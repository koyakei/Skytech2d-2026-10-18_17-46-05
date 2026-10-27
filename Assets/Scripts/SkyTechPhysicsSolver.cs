using System;
using UnityEngine;

namespace SkyTechSimulator
{
    public enum SkiGripState
    {
        Carving,
        Unweighted,
        Skidding,
        RailLimitHit,
        BootOut,
        HipCrash
    }

    public struct SimInput
    {
        public float u_ext_norm;
        public float u_flex_norm;
        public float l1_target;
        public float phi_lock_cmd;
        public float phi_steer_cmd;
        public float theta2_des_cmd;      // 地面基準の絶対目標上体ロール角 [rad] (AirPodsなどからの直接指令)
        public bool has_theta2_des_cmd;   // 直接上体目標角指令が有効かどうか
        public float mouse_lean_out_norm; // 中立基準線より下 (0..1: 鉛直を超えてリーンアウト)
        public float mouse_mid_norm;      // 中立基準線(0: 鉛直) ～ Lean-in line(1: ストレート内倒)
        public float mouse_lean_in_norm;  // Lean-in lineより上 (0..1: ストレート内倒を超えて内倒)
        public Vector2 mouse_world;
        public bool has_mouse_world;
        public bool steer_neutral_cmd;
        public bool w_reset;
    }

    public struct SimState
    {
        public float x_c, v_c, theta1, omega1, l1, dl1, phi, dphi;
        public float phi_bend, dphi_bend, P_hip_steer, tau_hip_steer;
        public float u_shift_filtered, theta2_ref, y_CoM_ref, z_CoM_ref;
        public float dy_cmd, dy_actual, dy_min_torque, dy_max_torque;
        public float V, stamina;
        public float y_CoM, z_CoM, Psi_CoM, R_CoM, z_H, F_leg, F_compress, u_ext_actual;
        public float I_eff, F_vert, G_z_up, F_lat_step, P_step_eff, snow_stress_ratio, skid_intensity;
        public float F_snow_normal, edge_penetration_mm, F_grip_max, F_grip_actual;
        public float v_c_target, v_c_recoil_max, skid_pushback_force;
        public float s_rom, knee_flex_deg, knee_int_deg, posture_force_ratio, P_leg_ext, P_leg_peak_avail;
        public float G_base, lambda_sustain, a_c, kappa, kappa_target;
        public SkiGripState gripState;
    }

    public static class SkyTechPhysicsSolver
    {
        public const float FreePlayDeg = 2.0f;
        public const float SupportHoldDropMeters = 0.15f;
        public const float SupportFadeZoneMeters = 0.03f;

        // リーンアウト時にストレート内倒へ移行する脚ロール角（80度）と移行幅（72度〜80度）
        public const float MaxLeanOutLegRollDeg = 80.0f;
        public const float StraightTransitionWidthDeg = 8.0f;

        private static float BoolGate(bool b) => Convert.ToSingle(b);
        private static float SmoothSign(float x, float eps = 1e-3f) => (float)Math.Tanh(x / SoftMax(1e-6f, eps));
        private static float SoftPlus(float x, float eps = 1e-5f) => 0.5f * (x + Mathf.Sqrt(x * x + eps * eps));
        private static float SoftMin(float a, float b, float eps = 1e-5f) => 0.5f * (a + b - Mathf.Sqrt((a - b) * (a - b) + eps * eps));
        private static float SoftMax(float a, float b, float eps = 1e-5f) => 0.5f * (a + b + Mathf.Sqrt((a - b) * (a - b) + eps * eps));

        private static float AnalyticClamp(float x, float minVal, float maxVal, float eps = 1e-5f)
        {
            float da = x - minVal, db = x - maxVal;
            return 0.5f * (minVal + maxVal + Mathf.Sqrt(da * da + eps * eps) - Mathf.Sqrt(db * db + eps * eps));
        }

        private static float AnalyticClamp01(float x) => AnalyticClamp(x, 0.0f, 1.0f);
        private static float AnalyticStep(float x, float center, float width) => 0.5f * (1.0f + (float)Math.Tanh((x - center) / SoftMax(1e-5f, width)));

        private static float Phi0(float z) => 1.0f + z * (0.5f + z * ((1.0f / 24.0f) + z * ((1.0f / 720.0f) + z * (1.0f / 40320.0f))));
        private static float Phi1(float z) => 1.0f + z * ((1.0f / 6.0f) + z * ((1.0f / 120.0f) + z * ((1.0f / 5040.0f) + z * (1.0f / 362880.0f))));

        private static float SolveFirstOrderExact(float z0, float zInf, float tau, float dt) =>
            zInf + (z0 - zInf) * Mathf.Exp(-dt / SoftMax(1e-4f, tau));

        private static void SolveSecondOrderExact(
            ref float q, ref float v, out float a,
            float qEq, float omega0Sq, float gamma, float fExt, float dt)
        {
            float g = AnalyticClamp(gamma, -15.0f, 250.0f);
            float f0 = fExt - omega0Sq * (q - qEq);
            float gdt = g * dt;
            float w2dt2 = omega0Sq * dt * dt;
            float z = gdt * gdt - w2dt2;

            float expFactor = Mathf.Exp(-gdt);
            float p0 = Phi0(z), p1 = Phi1(z);

            float G1 = expFactor * dt * p1;
            float G0 = expFactor * (p0 - gdt * p1);
            float G2 = expFactor * dt * dt * (0.5f + (1.0f / 6.0f) * gdt + (1.0f / 24.0f) * z + (1.0f / 60.0f) * gdt * z + (1.0f / 720.0f) * z * z);

            float vPrev = v;
            q += G1 * vPrev + G2 * f0;
            v = G0 * vPrev + G1 * f0 - w2dt2 * (G2 / dt) * vPrev;
            a = (v - vPrev) / dt;
        }

        private static void SolveSecondOrderBoundedExact(
            ref float q, ref float v, out float a,
            float qEq, float omega0Sq, float gamma, float fExt, float qMin, float qMax, float dt)
        {
            float qPrev = q, vPrev = v;
            SolveSecondOrderExact(ref q, ref v, out a, qEq, omega0Sq, gamma, fExt, dt);

            float qClamped = AnalyticClamp(q, qMin, qMax);
            float inBoundsGate = AnalyticStep(q - qMin, 0.0f, 1e-4f) * AnalyticStep(qMax - q, 0.0f, 1e-4f);

            v = v * inBoundsGate + ((qClamped - qPrev) / dt) * (1.0f - inBoundsGate);
            q = qClamped;
            a = (v - vPrev) / dt;
        }

        private static float SolveRiccatiSpeedExact(float v0, float aEff, float cDrag, float dt)
        {
            float z = aEff * cDrag * dt * dt;
            float p0 = Phi0(z), p1 = Phi1(z);
            return (p0 * v0 + aEff * dt * p1) / SoftMax(1e-4f, p0 + cDrag * v0 * dt * p1);
        }

        public static SimState CreateInitialState(SkyTechSimConfig cfg)
        {
            float initialL1 = AnalyticClamp(cfg.l_max - cfg.supportDropFromMaxMeters, cfg.l_min, cfg.l_max);
            float mg = cfg.M * cfg.g;

            var s = new SimState
            {
                l1 = initialL1,
                z_CoM_ref = initialL1,
                V = cfg.V_ref,
                stamina = cfg.maxStamina,
                F_compress = mg,
                F_leg = mg,
                F_vert = mg,
                F_snow_normal = mg,
                edge_penetration_mm = 0f,
                F_grip_max = mg,
                F_grip_actual = 0f,
                lambda_sustain = 1.0f,
                gripState = SkiGripState.Carving
            };

            UpdateKinematics(ref s, cfg);
            EvaluateLegBiomechanics(s.l1, cfg, out s.s_rom, out s.knee_flex_deg, out s.knee_int_deg, out s.posture_force_ratio, out _, out _, out s.P_leg_peak_avail);
            s.I_eff = cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1 + cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2);
            s.y_CoM_ref = s.y_CoM;
            s.z_CoM_ref = s.z_CoM;
            return s;
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float l_min = (cfg != null) ? cfg.l_min : 0.40f;
            float l_full = (cfg != null) ? SoftMax(l_max + 0.01f, cfg.l_full) : (l_max + 0.02f);
            float s_rom = AnalyticClamp01((l1 - l_min) / SoftMax(0.10f, l_max - l_min));
            float normCos = AnalyticClamp(l1 / l_full, 0.01f, 1.0f) / SoftMax(1e-4f, AnalyticClamp(l_max / l_full, 0.01f, 1.0f));
            return AnalyticClamp01(Mathf.Lerp(0.35f, 1.0f, s_rom) * normCos);
        }

        public static float ComputeSquatRecoveryFactor(float l1, float l_max, SkyTechSimConfig cfg = null) =>
            AnalyticStep(l_max - SupportHoldDropMeters - l1, 0.5f * SupportFadeZoneMeters, 0.35f * SupportFadeZoneMeters);

        public static void EvaluateLegBiomechanics(
            float l1, SkyTechSimConfig cfg,
            out float s_rom, out float knee_flex_deg, out float knee_int_deg,
            out float phi_F, out float F_iso, out float v_max_eff, out float P_peak_avail)
        {
            float romSpan = SoftMax(0.10f, cfg.l_max - cfg.l_min);
            s_rom = AnalyticClamp01((l1 - cfg.l_min) / romSpan);

            float l_full = SoftMax(cfg.l_max + 0.01f, cfg.l_full);
            knee_flex_deg = 2.0f * Mathf.Acos(AnalyticClamp(l1 / l_full, 0.01f, 0.9999f)) * Mathf.Rad2Deg;
            knee_int_deg = 180.0f - knee_flex_deg;

            float s_opt = AnalyticClamp(cfg.legOptRomRatio, 0.50f, 0.95f);
            float l_opt = cfg.l_min + s_opt * romSpan;
            float r_opt = Mathf.Sqrt(SoftPlus(l_full * l_full - l_opt * l_opt, 1e-4f));
            float r_cur = Mathf.Sqrt(SoftPlus(l_full * l_full - l1 * l1, 1e-4f));
            float leverTerm = SoftMin(1.0f, Mathf.Pow(r_opt / r_cur, cfg.kneeLeverPenaltyExp));

            float side = AnalyticStep(s_rom, s_opt, 0.012f);
            float ds = s_rom - s_opt;

            float sigmaF = SoftMax(0.05f, Mathf.Lerp(cfg.squatForceDropWidth, cfg.lockoutForceDropWidth, side));
            float fBase = AnalyticClamp01(Mathf.Lerp(cfg.squatMinForceRatio, cfg.lockoutMinForceRatio, side));
            phi_F = fBase + (1.0f - fBase) * leverTerm * Mathf.Exp(-(ds * ds) / (sigmaF * sigmaF));

            float sigmaV = SoftMax(0.05f, Mathf.Lerp(cfg.squatVelDropWidth, cfg.lockoutVelDropWidth, side));
            float vBase = AnalyticClamp01(Mathf.Lerp(cfg.squatMinVelRatio, cfg.lockoutMinVelRatio, side));
            float phi_v = vBase + (1.0f - vBase) * Mathf.Exp(-(ds * ds) / (sigmaV * sigmaV));

            F_iso = cfg.F_ext_max_G * cfg.M * cfg.g * phi_F;
            v_max_eff = SoftMax(0.20f, cfg.v_max * phi_v);

            float pHill = SoftMax(1.0f, cfg.hillPowerExponent);
            float vOptRatio = Mathf.Pow(1.0f / (pHill + 1.0f), 1.0f / pHill);
            P_peak_avail = F_iso * (v_max_eff * vOptRatio) * (1.0f - Mathf.Pow(vOptRatio, pHill));
        }

        public static void Step(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            float safeDt = SoftMax(1e-5f, dt);
            float totalMass = SoftMax(1.0f, cfg.m1 + cfg.m2);
            float mg = cfg.M * cfg.g;
            float m1c1Sq_plus_m2 = cfg.m1 * cfg.c1 * cfg.c1 + cfg.m2;
            float L1 = ((cfg.m1 * cfg.c1 + cfg.m2) / totalMass) * s.l1;
            float L2 = (cfg.m2 / totalMass) * cfg.l2;
            float m2_l2 = cfg.m2 * cfg.l2;
            float I2_hip = SoftMax(0.5f, cfg.m2 * cfg.l2 * cfg.l2);
            float prev_y_CoM = s.y_CoM;

            float cosPhi = Mathf.Cos(s.phi), sinPhi = Mathf.Sin(s.phi);
            s.I_eff = SoftMax(2.0f, m1c1Sq_plus_m2 * s.l1 * s.l1 + cfg.m2 * (cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2 * cosPhi));
            float half_dI_dl1 = m1c1Sq_plus_m2 * s.l1 + m2_l2 * cosPhi;
            float dI_dphi = -2.0f * m2_l2 * s.l1 * sinPhi;

            float phiMax = SoftMax(0.85f, cfg.phi_out_maxRad);
            float phiInMax = SoftMax(0.20f, Mathf.Abs(cfg.phi_in_maxRad));
            float tauHipMax = SoftMax(100.0f, cfg.hipSteerMaxIsoTorque);
            float sin1 = Mathf.Sin(s.theta1), cos1 = Mathf.Cos(s.theta1);
            float absTheta1 = SoftMin(Mathf.Abs(s.theta1), 90f * Mathf.Deg2Rad);
            float cosAbsTheta1 = SoftMax(0.15f, Mathf.Cos(absTheta1));
            float sinAbsTheta1 = Mathf.Sin(absTheta1);

            // =========================================================================
            // 1. マウス高さ基準線による姿勢マッピング ＆ 80度ストレート内倒移行制御
            // =========================================================================
            float gReset = BoolGate(input.w_reset);
            float activeGate = (1.0f - gReset) * (1.0f - BoolGate(input.steer_neutral_cmd));

            float u_x = activeGate * AnalyticClamp(-input.phi_steer_cmd, -1.0f, 1.0f);
            float tauShift = SoftMax(0.020f, cfg.comShiftResponseTau * 0.75f);
            float prev_u_shift = s.u_shift_filtered;
            s.u_shift_filtered = (1.0f - gReset) * SolveFirstOrderExact(prev_u_shift, u_x, tauShift, safeDt);
            float du_shift_rate = (s.u_shift_filtered - prev_u_shift) / safeDt;

            float inwardReturnRate = SoftPlus(-SmoothSign(s.u_shift_filtered, 0.05f) * du_shift_rate);
            float shiftActivity = AnalyticClamp01(SoftMax(Mathf.Abs(u_x), Mathf.Abs(s.u_shift_filtered)));
            float forceOverride = cfg.mouseForceOverrideRatio * shiftActivity;

            // --- 脚のロール角80度に向けたストレート内倒移行ブレンド係数の算出 ---
            float maxLegRollLimitRad = SoftMin(MaxLeanOutLegRollDeg * Mathf.Deg2Rad, cfg.theta_bootOutRad - 2.0f * Mathf.Deg2Rad);
            float transWidthRad = SoftMax(2.0f * Mathf.Deg2Rad, StraightTransitionWidthDeg * Mathf.Deg2Rad);
            float transStartRad = maxLegRollLimitRad - transWidthRad;

            // 現在の脚ロール角 |θ1| が 72°〜80° になるにつれて 0 -> 1 へ滑らかに遷移し、80度未満に起き上がると0へ戻る
            float rawLimitRatio = AnalyticClamp01((absTheta1 - transStartRad) / transWidthRad);
            float straightLockBlend = rawLimitRatio * rawLimitRatio * (3.0f - 2.0f * rawLimitRatio);

            // マウス操作による目標脚ロール角の上限を80度にする（それ以上は外傾で脚を押し倒さない）
            float maxTargetRollRad = maxLegRollLimitRad;
            float u_shift_abs = Mathf.Abs(s.u_shift_filtered);
            float smoothFactor = u_shift_abs / Mathf.Sqrt(u_shift_abs * u_shift_abs + 0.015f);
            float mouseTargetTheta1 = s.u_shift_filtered * Mathf.Lerp(0.85f, 1.0f, smoothFactor) * maxTargetRollRad;
            
            float theta1_des = activeGate * AnalyticClamp(mouseTargetTheta1 + input.phi_lock_cmd, -maxTargetRollRad, maxTargetRollRad);
            float rollError = theta1_des - s.theta1;

            float rollSignSmooth = SmoothSign(s.theta1, 8.0f * Mathf.Deg2Rad);
            float shiftSignSmooth = SmoothSign(s.u_shift_filtered, 0.12f);
            float oppositeInputGate = AnalyticClamp01(SoftPlus(-rollSignSmooth * shiftSignSmooth));
            float leanOutGate = 1.0f - oppositeInputGate;

            // 80度に近づくにつれてリーンアウト入力を滑らかに絞り、ストレート内倒へ移行させる
            float leanOutNorm = AnalyticClamp01(input.mouse_lean_out_norm) * (1.0f - straightLockBlend);
            float effLeanOut = leanOutNorm * leanOutGate;
            float midNorm = AnalyticClamp01(input.mouse_mid_norm);
            float leanInNorm = AnalyticClamp01(input.mouse_lean_in_norm);
            float deepRollFactor = AnalyticClamp01(absTheta1 / (55.0f * Mathf.Deg2Rad));

            float neutralL1 = AnalyticClamp(cfg.l_max - cfg.supportDropFromMaxMeters, cfg.l_min, cfg.l_max);
            float headReach = 1.55f * cfg.l2;

            // 1. 重力と横方向慣性力(遠心力)の釣り合い角 theta_force
            float theta_force = Mathf.Atan2(s.a_c, cfg.g);
            float turnSign = SmoothSign(s.theta1, 1.5f * Mathf.Deg2Rad);
            float absForceAngle = SoftPlus(turnSign * theta_force);
            
            float theta2_at_leanInLine = s.theta1;
            float theta2_at_centerLine = s.theta1 * 0.15f;
            float theta2_sweet_spot = Mathf.Lerp(theta2_at_centerLine, theta2_at_leanInLine, 0.48f);

            // 解剖学的な股関節可動域 (Configの設定値を厳守: 外傾上限 phiMax, 内倒上限 phiInMax)
            float turnDirSign = SmoothSign(s.u_shift_filtered + 0.65f * Mathf.Sin(s.theta1), 0.05f);
            float maxPosPhi = Mathf.Lerp(phiInMax, phiMax, 0.5f * (1.0f + turnDirSign));
            float maxNegPhi = Mathf.Lerp(phiMax, phiInMax, 0.5f * (1.0f + turnDirSign));
            float phi_target;
            float activeLeanInIntent = leanInNorm;
            float activePostureIntent = AnalyticClamp01(leanOutNorm + leanInNorm);

            if (input.has_theta2_des_cmd)
            {
                // ★ 顔(AirPods)による上体姿勢制御:
                // 顔のロール角度がそのまま地面基準の上体絶対ロール角 (θ2,target) になり、
                // 顔が鉛直(0 rad)の時は地面基準で完全に鉛直 (θ2,target = 0) を維持する。
                float theta2_cmd_target = input.theta2_des_cmd;
                float phi_direct = s.theta1 - theta2_cmd_target - input.phi_lock_cmd;
                float phi_clamped = AnalyticClamp(phi_direct, -maxNegPhi, maxPosPhi);

                // 外傾成分（ターン外側への屈曲）のみ80度到達時に0へ絞り込み、内倒成分は解剖学的上限まで許容
                float outwardComponent = turnDirSign * SoftPlus(turnDirSign * phi_clamped);
                float inwardComponent = phi_clamped - outwardComponent;
                phi_target = (1.0f - gReset) * AnalyticClamp(
                    outwardComponent * (1.0f - straightLockBlend) + inwardComponent,
                    -maxNegPhi, maxPosPhi
                );

                activeLeanInIntent = AnalyticClamp01(SoftPlus(-turnDirSign * phi_clamped) / SoftMax(0.1f, phiInMax));
                activePostureIntent = AnalyticClamp01(Mathf.Abs(phi_clamped) / SoftMax(0.1f, phiInMax));
            }
            else
            {
                float highRollGate = AnalyticClamp01((absTheta1 - 22.0f * Mathf.Deg2Rad) / (25.0f * Mathf.Deg2Rad));
                float fastTurnGate = AnalyticClamp01((Mathf.Abs(s.a_c) / cfg.g - 0.5f) / 1.0f);
                // midNorm が 0.35 (LEAN-IN基準線) を超えたら即座に外傾固定を急解除し、ストレート内倒へ移行させる
                float manualStraightRelease = AnalyticClamp01((midNorm - 0.35f) / 0.45f);
                float sweetSpotLock = 0.65f * highRollGate * fastTurnGate * (1.0f - manualStraightRelease) * (1.0f - leanInNorm);
                float raw_theta2_mid = Mathf.Lerp(theta2_at_centerLine, theta2_at_leanInLine, midNorm);
                float theta2_mid_target = Mathf.Lerp(raw_theta2_mid, theta2_sweet_spot, sweetSpotLock);

                float phi_mid_zone = s.theta1 - theta2_mid_target;
                float phi_extra_lean_out = leanOutNorm * (theta1_des * cfg.trunkLeanOutRatio + s.u_shift_filtered * (phiMax * 0.45f) * (1.0f - deepRollFactor * 0.4f));
                // STRAIGHT LINE 以上では、旋回・ステア方向へストレート内倒を超えてリーンイン（解剖学的内倒上限 phiInMax まで）
                float turnOrSteerGate = AnalyticClamp01(SoftMax(Mathf.Abs(s.u_shift_filtered) * 2.2f, Mathf.Abs(s.theta1) / (12.0f * Mathf.Deg2Rad)));
                float phi_extra_lean_in = -leanInNorm * turnDirSign * turnOrSteerGate * phiInMax;

                // 80度到達時は外傾成分のみを 0 (ストレート内倒) へ滑らかに収束させる（手動リーンイン操作は妨げない）
                float phi_without_lean_in = AnalyticClamp(phi_mid_zone + phi_extra_lean_out - input.phi_lock_cmd, -phiMax, phiMax);
                phi_target = (1.0f - gReset) * AnalyticClamp(
                    phi_without_lean_in * (1.0f - straightLockBlend) + phi_extra_lean_in,
                    -maxNegPhi, maxPosPhi
                );
            }

            s.theta2_ref = s.theta1 - phi_target;
            s.y_CoM_ref = L1 * sin1 + L2 * Mathf.Sin(s.theta2_ref);
            s.z_CoM_ref = SoftMax(0.18f, L1 * cos1 + L2 * Mathf.Cos(s.theta2_ref));
            float G_eq = Mathf.Lerp(s.y_CoM_ref / s.z_CoM_ref, s.y_CoM / SoftMax(0.18f, s.z_CoM), 0.75f);

            float L0 = SoftMax(0.1f, cfg.hipMuscleOptLengthL0);
            float strainScale = -0.35f * (L0 - AnalyticClamp(cfg.hipMuscleMinLengthLmin, 0.05f, L0)) / (SoftMax(0.25f, cfg.hipMuscleGaussianWidthW) * L0);
            float fullRomPhi = phiMax + 45.0f * Mathf.Deg2Rad;
            float EvalHipTorque(float dirPhi)
            {
                float ns = strainScale * AnalyticClamp01(SoftPlus(dirPhi) / fullRomPhi);
                return tauHipMax * Mathf.Exp(-(ns * ns));
            }

            float tau_hip_avail_pos = EvalHipTorque(s.phi);
            float tau_hip_avail_neg = EvalHipTorque(-s.phi);

            float g_eff_limit = Mathf.Lerp(Mathf.Sqrt(cfg.g * cfg.g + s.a_c * s.a_c), cfg.g * 0.65f, forceOverride);
            float invM2L2G = 1.0f / SoftMax(1.0f, m2_l2 * g_eff_limit);
            float maxAngShiftRadPos = SoftMin(phiMax, Mathf.Asin(AnalyticClamp(tau_hip_avail_pos * invM2L2G, 0.0f, 0.999f)));
            float maxAngShiftRadNeg = SoftMin(phiMax, Mathf.Asin(AnalyticClamp(tau_hip_avail_neg * invM2L2G, 0.0f, 0.999f)));

            float L_eff = SoftMax(0.22f, L1 + 0.35f * L2);
            float halfPi = 90.0f * Mathf.Deg2Rad;
            s.dy_min_torque = L_eff * (Mathf.Sin(SoftMax(-halfPi, s.theta1 - maxAngShiftRadNeg)) - sin1);
            s.dy_max_torque = L_eff * (Mathf.Sin(SoftMin(halfPi, s.theta1 + maxAngShiftRadPos)) - sin1);

            float dy_desired = s.u_shift_filtered * SoftMax(0.05f, cfg.comShiftMaxMeters) * (1.0f + 0.55f * forceOverride);
            s.dy_cmd = Mathf.Lerp(AnalyticClamp(dy_desired, s.dy_min_torque, s.dy_max_torque), dy_desired, forceOverride);
            float delta_theta1_cmd = Mathf.Asin(AnalyticClamp(sin1 + s.dy_cmd / L_eff, -0.99f, 0.99f)) - s.theta1;

            float chi_up = oppositeInputGate;

            // =========================================================================
            // 2. 下肢伸縮運動方程式 (l1, dl1) ＆ 頭頂高さ維持のマリオネット張力補償
            // =========================================================================
            EvaluateLegBiomechanics(s.l1, cfg, out s.s_rom, out s.knee_flex_deg, out s.knee_int_deg, out s.posture_force_ratio, out float F_iso, out float v_max_eff, out s.P_leg_peak_avail);

            float dropHold = cfg.supportDropFromMaxMeters;
            float hasTargetGate = AnalyticStep(input.l1_target, 0.05f, 0.015f);
            float u_flex = AnalyticClamp01(input.u_flex_norm);
            float baseKeyHold = Mathf.Lerp(Mathf.Lerp(neutralL1, cfg.l_max, AnalyticClamp01(input.u_ext_norm)), cfg.l_min, u_flex);
            float baseDesiredL1 = Mathf.Lerp(baseKeyHold, AnalyticClamp(input.l1_target, cfg.l_min, cfg.l_max), hasTargetGate);

            float hammockEngage = input.has_theta2_des_cmd
                ? 0.0f
                : (1.0f - midNorm) * leanOutGate * (1.0f - leanInNorm) * (1.0f - straightLockBlend);
            float curHeadZ = s.l1 * cos1 + headReach * Mathf.Cos(s.theta1 - s.phi);
            float targetHeadZ = baseDesiredL1 + headReach * Mathf.Cos(s.theta2_ref);
            float headHeightDeficit = SoftPlus(targetHeadZ - curHeadZ) * hammockEngage * (1.0f - u_flex);
            float puppetLiftL1 = AnalyticClamp(headHeightDeficit * 0.65f / SoftMax(0.40f, cosAbsTheta1), 0.0f, cfg.l_max - baseDesiredL1);
            float l_hold = AnalyticClamp(baseDesiredL1 + puppetLiftL1, cfg.l_min, cfg.l_max);

            float pHill = SoftMax(1.0f, cfg.hillPowerExponent);
            float fvFactor = 1.0f - Mathf.Pow(AnalyticClamp(SoftPlus(s.dl1) / v_max_eff, 0.0f, 0.999f), pHill) + 0.15f * AnalyticClamp01(SoftPlus(-s.dl1) / 1.50f);
            float returnPushBoost = AnalyticClamp01(inwardReturnRate / 2.5f) * AnalyticStep(absTheta1, 10.0f * Mathf.Deg2Rad, 5.0f * Mathf.Deg2Rad);
            float puppetExtDrive = AnalyticClamp01((l_hold - s.l1) / 0.08f) * hammockEngage;
            float u_norm = AnalyticClamp01(SoftMax(input.u_ext_norm, SoftMax(0.55f * puppetExtDrive, 0.65f * returnPushBoost * (1.0f - effLeanOut))));

            s.u_ext_actual = u_norm * (1.0f - u_flex) * F_iso * fvFactor * AnalyticClamp01((cfg.l_max - s.l1) / 0.025f) * AnalyticClamp01(s.stamina / 20.0f);

            float phi_support = ComputeSelfWeightSupportFactor(s.l1, cfg.l_max, cfg);
            float absPsi = Mathf.Abs(s.Psi_CoM);

            s.F_compress = SoftMax(0.1f * mg, mg * (Mathf.Cos(absPsi) + (Mathf.Abs(s.a_c) / cfg.g) * Mathf.Sin(absPsi)));

            float maxSpringForce = cfg.passiveSpringMaxG * mg;
            float springArg = (cfg.k_passive * Mathf.Abs(l_hold - s.l1) + 1.0f) / SoftMax(1.0f, maxSpringForce);
            float u_flex_eff = u_flex * AnalyticClamp01((s.l1 - l_hold) / 0.04f) * (1.0f - 0.65f * shiftActivity);
            float belowHoldGate = AnalyticStep(l_hold - s.l1, 0.0f, 0.005f);
            float k_stance_eff = cfg.k_passive * ((float)Math.Tanh(springArg) / springArg) * (1.0f - u_flex_eff) * belowHoldGate;

            float l_eq_safe = SoftMax(0.45f, cfg.l_eq);
            float c_ecc = SoftMax(2.0f * Mathf.Sqrt(cfg.M * SoftMax(100.0f, cfg.k_passive)) * 0.85f, cfg.c_ecc_factor * cfg.M * Mathf.Sqrt(cfg.g / l_eq_safe));
            float c_leg_eff = Mathf.Lerp(1.05f, 0.45f, AnalyticClamp01(u_norm + u_flex)) * c_ecc;

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex_eff);
            float effectiveLoad = Mathf.Lerp(mg, s.F_compress, AnalyticClamp01(u_norm + u_flex_eff + (1.0f - phi_support)));
            float fExtLeg = (selfWeightSupport + s.u_ext_actual + half_dI_dl1 * (s.omega1 * s.omega1) - effectiveLoad) / cfg.M;

            SolveSecondOrderBoundedExact(ref s.l1, ref s.dl1, out float ddl1, l_hold, k_stance_eff / cfg.M, c_leg_eff / (2.0f * cfg.M), fExtLeg, cfg.l_min, cfg.l_max, safeDt);

            float v_com_ext = SoftPlus(s.dl1);
            float v_z_ext = v_com_ext * cosAbsTheta1;
            float G_z_kin = (v_z_ext * v_z_ext) / (2.0f * cfg.g * SoftMax(0.12f, dropHold));
            float G_z_inst = (SoftPlus(ddl1) * cosAbsTheta1) / cfg.g;
            float extMotionGate = AnalyticClamp01(v_com_ext * 100.0f);
            s.G_z_up = SoftMax(u_norm * extMotionGate * SoftMax(G_z_kin, 0.55f * G_z_inst), SoftPlus(s.G_z_up - 3.8f * safeDt));

            float inertiaFactor = AnalyticClamp(s.I_eff / SoftMax(1.0f, cfg.M * l_eq_safe * l_eq_safe * 0.65f), 0.4f, 1.6f);
            s.u_ext_actual *= extMotionGate;
            s.P_step_eff = s.P_leg_ext = s.u_ext_actual * v_com_ext * inertiaFactor;

            float recoveryRate = (1.0f - AnalyticClamp01(u_norm * 20.0f)) * cfg.staminaRecoveryRate;
            s.stamina = AnalyticClamp(s.stamina + (recoveryRate - s.P_leg_ext) * safeDt, 0.0f, cfg.maxStamina);

            float stanceReturnForce = belowHoldGate * maxSpringForce * (float)Math.Tanh((cfg.k_passive * SoftPlus(l_hold - s.l1)) / SoftMax(1.0f, maxSpringForce)) * (1.0f - u_flex_eff);
            float vertSupportG = (Mathf.Lerp(phi_support, 1.0f, AnalyticClamp01(u_norm + hasTargetGate)) - (1.0f - phi_support) * s.G_z_up) * (1.0f - 0.85f * u_flex_eff);
            s.F_vert = mg * vertSupportG + stanceReturnForce - c_leg_eff * s.dl1 * cosAbsTheta1 + s.u_ext_actual * cosAbsTheta1;

            float F_vert_pos = SoftPlus(s.F_vert);
            float N_min = cfg.N_min_G * mg;
            float weightGate = AnalyticClamp01(F_vert_pos / SoftMax(1.0f, N_min));

            float v_y_ext = v_com_ext * sinAbsTheta1;
            s.F_lat_step = weightGate * ((s.u_ext_actual + cfg.M * SoftPlus(ddl1)) * sinAbsTheta1 + cfg.M * Mathf.Abs(s.a_c) * Mathf.Sin(absPsi));
            s.F_leg = SoftMax(F_vert_pos, F_vert_pos * cosAbsTheta1 + s.F_lat_step * sinAbsTheta1);

            // =========================================================================
            // 3. エッジ貫入深さ (Edge Penetration) と雪面グリップ力の物理計算
            // =========================================================================
            float effectiveRollRate = s.omega1 + 0.35f * (rollError / SoftMax(0.06f, tauShift));
            float sigma_phase = rollSignSmooth * SmoothSign(effectiveRollRate, 0.65f);

            float rollInDeficit = SoftPlus(SmoothSign(theta1_des, 2.0f * Mathf.Deg2Rad) * rollError);
            float intentionalRollIn = leanOutGate * AnalyticClamp01(rollInDeficit / (10.0f * Mathf.Deg2Rad));

            float sigma_in = AnalyticClamp01(SoftPlus(sigma_phase)) * AnalyticStep(absTheta1, 15.0f * Mathf.Deg2Rad, 6.0f * Mathf.Deg2Rad);
            float sigma_out = AnalyticClamp01(SoftPlus(-sigma_phase));

            float legNormalComponent = SoftPlus(s.F_leg) * Mathf.Cos(absTheta1);
            float wLeg = AnalyticClamp01(cfg.normalForceLegWeight);
            s.F_snow_normal = SoftPlus((1.0f - wLeg) * F_vert_pos + wLeg * legNormalComponent);

            float biteRefRad = SoftMax(0.25f, cfg.edgeBiteRefDeg) * Mathf.Deg2Rad;
            float edgeContactEngagement = absTheta1 / Mathf.Sqrt(absTheta1 * absTheta1 + biteRefRad * biteRefRad);
            float sinWedge = Mathf.Pow(SoftMax(1e-4f, sinAbsTheta1), SoftMax(0.5f, cfg.edgeWedgeExponent));
            float cosWedge = Mathf.Sqrt(cos1 * cos1 + 0.0625f);
            float wedgeEfficiency = edgeContactEngagement * (1.0f + SoftMax(0.5f, cfg.edgeWedgeBoost) * (sinWedge / cosWedge));

            float K_snow_safe = SoftMax(50000.0f, cfg.snowHardnessK);
            float L_edge_safe = SoftMax(0.50f, cfg.effectiveEdgeLength);
            float rawPenetrationMeters = (s.F_snow_normal / (K_snow_safe * L_edge_safe)) * wedgeEfficiency;
            float maxPenMeters = SoftMax(0.005f, cfg.maxEdgePenetrationMeters);
            float d_p = maxPenMeters * (float)Math.Tanh(rawPenetrationMeters / maxPenMeters);
            s.edge_penetration_mm = d_p * 1000.0f;

            s.F_grip_max = L_edge_safe * d_p * K_snow_safe;
            float safeGripMax = SoftMax(1.0f, s.F_grip_max);

            float vRefSafe = SoftMax(4.0f, cfg.V_ref);
            float rScSafe = SoftMax(3.0f, cfg.R_sc);
            float speedRatio = s.V / vRefSafe;
            float speedScale = 0.55f + 0.45f * AnalyticClamp(speedRatio, 0.50f, 1.65f);

            float G0_effective = 0.5f * (cfg.G0_curve + 2.20f * (vRefSafe * vRefSafe) / (cfg.g * rScSafe));
            float edgeSignSmooth = SmoothSign(s.theta1, SoftMax(0.3f * Mathf.Deg2Rad, cfg.theta_deadRad));
            float howeCurve = edgeSignSmooth * (Mathf.Pow(SoftMax(1e-4f, sinAbsTheta1), SoftMax(0.75f, cfg.sinPower_p)) / SoftMax(0.18f, Mathf.Cos(cfg.k_flex * absTheta1)));

            float angulationAlignment = SoftPlus(edgeSignSmooth * (s.phi / phiMax));
            float angulationBoost = 1.0f + cfg.k_ang_boost * angulationAlignment;
            float G_howe = G0_effective * angulationBoost * howeCurve * (speedRatio * speedRatio);

            float penetrationBite = AnalyticClamp01(s.edge_penetration_mm / 0.65f);
            float shallowRollEntryGate = AnalyticStep(absTheta1, 20.0f * Mathf.Deg2Rad, 7.0f * Mathf.Deg2Rad);
            float eqScale = Mathf.Lerp(0.85f, 1.15f, penetrationBite) * (1.0f - 0.35f * intentionalRollIn * (1.0f - shallowRollEntryGate));
            float G_com_equilibrium = Mathf.Lerp(G_eq, sin1 / cosAbsTheta1, 0.15f) * eqScale * speedScale;

            float howeBlend = AnalyticClamp01(0.25f * penetrationBite + 0.75f * shallowRollEntryGate * (1.0f - 0.30f * intentionalRollIn) + 0.15f * angulationAlignment);
            float phaseGMod = (1.0f - 0.10f * sigma_in + 0.22f * sigma_out) * Mathf.Lerp(1.0f, 0.72f, intentionalRollIn * (1.0f - shallowRollEntryGate));

            s.G_base = Mathf.Lerp(G_com_equilibrium, G_howe, howeBlend) * penetrationBite * phaseGMod;

            float F_req_ideal = mg * Mathf.Cos(absPsi);
            float loadRatio = s.F_leg / SoftMax(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float maxLambda = Mathf.Lerp(1.0f, SoftMax(1.2f, cfg.lambda_max), AnalyticClamp01((s.P_leg_ext + s.F_lat_step * v_y_ext) / 600.0f));
            s.lambda_sustain = AnalyticClamp(Mathf.Pow(AnalyticClamp(loadRatio, 0.85f, 1.8f), cfg.loadSustainPower_q), 0.92f, maxLambda);

            float viableGate = AnalyticStep(s.z_H, cfg.z_H_min, 2e-3f) * AnalyticStep(cfg.theta_bootOutRad, absTheta1, 2e-3f);
            s.v_c_recoil_max = v_y_ext;

            float F_carve_demand = cfg.M * (s.G_base * cfg.g) * s.lambda_sustain * weightGate;
            float F_camber_rebound = weightGate * sigma_out * 0.22f * s.F_leg * sin1 * speedScale;
            float activeExtDrive = weightGate * Mathf.Lerp(-0.5f, 1.0f, sigma_out) * s.u_ext_actual * sin1 * 0.45f;
            float F_total_lat_demand = F_carve_demand + F_camber_rebound + activeExtDrive;

            float absDemand = Mathf.Abs(F_total_lat_demand);
            s.snow_stress_ratio = absDemand / safeGripMax;

            float targetSkid = AnalyticClamp01((s.snow_stress_ratio - 1.0f) / 0.50f);
            float skidTau = (targetSkid > s.skid_intensity) ? 0.04f : SoftMax(0.05f, cfg.skidRecoveryTau);
            s.skid_intensity = SolveFirstOrderExact(s.skid_intensity, targetSkid, skidTau, safeDt);

            float carvingRetention = 1.0f - (1.0f - cfg.skidGripMultiplier) * s.skid_intensity;
            float normDemand = absDemand / safeGripMax;
            float linearHoldMag = absDemand / Mathf.Pow(1.0f + Mathf.Pow(normDemand, 4.0f), 0.25f);
            s.F_grip_actual = viableGate * SmoothSign(F_total_lat_demand, 1.0f) * linearHoldMag * carvingRetention;

            s.skid_pushback_force = -SmoothSign(s.theta1, 1e-2f) * s.skid_intensity * cfg.carriagePushbackGain * SoftPlus(absDemand - s.F_grip_max);

            float a_c_target = viableGate * ((s.F_grip_actual + s.skid_pushback_force) / cfg.M);
            s.a_c = viableGate * SolveFirstOrderExact(s.a_c, a_c_target, AnalyticClamp(cfg.tau_G_lag, 0.010f, 0.025f), safeDt);

            SolveSecondOrderExact(ref s.x_c, ref s.v_c, out float ddx_c, 0.0f, 0.0f, 0.0f, s.a_c, safeDt);
            s.v_c_target = s.v_c;

            float vSqSafe = SoftMax(4.0f, s.V * s.V);
            s.kappa = ddx_c / vSqSafe;
            s.kappa_target = a_c_target / vSqSafe;

            float a_pivot_total = ddx_c;

            // =========================================================================
            // 4. 2リンク回転運動方程式 ＆ 股関節サーボ
            // =========================================================================
            float ankleCopShift = 0.02f * (theta1_des - s.theta1) * (1.0f - 0.5f * shallowRollEntryGate);
            float effective_y_CoM = s.y_CoM + ankleCopShift;

            float tau_gravity = (1.0f - gReset) * mg * effective_y_CoM * (1.0f + 0.15f * sigma_in * Mathf.Sin(cfg.alphaRad));
            float tau_inertial_com = -(1.0f - gReset) * cfg.M * a_pivot_total * s.z_CoM;

            float pendulumCritDamping = 2.0f * Mathf.Sqrt(s.I_eff * mg * SoftMax(0.30f, s.z_CoM));
            float coContractGain = AnalyticClamp(cfg.adaptiveDampingBoost, 0.5f, 5.0f);
            float rollReleaseFactor = (1.0f - 0.85f * intentionalRollIn) * (1.0f - 0.75f * oppositeInputGate) * Mathf.Lerp(0.35f, 1.0f, shallowRollEntryGate);
            float autoImpedanceDamping = pendulumCritDamping * (0.12f * rollReleaseFactor + 0.20f * coContractGain * (1.0f - shiftActivity));
            float totalRollDamping = SoftMax(cfg.gamma_roll * 0.45f * rollReleaseFactor, autoImpedanceDamping) + SoftMax(40.0f, cfg.gamma_quad) * 0.18f * Mathf.Abs(s.omega1) * rollReleaseFactor;

            // A. 股関節相対角 phi(t)
            float theta2_cur = s.theta1 - s.phi;
            float theta2_target = s.theta1 - phi_target;
            float a_hip_inertial = a_pivot_total;
            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - a_hip_inertial * Mathf.Cos(theta2_cur));
            float tau_hip_ff = (1.0f - gReset) * m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - a_hip_inertial * Mathf.Cos(theta2_target));

            float kp_hip_eff = SoftMax(950.0f, cfg.kp_hip) * (1.0f + 0.85f * shiftActivity + 0.65f * chi_up + 0.65f * hammockEngage + 0.85f * straightLockBlend);
            float postureOmegaFollow = input.has_theta2_des_cmd
                ? (1.0f - activeLeanInIntent) * (1.0f - straightLockBlend)
                : ((1.0f - midNorm) * (1.0f - leanInNorm) + 0.35f * effLeanOut) * (1.0f - straightLockBlend);
            float dphi_target = (1.0f - gReset) * postureOmegaFollow * s.omega1;
            float dphi_error = s.dphi - dphi_target;

            float hipCoContraction = 1.0f + 0.35f * cfg.adaptiveDampingBoost * (1.0f - 0.5f * shiftActivity) + 0.25f * cfg.adaptiveDampingBoost * AnalyticClamp01(Mathf.Abs(dphi_error) / SoftMax(0.05f, cfg.coContractionVelThreshold));
            float kd_hip_eff = SoftMax(cfg.kd_hip, 1.95f * Mathf.Sqrt(I2_hip * kp_hip_eff) * hipCoContraction);

            float I_couple = I2_hip + m2_l2 * s.l1 * cosPhi;
            float tau_hip_pd = kp_hip_eff * (phi_target - s.phi) - kd_hip_eff * dphi_error;
            float limitTorqueBoost = 1.0f + 0.65f * straightLockBlend;
            // フィードフォワード補償(tau_hip_ff)も含めた股関節の総発生トルクを、生体筋力上限(tau_hip_avail)内に厳密にクランプする
            s.tau_hip_steer = (1.0f - gReset) * AnalyticClamp(
                tau_hip_ff + tau_hip_pd,
                -tau_hip_avail_neg * 1.5f * limitTorqueBoost,
                tau_hip_avail_pos * 1.5f * limitTorqueBoost
            );

            float lockAllowance = Mathf.Abs(input.phi_lock_cmd) * (1.0f - straightLockBlend);
            float allowPos = SoftMax(0.0f, SoftMax(s.theta1, theta1_des)) + lockAllowance + 15.0f * Mathf.Deg2Rad;
            float allowNeg = SoftMin(0.0f, SoftMin(s.theta1, theta1_des)) - lockAllowance - 15.0f * Mathf.Deg2Rad;
            
            float baseDynMaxPhi = Mathf.Lerp(SoftMin(maxPosPhi, allowPos), maxPosPhi, activePostureIntent);
            float baseDynMinPhi = Mathf.Lerp(SoftMax(-maxNegPhi, allowNeg), -maxNegPhi, activePostureIntent);

            // 80度に達するにつれて外傾側の可動上限を 0 (ストレート内倒) へ滑らかに絞り込むとともに、
            // 急ロール時の勢いで phi = 0 を通り越してリーンイン(逆屈曲)しないよう反対側も 0 へガードする
            float posTurnGate = AnalyticStep(s.theta1, 0.0f, 2.0f * Mathf.Deg2Rad);
            float negTurnGate = 1.0f - posTurnGate;
            float dynMaxPhi = baseDynMaxPhi * (1.0f - straightLockBlend * posTurnGate);
            float dynMinPhi = baseDynMinPhi * (1.0f - straightLockBlend * negTurnGate);

            float allowLeanInAtLimit = activeLeanInIntent;
            dynMinPhi = Mathf.Lerp(dynMinPhi, SoftMax(dynMinPhi, -allowLeanInAtLimit * phiInMax), straightLockBlend * posTurnGate);
            dynMaxPhi = Mathf.Lerp(dynMaxPhi, SoftMin(dynMaxPhi,  allowLeanInAtLimit * phiInMax), straightLockBlend * negTurnGate);

            float tau_hip_coriolis = -m2_l2 * (2.0f * s.dl1 * s.omega1 * cosPhi + s.l1 * s.omega1 * s.omega1 * sinPhi);
            SolveSecondOrderBoundedExact(
                ref s.phi, ref s.dphi, out float ddphi,
                0.0f, (gReset * 900.0f) / I2_hip, (SoftMax(25.0f, cfg.kd_hip_steer) + gReset * 160.0f) / (2.0f * I2_hip),
                (s.tau_hip_steer - (1.0f - gReset) * (tau_hip_load - tau_hip_coriolis)) / I2_hip, dynMinPhi, dynMaxPhi, safeDt
            );

            // B. 下部リンク角 theta1(t)
            // 外傾(phi)をストレート(0)へ戻す際の急激な角加速度 ddphi が、反作用で脚を引き起こしてしまうのを防ぐ
            float recoveringDdphiGate = AnalyticStep(-turnSign * ddphi, 0.0f, 2.0f);
            float straightReactionDamp = 1.0f - straightLockBlend * recoveringDdphiGate * 0.90f;
            float reactionScale = Mathf.Lerp(AnalyticClamp(cfg.hipReactionGain, 0.65f, 1.0f), 1.0f, shiftActivity) * straightReactionDamp;

            float tau_hip_inertial_reaction = (1.0f - gReset) * reactionScale * (I_couple * AnalyticClamp(ddphi, -60.0f, 60.0f) + m2_l2 * s.l1 * sinPhi * s.dphi * s.dphi);
            float dI_eff_dt = 2.0f * half_dI_dl1 * s.dl1 + dI_dphi * s.dphi;

            // 物理境界は -halfPi, halfPi (±90度) とし、バランス崩壊時は80度を超えて自然に転倒(BootOut / HipCrash)させる
            SolveSecondOrderBoundedExact(
                ref s.theta1, ref s.omega1, out _,
                0.0f, (gReset * 850.0f) / s.I_eff, (weightGate * totalRollDamping + SoftMax(0.0f, dI_eff_dt) + gReset * 160.0f) / (2.0f * s.I_eff),
                (tau_gravity + tau_inertial_com + tau_hip_inertial_reaction) / s.I_eff,
                -halfPi, halfPi, safeDt
            );

            if (gReset < 0.5f)
            {
                s.phi = AnalyticClamp(s.phi, dynMinPhi, dynMaxPhi);
            }

            s.P_hip_steer = (1.0f - gReset) * Mathf.Abs(s.tau_hip_steer * s.dphi);
            s.phi_bend = (1.0f - gReset) * (s.phi - phi_target);
            s.dphi_bend = (1.0f - gReset) * s.dphi;

            UpdateKinematics(ref s, cfg);

            float v_y_com_actual = (s.y_CoM - prev_y_CoM) / safeDt;
            s.F_lat_step += weightGate * cfg.M * 1.8f * SoftPlus(SmoothSign(s.theta1 + 0.1f * s.y_CoM, 2e-3f) * v_y_com_actual);
            s.dy_actual = s.y_CoM - (L1 * Mathf.Sin(s.theta1 - delta_theta1_cmd) + L2 * Mathf.Sin(s.theta2_ref));

            // 5. 進行方向速度 V(t)
            float gravityDrive = cfg.g * Mathf.Sin(cfg.alphaRad) * 0.35f * (1.0f + 0.35f * sigma_in - 0.30f * sigma_out);
            float snowFriction = weightGate * (cfg.mu_snow * (1.0f + 2.0f * s.skid_intensity) * (SoftPlus(s.F_leg) / cfg.M) + 0.012f * sinAbsTheta1 * sinAbsTheta1 * Mathf.Abs(s.G_base) * cfg.g);
            float pumpPropulsion = (0.045f * s.P_leg_ext) / (cfg.M * SoftMax(2.0f, s.V));
            s.V = AnalyticClamp(SolveRiccatiSpeedExact(s.V, gravityDrive - snowFriction + pumpPropulsion, Mathf.Lerp(0.0022f, 0.0036f, s.s_rom), safeDt), 4.0f, 32.0f);

            // 6. テレメトリ表示用ラベルへのマッピング
            s.gripState = EvaluateGripStateLabel(in s, cfg, cfg.W_rail * 0.5f, N_min);
        }

        private static SkiGripState EvaluateGripStateLabel(in SimState s, SkyTechSimConfig cfg, float halfRail, float N_min)
        {
            if (Mathf.Abs(s.x_c) >= halfRail) return SkiGripState.RailLimitHit;
            if (s.z_H < cfg.z_H_min) return SkiGripState.HipCrash;
            if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutRad) return SkiGripState.BootOut;
            if (s.snow_stress_ratio >= 1.0f || s.skid_intensity > 0.15f) return SkiGripState.Skidding;
            if (s.F_vert < N_min && s.F_leg < N_min) return SkiGripState.Unweighted;
            return SkiGripState.Carving;
        }

        public static void UpdateKinematics(ref SimState s, SkyTechSimConfig cfg)
        {
            float theta2 = s.theta1 - s.phi;
            float sin1 = Mathf.Sin(s.theta1), cos1 = Mathf.Cos(s.theta1);
            float totalMass = SoftMax(1.0f, cfg.m1 + cfg.m2);

            s.y_CoM = (cfg.m1 * cfg.c1 * s.l1 * sin1 + cfg.m2 * (s.l1 * sin1 + cfg.l2 * Mathf.Sin(theta2))) / totalMass;
            s.z_CoM = (cfg.m1 * cfg.c1 * s.l1 * cos1 + cfg.m2 * (s.l1 * cos1 + cfg.l2 * Mathf.Cos(theta2))) / totalMass;
            s.Psi_CoM = Mathf.Atan2(s.y_CoM, SoftMax(0.01f, s.z_CoM));
            s.R_CoM = Mathf.Sqrt(s.y_CoM * s.y_CoM + s.z_CoM * s.z_CoM);
            s.z_H = s.l1 * cos1;
        }
    }
}
