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
        public Vector2 mouse_world;
        public bool has_mouse_world;
        public bool steer_neutral_cmd;
        public bool w_reset;
    }

    public struct SimState
    {
        public float x_c;
        public float v_c;
        public float theta1;
        public float omega1;
        public float l1;
        public float dl1;
        public float phi;
        public float dphi;

        public float phi_bend;
        public float dphi_bend;
        public float P_hip_steer;
        public float tau_hip_steer;

        public float u_shift_filtered;
        public float theta2_ref;
        public float y_CoM_ref;
        public float z_CoM_ref;
        public float dy_cmd;
        public float dy_actual;
        public float dy_min_torque;
        public float dy_max_torque;

        public float V;
        public float stamina;

        public float y_CoM;
        public float z_CoM;
        public float Psi_CoM;
        public float R_CoM;
        public float z_H;
        public float F_leg;
        public float F_compress;
        public float u_ext_actual;

        public float I_eff;
        public float F_vert;
        public float G_z_up;
        public float F_lat_step;
        public float P_step_eff;
        public float snow_stress_ratio;
        public float skid_intensity;
        public float v_c_target;
        public float v_c_recoil_max;
        public float skid_pushback_force;

        public float s_rom;
        public float knee_flex_deg;
        public float knee_int_deg;
        public float posture_force_ratio;
        public float P_leg_ext;
        public float P_leg_peak_avail;

        public float G_base;
        public float lambda_sustain;
        public float a_c;
        public float kappa;
        public float kappa_target;
        public SkiGripState gripState;
    }

    public static class SkyTechPhysicsSolver
    {
        public const float FreePlayDeg = 2.0f;
        public const float SupportHoldDropMeters = 0.15f;
        public const float SupportFadeZoneMeters = 0.03f;

        // =========================================================================
        // 解析的プリミティブ関数群（条件分岐なし・区間内部での歪みゼロ）
        // =========================================================================

        private static float BoolGate(bool b) => Convert.ToSingle(b);

        private static float SmoothSign(float x, float eps = 1e-3f) =>
            (float)Math.Tanh(x / SoftMax(1e-6f, eps));

        private static float SoftPlus(float x, float eps = 1e-5f) =>
            0.5f * (x + Mathf.Sqrt(x * x + eps * eps));

        private static float SoftMin(float a, float b, float eps = 1e-5f) =>
            0.5f * (a + b - Mathf.Sqrt((a - b) * (a - b) + eps * eps));

        private static float SoftMax(float a, float b, float eps = 1e-5f) =>
            0.5f * (a + b + Mathf.Sqrt((a - b) * (a - b) + eps * eps));

        private static float AnalyticClamp(float x, float minVal, float maxVal, float eps = 1e-5f)
        {
            float da = x - minVal;
            float db = x - maxVal;
            return 0.5f * (minVal + maxVal + Mathf.Sqrt(da * da + eps * eps) - Mathf.Sqrt(db * db + eps * eps));
        }

        private static float AnalyticClamp01(float x) => AnalyticClamp(x, 0.0f, 1.0f);

        private static float AnalyticStep(float x, float center, float width) =>
            0.5f * (1.0f + (float)Math.Tanh((x - center) / SoftMax(1e-5f, width)));

        private static float Phi0(float z) =>
            1.0f + z * (0.5f + z * ((1.0f / 24.0f) + z * ((1.0f / 720.0f) + z * (1.0f / 40320.0f))));

        private static float Phi1(float z) =>
            1.0f + z * ((1.0f / 6.0f) + z * ((1.0f / 120.0f) + z * ((1.0f / 5040.0f) + z * (1.0f / 362880.0f))));

        private static float SolveFirstOrderExact(float z0, float zInf, float tau, float dt)
        {
            float decay = Mathf.Exp(-dt / SoftMax(1e-4f, tau));
            return zInf + (z0 - zInf) * decay;
        }

        private static void SolveSecondOrderBoundedExact(
            ref float q,
            ref float v,
            out float a,
            float qEq,
            float omega0Sq,
            float gamma,
            float fExt,
            float qMin,
            float qMax,
            float dt)
        {
            float g = AnalyticClamp(gamma, -15.0f, 250.0f);
            float f0 = fExt - omega0Sq * (q - qEq);

            float gdt = g * dt;
            float w2dt2 = omega0Sq * dt * dt;
            float z = gdt * gdt - w2dt2;

            float expFactor = Mathf.Exp(-gdt);
            float p0 = Phi0(z);
            float p1 = Phi1(z);

            float G1 = expFactor * dt * p1;
            float G0 = expFactor * (p0 - gdt * p1);
            float G2 = expFactor * dt * dt * (
                0.5f
                + (1.0f / 6.0f) * gdt
                + (1.0f / 24.0f) * z
                + (1.0f / 60.0f) * gdt * z
                + (1.0f / 720.0f) * z * z
            );

            float qPrev = q;
            float qFree = qPrev + G1 * v + G2 * f0;
            float vFree = G0 * v + G1 * f0 - w2dt2 * (G2 / dt) * v;

            float qClamped = AnalyticClamp(qFree, qMin, qMax);
            float inBoundsGate = AnalyticStep(qFree - qMin, 0.0f, 1e-4f) * AnalyticStep(qMax - qFree, 0.0f, 1e-4f);

            q = qClamped;
            v = vFree * inBoundsGate + ((qClamped - qPrev) / dt) * (1.0f - inBoundsGate);
            a = (v - vFree) / dt + (vFree - G0 * v) / dt;
        }

        private static float SolveRiccatiSpeedExact(float v0, float aEff, float cDrag, float dt)
        {
            float z = aEff * cDrag * dt * dt;
            float p0 = Phi0(z);
            float p1 = Phi1(z);
            float num = p0 * v0 + aEff * dt * p1;
            float den = SoftMax(1e-4f, p0 + cDrag * v0 * dt * p1);
            return num / den;
        }

        // =========================================================================
        // 公開API・初期化・バイオメカニクス計算
        // =========================================================================

        public static SimState CreateInitialState(SkyTechSimConfig cfg)
        {
            float drop = cfg.supportDropFromMaxMeters;
            float initialL1 = AnalyticClamp(cfg.l_max - drop, cfg.l_min, cfg.l_max);
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
                lambda_sustain = 1.0f,
                gripState = SkiGripState.Carving
            };

            UpdateKinematics(ref s, cfg);
            EvaluateLegBiomechanics(
                s.l1, cfg,
                out s.s_rom, out s.knee_flex_deg, out s.knee_int_deg,
                out s.posture_force_ratio, out _, out _, out s.P_leg_peak_avail
            );

            s.I_eff = cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1
                    + cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2);
            s.y_CoM_ref = s.y_CoM;
            s.z_CoM_ref = s.z_CoM;
            return s;
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float l_min = (cfg != null) ? cfg.l_min : 0.40f;
            float l_full = (cfg != null) ? SoftMax(l_max + 0.01f, cfg.l_full) : (l_max + 0.02f);
            float romSpan = SoftMax(0.10f, l_max - l_min);
            float s_rom = AnalyticClamp01((l1 - l_min) / romSpan);
            float cosHalfKnee = AnalyticClamp(l1 / l_full, 0.01f, 1.0f);
            float cosMax = AnalyticClamp(l_max / l_full, 0.01f, 1.0f);
            float normCos = cosHalfKnee / SoftMax(1e-4f, cosMax);
            return AnalyticClamp01(Mathf.Lerp(0.35f, 1.0f, s_rom) * normCos);
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max) =>
            ComputeSelfWeightSupportFactor(l1, l_max, null);

        public static float ComputeSquatRecoveryFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float drop = SupportHoldDropMeters;
            float fade = SupportFadeZoneMeters;
            return AnalyticStep(l_max - drop - l1, 0.5f * fade, 0.35f * fade);
        }

        public static float ComputeSquatRecoveryFactor(float l1, float l_max) =>
            ComputeSquatRecoveryFactor(l1, l_max, null);

        public static void EvaluateLegBiomechanics(
            float l1,
            SkyTechSimConfig cfg,
            out float s_rom,
            out float knee_flex_deg,
            out float knee_int_deg,
            out float phi_F,
            out float F_iso,
            out float v_max_eff,
            out float P_peak_avail)
        {
            float romSpan = SoftMax(0.10f, cfg.l_max - cfg.l_min);
            s_rom = AnalyticClamp01((l1 - cfg.l_min) / romSpan);

            float l_full = SoftMax(cfg.l_max + 0.01f, cfg.l_full);
            float cosHalfKnee = AnalyticClamp(l1 / l_full, 0.01f, 0.9999f);
            knee_flex_deg = 2.0f * Mathf.Acos(cosHalfKnee) * Mathf.Rad2Deg;
            knee_int_deg = 180.0f - knee_flex_deg;

            float s_opt = AnalyticClamp(cfg.legOptRomRatio, 0.50f, 0.95f);
            float l_opt = cfg.l_min + s_opt * romSpan;

            float r_opt = Mathf.Sqrt(SoftPlus(l_full * l_full - l_opt * l_opt, 1e-4f));
            float r_cur = Mathf.Sqrt(SoftPlus(l_full * l_full - l1 * l1, 1e-4f));
            float leverTerm = SoftMin(1.0f, Mathf.Pow(r_opt / r_cur, cfg.kneeLeverPenaltyExp));

            float side = AnalyticStep(s_rom, s_opt, 0.012f);
            float ds = s_rom - s_opt;

            float sigmaF = SoftMax(0.05f, Mathf.Lerp(cfg.squatForceDropWidth, cfg.lockoutForceDropWidth, side));
            float fBase  = AnalyticClamp01(Mathf.Lerp(cfg.squatMinForceRatio, cfg.lockoutMinForceRatio, side));
            phi_F = fBase + (1.0f - fBase) * leverTerm * Mathf.Exp(-(ds * ds) / (sigmaF * sigmaF));

            float sigmaV = SoftMax(0.05f, Mathf.Lerp(cfg.squatVelDropWidth, cfg.lockoutVelDropWidth, side));
            float vBase  = AnalyticClamp01(Mathf.Lerp(cfg.squatMinVelRatio, cfg.lockoutMinVelRatio, side));
            float phi_v  = vBase + (1.0f - vBase) * Mathf.Exp(-(ds * ds) / (sigmaV * sigmaV));

            F_iso = cfg.F_ext_max_G * cfg.M * cfg.g * phi_F;
            v_max_eff = SoftMax(0.20f, cfg.v_max * phi_v);

            float pHill = SoftMax(1.0f, cfg.hillPowerExponent);
            float vOptRatio = Mathf.Pow(1.0f / (pHill + 1.0f), 1.0f / pHill);
            P_peak_avail = F_iso * (v_max_eff * vOptRatio) * (1.0f - Mathf.Pow(vOptRatio, pHill));
        }

        // =========================================================================
        // メイン時間発展ステップ
        // =========================================================================

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

            float cosPhi = Mathf.Cos(s.phi);
            float sinPhi = Mathf.Sin(s.phi);

            s.I_eff = SoftMax(2.0f,
                m1c1Sq_plus_m2 * s.l1 * s.l1 +
                cfg.m2 * (cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2 * cosPhi));
            float half_dI_dl1 = m1c1Sq_plus_m2 * s.l1 + m2_l2 * cosPhi;
            float dI_dphi     = -2.0f * m2_l2 * s.l1 * sinPhi;

            float phiMax = SoftMax(0.15f, SoftMax(0.85f, cfg.phi_out_maxRad));
            float tauHipMax = SoftMax(100.0f, cfg.hipSteerMaxIsoTorque);
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);

            // =========================================================================
            // 1. 台車-重心直線からのマウス距離に基づく側屈・リーンアウト制御
            // =========================================================================
            float gReset   = BoolGate(input.w_reset);
            float gNeutral = BoolGate(input.steer_neutral_cmd);
            float activeGate = (1.0f - gReset) * (1.0f - gNeutral);

            float u_x = activeGate * AnalyticClamp(-input.phi_steer_cmd, -1.0f, 1.0f);
            float tauShift = SoftMax(0.020f, cfg.comShiftResponseTau * 0.75f);
            float prev_u_shift = s.u_shift_filtered;
            s.u_shift_filtered = (1.0f - gReset) * SolveFirstOrderExact(prev_u_shift, u_x, tauShift, safeDt);
            float du_shift_rate = (s.u_shift_filtered - prev_u_shift) / safeDt;

            float inwardReturnRate = SoftPlus(-SmoothSign(s.u_shift_filtered, 0.05f) * du_shift_rate);
            float shiftActivity = AnalyticClamp01(SoftMax(Mathf.Abs(u_x), Mathf.Abs(s.u_shift_filtered)));
            float forceOverride = cfg.mouseForceOverrideRatio * shiftActivity;

            // 目標ロール角 theta1_des（最大 ±72°）
            float maxTargetRollRad = SoftMin(72.0f * Mathf.Deg2Rad, cfg.theta_bootOutRad - 4.0f * Mathf.Deg2Rad);
            float signedShift = SmoothSign(s.u_shift_filtered, 0.02f) * Mathf.Pow(Mathf.Abs(s.u_shift_filtered), 0.82f);
            float mouseTargetTheta1 = signedShift * maxTargetRollRad;
            float theta1_des = activeGate * AnalyticClamp(mouseTargetTheta1 + input.phi_lock_cmd, -maxTargetRollRad, maxTargetRollRad);
            float rollError = theta1_des - s.theta1;

            // ★台車 P(x_c, 0) と重心 C(x_c + y_CoM, z_CoM) を結ぶ直線成分でのマウス距離を測定
            float comDist = SoftMax(0.15f, Mathf.Sqrt(s.y_CoM * s.y_CoM + s.z_CoM * s.z_CoM));
            float lineDirX = s.y_CoM / comDist;
            float lineDirZ = s.z_CoM / comDist;

            float mouseDistAlongComLine;
            if (input.has_mouse_world)
            {
                // 台車位置 (s.x_c, 0) からマウスワールド位置へのベクトルを台車-重心直線へ射影（内積）
                float mouseRelX = input.mouse_world.x - s.x_c;
                float mouseRelZ = input.mouse_world.y;
                mouseDistAlongComLine = mouseRelX * lineDirX + mouseRelZ * lineDirZ;
            }
            else
            {
                mouseDistAlongComLine = comDist;
            }

            // 距離が近い(<= nearDist)とき straightFactor = 0 (bendRatio = 1: 上体を鉛直まで立てる)
            // 距離が遠い(>= farDist)とき  straightFactor = 1 (bendRatio = 0: 側屈0で体がストレート)
            float nearDist = SoftMax(0.10f, cfg.comLineNearDist);
            float farDist  = SoftMax(nearDist + 0.15f, cfg.comLineFarDist);
            float straightFactor = AnalyticClamp01((mouseDistAlongComLine - nearDist) / (farDist - nearDist));
            float bendRatio = 1.0f - straightFactor;

            // 後方互換用（他セクションの参照用）
            float mouseDistanceFactor = bendRatio;
            float deepRollFactor = AnalyticClamp01(Mathf.Abs(s.theta1) / (55.0f * Mathf.Deg2Rad));

            // 基準姿勢: bendRatio = 1 で上体鉛直(theta2_ref = 0, phi_ref = theta1)
            //          bendRatio = 0 で体ストレート(theta2_ref = theta1, phi_ref = 0)
            float phi_ref = AnalyticClamp(bendRatio * s.theta1, -phiMax, phiMax);
            s.theta2_ref = s.theta1 - phi_ref;

            s.y_CoM_ref = L1 * sin1 + L2 * Mathf.Sin(s.theta2_ref);
            s.z_CoM_ref = SoftMax(0.18f, L1 * cos1 + L2 * Mathf.Cos(s.theta2_ref));
            float G_eq = Mathf.Lerp(s.y_CoM_ref / s.z_CoM_ref, s.y_CoM / SoftMax(0.18f, s.z_CoM), 0.75f);

            float L0 = SoftMax(0.1f, cfg.hipMuscleOptLengthL0);
            float Lmin = AnalyticClamp(cfg.hipMuscleMinLengthLmin, 0.05f, L0);
            float gaussW = SoftMax(0.25f, cfg.hipMuscleGaussianWidthW);

            float fullRomPhi = phiMax + 45.0f * Mathf.Deg2Rad;
            float u_stroke_pos = AnalyticClamp01(SoftPlus(s.phi) / fullRomPhi);
            float normStrain_pos = -0.35f * u_stroke_pos * (L0 - Lmin) / (gaussW * L0);
            float tau_hip_avail_pos = tauHipMax * Mathf.Exp(-(normStrain_pos * normStrain_pos));

            float u_stroke_neg = AnalyticClamp01(SoftPlus(-s.phi) / fullRomPhi);
            float normStrain_neg = -0.35f * u_stroke_neg * (L0 - Lmin) / (gaussW * L0);
            float tau_hip_avail_neg = tauHipMax * Mathf.Exp(-(normStrain_neg * normStrain_neg));

            float g_eff = Mathf.Sqrt(cfg.g * cfg.g + s.a_c * s.a_c);
            float g_eff_limit = Mathf.Lerp(g_eff, cfg.g * 0.65f, forceOverride);
            float deltaThetaTorquePos = Mathf.Asin(AnalyticClamp(tau_hip_avail_pos / SoftMax(1.0f, m2_l2 * g_eff_limit), 0.0f, 0.999f));
            float deltaThetaTorqueNeg = Mathf.Asin(AnalyticClamp(tau_hip_avail_neg / SoftMax(1.0f, m2_l2 * g_eff_limit), 0.0f, 0.999f));

            float L_eff = SoftMax(0.22f, L1 + 0.35f * L2);
            float maxAngShiftRadPos = SoftMin(phiMax, deltaThetaTorquePos);
            float maxAngShiftRadNeg = SoftMin(phiMax, deltaThetaTorqueNeg);
            float halfPi = 85.0f * Mathf.Deg2Rad;

            s.dy_min_torque = L_eff * (Mathf.Sin(SoftMax(-halfPi, s.theta1 - maxAngShiftRadNeg)) - sin1);
            s.dy_max_torque = L_eff * (Mathf.Sin(SoftMin( halfPi, s.theta1 + maxAngShiftRadPos)) - sin1);

            float maxShiftDesign = SoftMax(0.05f, cfg.comShiftMaxMeters) * (1.0f + 0.55f * forceOverride);
            float dy_desired = s.u_shift_filtered * maxShiftDesign;
            s.dy_cmd = Mathf.Lerp(AnalyticClamp(dy_desired, s.dy_min_torque, s.dy_max_torque), dy_desired, forceOverride);

            float delta_theta1_cmd = Mathf.Asin(AnalyticClamp(sin1 + s.dy_cmd / L_eff, -0.99f, 0.99f)) - s.theta1;

            float tiltDir  = SmoothSign(s.theta1, 8.0f * Mathf.Deg2Rad);
            float shiftDir = SmoothSign(s.u_shift_filtered, 0.15f);
            float chi_up   = AnalyticClamp01(SoftPlus(-tiltDir * shiftDir));

            float shallowAngleGate = 1.0f - AnalyticStep(Mathf.Abs(s.theta1), 18.0f * Mathf.Deg2Rad, 8.0f * Mathf.Deg2Rad);

            float rollSign = SmoothSign(s.theta1, 1.0f * Mathf.Deg2Rad);
            float inwardOmega = SoftPlus(rollSign * s.omega1);
            float leadTheta1ForHip = s.theta1 + 0.10f * rollSign * inwardOmega;

            // 目標股関節側屈角: bendRatio = 0 のとき phi_target = 0 (ストレート), bendRatio = 1 のとき phi_target = theta1 (上体鉛直)
            float phi_target = (1.0f - gReset) * AnalyticClamp(bendRatio * leadTheta1ForHip, -phiMax, phiMax);

            // phi = 0 (ストレート) から phi = theta1 (上体鉛直) までの全域を許容する可動範囲
            float slackRad = 3.0f * Mathf.Deg2Rad;
            float minPhiActive = AnalyticClamp(SoftMin(0.0f, s.theta1) - slackRad, -phiMax, 0.0f);
            float maxPhiActive = AnalyticClamp(SoftMax(0.0f, s.theta1) + slackRad, 0.0f, phiMax);
            float minPhiLimit = Mathf.Lerp(minPhiActive, -phiMax, gReset);
            float maxPhiLimit = Mathf.Lerp(maxPhiActive,  phiMax, gReset);

            // =========================================================================
            // 2. 下肢伸縮運動方程式 (l1, dl1)
            // =========================================================================
            EvaluateLegBiomechanics(
                s.l1, cfg,
                out s.s_rom, out s.knee_flex_deg, out s.knee_int_deg,
                out s.posture_force_ratio, out float F_iso, out float v_max_eff, out s.P_leg_peak_avail
            );

            float pHill = SoftMax(1.0f, cfg.hillPowerExponent);
            float posVelRatio = AnalyticClamp(SoftPlus(s.dl1) / v_max_eff, 0.0f, 0.999f);
            float negVelRatio = AnalyticClamp01(SoftPlus(-s.dl1) / 1.50f);
            float fvFactor = 1.0f - Mathf.Pow(posVelRatio, pHill) + 0.15f * negVelRatio;
            float strokeRoomGate = AnalyticClamp01((cfg.l_max - s.l1) / 0.025f);
            float staminaGate = AnalyticClamp01(s.stamina / 20.0f);

            float returnPushBoost = AnalyticClamp01(inwardReturnRate / 2.5f) * AnalyticStep(Mathf.Abs(s.theta1), 10.0f * Mathf.Deg2Rad, 5.0f * Mathf.Deg2Rad);
            float u_norm = AnalyticClamp01(SoftMax(input.u_ext_norm, 0.65f * returnPushBoost * (1.0f - mouseDistanceFactor)));
            float u_flex = AnalyticClamp01(input.u_flex_norm);
            float F_avail = F_iso * fvFactor * strokeRoomGate * staminaGate;
            s.u_ext_actual = u_norm * (1.0f - u_flex) * F_avail;

            float dropHold = cfg.supportDropFromMaxMeters;
            float hasTargetGate = AnalyticStep(input.l1_target, 0.05f, 0.015f);
            float defaultHold = cfg.l_max - dropHold;
            float keyHold = Mathf.Lerp(Mathf.Lerp(defaultHold, cfg.l_max, u_norm), cfg.l_min, u_flex);
            float l_hold = Mathf.Lerp(keyHold, AnalyticClamp(input.l1_target, cfg.l_min, cfg.l_max), hasTargetGate);

            float phi_support = ComputeSelfWeightSupportFactor(s.l1, cfg.l_max, cfg);

            float absTheta1 = SoftMin(Mathf.Abs(s.theta1), 82f * Mathf.Deg2Rad);
            float cosAbsTheta1 = SoftMax(0.15f, Mathf.Cos(absTheta1));
            float sinAbsTheta1 = Mathf.Sin(absTheta1);
            float absPsi = Mathf.Abs(s.Psi_CoM);
            float currentLatG = Mathf.Abs(s.a_c) / cfg.g;

            s.F_compress = SoftMax(0.1f * mg, mg * (Mathf.Cos(absPsi) + currentLatG * Mathf.Sin(absPsi)));

            float maxSpringForce = cfg.passiveSpringMaxG * mg;
            float posError = l_hold - s.l1;
            float springArg = (cfg.k_passive * Mathf.Abs(posError) + 1.0f) / SoftMax(1.0f, maxSpringForce);
            float secantSaturation = (float)Math.Tanh(springArg) / springArg;

            float movingDownGate = AnalyticClamp01((s.l1 - l_hold) / 0.04f);
            float flexUnweightGate = movingDownGate * (1.0f - 0.65f * shiftActivity);
            float u_flex_eff = u_flex * flexUnweightGate;

            float belowHoldGate = AnalyticStep(l_hold - s.l1, 0.0f, 0.005f);
            float k_stance_eff = cfg.k_passive * secantSaturation * (1.0f - u_flex_eff) * belowHoldGate;
            float l_eq_safe = SoftMax(0.45f, cfg.l_eq);
            float c_crit = 2.0f * Mathf.Sqrt(cfg.M * SoftMax(100.0f, cfg.k_passive));
            float c_ecc = SoftMax(c_crit * 0.85f, cfg.c_ecc_factor * cfg.M * Mathf.Sqrt(cfg.g / l_eq_safe));
            float activeInputMag = AnalyticClamp01(u_norm + u_flex);
            float dampingMult = Mathf.Lerp(1.05f, 0.45f, activeInputMag);
            float c_leg_eff = dampingMult * c_ecc;

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex_eff);
            float loadBlend = AnalyticClamp01(u_norm + u_flex_eff + (1.0f - phi_support));
            float effectiveLoad = Mathf.Lerp(mg, s.F_compress, loadBlend);

            float F_rot_centrifugal = half_dI_dl1 * (s.omega1 * s.omega1);

            float fExtLeg = (selfWeightSupport + s.u_ext_actual + F_rot_centrifugal - effectiveLoad) / cfg.M;
            float omegaLegSq = k_stance_eff / cfg.M;
            float gammaLeg = c_leg_eff / (2.0f * cfg.M);

            SolveSecondOrderBoundedExact(
                ref s.l1, ref s.dl1, out float ddl1,
                l_hold, omegaLegSq, gammaLeg, fExtLeg,
                cfg.l_min, cfg.l_max, safeDt
            );

            float v_com_ext = SoftPlus(s.dl1);
            float v_z_ext = v_com_ext * cosAbsTheta1;
            float G_z_kin = (v_z_ext * v_z_ext) / (2.0f * cfg.g * SoftMax(0.12f, dropHold));
            float G_z_inst = (SoftPlus(ddl1) * cosAbsTheta1) / cfg.g;
            float extMotionGate = AnalyticClamp01(v_com_ext * 100.0f);
            float targetUpG = u_norm * extMotionGate * SoftMax(G_z_kin, 0.55f * G_z_inst);
            s.G_z_up = SoftMax(targetUpG, SoftPlus(s.G_z_up - 3.8f * safeDt));

            float refInertia = SoftMax(1.0f, cfg.M * l_eq_safe * l_eq_safe * 0.65f);
            float inertiaFactor = AnalyticClamp(s.I_eff / refInertia, 0.4f, 1.6f);
            s.u_ext_actual *= extMotionGate;
            s.P_leg_ext = s.u_ext_actual * v_com_ext * inertiaFactor;
            s.P_step_eff = s.P_leg_ext;

            float recoveryRate = (1.0f - AnalyticClamp01(u_norm * 20.0f)) * cfg.staminaRecoveryRate;
            s.stamina = AnalyticClamp(s.stamina + (recoveryRate - s.P_leg_ext) * safeDt, 0.0f, cfg.maxStamina);

            float stanceSpringForce = belowHoldGate * maxSpringForce * (float)Math.Tanh((cfg.k_passive * SoftPlus(l_hold - s.l1)) / SoftMax(1.0f, maxSpringForce));
            float stanceReturnForce = stanceSpringForce * (1.0f - u_flex_eff);
            float xi_ext = 1.0f - phi_support;
            float baseSupportCoeff = Mathf.Lerp(phi_support, 1.0f, AnalyticClamp01(u_norm + hasTargetGate));
            float vertSupportG = (baseSupportCoeff - xi_ext * s.G_z_up) * (1.0f - 0.85f * u_flex_eff);

            s.F_vert = mg * vertSupportG + stanceReturnForce
                     - c_leg_eff * s.dl1 * cosAbsTheta1 + s.u_ext_actual * cosAbsTheta1;

            float F_vert_pos = SoftPlus(s.F_vert);
            float N_min = cfg.N_min_G * mg;
            float weightGate = AnalyticClamp01(F_vert_pos / SoftMax(1.0f, N_min));

            float v_y_ext = v_com_ext * sinAbsTheta1;
            s.F_lat_step = weightGate * ((s.u_ext_actual + cfg.M * SoftPlus(ddl1)) * sinAbsTheta1
                         + cfg.M * Mathf.Abs(s.a_c) * Mathf.Sin(absPsi));

            s.F_leg = SoftMax(F_vert_pos, F_vert_pos * cosAbsTheta1 + s.F_lat_step * sinAbsTheta1);

            // =========================================================================
            // 3. SkyTechSport 屋内レール式キャリッジ駆動 ＆ 自己整合カービング物理モデル
            // =========================================================================
            float effectiveRollRate = s.omega1 + 0.35f * (rollError / SoftMax(0.06f, tauShift));
            float rollDirSign = SmoothSign(s.theta1, 6.0f * Mathf.Deg2Rad);
            float rateDirSign = SmoothSign(effectiveRollRate, 0.65f);
            float sigma_phase = rollDirSign * rateDirSign;

            float targetDirSign = SmoothSign(theta1_des, 2.0f * Mathf.Deg2Rad);
            float rollInDeficit = SoftPlus(targetDirSign * (theta1_des - s.theta1));
            float intentionalRollIn = AnalyticClamp01(rollInDeficit / (18.0f * Mathf.Deg2Rad));

            float deepAnglePhaseGate = AnalyticStep(absTheta1, 15.0f * Mathf.Deg2Rad, 6.0f * Mathf.Deg2Rad);
            float sigma_in  = AnalyticClamp01(SoftPlus( sigma_phase)) * deepAnglePhaseGate;
            float sigma_out = AnalyticClamp01(SoftPlus(-sigma_phase));

            float vRefSafe = SoftMax(4.0f, cfg.V_ref);
            float rScSafe  = SoftMax(3.0f, cfg.R_sc);
            float speedRatio = s.V / vRefSafe;
            float speedSqFactor = speedRatio * speedRatio;

            float sidecutRefG = (vRefSafe * vRefSafe) / (cfg.g * rScSafe);
            float G0_effective = 0.5f * (cfg.G0_curve + 2.20f * sidecutRefG);

            float deadRad = SoftMax(0.5f * Mathf.Deg2Rad, cfg.theta_deadRad);
            float edgeSignSmooth = SmoothSign(s.theta1, deadRad);

            float effectiveP = SoftMax(0.75f, cfg.sinPower_p);
            float sinPowP = Mathf.Pow(SoftMax(1e-4f, sinAbsTheta1), effectiveP);
            float cosFlex = SoftMax(0.18f, Mathf.Cos(cfg.k_flex * absTheta1));
            float howeCurve = edgeSignSmooth * (sinPowP / cosFlex);

            float angulationAlignment = SoftPlus(edgeSignSmooth * (s.phi / phiMax));
            float angulationBoost = 1.0f + cfg.k_ang_boost * angulationAlignment;

            float G_howe = G0_effective * angulationBoost * howeCurve * speedSqFactor;

            float legTanG = sin1 / cosAbsTheta1;
            float effectiveEqG = Mathf.Lerp(G_eq, legTanG, 0.45f);
            float carveEngageEnvelope = AnalyticStep(absTheta1, 11.0f * Mathf.Deg2Rad, 5.5f * Mathf.Deg2Rad);
            float rollInCentrifugalRelease = 1.0f - 0.55f * intentionalRollIn;
            float eqScale = Mathf.Lerp(0.55f, 1.08f, carveEngageEnvelope) * rollInCentrifugalRelease;
            float G_com_equilibrium = effectiveEqG * (eqScale + 0.18f * angulationAlignment + 0.30f * mouseDistanceFactor) * SoftBoxSpeedScale(speedRatio);

            float howeBlend = AnalyticClamp01(
                0.12f
                + 0.55f * AnalyticStep(absTheta1, 22.0f * Mathf.Deg2Rad, 10.0f * Mathf.Deg2Rad) * (1.0f - 0.50f * intentionalRollIn)
                + 0.18f * angulationAlignment
            );

            float playRad = SoftMax(0.5f * Mathf.Deg2Rad, cfg.carriageDeadzoneRad);
            float biteRatio = AnalyticClamp01(absTheta1 / playRad);
            float effectiveBite = Mathf.Lerp(biteRatio * biteRatio, biteRatio, carveEngageEnvelope);
            float phaseGMod = (1.0f - 0.15f * sigma_in + 0.22f * sigma_out) * Mathf.Lerp(1.0f, 0.65f, intentionalRollIn);

            s.G_base = Mathf.Lerp(G_com_equilibrium, G_howe, howeBlend) * effectiveBite * phaseGMod;

            float F_req_ideal = mg * (Mathf.Cos(absPsi) + Mathf.Abs(s.G_base) * Mathf.Sin(absPsi));
            float loadRatio = s.F_leg / SoftMax(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float maxLambda = Mathf.Lerp(1.0f, SoftMax(1.2f, cfg.lambda_max), AnalyticClamp01((s.P_leg_ext + s.F_lat_step * v_y_ext) / 600.0f));
            s.lambda_sustain = AnalyticClamp(Mathf.Pow(AnalyticClamp(loadRatio, 0.65f, 1.8f), cfg.loadSustainPower_q), 0.72f, maxLambda);

            float minSkidRad = SoftMax(22.0f, cfg.skidMinAngleDeg) * Mathf.Deg2Rad;
            float refSkidRad = SoftMax(minSkidRad + 12.0f * Mathf.Deg2Rad, cfg.skidRefAngleDeg * Mathf.Deg2Rad);
            float angleGate = AnalyticStep(absTheta1, minSkidRad, 0.25f * minSkidRad);
            float phi_angle = angleGate * Mathf.Pow(sinAbsTheta1 / SoftMax(0.25f, Mathf.Sin(refSkidRad)), cfg.snowAngleVulnerabilityExp);

            float limitVertG = SoftMax(1.2f, cfg.snowFailureVerticalForceG) * Mathf.Lerp(1.0f, cosAbsTheta1, 0.40f);
            float F_lat_cent_demand = mg * Mathf.Abs(s.G_base) * s.lambda_sustain;
            float steadyVertLoad = SoftMin(F_vert_pos, s.F_compress + 0.65f * s.u_ext_actual);
            float combinedLoadG = (steadyVertLoad + 0.45f * F_lat_cent_demand) / mg;
            float sigma_force = SoftPlus(combinedLoadG - 1.10f * cosAbsTheta1) / SoftMax(0.50f, limitVertG);

            float rawStress = phi_angle * (sigma_force * sigma_force);
            s.snow_stress_ratio = SolveFirstOrderExact(s.snow_stress_ratio, rawStress, 0.06f, safeDt);

            float skidOnset = AnalyticStep(s.snow_stress_ratio, 1.0f, 0.08f);
            float targetSkid = skidOnset * AnalyticClamp01(0.50f + 0.35f * (s.snow_stress_ratio - 1.0f));
            float recTau = SoftMax(0.06f, cfg.skidRecoveryTau);
            float skidTau = Mathf.Lerp(recTau, 0.05f, skidOnset);
            s.skid_intensity = SolveFirstOrderExact(s.skid_intensity, targetSkid, skidTau, safeDt);

            float m_c_hw = SoftMax(1.0f, cfg.carriageMass);
            float M_coupled_horiz = cfg.M * (1.0f - (cfg.M * s.z_CoM * s.z_CoM) / SoftMax(1.0f, s.I_eff + cfg.M * s.R_CoM * s.R_CoM));
            float m_c_eff = m_c_hw + M_coupled_horiz;

            float effectiveAngle = SoftMax(absTheta1, absPsi);
            const float C_rail = 95.0f;
            float c_glide_base = Mathf.Lerp(C_rail * cfg.shallowGlideStiffnessMinRatio, C_rail, AnalyticClamp01(effectiveAngle / (15f * Mathf.Deg2Rad)));
            float c_glide = c_glide_base * (1.0f - 0.06f * sigma_in + 0.20f * sigma_out);

            float sinRoll = Mathf.Sin(effectiveAngle);
            float angleShapeFactor = 0.85f + 0.55f * (sinRoll * sinRoll);

            float comTanDrive = s.y_CoM / SoftMax(0.22f, s.z_CoM);
            float howeTanDrive = 0.50f * howeCurve * angulationBoost * (12.0f / rScSafe);
            float rawTanDrive = (0.50f * comTanDrive + 0.50f * howeTanDrive) * SoftBoxSpeedScale(speedRatio);
            float tanAngleDrive = 1.85f * (float)Math.Tanh(rawTanDrive / 1.85f);
            float railScaleNorm = SoftMax(0.5f, cfg.railScale / 4.87f);
            float v_c_sym = (tanAngleDrive * cfg.g) * 1.15f * railScaleNorm * angleShapeFactor;

            float inwardFallingRate = rollDirSign * SoftPlus(rollDirSign * s.omega1);
            float boundedFallingRate = 0.45f * (float)Math.Tanh(inwardFallingRate / 0.45f);
            float captureSuppression = 1.0f - 0.80f * intentionalRollIn;
            float v_c_capture_lead = cfg.capturePointLeadGain * captureSuppression * s.z_CoM * boundedFallingRate * railScaleNorm;

            float phaseSpeedGain = 1.0f - 0.08f * sigma_in + 0.22f * sigma_out;
            float desired_v_c_target = v_c_sym * phaseSpeedGain + v_c_capture_lead;
            s.v_c_target = SolveFirstOrderExact(s.v_c_target, desired_v_c_target, 0.035f, safeDt);

            float safeSkidGrip = SoftMax(0.55f, cfg.skidGripMultiplier);
            float carvingRetention = Mathf.Lerp(1.0f, safeSkidGrip, s.skid_intensity);
            float F_drive = weightGate * c_glide * s.v_c_target * carvingRetention;

            float F_camber_rebound = weightGate * sigma_out * 0.22f * s.F_leg * sin1 * carvingRetention * SoftBoxSpeedScale(speedRatio);

            float M_inertia = s.I_eff / SoftMax(0.15f, s.R_CoM * s.R_CoM);
            float inertiaMassRatio = M_inertia / (m_c_hw + M_inertia);
            s.v_c_recoil_max = inertiaMassRatio * v_y_ext;

            float dirSign = SmoothSign(s.y_CoM + 0.1f * s.theta1, 2e-3f);
            float dynamicFootForce = ((s.P_leg_ext / SoftMax(0.3f, v_max_eff)) + 0.20f * F_lat_cent_demand) * sinAbsTheta1;
            s.skid_pushback_force = -dirSign * SoftMin(0.35f, cfg.carriagePushbackGain) * inertiaMassRatio * dynamicFootForce * s.skid_intensity;

            float switchCrossUnderSign = Mathf.Lerp(-1.0f, 1.0f, sigma_out);
            float activeExtCarriageDrive = switchCrossUnderSign * s.u_ext_actual * sin1 * (1.0f - s.skid_intensity) * 0.35f;
            float crossUnderPushForce = -cfg.M * maxShiftDesign * du_shift_rate * (0.45f + 0.35f * shallowAngleGate);

            float halfRail = cfg.W_rail * 0.5f;
            float v_c_prev = s.v_c;
            float omegaRailSq = cfg.k_center;
            float gammaRail = c_glide / (2.0f * m_c_eff);
            float fExtRail = (F_drive + F_camber_rebound + activeExtCarriageDrive + crossUnderPushForce + s.skid_pushback_force) / m_c_eff;

            SolveSecondOrderBoundedExact(
                ref s.x_c, ref s.v_c, out _,
                0.0f, omegaRailSq, gammaRail, fExtRail,
                -halfRail, halfRail, safeDt
            );

            float atRailEndGate = AnalyticStep(Mathf.Abs(s.x_c), halfRail - 1e-3f, 5e-4f);
            float pushingOutwardGate = AnalyticStep(s.x_c * v_c_prev, 0.0f, 1e-3f);
            float bounceGate = atRailEndGate * pushingOutwardGate;
            s.v_c = Mathf.Lerp(s.v_c, -0.30f * v_c_prev, bounceGate);

            float ddx_c = (s.v_c - v_c_prev) / safeDt;

            float outsideSameSide = AnalyticStep(s.x_c * s.y_CoM, 0.0f, 2e-3f);
            float outwardFreeGate = 1.0f - atRailEndGate * outsideSameSide;

            float v_req = SoftMax(0.08f, Mathf.Abs(s.v_c_target));
            float v_realized = SoftPlus(SmoothSign(s.v_c_target, 0.01f) * s.v_c);
            float raw_eta = 1.0f - AnalyticClamp01((v_req - v_realized) / v_req);
            float rightingFloor = Mathf.Lerp(0.25f, SoftMax(0.75f, cfg.directCarveRightingFloor), carveEngageEnvelope * (1.0f - 0.65f * intentionalRollIn));
            float eta_carriage = outwardFreeGate * Mathf.Lerp(raw_eta, 1.0f, rightingFloor);

            float skidGripGate = Mathf.Lerp(1.0f, safeSkidGrip, AnalyticClamp01(s.skid_intensity));
            float hipViableGate  = AnalyticStep(s.z_H, cfg.z_H_min, 2e-3f);
            float bootViableGate = AnalyticStep(cfg.theta_bootOutRad, absTheta1, 2e-3f);
            float viableGate = hipViableGate * bootViableGate;
            float gripFactor = viableGate * weightGate * skidGripGate;

            float a_c_target = (s.G_base * cfg.g) * s.lambda_sustain * gripFactor * eta_carriage;
            float tauG = AnalyticClamp(cfg.tau_G_lag, 0.010f, 0.025f);
            s.a_c = viableGate * SolveFirstOrderExact(s.a_c, a_c_target, tauG, safeDt);

            float vSqSafe = SoftMax(4.0f, s.V * s.V);
            s.kappa = s.a_c / vSqSafe;
            s.kappa_target = a_c_target / vSqSafe;

            float railImpactBlend = atRailEndGate * outsideSameSide;
            float baseCoupling = Mathf.Lerp(0.30f, 0.55f, shallowAngleGate * shiftActivity);
            float ddx_c_coupled = Mathf.Lerp(baseCoupling * AnalyticClamp(ddx_c, -6.5f, 6.5f), ddx_c, railImpactBlend);
            float a_pivot_total = s.a_c + ddx_c_coupled;

            // =========================================================================
            // 4. 2リンク回転運動方程式 ＆ 股関節リーンアウト追従サーボ
            // =========================================================================

            float valleyGravityAssist = 1.0f + 0.15f * sigma_in * Mathf.Sin(cfg.alphaRad);
            float tau_gravity = (1.0f - gReset) * mg * s.y_CoM * valleyGravityAssist;

            float z_CoM_cuff = Mathf.Lerp(s.z_CoM, s.R_CoM, AnalyticClamp01(cfg.beta_cuff) * sinAbsTheta1 * sinAbsTheta1);
            float tau_inertial_com = -(1.0f - gReset) * cfg.M * a_pivot_total * z_CoM_cuff;

            float driveGain = SoftMax(1.2f, cfg.comShiftRollDriveGain) * SoftMax(1.2f, cfg.mouseRollMomentGain * 0.75f);
            float kp_roll_drive = mg * SoftMax(0.45f, s.z_CoM) * 0.52f * driveGain;
            float tau_roll_drive = (1.0f - gReset) * outwardFreeGate * kp_roll_drive * rollError;

            float tau_switch_push = -(1.0f - gReset) * sigma_out * (1.0f - mouseDistanceFactor)
                                  * SoftMax(0.5f, cfg.extVelocityRightingGain) * mg * 0.28f * sin1;

            float pendulumCritDamping = 2.0f * Mathf.Sqrt(s.I_eff * mg * SoftMax(0.30f, s.z_CoM));
            float centerHoldFactor  = 1.0f - shiftActivity;
            float postureHoldFactor = 1.0f - AnalyticClamp01(Mathf.Abs(du_shift_rate) / 2.0f);
            float coContractGain    = AnalyticClamp(cfg.adaptiveDampingBoost, 0.5f, 5.0f);

            float rollReleaseFactor = 1.0f - 0.72f * intentionalRollIn;

            float autoImpedanceDamping = pendulumCritDamping * (
                0.20f * rollReleaseFactor
                + 0.28f * coContractGain * centerHoldFactor
                + 0.18f * coContractGain * postureHoldFactor * rollReleaseFactor
            );

            float quadDamping = SoftMax(60.0f, cfg.gamma_quad) * 0.30f * Mathf.Abs(s.omega1) * rollReleaseFactor;
            float baseRollDamping = cfg.gamma_roll * rollReleaseFactor;
            float totalRollDamping = SoftMax(baseRollDamping, autoImpedanceDamping) + quadDamping;

            float ddtheta1_est = (tau_gravity + tau_inertial_com + tau_roll_drive + tau_switch_push - weightGate * totalRollDamping * s.omega1) / s.I_eff;

            // --- A. 股関節相対角 phi(t) = theta1(t) - theta2(t) の運動方程式 ---
            float theta2_cur = s.theta1 - s.phi;
            float theta2_target = s.theta1 - phi_target;
            float a_hip_inertial = Mathf.Lerp(s.a_c, a_pivot_total, 0.25f);
            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - a_hip_inertial * Mathf.Cos(theta2_cur));
            float tau_hip_ff   = (1.0f - gReset) * m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - a_hip_inertial * Mathf.Cos(theta2_target));

            float kp_hip_eff = SoftMax(850.0f, cfg.kp_hip) * (1.0f + 0.85f * shiftActivity + 0.65f * chi_up + 0.55f * deepRollFactor);
            float kd_hip_critical = 2.0f * Mathf.Sqrt(I2_hip * kp_hip_eff);
            float velThresh = SoftMax(0.05f, cfg.coContractionVelThreshold);

            // 側屈率 bendRatio に応じた目標側屈角速度（ストレート時は dphi_target = 0）
            float dphi_target = (1.0f - gReset) * bendRatio * s.omega1;
            float dphi_error  = s.dphi - dphi_target;

            float hipCoContraction = 1.0f + 0.35f * cfg.adaptiveDampingBoost * (1.0f - 0.5f * shiftActivity)
                                   + 0.25f * cfg.adaptiveDampingBoost * AnalyticClamp01(Mathf.Abs(dphi_error) / velThresh);
            float kd_hip_eff = SoftMax(cfg.kd_hip, 0.95f * kd_hip_critical * hipCoContraction);

            float I_couple = I2_hip + m2_l2 * s.l1 * cosPhi;
            float tau_hip_leg_accel_comp = (1.0f - gReset) * I_couple * bendRatio * ddtheta1_est;
            float tau_hip_pd = kp_hip_eff * (phi_target - s.phi) - kd_hip_eff * dphi_error;
            float clampedHipPd = (1.0f - gReset) * AnalyticClamp(tau_hip_pd, -tau_hip_avail_neg * 1.5f, tau_hip_avail_pos * 1.5f);
            s.tau_hip_steer = tau_hip_ff + clampedHipPd + tau_hip_leg_accel_comp;

            float tau_hip_coriolis = -m2_l2 * (2.0f * s.dl1 * s.omega1 * cosPhi + s.l1 * s.omega1 * s.omega1 * sinPhi);

            float omegaPhiSq = (gReset * 900.0f) / I2_hip;
            float gammaPhi   = (SoftMax(25.0f, cfg.kd_hip_steer) + gReset * 160.0f) / (2.0f * I2_hip);
            float fExtPhi    = (s.tau_hip_steer - (1.0f - gReset) * (tau_hip_load - tau_hip_coriolis)) / I2_hip;

            SolveSecondOrderBoundedExact(
                ref s.phi, ref s.dphi, out float ddphi,
                0.0f, omegaPhiSq, gammaPhi, fExtPhi,
                minPhiLimit, maxPhiLimit, safeDt
            );

            // --- B. 下部リンク角 theta1(t) の運動方程式 ---
            float reactionScale = Mathf.Lerp(AnalyticClamp(cfg.hipReactionGain, 0.35f, 1.0f), 1.0f, shiftActivity);
            float tau_hip_centrifugal = m2_l2 * s.l1 * sinPhi * (s.dphi * s.dphi);
            float ddphi_active = AnalyticClamp(ddphi - bendRatio * ddtheta1_est, -25.0f, 25.0f);
            float tau_hip_inertial_reaction = (1.0f - gReset) * reactionScale * (I_couple * ddphi_active + tau_hip_centrifugal);

            float dI_eff_dt = 2.0f * half_dI_dl1 * s.dl1 + dI_dphi * s.dphi;

            float omegaTheta1Sq = (gReset * 850.0f) / s.I_eff;
            float gammaTheta1   = (weightGate * totalRollDamping + SoftMax(0.0f, dI_eff_dt) + gReset * 160.0f) / (2.0f * s.I_eff);
            float fExtTheta1    = (tau_gravity + tau_inertial_com + tau_roll_drive + tau_switch_push + tau_hip_inertial_reaction) / s.I_eff;

            SolveSecondOrderBoundedExact(
                ref s.theta1, ref s.omega1, out _,
                0.0f, omegaTheta1Sq, gammaTheta1, fExtTheta1,
                -halfPi, halfPi, safeDt
            );

            if (gReset < 0.5f)
            {
                // ストレート(phi = 0)〜上体鉛直(phi = theta1)の範囲を阻害しないクランプ
                float postMinPhi = AnalyticClamp(SoftMin(0.0f, s.theta1) - slackRad, -phiMax, 0.0f);
                float postMaxPhi = AnalyticClamp(SoftMax(0.0f, s.theta1) + slackRad, 0.0f, phiMax);
                s.phi = AnalyticClamp(s.phi, postMinPhi, postMaxPhi);
            }

            s.P_hip_steer = (1.0f - gReset) * Mathf.Abs(s.tau_hip_steer * s.dphi);
            s.phi_bend = (1.0f - gReset) * (s.phi - phi_ref);
            s.dphi_bend = (1.0f - gReset) * s.dphi;

            UpdateKinematics(ref s, cfg);

            float v_y_com_actual = (s.y_CoM - prev_y_CoM) / safeDt;
            float dirSignPost = SmoothSign(s.theta1 + 0.1f * s.y_CoM, 2e-3f);
            s.F_lat_step += weightGate * cfg.M * 1.8f * SoftPlus(dirSignPost * v_y_com_actual);
            s.dy_actual = s.y_CoM - (L1 * Mathf.Sin(s.theta1 - delta_theta1_cmd) + L2 * Mathf.Sin(s.theta2_ref));

            // =========================================================================
            // 5. 進行方向速度 V(t)
            // =========================================================================
            float yawCosineApprox = 1.0f + 0.35f * sigma_in - 0.30f * sigma_out;
            float gravityDrive = cfg.g * Mathf.Sin(cfg.alphaRad) * 0.35f * yawCosineApprox;
            float frictionMult = 1.0f + 2.0f * s.skid_intensity;
            float carvingInducedDrag = 0.012f * sinAbsTheta1 * sinAbsTheta1 * Mathf.Abs(s.G_base) * cfg.g;
            float snowFriction = weightGate * (cfg.mu_snow * frictionMult * (SoftPlus(s.F_leg) / cfg.M) + carvingInducedDrag);
            float dragCoeffPerMass = Mathf.Lerp(0.0022f, 0.0036f, s.s_rom);
            float pumpPropulsion = (0.045f * s.P_leg_ext) / (cfg.M * SoftMax(2.0f, s.V));
            float aNetDrive = gravityDrive - snowFriction + pumpPropulsion;

            s.V = AnalyticClamp(SolveRiccatiSpeedExact(s.V, aNetDrive, dragCoeffPerMass, safeDt), 4.0f, 32.0f);

            // =========================================================================
            // 6. テレメトリ表示用ラベルへのマッピング
            // =========================================================================
            s.gripState = EvaluateGripStateLabel(in s, cfg, halfRail, N_min);
        }

        private static float SoftBoxSpeedScale(float speedRatio)
        {
            return 0.55f + 0.45f * AnalyticClamp(speedRatio, 0.50f, 1.65f);
        }

        private static SkiGripState EvaluateGripStateLabel(in SimState s, SkyTechSimConfig cfg, float halfRail, float N_min)
        {
            if (s.z_H < cfg.z_H_min) return SkiGripState.HipCrash;
            if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutDeg * Mathf.Deg2Rad) return SkiGripState.BootOut;
            if (Mathf.Abs(s.x_c) >= halfRail - 1e-3f) return SkiGripState.RailLimitHit;
            if (s.snow_stress_ratio >= 1.0f || s.skid_intensity > 0.15f) return SkiGripState.Skidding;
            if (s.F_vert < N_min && s.F_leg < N_min) return SkiGripState.Unweighted;
            return SkiGripState.Carving;
        }

        public static void UpdateKinematics(ref SimState s, SkyTechSimConfig cfg)
        {
            float theta2 = s.theta1 - s.phi;
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);
            float totalMass = SoftMax(1.0f, cfg.m1 + cfg.m2);

            s.y_CoM = (cfg.m1 * cfg.c1 * s.l1 * sin1 + cfg.m2 * (s.l1 * sin1 + cfg.l2 * Mathf.Sin(theta2))) / totalMass;
            s.z_CoM = (cfg.m1 * cfg.c1 * s.l1 * cos1 + cfg.m2 * (s.l1 * cos1 + cfg.l2 * Mathf.Cos(theta2))) / totalMass;
            s.Psi_CoM = Mathf.Atan2(s.y_CoM, SoftMax(0.01f, s.z_CoM));
            s.R_CoM = Mathf.Sqrt(s.y_CoM * s.y_CoM + s.z_CoM * s.z_CoM);
            s.z_H = s.l1 * cos1;
        }
    }
}