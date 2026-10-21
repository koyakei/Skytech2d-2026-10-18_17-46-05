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

        /// <summary>ブール値を条件分岐なしで {0f, 1f} の連続スカラーへ変換</summary>
        private static float BoolGate(bool b) => Convert.ToSingle(b);

        /// <summary>滑らかな符号関数 tanh(x / eps)</summary>
        private static float SmoothSign(float x, float eps = 1e-3f) =>
            (float)Math.Tanh(x / eps);

        /// <summary>解析的ソフトプラス関数: max(0, x) = 0.5 * (x + sqrt(x^2 + eps^2))</summary>
        private static float SoftPlus(float x, float eps = 1e-5f) =>
            0.5f * (x + Mathf.Sqrt(x * x + eps * eps));

        /// <summary>解析的ソフトミニマム関数: min(a, b) = 0.5 * (a + b - sqrt((a - b)^2 + eps^2))</summary>
        private static float SoftMin(float a, float b, float eps = 1e-5f) =>
            0.5f * (a + b - Mathf.Sqrt((a - b) * (a - b) + eps * eps));

        /// <summary>解析的ソフトマキシマム関数: max(a, b) = 0.5 * (a + b + sqrt((a - b)^2 + eps^2))</summary>
        private static float SoftMax(float a, float b, float eps = 1e-5f) =>
            0.5f * (a + b + Mathf.Sqrt((a - b) * (a - b) + eps * eps));

        /// <summary>
        /// 区間内部 (minVal, maxVal) で厳密に恒等写像となる解析的クランプ関数:
        /// 0.5 * (a + b + sqrt((x - a)^2 + eps^2) - sqrt((x - b)^2 + eps^2))
        /// </summary>
        private static float AnalyticClamp(float x, float minVal, float maxVal, float eps = 1e-5f)
        {
            float da = x - minVal;
            float db = x - maxVal;
            return 0.5f * (minVal + maxVal + Mathf.Sqrt(da * da + eps * eps) - Mathf.Sqrt(db * db + eps * eps));
        }

        /// <summary>[0, 1] 区間の解析的クランプ関数</summary>
        private static float AnalyticClamp01(float x) => AnalyticClamp(x, 0.0f, 1.0f);

        /// <summary>解析的シグモイド・ステップ関数: 0.5 * (1 + tanh((x - center) / width))</summary>
        private static float AnalyticStep(float x, float center, float width) =>
            0.5f * (1.0f + (float)Math.Tanh((x - center) / SoftMax(1e-5f, width)));

        /// <summary>普遍整関数 Φ_0(z) = cosh(sqrt(z)) のテイラー展開（z の正負・ゼロを問わず分岐なしで厳密）</summary>
        private static float Phi0(float z) =>
            1.0f + z * (0.5f + z * ((1.0f / 24.0f) + z * ((1.0f / 720.0f) + z * (1.0f / 40320.0f))));

        /// <summary>普遍整関数 Φ_1(z) = sinh(sqrt(z)) / sqrt(z) のテイラー展開</summary>
        private static float Phi1(float z) =>
            1.0f + z * ((1.0f / 6.0f) + z * ((1.0f / 120.0f) + z * ((1.0f / 5040.0f) + z * (1.0f / 362880.0f))));

        /// <summary>1階線形微分方程式 dz/dt = (z_inf - z) / tau の厳密解析解</summary>
        private static float SolveFirstOrderExact(float z0, float zInf, float tau, float dt)
        {
            float decay = Mathf.Exp(-dt / SoftMax(1e-4f, tau));
            return zInf + (z0 - zInf) * decay;
        }

        /// <summary>
        /// 2階線形微分方程式 d^2q/dt^2 + 2*gamma*dq/dt + omega0Sq*(q - qEq) = fExt の厳密解析解プロパゲータ。
        /// omega0Sq = 0（純粘性系）、omega0Sq > 0（バネ減衰系）、omega0Sq < 0（倒立振子系）のすべてにおいて
        /// ゼロ除算や桁落ちを一切起こさず、境界 [qMin, qMax] を含めて単一式で積分します。
        /// </summary>
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
            float g = SoftPlus(gamma);
            float f0 = fExt - omega0Sq * (q - qEq);

            float gdt = g * dt;
            float w2dt2 = omega0Sq * dt * dt;
            float z = gdt * gdt - w2dt2;

            float expFactor = Mathf.Exp(-gdt);
            float p0 = Phi0(z);
            float p1 = Phi1(z);

            // 速度グリーン関数 G1(dt) とその時間微分 G0(dt)
            float G1 = expFactor * dt * p1;
            float G0 = expFactor * (p0 - gdt * p1);

            // 変位グリーン関数 G2(dt) = \int_0^{dt} G1(\tau) d\tau （omega0Sq での除算を含まない整関数展開）
            float G2 = expFactor * dt * dt * (
                0.5f
                + (1.0f / 6.0f) * gdt
                + (1.0f / 24.0f) * z
                + (1.0f / 60.0f) * gdt * z
                + (1.0f / 720.0f) * z * z
            );

            // 初期加速度 a_init = f0 - 2*g*v を用いた状態更新:
            // q(t+dt) = q + (G1 + 2*g*G2)*v + G2*(f0 - 2*g*v) = q + G1*v + G2*f0
            float qFree = q + G1 * v + G2 * f0;
            float vFree = G0 * v + G1 * f0 - w2dt2 * (G2 / dt) * v;

            float qClamped = AnalyticClamp(qFree, qMin, qMax);
            // 境界内にいるときは 1、境界端で外向きに押し付けられているときは 0 になる解析的ゲート
            float inBoundsGate = AnalyticStep(qFree - qMin, 0.0f, 1e-4f) * AnalyticStep(qMax - qFree, 0.0f, 1e-4f);

            a = (vFree - v) / dt;
            q = qClamped;
            v = vFree * inBoundsGate + ((qClamped - q) / dt) * (1.0f - inBoundsGate);
        }

        /// <summary>
        /// リッカチ微分方程式 dV/dt = a_eff - c_d * V^2 のコール・ホップ変換による厳密解析解。
        /// </summary>
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
            float drop = SupportHoldDropMeters;
            float fade = SupportFadeZoneMeters;
            return 1.0f - AnalyticStep(l1, l_max - drop + 0.5f * fade, 0.35f * fade);
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
        // メイン時間発展ステップ（解析的運動方程式による単一ステップ厳密積分）
        // =========================================================================

        public static void Step(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            float safeDt = SoftMax(1e-5f, dt);

            float totalMass = SoftMax(1.0f, cfg.m1 + cfg.m2);
            float mg = cfg.M * cfg.g;
            float L1 = ((cfg.m1 * cfg.c1 + cfg.m2) / totalMass) * s.l1;
            float L2 = (cfg.m2 / totalMass) * cfg.l2;
            float m2_l2 = cfg.m2 * cfg.l2;
            float I2_hip = SoftMax(0.5f, cfg.m2 * cfg.l2 * cfg.l2);
            float prev_y_CoM = s.y_CoM;

            s.I_eff = SoftMax(2.0f,
                cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1 +
                cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2 * Mathf.Cos(s.phi)));

            float phiMax = SoftMax(0.08f, cfg.phi_out_maxRad);
            float tauHipMax = SoftMax(100.0f, cfg.hipSteerMaxIsoTorque);
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);

            // =========================================================================
            // 1. 基準姿勢 θ_{2,ref} と 重心シフト入力の1階解析解
            // =========================================================================
            float gReset   = BoolGate(input.w_reset);
            float gNeutral = BoolGate(input.steer_neutral_cmd);
            float activeGate = (1.0f - gReset) * (1.0f - gNeutral);

            float u_x = activeGate * AnalyticClamp(-input.phi_steer_cmd, -1.0f, 1.0f);
            float tauShift = SoftMax(0.01f, cfg.comShiftResponseTau);
            float prev_u_shift = s.u_shift_filtered;
            s.u_shift_filtered = (1.0f - gReset) * SolveFirstOrderExact(prev_u_shift, u_x, tauShift, safeDt);
            // 重心シフトの瞬間的な変化率（定常保持時は 0 になる）
            float du_shift_rate = (s.u_shift_filtered - prev_u_shift) / safeDt;

            float shiftActivity = AnalyticClamp01(SoftMax(Mathf.Abs(u_x), Mathf.Abs(s.u_shift_filtered)));
            float forceOverride = cfg.mouseForceOverrideRatio * shiftActivity;

            float G0_approx = (L1 * sin1) / SoftMax(0.20f, L1 * cos1 + L2);
            float theta_res0 = Mathf.Atan(G0_approx);
            float g_eff0 = cfg.g * Mathf.Sqrt(1.0f + G0_approx * G0_approx);

            float etaVertBudget = (1.0f - AnalyticClamp(cfg.shiftPriorityReserveRatio, 0.65f, 0.95f)) * (1.0f - 0.5f * forceOverride);
            float alphaVert = AnalyticClamp01(cfg.refVerticalKeepRatio);
            float deltaVertMax = Mathf.Asin(AnalyticClamp((etaVertBudget * tauHipMax) / SoftMax(1.0f, m2_l2 * g_eff0), 0.0f, 0.999f));

            float theta2_des = theta_res0 - AnalyticClamp(alphaVert * theta_res0, -deltaVertMax, deltaVertMax);
            float phiRefLimit = SoftMax(0.25f * phiMax, etaVertBudget * phiMax);
            s.theta2_ref = AnalyticClamp(theta2_des, s.theta1 - phiRefLimit, s.theta1 + phiRefLimit);
            float phi_ref = s.theta1 - s.theta2_ref;

            s.y_CoM_ref = L1 * sin1 + L2 * Mathf.Sin(s.theta2_ref);
            s.z_CoM_ref = SoftMax(0.18f, L1 * cos1 + L2 * Mathf.Cos(s.theta2_ref));
            float G_eq = Mathf.Lerp(s.y_CoM_ref / s.z_CoM_ref, s.y_CoM / SoftMax(0.18f, s.z_CoM), 0.75f);

            // 股関節の筋長-張力ガウス特性と目標角
            float u_stroke_mag = AnalyticClamp01(Mathf.Abs(s.phi) / phiMax);
            float L0 = SoftMax(0.1f, cfg.hipMuscleOptLengthL0);
            float Lmin = AnalyticClamp(cfg.hipMuscleMinLengthLmin, 0.05f, L0);
            float normStrain = -0.5f * u_stroke_mag * (L0 - Lmin) / (SoftMax(0.1f, cfg.hipMuscleGaussianWidthW) * L0);
            float tau_hip_avail = tauHipMax * Mathf.Exp(-(normStrain * normStrain));

            float g_eff = Mathf.Sqrt(cfg.g * cfg.g + s.a_c * s.a_c);
            float g_eff_limit = Mathf.Lerp(g_eff, cfg.g * 0.65f, forceOverride);
            float deltaThetaTorque = Mathf.Asin(AnalyticClamp(tau_hip_avail / SoftMax(1.0f, m2_l2 * g_eff_limit), 0.0f, 0.999f));

            float kappaTrunk = AnalyticClamp(cfg.trunkInPhaseRatio, 0.25f, 0.85f);
            float angulationRatio = 1.0f - kappaTrunk;
            float L_eff = SoftMax(0.20f, L1 + kappaTrunk * L2);
            float maxAngShiftRad = SoftMin(phiMax, deltaThetaTorque) / SoftMax(0.15f, angulationRatio);
            float halfPi = 85.0f * Mathf.Deg2Rad;

            s.dy_min_torque = L_eff * (Mathf.Sin(SoftMax(-halfPi, s.theta1 - maxAngShiftRad)) - sin1);
            s.dy_max_torque = L_eff * (Mathf.Sin(SoftMin( halfPi, s.theta1 + maxAngShiftRad)) - sin1);

            float maxShiftDesign = SoftMax(0.05f, cfg.comShiftMaxMeters) * (1.0f + 0.55f * forceOverride);
            float dy_desired = s.u_shift_filtered * maxShiftDesign;
            s.dy_cmd = Mathf.Lerp(AnalyticClamp(dy_desired, s.dy_min_torque, s.dy_max_torque), dy_desired, forceOverride);

            float delta_theta1_cmd = Mathf.Asin(AnalyticClamp(sin1 + s.dy_cmd / L_eff, -0.99f, 0.99f)) - s.theta1;

            float lowPostureFactor = 1.0f - s.s_rom;
            float crossOverGate = AnalyticClamp01((-s.theta1 * s.u_shift_filtered) / (15.0f * Mathf.Deg2Rad));
            float extIntent = AnalyticClamp01(SoftMax(input.u_ext_norm, s.dl1 / 0.35f)) * s.s_rom;
            float headDiveRad = s.u_shift_filtered * (cfg.extHeadDiveMaxDeg * Mathf.Deg2Rad) * (0.25f * extIntent * crossOverGate);

            float effectiveAngRatio = AnalyticClamp(angulationRatio + 0.35f * lowPostureFactor, 0.35f, 0.90f);
            float minPhiLimit = SoftMin(-phiMax, cfg.phi_in_maxRad * 1.35f);
            float maxPhiLimit = SoftMax( phiMax, -cfg.phi_in_maxRad * 1.35f);
            float phi_target = (1.0f - gReset) * AnalyticClamp(effectiveAngRatio * (s.theta1 + delta_theta1_cmd) - headDiveRad, minPhiLimit, maxPhiLimit);

            // =========================================================================
            // 2. 下肢伸縮運動方程式 (l1, dl1) の解析解
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

            float u_norm = AnalyticClamp01(input.u_ext_norm);
            float u_flex = AnalyticClamp01(input.u_flex_norm);
            float F_avail = F_iso * fvFactor * strokeRoomGate * staminaGate;
            s.u_ext_actual = u_norm * (1.0f - u_flex) * F_avail;

            float dropHold = cfg.supportDropFromMaxMeters;
            float fadeWidth = SoftMax(0.005f, cfg.supportFadeWidthMeters);
            float hasTargetGate = AnalyticStep(input.l1_target, 0.05f, 0.015f);
            float defaultHold = cfg.l_max - dropHold;
            // Sキー押下時（hasTargetGate == 0 かつ u_flex > 0）も目標保持長 l_hold が滑らかに l_min へ移行する
            float keyHold = Mathf.Lerp(defaultHold, cfg.l_min, u_flex);
            float l_hold = Mathf.Lerp(keyHold, AnalyticClamp(input.l1_target, cfg.l_min, cfg.l_max), hasTargetGate);

            float phi_support = 1.0f - AnalyticStep(s.l1, l_hold + 0.5f * fadeWidth, 0.35f * fadeWidth);

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

            // 現在の脚長 s.l1 が目標保持長 l_hold より上にある（実際に縮んでいる）最中のみ抜重し、低姿勢到達後はしっかり踏力を維持する
            float movingDownGate = AnalyticClamp01((s.l1 - l_hold) / 0.04f);
            float flexUnweightGate = movingDownGate * (1.0f - 0.65f * shiftActivity);
            float u_flex_eff = u_flex * flexUnweightGate;

            float k_stance_eff = cfg.k_passive * secantSaturation * (1.0f - u_flex_eff);
            float l_eq_safe = SoftMax(0.45f, cfg.l_eq);
            float c_crit = 2.0f * Mathf.Sqrt(cfg.M * SoftMax(100.0f, cfg.k_passive));
            float c_ecc = SoftMax(c_crit * 0.85f, cfg.c_ecc_factor * cfg.M * Mathf.Sqrt(cfg.g / l_eq_safe));
            float activeInputMag = AnalyticClamp01(u_norm + u_flex);
            float dampingMult = Mathf.Lerp(1.05f, 0.40f, activeInputMag);
            float c_leg_eff = dampingMult * c_ecc;

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex_eff);
            float activeTuckPull = u_flex_eff * (0.95f + cfg.releaseTuckPullG) * mg;
            float loadBlend = AnalyticClamp01(u_norm + u_flex_eff + (1.0f - phi_support));
            float effectiveLoad = Mathf.Lerp(mg, s.F_compress, loadBlend);

            float fExtLeg = (selfWeightSupport + s.u_ext_actual - activeTuckPull - effectiveLoad) / cfg.M;
            float omegaLegSq = k_stance_eff / cfg.M;
            float gammaLeg = c_leg_eff / (2.0f * cfg.M);

            SolveSecondOrderBoundedExact(
                ref s.l1, ref s.dl1, out float ddl1,
                l_hold, omegaLegSq, gammaLeg, fExtLeg,
                cfg.l_min, cfg.l_max, safeDt
            );

            // 上方加速度 G_z_up・パワー・スタミナの連続更新
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

            float stanceSpringForce = maxSpringForce * (float)Math.Tanh((cfg.k_passive * (l_hold - s.l1)) / SoftMax(1.0f, maxSpringForce));
            float stanceReturnForce = stanceSpringForce * (1.0f - u_flex_eff);
            float xi_ext = 1.0f - phi_support;
            float baseSupportCoeff = Mathf.Lerp(phi_support, 1.0f, AnalyticClamp01(u_norm + hasTargetGate));
            float vertSupportG = (baseSupportCoeff - xi_ext * s.G_z_up) * (1.0f - 0.85f * u_flex_eff);

            s.F_vert = mg * vertSupportG + stanceReturnForce - activeTuckPull
                     - c_leg_eff * s.dl1 * cosAbsTheta1 + s.u_ext_actual * cosAbsTheta1;

            float v_y_ext = v_com_ext * sinAbsTheta1;
            float a_y_ext = SoftPlus(ddl1) * sinAbsTheta1;
            float dy_shift_speed = Mathf.Abs(du_shift_rate) * maxShiftDesign;
            s.F_lat_step = s.u_ext_actual * sinAbsTheta1
                         + cfg.M * (a_y_ext + 2.5f * v_y_ext + 1.5f * dy_shift_speed + Mathf.Abs(s.a_c) * Mathf.Sin(absPsi));

            float F_vert_pos = SoftPlus(s.F_vert);
            s.F_leg = SoftMax(F_vert_pos, F_vert_pos * cosAbsTheta1 + s.F_lat_step * sinAbsTheta1);

            // =========================================================================
            // 3. 台車水平運動方程式 (x_c, v_c) と 実効遠心加速度 a_c の解析解
            // =========================================================================
            float playRad = FreePlayDeg * Mathf.Deg2Rad;
            float biteRatio = AnalyticClamp01(absTheta1 / playRad + shiftActivity * 10.0f);
            s.G_base = G_eq * biteRatio * biteRatio;

            float F_req_ideal = mg * (Mathf.Cos(absPsi) + Mathf.Abs(G_eq) * Mathf.Sin(absPsi));
            float loadRatio = s.F_leg / SoftMax(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float maxLambda = Mathf.Lerp(1.0f, SoftMax(1.2f, cfg.lambda_max), AnalyticClamp01((s.P_leg_ext + s.F_lat_step * v_y_ext) / 600.0f));
            s.lambda_sustain = AnalyticClamp(Mathf.Pow(AnalyticClamp(loadRatio, 0.0f, 1.8f), cfg.loadSustainPower_q), 0.0f, maxLambda);

            // 雪面挫折ストレスとスキッド強度の1階解析解
            float minSkidRad = SoftMax(3.0f, cfg.skidMinAngleDeg) * Mathf.Deg2Rad;
            float refSkidRad = SoftMax(cfg.skidMinAngleDeg + 5.0f, cfg.skidRefAngleDeg) * Mathf.Deg2Rad;
            float angleGate = AnalyticStep(absTheta1, minSkidRad, 0.25f * minSkidRad);
            float phi_angle = angleGate * Mathf.Pow(sinAbsTheta1 / SoftMax(0.2f, Mathf.Sin(refSkidRad)), cfg.snowAngleVulnerabilityExp);

            float limitVertG = SoftMax(0.4f, cfg.snowFailureVerticalForceG) * Mathf.Lerp(1.0f, cosAbsTheta1, 0.55f);
            float phi_vert = SoftPlus(s.F_vert / mg - 0.55f * cosAbsTheta1) / SoftMax(0.25f, limitVertG);
            float phi_power = s.P_step_eff / SoftMax(150.0f, cfg.snowFailurePowerThreshold);

            s.snow_stress_ratio = phi_angle * phi_vert * phi_power;

            float skidOnset = AnalyticStep(s.snow_stress_ratio, 1.0f, 0.04f);
            float targetSkid = skidOnset * AnalyticClamp01(0.65f + 0.35f * (s.snow_stress_ratio - 1.0f));
            float recTau = Mathf.Lerp(SoftMin(0.06f, cfg.skidRecoveryTau), SoftMax(0.05f, cfg.skidRecoveryTau), extMotionGate);
            float skidTau = Mathf.Lerp(recTau, 0.025f, skidOnset);
            s.skid_intensity = SolveFirstOrderExact(s.skid_intensity, targetSkid, skidTau, safeDt);

            // 台車の運動方程式: m_c * d^2(x_c)/dt^2 + c_glide * d(x_c)/dt + k_center * x_c = F_drive + ...
            float m_c = SoftMax(1.0f, cfg.carriageMass);
            float effectiveAngle = SoftMax(absTheta1, absPsi);
            const float C_rail = 75.0f;
            float c_glide = Mathf.Lerp(C_rail * 0.35f, C_rail, AnalyticClamp01(effectiveAngle / (20f * Mathf.Deg2Rad)));

            float sinRoll = Mathf.Sin(effectiveAngle);
            float angleShapeFactor = Mathf.Lerp(1.0f, 0.40f, AnalyticClamp01((effectiveAngle - 5f * Mathf.Deg2Rad) / (25f * Mathf.Deg2Rad)))
                                   + 0.35f * (sinRoll * sinRoll);

            // 低姿勢時でも重心角度 y_CoM / z_CoM に加えて目標シフト dy_cmd がエッジ駆動力へ滑らかに寄与する
            float rawTanDrive = (s.y_CoM + 0.35f * s.dy_cmd) / SoftMax(0.18f, s.z_CoM);
            float tanAngleDrive = 1.65f * (float)Math.Tanh(rawTanDrive / 1.65f);
            s.v_c_target = (tanAngleDrive * cfg.g) * 1.217f * angleShapeFactor;

            float carvingRetention = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, s.skid_intensity);
            float F_drive = C_rail * s.v_c_target * carvingRetention;

            float M_inertia = s.I_eff / SoftMax(0.15f, s.R_CoM * s.R_CoM);
            float inertiaMassRatio = M_inertia / (m_c + M_inertia);
            s.v_c_recoil_max = inertiaMassRatio * v_y_ext;

            float dirSign = SmoothSign(s.y_CoM + 0.1f * s.theta1, 2e-3f);
            float dynamicFootForce = (s.P_leg_ext / SoftMax(0.3f, v_max_eff)) * sinAbsTheta1;
            s.skid_pushback_force = -dirSign * cfg.carriagePushbackGain * inertiaMassRatio * dynamicFootForce * s.skid_intensity;

            // 重心シフトの過渡速度 du_shift_rate にのみ比例する瞬間反作用（定常保持時は 0 となり逆走を引き起こさない）
            float transientShiftImpulse = -cfg.M * maxShiftDesign * du_shift_rate * 0.25f * inertiaMassRatio;

            float halfRail = cfg.W_rail * 0.5f;
            float v_c_prev = s.v_c;
            float omegaRailSq = cfg.k_center;
            float gammaRail = c_glide / (2.0f * m_c);
            float fExtRail = (F_drive + s.skid_pushback_force + transientShiftImpulse) / m_c;

            SolveSecondOrderBoundedExact(
                ref s.x_c, ref s.v_c, out _,
                0.0f, omegaRailSq, gammaRail, fExtRail,
                -halfRail, halfRail, safeDt
            );

            // レール端ストッパー接触時の反発（係数 0.30）を連続ゲートで融合
            float atRailEndGate = AnalyticStep(Mathf.Abs(s.x_c), halfRail - 1e-3f, 5e-4f);
            float pushingOutwardGate = AnalyticStep(s.x_c * v_c_prev, 0.0f, 1e-3f);
            float bounceGate = atRailEndGate * pushingOutwardGate;
            s.v_c = Mathf.Lerp(s.v_c, -0.30f * v_c_prev, bounceGate);
            float ddx_c = (s.v_c - v_c_prev) / safeDt;

            // 車止め端に達し、かつ重心が同じ外側にあるときのみ 0 になる連続ゲート
            float outsideSameSide = AnalyticStep(s.x_c * s.y_CoM, 0.0f, 2e-3f);
            float outwardFreeGate = 1.0f - atRailEndGate * outsideSameSide;

            float v_req = SoftMax(0.20f, Mathf.Abs(s.v_c_target));
            float v_realized = SoftPlus(SmoothSign(s.v_c_target, 0.01f) * s.v_c);
            float eta_carriage = outwardFreeGate * (1.0f - AnalyticClamp01((v_req - v_realized) / v_req));

            float N_min = cfg.N_min_G * mg;
            float weightGate = Mathf.Lerp(0.75f, 1.0f, AnalyticClamp01(SoftMax(s.F_vert, s.F_leg) / SoftMax(1.0f, N_min)));
            float skidGripGate = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, AnalyticClamp01(s.skid_intensity * 1.25f));
            float hipViableGate  = AnalyticStep(s.z_H, cfg.z_H_min, 2e-3f);
            float bootViableGate = AnalyticStep(cfg.theta_bootOutRad, absTheta1, 2e-3f);
            float viableGate = hipViableGate * bootViableGate;
            float gripFactor = viableGate * weightGate * skidGripGate;

            float a_c_target = (s.G_base * cfg.g) * s.lambda_sustain * gripFactor * eta_carriage;
            float tauG = AnalyticClamp(cfg.tau_G_lag, 0.005f, 0.015f);
            s.a_c = viableGate * SolveFirstOrderExact(s.a_c, a_c_target, tauG, safeDt);

            float vSqSafe = SoftMax(4.0f, s.V * s.V);
            s.kappa = s.a_c / vSqSafe;
            s.kappa_target = a_c_target / vSqSafe;

            // =========================================================================
            // 4. 2リンク回転運動方程式 (phi, dphi) および (theta1, omega1) の解析解
            // =========================================================================

            // --- A. 股関節相対角 phi(t) の2階線形運動方程式 ---
            float theta2_cur = s.theta1 - s.phi;
            float theta2_target = s.theta1 - phi_target;
            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - s.a_c * Mathf.Cos(theta2_cur));
            float tau_hip_ff   = (1.0f - gReset) * m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - s.a_c * (1.0f - 0.85f * forceOverride) * Mathf.Cos(theta2_target));

            float kp_hip_eff = SoftMax(600.0f, cfg.kp_hip * 1.4f) * (1.0f + 1.25f * shiftActivity);
            float kd_hip_eff = SoftMax(45.0f, cfg.kd_hip * 1.2f);

            float steerScale = (1.0f - gReset) * Mathf.Lerp(0.35f, 1.0f, outwardFreeGate);
            float rawHipSteer = tau_hip_ff + kp_hip_eff * (phi_target - s.phi) - kd_hip_eff * s.dphi;
            s.tau_hip_steer = steerScale * AnalyticClamp(rawHipSteer, -tau_hip_avail, tau_hip_avail);

            float ddxCoupling = Mathf.Lerp(0.35f, 1.0f, atRailEndGate);
            float tau_hip_impact = -(1.0f - gReset) * m2_l2 * (ddxCoupling * ddx_c) * Mathf.Cos(theta2_cur);

            float omegaPhiSq = (gReset * 900.0f) / I2_hip;
            float gammaPhi   = (cfg.kd_hip_steer + gReset * 160.0f) / (2.0f * I2_hip);
            float fExtPhi    = (s.tau_hip_steer - (1.0f - gReset) * (tau_hip_load + tau_hip_impact)) / I2_hip;

            SolveSecondOrderBoundedExact(
                ref s.phi, ref s.dphi, out float ddphi,
                0.0f, omegaPhiSq, gammaPhi, fExtPhi,
                minPhiLimit, maxPhiLimit, safeDt
            );

            // --- B. 下部リンク角 theta1(t) の2階線形運動方程式 ---
            float tau_gravity = (1.0f - gReset) * mg * s.y_CoM;
            float tau_carriage_com = -(1.0f - gReset) * cfg.M * (ddxCoupling * ddx_c + s.a_c) * s.z_CoM;

            // 低姿勢時（s.l1 が短いとき）でもロール駆動モーメントが低下しないよう基準長 l_eq_safe でスケーリング
            float lowStanceBoost = 1.0f + 0.75f * (1.0f - s.s_rom);
            float legSupportRatio = AnalyticClamp(s.F_leg / mg, 0.50f, 1.8f);
            float kp_leg_roll = outwardFreeGate * cfg.comShiftRollDriveGain * cfg.mouseRollMomentGain * mg * l_eq_safe * lowStanceBoost * legSupportRatio;
            float kd_leg_roll = 0.18f * kp_leg_roll;
            float tau_leg_lateral_drive = (1.0f - gReset) * kp_leg_roll * delta_theta1_cmd;

            float oncomingPushTorque = (1.0f - gReset) * outwardFreeGate * (s.u_shift_filtered * Mathf.Abs(s.v_c) * (0.45f * mg * s.z_CoM) * lowStanceBoost);

            float skidRightingPenalty = 1.0f - 0.40f * s.skid_intensity;
            float activeExtMotionGate = AnalyticClamp01(SoftMax(v_com_ext / 0.20f, (s.u_ext_actual / (1.2f * mg)) * 0.85f));
            float rightingSuppress = 1.0f - 0.80f * AnalyticClamp01(s.u_shift_filtered * SmoothSign(s.theta1, 0.01f));
            float tau_leg_carriage_push = -(1.0f - gReset) * eta_carriage * activeExtMotionGate * skidRightingPenalty * rightingSuppress
                * (cfg.extRightingTorqueGain * s.u_ext_actual * s.l1 + cfg.extVelocityRightingGain * mg * v_com_ext) * sin1;

            float I_couple = I2_hip + cfg.m2 * s.l1 * cfg.l2 * Mathf.Cos(s.phi);
            float tau_hip_inertial_reaction = (1.0f - gReset) * Mathf.Lerp(1.0f, 0.25f, eta_carriage) * I_couple * ddphi;

            float omegaTheta1Sq = (gReset * 850.0f) / s.I_eff;
            float gammaTheta1   = (kd_leg_roll + SoftMin(cfg.gamma_roll, 10.0f) + gReset * 160.0f) / (2.0f * s.I_eff);
            float fExtTheta1    = (tau_gravity + tau_carriage_com + tau_leg_lateral_drive + oncomingPushTorque + tau_leg_carriage_push + tau_hip_inertial_reaction) / s.I_eff;

            SolveSecondOrderBoundedExact(
                ref s.theta1, ref s.omega1, out _,
                0.0f, omegaTheta1Sq, gammaTheta1, fExtTheta1,
                -halfPi, halfPi, safeDt
            );

            s.P_hip_steer = (1.0f - gReset) * Mathf.Abs(s.tau_hip_steer * s.dphi);
            s.phi_bend = (1.0f - gReset) * (s.phi - phi_ref);
            s.dphi_bend = (1.0f - gReset) * s.dphi;

            UpdateKinematics(ref s, cfg);

            float v_y_com_actual = (s.y_CoM - prev_y_CoM) / safeDt;
            float dirSignPost = SmoothSign(s.theta1 + 0.1f * s.y_CoM, 2e-3f);
            s.F_lat_step += cfg.M * 1.8f * SoftPlus(dirSignPost * v_y_com_actual);
            s.dy_actual = s.y_CoM - (L1 * Mathf.Sin(s.theta1 - delta_theta1_cmd) + L2 * Mathf.Sin(s.theta2_ref));

            // =========================================================================
            // 5. 進行方向速度 V(t) のリッカチ微分方程式の解析解
            // =========================================================================
            float gravityDrive = cfg.g * Mathf.Sin(cfg.alphaRad) * 0.35f;
            float frictionMult = 1.0f + 2.0f * s.skid_intensity;
            float snowFriction = cfg.mu_snow * frictionMult * (SoftPlus(s.F_leg) / cfg.M);
            float dragCoeffPerMass = Mathf.Lerp(0.0022f, 0.0036f, s.s_rom);
            float pumpPropulsion = (0.045f * s.P_leg_ext) / (cfg.M * SoftMax(2.0f, s.V));
            float aNetDrive = gravityDrive - snowFriction + pumpPropulsion;

            s.V = AnalyticClamp(SolveRiccatiSpeedExact(s.V, aNetDrive, dragCoeffPerMass, safeDt), 4.0f, 32.0f);

            // =========================================================================
            // 6. テレメトリ表示用ラベルへのマッピング
            // =========================================================================
            s.gripState = EvaluateGripStateLabel(in s, cfg, halfRail, N_min);
        }

        private static SkiGripState EvaluateGripStateLabel(in SimState s, SkyTechSimConfig cfg, float halfRail, float N_min)
        {
            if (s.z_H < cfg.z_H_min) return SkiGripState.HipCrash;
            if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutRad) return SkiGripState.BootOut;
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