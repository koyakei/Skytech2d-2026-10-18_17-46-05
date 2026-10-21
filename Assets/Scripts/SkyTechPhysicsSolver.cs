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
        private const int SubSteps = 4;
        public const float FreePlayDeg = 2.0f;
        public const float SupportHoldDropMeters = 0.15f;
        public const float SupportFadeZoneMeters = 0.03f;

        public static SimState CreateInitialState(SkyTechSimConfig cfg)
        {
            float drop = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float initialL1 = Mathf.Clamp(cfg.l_max - drop, cfg.l_min, cfg.l_max);
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
            float drop = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float fade = Mathf.Max(0.001f, (cfg != null) ? cfg.supportFadeWidthMeters : SupportFadeZoneMeters);
            return 1.0f - Mathf.SmoothStep(0f, 1f, (l1 - (l_max - drop)) / fade);
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max) =>
            ComputeSelfWeightSupportFactor(l1, l_max, null);

        public static float ComputeSquatRecoveryFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float drop = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float fade = Mathf.Max(0.001f, (cfg != null) ? cfg.supportFadeWidthMeters : SupportFadeZoneMeters);
            return Mathf.SmoothStep(0f, 1f, ((l_max - drop) - l1) / fade);
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
            float romSpan = Mathf.Max(0.10f, cfg.l_max - cfg.l_min);
            s_rom = Mathf.Clamp01((l1 - cfg.l_min) / romSpan);

            float l_full = Mathf.Max(cfg.l_max + 0.01f, cfg.l_full);
            knee_flex_deg = 2.0f * Mathf.Acos(Mathf.Clamp01(l1 / l_full)) * Mathf.Rad2Deg;
            knee_int_deg = 180.0f - knee_flex_deg;

            float s_opt = Mathf.Clamp(cfg.legOptRomRatio, 0.50f, 0.95f);
            float l_opt = cfg.l_min + s_opt * romSpan;

            float r_opt = Mathf.Sqrt(Mathf.Max(1e-4f, l_full * l_full - l_opt * l_opt));
            float r_cur = Mathf.Sqrt(Mathf.Max(1e-4f, l_full * l_full - l1 * l1));
            float leverTerm = Mathf.Min(1.0f, Mathf.Pow(r_opt / r_cur, cfg.kneeLeverPenaltyExp));

            // s_rom <= s_opt (0) と s_rom > s_opt (1) を滑らかに補間して単一のガウス式に統合
            float side = Mathf.SmoothStep(0f, 1f, (s_rom - s_opt) / 0.02f);
            float ds = s_rom - s_opt;

            float sigmaF = Mathf.Max(0.05f, Mathf.Lerp(cfg.squatForceDropWidth, cfg.lockoutForceDropWidth, side));
            float fBase  = Mathf.Clamp01(Mathf.Lerp(cfg.squatMinForceRatio, cfg.lockoutMinForceRatio, side));
            phi_F = fBase + (1.0f - fBase) * leverTerm * Mathf.Exp(-(ds * ds) / (sigmaF * sigmaF));

            float sigmaV = Mathf.Max(0.05f, Mathf.Lerp(cfg.squatVelDropWidth, cfg.lockoutVelDropWidth, side));
            float vBase  = Mathf.Clamp01(Mathf.Lerp(cfg.squatMinVelRatio, cfg.lockoutMinVelRatio, side));
            float phi_v  = vBase + (1.0f - vBase) * Mathf.Exp(-(ds * ds) / (sigmaV * sigmaV));

            F_iso = cfg.F_ext_max_G * cfg.M * cfg.g * phi_F;
            v_max_eff = Mathf.Max(0.20f, cfg.v_max * phi_v);

            float pHill = Mathf.Max(1.0f, cfg.hillPowerExponent);
            float vOptRatio = Mathf.Pow(1.0f / (pHill + 1.0f), 1.0f / pHill);
            P_peak_avail = F_iso * (v_max_eff * vOptRatio) * (1.0f - Mathf.Pow(vOptRatio, pHill));
        }

        public static void Step(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            if (dt <= 0f) return;
            float subDt = dt / SubSteps;
            for (int i = 0; i < SubSteps; i++)
            {
                StepSub(ref s, in input, cfg, subDt);
                if (s.gripState == SkiGripState.HipCrash || s.gripState == SkiGripState.BootOut) break;
            }
        }

        private static void StepSub(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            float totalMass = Mathf.Max(1.0f, cfg.m1 + cfg.m2);
            float mg = cfg.M * cfg.g;
            float L1 = ((cfg.m1 * cfg.c1 + cfg.m2) / totalMass) * s.l1;
            float L2 = (cfg.m2 / totalMass) * cfg.l2;
            float m2_l2 = cfg.m2 * cfg.l2;
            float I2_hip = Mathf.Max(0.5f, cfg.m2 * cfg.l2 * cfg.l2);
            float prev_y_CoM = s.y_CoM;

            s.I_eff = Mathf.Max(2.0f,
                cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1 +
                cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2 * Mathf.Cos(s.phi)));

            float phiMax = Mathf.Max(0.08f, cfg.phi_out_maxRad);
            float tauHipMax = Mathf.Max(100.0f, cfg.hipSteerMaxIsoTorque);
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);

            // =========================================================================
            // 1. 基準姿勢 θ_{2,ref} と 股関節アクチュエーター制御 (tau_hip_steer)
            // =========================================================================
            float activeGate = (input.w_reset || input.steer_neutral_cmd) ? 0f : 1f;
            float u_x = activeGate * Mathf.Clamp(-input.phi_steer_cmd, -1.0f, 1.0f);
            float tauShift = Mathf.Max(0.01f, cfg.comShiftResponseTau);
            s.u_shift_filtered = input.w_reset ? 0f : s.u_shift_filtered + ((u_x - s.u_shift_filtered) / tauShift) * dt;

            float shiftActivity = Mathf.Clamp01(Mathf.Max(Mathf.Abs(u_x), Mathf.Abs(s.u_shift_filtered)));
            float forceOverride = cfg.mouseForceOverrideRatio * shiftActivity;

            float G0_approx = (L1 * sin1) / Mathf.Max(0.20f, L1 * cos1 + L2);
            float theta_res0 = Mathf.Atan(G0_approx);
            float g_eff0 = cfg.g * Mathf.Sqrt(1.0f + G0_approx * G0_approx);

            float etaVertBudget = (1.0f - Mathf.Clamp(cfg.shiftPriorityReserveRatio, 0.65f, 0.95f)) * (1.0f - 0.5f * forceOverride);
            float alphaVert = Mathf.Clamp01(cfg.refVerticalKeepRatio);
            float deltaVertMax = Mathf.Asin(Mathf.Clamp((etaVertBudget * tauHipMax) / Mathf.Max(1.0f, m2_l2 * g_eff0), 0f, 1.0f));

            float theta2_des = theta_res0 - Mathf.Clamp(alphaVert * theta_res0, -deltaVertMax, deltaVertMax);
            float phiRefLimit = Mathf.Max(0.25f * phiMax, etaVertBudget * phiMax);
            s.theta2_ref = Mathf.Clamp(theta2_des, s.theta1 - phiRefLimit, s.theta1 + phiRefLimit);
            float phi_ref = s.theta1 - s.theta2_ref;

            s.y_CoM_ref = L1 * sin1 + L2 * Mathf.Sin(s.theta2_ref);
            s.z_CoM_ref = Mathf.Max(0.18f, L1 * cos1 + L2 * Mathf.Cos(s.theta2_ref));
            // 外向傾で腰を出した際に過大な仮想遠心力で右へ押し戻されないよう、実重心位置 s.y_CoM を反映
            float G_eq = Mathf.Lerp(s.y_CoM_ref / s.z_CoM_ref, s.y_CoM / Mathf.Max(0.18f, s.z_CoM), 0.75f);

            // 股関節の筋長-張力ガウス特性と目標角
            float u_stroke_mag = Mathf.Clamp01(Mathf.Abs(s.phi) / phiMax);
            float L0 = Mathf.Max(0.1f, cfg.hipMuscleOptLengthL0);
            float Lmin = Mathf.Clamp(cfg.hipMuscleMinLengthLmin, 0.05f, L0);
            float normStrain = -0.5f * u_stroke_mag * (L0 - Lmin) / (Mathf.Max(0.1f, cfg.hipMuscleGaussianWidthW) * L0);
            float tau_hip_avail = tauHipMax * Mathf.Exp(-(normStrain * normStrain));

            float g_eff = Mathf.Sqrt(cfg.g * cfg.g + s.a_c * s.a_c);
            float g_eff_limit = Mathf.Lerp(g_eff, cfg.g * 0.65f, forceOverride);
            float deltaThetaTorque = Mathf.Asin(Mathf.Clamp(tau_hip_avail / Mathf.Max(1.0f, m2_l2 * g_eff_limit), 0f, 1.0f));

            float kappaTrunk = Mathf.Clamp(cfg.trunkInPhaseRatio, 0.25f, 0.85f);
            float angulationRatio = 1.0f - kappaTrunk;
            float L_eff = Mathf.Max(0.20f, L1 + kappaTrunk * L2);
            float maxAngShiftRad = Mathf.Min(phiMax, deltaThetaTorque) / Mathf.Max(0.15f, angulationRatio);
            float halfPi = 85.0f * Mathf.Deg2Rad;

            s.dy_min_torque = L_eff * (Mathf.Sin(Mathf.Max(-halfPi, s.theta1 - maxAngShiftRad)) - sin1);
            s.dy_max_torque = L_eff * (Mathf.Sin(Mathf.Min( halfPi, s.theta1 + maxAngShiftRad)) - sin1);

            float maxShiftDesign = Mathf.Max(0.05f, cfg.comShiftMaxMeters) * (1.0f + 0.55f * forceOverride);
            float dy_desired = s.u_shift_filtered * maxShiftDesign;
            s.dy_cmd = Mathf.Lerp(Mathf.Clamp(dy_desired, s.dy_min_torque, s.dy_max_torque), dy_desired, forceOverride);

            float delta_theta1_cmd = Mathf.Asin(Mathf.Clamp(sin1 + s.dy_cmd / L_eff, -0.99f, 0.99f)) - s.theta1;

            // 姿勢が低いほど「腰を横に出して上体を残す（外向傾）」比率を高め、ヘッドダイブを抑制する
            float lowPostureFactor = 1.0f - s.s_rom; // 低い姿勢ほど 1.0 に近づく
            float crossOverGate = Mathf.Clamp01((-s.theta1 * s.u_shift_filtered) / (15.0f * Mathf.Deg2Rad));
            float extIntent = Mathf.Clamp01(Mathf.Max(input.u_ext_norm, s.dl1 / 0.35f)) * s.s_rom;
            // 常時70%のヘッドダイブを廃止し、高姿勢での強い伸展クロスオーバー時のみ発生させる
            float headDiveRad = s.u_shift_filtered * (cfg.extHeadDiveMaxDeg * Mathf.Deg2Rad) * (0.25f * extIntent * crossOverGate);

            // マウス横移動時かつ低姿勢時は、頭ではなく腰が先に出るよう外向傾（phiと同符号）をしっかり効かせる
            float effectiveAngRatio = Mathf.Clamp(angulationRatio + 0.35f * lowPostureFactor, 0.35f, 0.90f);
            float minPhiLimit = Mathf.Min(-phiMax, cfg.phi_in_maxRad * 1.35f);
            float maxPhiLimit = Mathf.Max( phiMax, -cfg.phi_in_maxRad * 1.35f);
            float phi_target = Mathf.Clamp(effectiveAngRatio * (s.theta1 + delta_theta1_cmd) - headDiveRad, minPhiLimit, maxPhiLimit);

            float theta2_cur = s.theta1 - s.phi;
            float theta2_target = s.theta1 - phi_target;
            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - s.a_c * Mathf.Cos(theta2_cur));
            float tau_hip_ff   = m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - s.a_c * (1.0f - 0.85f * forceOverride) * Mathf.Cos(theta2_target));

            float kp_hip_eff = Mathf.Max(600.0f, cfg.kp_hip * 1.4f) * (1.0f + 1.25f * shiftActivity);
            float kd_hip_eff = Mathf.Max(45.0f, cfg.kd_hip * 1.2f);
            s.tau_hip_steer = Mathf.Clamp(tau_hip_ff + kp_hip_eff * (phi_target - s.phi) - kd_hip_eff * s.dphi, -tau_hip_avail, tau_hip_avail);

            // =========================================================================
            // 2. 下肢アクチュエーターと伸縮運動方程式 (l1, dl1)
            // =========================================================================
            EvaluateLegBiomechanics(
                s.l1, cfg,
                out s.s_rom, out s.knee_flex_deg, out s.knee_int_deg,
                out s.posture_force_ratio, out float F_iso, out float v_max_eff, out s.P_leg_peak_avail
            );

            float pHill = Mathf.Max(1.0f, cfg.hillPowerExponent);
            float fvFactor = 1.0f - Mathf.Pow(Mathf.Clamp01(s.dl1 / v_max_eff), pHill) + 0.15f * Mathf.Clamp01(-s.dl1 / 1.50f);
            float strokeRoomGate = Mathf.Clamp01((cfg.l_max - s.l1) / 0.025f);
            float staminaGate = Mathf.Clamp01(s.stamina / 20.0f);

            float u_norm = Mathf.Clamp01(input.u_ext_norm);
            float u_flex = Mathf.Clamp01(input.u_flex_norm);
            float F_avail = F_iso * fvFactor * strokeRoomGate * staminaGate;
            s.u_ext_actual = u_norm * (1.0f - u_flex) * F_avail;

            float dropHold = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float fadeWidth = Mathf.Max(0.005f, (cfg != null) ? cfg.supportFadeWidthMeters : SupportFadeZoneMeters);
            float hasTargetGate = Mathf.Clamp01(input.l1_target * 100.0f);
            float defaultHold = cfg.l_max - dropHold;
            float l_hold = Mathf.Lerp(defaultHold, Mathf.Clamp(input.l1_target, cfg.l_min, cfg.l_max), hasTargetGate);

            // 現在の保持長 l_hold を基準にした連続サポート係数（l_holdより上へ行くと滑らかに支持力を抜く）
            float phi_support = 1.0f - Mathf.SmoothStep(0f, 1f, (s.l1 - l_hold) / fadeWidth);
            float phi_recover = Mathf.SmoothStep(0f, 1f, (l_hold - s.l1) / fadeWidth);

            float absTheta1 = Mathf.Min(Mathf.Abs(s.theta1), 82f * Mathf.Deg2Rad);
            float cosAbsTheta1 = Mathf.Max(0.15f, Mathf.Cos(absTheta1));
            float sinAbsTheta1 = Mathf.Sin(absTheta1);
            float absPsi = Mathf.Abs(s.Psi_CoM);
            float currentLatG = Mathf.Abs(s.a_c) / cfg.g;

            s.F_compress = Mathf.Max(0.1f * mg, mg * (Mathf.Cos(absPsi) + currentLatG * Mathf.Sin(absPsi)));

            // 保持長 l_hold に対する対称で滑らかな姿勢維持バネ力と臨界減衰
            float posError = l_hold - s.l1;
            float maxSpringForce = cfg.passiveSpringMaxG * mg;
            float stanceSpringForce = maxSpringForce * (float)Math.Tanh((cfg.k_passive * posError) / Mathf.Max(1.0f, maxSpringForce));
            float stanceReturnForce = stanceSpringForce * (1.0f - u_flex);

            float l_eq_safe = Mathf.Max(0.45f, cfg.l_eq);
            float c_crit = 2.0f * Mathf.Sqrt(cfg.M * Mathf.Max(100.0f, cfg.k_passive));
            float c_ecc = Mathf.Max(c_crit * 0.85f, cfg.c_ecc_factor * cfg.M * Mathf.Sqrt(cfg.g / l_eq_safe));

            // 強い伸展・屈曲入力時のみダンピングを緩め、中立保持時は臨界減衰(dampingMult ≈ 1)でピタッと止める
            float activeInputMag = Mathf.Clamp01(u_norm + u_flex);
            float dampingMult = Mathf.Lerp(1.05f, 0.40f, activeInputMag);

            // 目標高より高い位置から縮んでいる最中のみ抜重(tuck)を効かせ、低姿勢保持・横プッシュ時は踏力を残す
            float flexUnweightGate = (hasTargetGate > 0.5f)
                ? Mathf.Clamp01((s.l1 - l_hold) / 0.04f) * (1.0f - 0.65f * shiftActivity)
                : (1.0f - 0.55f * shiftActivity);
            float u_flex_eff = u_flex * flexUnweightGate;

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex_eff);
            float activeTuckPull = u_flex_eff * (0.95f + cfg.releaseTuckPullG) * mg;

            float passiveForce = selfWeightSupport
                               + stanceSpringForce * (1.0f - u_flex_eff)
                               - activeTuckPull
                               - dampingMult * c_ecc * s.dl1;

            s.F_leg = Mathf.Max(0f, passiveForce + s.u_ext_actual);

            // 保持時は自重 mg、伸展・深バンク時は F_compress と滑らかにブレンド
            float loadBlend = Mathf.Clamp01(u_norm + u_flex + Mathf.SmoothStep(0f, 1f, (s.l1 - l_hold) / fadeWidth));
            float effectiveLoad = Mathf.Lerp(mg, s.F_compress, loadBlend);

            float ddl1 = (passiveForce + s.u_ext_actual - effectiveLoad) / cfg.M;
            float dl1_free = s.dl1 + ddl1 * dt;
            float l1_prev = s.l1;
            s.l1 = Mathf.Clamp(l1_prev + dl1_free * dt, cfg.l_min, cfg.l_max);
            s.dl1 = (s.l1 - l1_prev) / dt;

            // 上方加速度 G_z_up・パワー・スタミナの連続更新
            float v_com_ext = Mathf.Max(0f, s.dl1);
            float v_z_ext = v_com_ext * cosAbsTheta1;
            float G_z_kin = (v_z_ext * v_z_ext) / (2.0f * cfg.g * Mathf.Max(0.12f, dropHold));
            float G_z_inst = (Mathf.Max(0f, ddl1) * cosAbsTheta1) / cfg.g;
            float targetUpG = u_norm * Mathf.Clamp01(v_com_ext * 100.0f) * Mathf.Max(G_z_kin, 0.55f * G_z_inst);
            s.G_z_up = Mathf.Max(targetUpG, Mathf.MoveTowards(s.G_z_up, 0f, 3.8f * dt));

            float refInertia = Mathf.Max(1.0f, cfg.M * l_eq_safe * l_eq_safe * 0.65f);
            float inertiaFactor = Mathf.Clamp(s.I_eff / refInertia, 0.4f, 1.6f);
            float extMotionGate = Mathf.Clamp01(v_com_ext * 100.0f);
            s.u_ext_actual *= extMotionGate;
            s.P_leg_ext = s.u_ext_actual * v_com_ext * inertiaFactor;
            s.P_step_eff = s.P_leg_ext;

            float recoveryRate = (1.0f - Mathf.Clamp01(u_norm * 20.0f)) * cfg.staminaRecoveryRate;
            s.stamina = Mathf.Clamp(s.stamina + (recoveryRate - s.P_leg_ext) * dt, 0f, cfg.maxStamina);

            float xi_ext = 1.0f - phi_support;
            float baseSupportCoeff = Mathf.Lerp(phi_support, 1.0f, Mathf.Clamp01(u_norm + hasTargetGate));
            float vertSupportG = (baseSupportCoeff - xi_ext * s.G_z_up) * (1.0f - 0.85f * u_flex_eff);

            s.F_vert = mg * vertSupportG + stanceReturnForce - activeTuckPull
                     - dampingMult * c_ecc * s.dl1 * cosAbsTheta1 + s.u_ext_actual * cosAbsTheta1;

            float v_y_ext = v_com_ext * sinAbsTheta1;
            float a_y_ext = Mathf.Max(0f, ddl1) * sinAbsTheta1;
            float dy_shift_rate = Mathf.Abs(u_x - s.u_shift_filtered) / tauShift * maxShiftDesign;
            s.F_lat_step = s.u_ext_actual * sinAbsTheta1
                         + cfg.M * (a_y_ext + 2.5f * v_y_ext + 1.5f * dy_shift_rate + Mathf.Abs(s.a_c) * Mathf.Sin(absPsi));

            float F_vert_pos = Mathf.Max(0f, s.F_vert);
            s.F_leg = Mathf.Max(F_vert_pos, F_vert_pos * cosAbsTheta1 + s.F_lat_step * sinAbsTheta1);

            // =========================================================================
            // 3. 台車の水平運動 (x_c, v_c) と 実効遠心加速度 a_c
            // =========================================================================
            float playRad = FreePlayDeg * Mathf.Deg2Rad;
            float biteRatio = Mathf.Clamp01(absTheta1 / playRad + shiftActivity * 10.0f);
            s.G_base = G_eq * biteRatio * biteRatio;

            float F_req_ideal = mg * (Mathf.Cos(absPsi) + Mathf.Abs(G_eq) * Mathf.Sin(absPsi));
            float loadRatio = s.F_leg / Mathf.Max(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float maxLambda = Mathf.Lerp(1.0f, Mathf.Max(1.2f, cfg.lambda_max), Mathf.Clamp01((s.P_leg_ext + s.F_lat_step * v_y_ext) / 600.0f));
            s.lambda_sustain = Mathf.Clamp(Mathf.Pow(Mathf.Clamp(loadRatio, 0f, 1.8f), cfg.loadSustainPower_q), 0f, maxLambda);

            // 雪面挫折 (Snow Shear Failure) 連続ストレス計算
            float minSkidRad = Mathf.Max(3.0f, cfg.skidMinAngleDeg) * Mathf.Deg2Rad;
            float refSkidRad = Mathf.Max(cfg.skidMinAngleDeg + 5.0f, cfg.skidRefAngleDeg) * Mathf.Deg2Rad;
            float angleGate = Mathf.SmoothStep(0f, 1f, (absTheta1 - minSkidRad * 0.75f) / Mathf.Max(0.02f, minSkidRad * 0.5f));
            float phi_angle = angleGate * Mathf.Pow(sinAbsTheta1 / Mathf.Max(0.2f, Mathf.Sin(refSkidRad)), cfg.snowAngleVulnerabilityExp);

            float limitVertG = Mathf.Max(0.4f, cfg.snowFailureVerticalForceG) * Mathf.Lerp(1.0f, cosAbsTheta1, 0.55f);
            float phi_vert = Mathf.Max(0f, (s.F_vert / mg - 0.55f * cosAbsTheta1) / Mathf.Max(0.25f, limitVertG));
            float phi_power = s.P_step_eff / Mathf.Max(150.0f, cfg.snowFailurePowerThreshold);

            s.snow_stress_ratio = phi_angle * phi_vert * phi_power;

            float skidOnset = Mathf.SmoothStep(0.95f, 1.05f, s.snow_stress_ratio);
            float targetSkid = skidOnset * Mathf.Clamp01(0.65f + 0.35f * (s.snow_stress_ratio - 1.0f));
            float recTau = Mathf.Lerp(Mathf.Min(0.06f, cfg.skidRecoveryTau), Mathf.Max(0.05f, cfg.skidRecoveryTau), extMotionGate);
            float skidTau = Mathf.Lerp(recTau, 0.025f, skidOnset);
            s.skid_intensity = Mathf.MoveTowards(s.skid_intensity, targetSkid, dt / skidTau);

            // 台車の運動方程式（Clampと位置差分によりストッパー衝突を単一式で処理）
            float m_c = Mathf.Max(1.0f, cfg.carriageMass);
            float effectiveAngle = Mathf.Max(absTheta1, absPsi);
            const float C_rail = 75.0f;
            float c_glide = Mathf.Lerp(C_rail * 0.35f, C_rail, Mathf.Clamp01(effectiveAngle / (20f * Mathf.Deg2Rad)));

            float sinRoll = Mathf.Sin(effectiveAngle);
            float angleShapeFactor = Mathf.Lerp(1.0f, 0.40f, Mathf.Clamp01((effectiveAngle - 5f * Mathf.Deg2Rad) / (25f * Mathf.Deg2Rad)))
                                   + 0.35f * (sinRoll * sinRoll);

            float rawTanDrive = s.y_CoM / Mathf.Max(0.18f, s.z_CoM);
            float tanAngleDrive = 1.65f * (float)Math.Tanh(rawTanDrive / 1.65f);
            s.v_c_target = (tanAngleDrive * cfg.g) * 1.217f * angleShapeFactor;

            float carvingRetention = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, s.skid_intensity);
            float F_drive = C_rail * s.v_c_target * carvingRetention;

            float M_inertia = s.I_eff / Mathf.Max(0.15f, s.R_CoM * s.R_CoM);
            float inertiaMassRatio = M_inertia / (m_c + M_inertia);
            s.v_c_recoil_max = inertiaMassRatio * v_y_ext;

            float dirSign = (Mathf.Abs(s.y_CoM) > 0.001f) ? Mathf.Sign(s.y_CoM) : Mathf.Sign(s.theta1);
            float dynamicFootForce = (s.P_leg_ext / Mathf.Max(0.3f, v_max_eff)) * sinAbsTheta1;
            s.skid_pushback_force = -dirSign * cfg.carriagePushbackGain * inertiaMassRatio * dynamicFootForce * s.skid_intensity;

            // 姿勢を低くして重心シフトする際、脚が台車を逆方向（左シフトなら右方向 +x）へ押す水平反力
            float lowPushLeverage = 1.0f + 0.65f * (1.0f - s.s_rom);
            float legPushForce = -s.u_shift_filtered * (0.35f * s.F_leg) * lowPushLeverage * inertiaMassRatio;
            // 台車が向かってくる（重心シフトと逆向きに動いている、または追いかけてくる）際の脚の踏ん張り抵抗
            float oncomingResist = (s.v_c * s.u_shift_filtered > 0f) ? (-c_glide * 0.45f * s.v_c * shiftActivity) : 0f;

            float halfRail = cfg.W_rail * 0.5f;
            float x_c_prev = s.x_c;
            float v_c_prev = s.v_c;
            float v_c_free = v_c_prev + ((F_drive + s.skid_pushback_force + legPushForce + oncomingResist - c_glide * v_c_prev) / m_c) * dt;
            s.x_c = Mathf.Clamp(x_c_prev + v_c_free * dt, -halfRail, halfRail);
            s.v_c = (s.x_c - x_c_prev) / dt;
            float ddx_c = (s.v_c - v_c_prev) / dt;

            float v_req = Mathf.Abs(s.v_c_target);
            float v_realized = Mathf.Max(0f, Mathf.Sign(s.v_c_target) * s.v_c);
            float eta_carriage = 1.0f - Mathf.Clamp01((v_req - v_realized) / Mathf.Max(0.20f, v_req));

            // 連続グリップ係数と実効遠心加速度 a_c
            float N_min = cfg.N_min_G * mg;
            float weightGate = Mathf.Lerp(0.75f, 1.0f, Mathf.Clamp01(Mathf.Max(s.F_vert, s.F_leg) / Mathf.Max(1.0f, N_min)));
            float skidGripGate = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, Mathf.Clamp01(s.skid_intensity * 1.25f));
            bool isHipCrash = s.z_H < cfg.z_H_min;
            bool isBootOut  = absTheta1 >= cfg.theta_bootOutRad;
            float gripFactor = (isHipCrash || isBootOut) ? 0f : (weightGate * skidGripGate);

            float a_c_target = (s.G_base * cfg.g) * s.lambda_sustain * gripFactor * eta_carriage;
            s.a_c += ((a_c_target - s.a_c) / Mathf.Max(0.005f, Mathf.Min(0.015f, cfg.tau_G_lag))) * dt;
            s.kappa = s.a_c / Mathf.Max(4.0f, s.V * s.V);
            s.kappa_target = a_c_target / Mathf.Max(4.0f, s.V * s.V);

            // =========================================================================
            // 4. ブーツ支点(台車)まわりのロール運動方程式 (theta1, omega1, phi, dphi)
            // =========================================================================
            if (input.w_reset)
            {
                s.theta1 = Mathf.MoveTowards(s.theta1, 0f, 3.0f * dt);
                s.omega1 = Mathf.MoveTowards(s.omega1, 0f, 15.0f * dt);
                s.phi = Mathf.MoveTowards(s.phi, 0f, 6.0f * dt);
                s.dphi = s.phi_bend = s.dphi_bend = s.tau_hip_steer = s.P_hip_steer = 0f;
            }
            else
            {
                float tau_gravity = mg * s.y_CoM;
                float tau_carriage_com = -cfg.M * (0.35f * ddx_c + s.a_c) * s.z_CoM;

                // 外向傾(phi)を作った際に腰を逆方向へ押し戻してしまう成分を除去し、純粋な脚の横押し駆動トルクにする
                float lowStanceBoost = 1.0f + 0.75f * (1.0f - s.s_rom);
                float legSupportRatio = Mathf.Clamp(s.F_leg / mg, 0.35f, 1.8f);
                float kp_leg_roll = cfg.comShiftRollDriveGain * cfg.mouseRollMomentGain * mg * s.l1 * lowStanceBoost * legSupportRatio;
                float kd_leg_roll = 0.18f * kp_leg_roll;
                float tau_leg_lateral_drive = kp_leg_roll * delta_theta1_cmd - kd_leg_roll * s.omega1;

                // 台車が進んでくる方向と逆に脚で踏ん張ることで、重心を台車の向こう側（左）へ送り出すトルク
                float oncomingPushTorque = s.u_shift_filtered * Mathf.Abs(s.v_c) * (0.45f * mg * s.z_CoM) * lowStanceBoost;

                float skidRightingPenalty = 1.0f - 0.40f * s.skid_intensity;
                float activeExtMotionGate = Mathf.Clamp01(Mathf.Max(v_com_ext / 0.20f, (s.u_ext_actual / (1.2f * mg)) * 0.85f));
                // マウスで同方向へ倒し込んでいる最中は、伸展による直立戻しトルク(tau_leg_carriage_push)を抑制
                float rightingSuppress = 1.0f - 0.80f * Mathf.Clamp01(s.u_shift_filtered * Mathf.Sign(s.theta1));
                float tau_leg_carriage_push = -eta_carriage * activeExtMotionGate * skidRightingPenalty * rightingSuppress
                    * (cfg.extRightingTorqueGain * s.u_ext_actual * s.l1 + cfg.extVelocityRightingGain * mg * v_com_ext) * sin1;

                // 股関節トルクによる脚リンクへの結合：外向傾(phi < 0)で腰が左(theta1 < 0)へ出るよう符号を一致させる
                float tau_hip_carriage = -eta_carriage * (0.25f * cfg.hipReactionGain) * s.tau_hip_steer;
                float tau_damping = -Mathf.Min(cfg.gamma_roll, 10.0f) * s.omega1;

                float domega1 = (tau_gravity + tau_carriage_com + tau_leg_lateral_drive + oncomingPushTorque
                               + tau_leg_carriage_push + tau_hip_carriage + tau_damping) / s.I_eff;

                float theta1_prev = s.theta1;
                float omega1_free = s.omega1 + domega1 * dt;
                s.theta1 = Mathf.Clamp(theta1_prev + omega1_free * dt, -halfPi, halfPi);
                s.omega1 = (s.theta1 - theta1_prev) / dt;

                float ddphi = (s.tau_hip_steer - tau_hip_load - cfg.kd_hip_steer * s.dphi) / I2_hip;
                float maxHipVel = Mathf.Max(10.0f, cfg.dphi_max * (2.0f + 1.5f * shiftActivity));
                float dphi_free = Mathf.Clamp(s.dphi + ddphi * dt, -maxHipVel, maxHipVel);

                float phi_prev = s.phi;
                s.phi = Mathf.Clamp(phi_prev + dphi_free * dt, minPhiLimit, maxPhiLimit);
                s.dphi = (s.phi - phi_prev) / dt;

                s.P_hip_steer = Mathf.Abs(s.tau_hip_steer * s.dphi);
                s.phi_bend = s.phi - phi_ref;
                s.dphi_bend = s.dphi;
            }

            UpdateKinematics(ref s, cfg);

            float v_y_com_actual = (s.y_CoM - prev_y_CoM) / dt;
            float dirSignPost = (Mathf.Abs(s.theta1) > 0.001f) ? Mathf.Sign(s.theta1) : Mathf.Sign(s.y_CoM);
            s.F_lat_step += cfg.M * 1.8f * Mathf.Max(0f, dirSignPost * v_y_com_actual);
            s.dy_actual = s.y_CoM - (L1 * Mathf.Sin(s.theta1 - delta_theta1_cmd) + L2 * Mathf.Sin(s.theta2_ref));

            // テレメトリ・クラッシュ状態判定
            bool isAtRailLimit = Mathf.Abs(s.x_c) >= (halfRail - 1e-4f);
            if (s.z_H < cfg.z_H_min)                                      { s.gripState = SkiGripState.HipCrash;     s.a_c = 0f; }
            else if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutRad)          { s.gripState = SkiGripState.BootOut;      s.a_c = 0f; }
            else if (isAtRailLimit)                                        { s.gripState = SkiGripState.RailLimitHit; }
            else if (s.snow_stress_ratio >= 1.0f || s.skid_intensity > 0.15f) { s.gripState = SkiGripState.Skidding;  }
            else if (s.F_vert < N_min && s.F_leg < N_min)                  { s.gripState = SkiGripState.Unweighted;   }
            else                                                           { s.gripState = SkiGripState.Carving;      }

            // =========================================================================
            // 5. 進行方向速度 V(t)（スキッド摩擦も連続変化）
            // =========================================================================
            float gravityDrive = cfg.g * Mathf.Sin(cfg.alphaRad) * 0.35f;
            float frictionMult = 1.0f + 2.0f * s.skid_intensity;
            float snowFriction = cfg.mu_snow * frictionMult * (Mathf.Max(0f, s.F_leg) / cfg.M);
            float dragCoeffPerMass = Mathf.Lerp(0.0022f, 0.0036f, s.s_rom);
            float airDrag = dragCoeffPerMass * s.V * s.V;
            float pumpPropulsion = (0.045f * s.P_leg_ext) / (cfg.M * Mathf.Max(2.0f, s.V));

            s.V = Mathf.Clamp(s.V + (gravityDrive - snowFriction - airDrag + pumpPropulsion) * dt, 4.0f, 32.0f);
        }

        public static void UpdateKinematics(ref SimState s, SkyTechSimConfig cfg)
        {
            float theta2 = s.theta1 - s.phi;
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);
            float totalMass = Mathf.Max(1f, cfg.m1 + cfg.m2);

            s.y_CoM = (cfg.m1 * cfg.c1 * s.l1 * sin1 + cfg.m2 * (s.l1 * sin1 + cfg.l2 * Mathf.Sin(theta2))) / totalMass;
            s.z_CoM = (cfg.m1 * cfg.c1 * s.l1 * cos1 + cfg.m2 * (s.l1 * cos1 + cfg.l2 * Mathf.Cos(theta2))) / totalMass;
            s.Psi_CoM = Mathf.Atan2(s.y_CoM, Mathf.Max(0.01f, s.z_CoM));
            s.R_CoM = Mathf.Sqrt(s.y_CoM * s.y_CoM + s.z_CoM * s.z_CoM);
            s.z_H = s.l1 * cos1;
        }
    }
}