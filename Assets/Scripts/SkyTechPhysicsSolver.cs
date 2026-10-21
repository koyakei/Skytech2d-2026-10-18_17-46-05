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

        /// <summary>(kg*m^2) 現在の重心まわりの実効慣性モーメント I_eff</summary>
        public float I_eff;
        /// <summary>(N) 鉛直下方向の荷重（2G上方加速での伸び切り時は -1G = -M*g）</summary>
        public float F_vert;
        /// <summary>(G) 重心の鉛直上方加速度 G_z_up</summary>
        public float G_z_up;
        /// <summary>(N) 横方向の重心移動による横方向踏み込み力</summary>
        public float F_lat_step;
        /// <summary>(W) 雪面挫折判定用の実効踏み込みパワー（重心が動いているときのみ発生）</summary>
        public float P_step_eff;
        /// <summary>[0..2+] 雪面挫折限界に対する現在の負荷率（1.0以上でスキッディング）</summary>
        public float snow_stress_ratio;
        /// <summary>[0..1] スキッディング（雪面挫折）の持続強度</summary>
        public float skid_intensity;
        /// <summary>(m/s) 角度依存の台車目標移動速度</summary>
        public float v_c_target;
        /// <summary>(m/s) 重心の慣性モーメント反作用による台車の反対側への最大許容速度（テレメトリ用）</summary>
        public float v_c_recoil_max;
        /// <summary>(N) 雪面挫折により台車が脚に押し負ける反力</summary>
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
            float defaultStanceL1 = cfg.l_max - drop;
            float initialL1 = Mathf.Clamp(defaultStanceL1, cfg.l_min, cfg.l_max);

            var s = new SimState
            {
                x_c = 0f,
                v_c = 0f,
                theta1 = 0.0f,
                omega1 = 0f,
                l1 = initialL1,
                dl1 = 0f,
                phi = 0f,
                dphi = 0f,
                phi_bend = 0f,
                dphi_bend = 0f,
                P_hip_steer = 0f,
                tau_hip_steer = 0f,
                u_shift_filtered = 0f,
                theta2_ref = 0f,
                y_CoM_ref = 0f,
                z_CoM_ref = initialL1,
                dy_cmd = 0f,
                dy_actual = 0f,
                dy_min_torque = 0f,
                dy_max_torque = 0f,
                V = cfg.V_ref,
                stamina = cfg.maxStamina,
                gripState = SkiGripState.Carving
            };

            UpdateKinematics(ref s, cfg);
            EvaluateLegBiomechanics(
                s.l1,
                cfg,
                out s.s_rom,
                out s.knee_flex_deg,
                out s.knee_int_deg,
                out s.posture_force_ratio,
                out _,
                out _,
                out s.P_leg_peak_avail
            );

            s.I_eff = cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1
                    + cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2);
            s.y_CoM_ref = s.y_CoM;
            s.z_CoM_ref = s.z_CoM;
            s.F_compress = cfg.M * cfg.g;
            s.F_leg = cfg.M * cfg.g;
            s.F_vert = cfg.M * cfg.g;
            s.G_z_up = 0f;
            s.F_lat_step = 0f;
            s.P_step_eff = 0f;
            s.snow_stress_ratio = 0f;
            s.skid_intensity = 0f;
            s.v_c_target = 0f;
            s.v_c_recoil_max = 0f;
            s.skid_pushback_force = 0f;
            s.lambda_sustain = 1.0f;
            return s;
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float drop = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float fade = (cfg != null) ? cfg.supportFadeWidthMeters : SupportFadeZoneMeters;
            float l_hold = l_max - drop;
            float xi = Mathf.Clamp01((l1 - l_hold) / Mathf.Max(0.001f, fade));
            return 1.0f - (xi * xi * (3.0f - 2.0f * xi));
        }

        public static float ComputeSelfWeightSupportFactor(float l1, float l_max)
        {
            return ComputeSelfWeightSupportFactor(l1, l_max, null);
        }

        public static float ComputeSquatRecoveryFactor(float l1, float l_max, SkyTechSimConfig cfg = null)
        {
            float drop = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            float fade = (cfg != null) ? cfg.supportFadeWidthMeters : SupportFadeZoneMeters;
            float l_hold = l_max - drop;
            float xi = Mathf.Clamp01((l_hold - l1) / Mathf.Max(0.001f, fade));
            return xi * xi * (3.0f - 2.0f * xi);
        }

        public static float ComputeSquatRecoveryFactor(float l1, float l_max)
        {
            return ComputeSquatRecoveryFactor(l1, l_max, null);
        }

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
            float cosHalfKnee = Mathf.Clamp(l1 / l_full, 0.0f, 1.0f);
            knee_flex_deg = 2.0f * Mathf.Acos(cosHalfKnee) * Mathf.Rad2Deg;
            knee_int_deg = 180.0f - knee_flex_deg;

            float s_opt = Mathf.Clamp(cfg.legOptRomRatio, 0.50f, 0.95f);
            float l_opt = cfg.l_min + s_opt * romSpan;

            float r_opt = Mathf.Sqrt(Mathf.Max(1e-4f, l_full * l_full - l_opt * l_opt));
            float r_cur = Mathf.Sqrt(Mathf.Max(1e-4f, l_full * l_full - l1 * l1));
            float leverRatio = r_opt / r_cur;

            float phi_v;
            if (s_rom <= s_opt)
            {
                float sigmaF = Mathf.Max(0.08f, cfg.squatForceDropWidth);
                float dsF = (s_opt - s_rom) / sigmaF;
                float leverTerm = Mathf.Min(1.0f, Mathf.Pow(leverRatio, cfg.kneeLeverPenaltyExp));
                float fMin = Mathf.Clamp01(cfg.squatMinForceRatio);
                phi_F = fMin + (1.0f - fMin) * leverTerm * Mathf.Exp(-(dsF * dsF));

                float sigmaV = Mathf.Max(0.10f, cfg.squatVelDropWidth);
                float dsV = (s_opt - s_rom) / sigmaV;
                float vMin = Mathf.Clamp01(cfg.squatMinVelRatio);
                phi_v = vMin + (1.0f - vMin) * Mathf.Exp(-(dsV * dsV));
            }
            else
            {
                float sigmaF = Mathf.Max(0.05f, cfg.lockoutForceDropWidth);
                float dsF = (s_rom - s_opt) / sigmaF;
                float fLock = Mathf.Clamp01(cfg.lockoutMinForceRatio);
                phi_F = fLock + (1.0f - fLock) * Mathf.Exp(-(dsF * dsF));

                float sigmaV = Mathf.Max(0.05f, cfg.lockoutVelDropWidth);
                float dsV = (s_rom - s_opt) / sigmaV;
                float vLock = Mathf.Clamp01(cfg.lockoutMinVelRatio);
                phi_v = vLock + (1.0f - vLock) * Mathf.Exp(-(dsV * dsV));
            }

            float mg = cfg.M * cfg.g;
            F_iso = cfg.F_ext_max_G * mg * phi_F;
            v_max_eff = Mathf.Max(0.20f, cfg.v_max * phi_v);

            float pHill = Mathf.Max(1.0f, cfg.hillPowerExponent);
            float vOptRatio = Mathf.Pow(1.0f / (pHill + 1.0f), 1.0f / pHill);
            float forceAtOptVel = 1.0f - Mathf.Pow(vOptRatio, pHill);
            P_peak_avail = F_iso * (v_max_eff * vOptRatio) * forceAtOptVel;
        }

        public static void Step(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            if (dt <= 0f) return;

            float subDt = dt / SubSteps;
            for (int i = 0; i < SubSteps; i++)
            {
                StepSub(ref s, in input, cfg, subDt);
                if (s.gripState == SkiGripState.HipCrash || s.gripState == SkiGripState.BootOut)
                {
                    break;
                }
            }
        }

        private static void StepSub(ref SimState s, in SimInput input, SkyTechSimConfig cfg, float dt)
        {
            float totalMass = Mathf.Max(1.0f, cfg.m1 + cfg.m2);
            float L1 = ((cfg.m1 * cfg.c1 + cfg.m2) / totalMass) * s.l1;
            float L2 = (cfg.m2 / totalMass) * cfg.l2;
            float m2_l2 = cfg.m2 * cfg.l2;
            float I2_hip = Mathf.Max(0.5f, cfg.m2 * cfg.l2 * cfg.l2);

            float prev_y_CoM = s.y_CoM;

            // ブーツ支点まわりの合成慣性モーメント I_eff
            s.I_eff = cfg.m1 * cfg.c1 * cfg.c1 * s.l1 * s.l1
                    + cfg.m2 * (s.l1 * s.l1 + cfg.l2 * cfg.l2 + 2.0f * s.l1 * cfg.l2 * Mathf.Cos(s.phi));
            s.I_eff = Mathf.Max(2.0f, s.I_eff);

            float phiMax = Mathf.Max(0.08f, cfg.phi_out_maxRad);
            float tauHipMax = Mathf.Max(100.0f, cfg.hipSteerMaxIsoTorque);

            // =========================================================================
            // 1. 基準姿勢 θ_{2,ref}(θ_1) と定常釣り合い横G (G_eq)
            // =========================================================================
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);

            float u_x = (input.w_reset || input.steer_neutral_cmd) ? 0f : Mathf.Clamp(-input.phi_steer_cmd, -1.0f, 1.0f);
            float tauShift = Mathf.Max(0.01f, cfg.comShiftResponseTau);
            if (input.w_reset)
            {
                s.u_shift_filtered = 0f;
            }
            else
            {
                s.u_shift_filtered += ((u_x - s.u_shift_filtered) / tauShift) * dt;
            }

            float shiftActivity = Mathf.Clamp01(Mathf.Max(Mathf.Abs(u_x), Mathf.Abs(s.u_shift_filtered)));

            // マウス横入力時は合力の延長線上に留まる拘束を解除し、マウス方向への傾斜を優先する
            float forceOverride = 0.88f * shiftActivity;

            float z_upright_approx = Mathf.Max(0.20f, L1 * cos1 + L2);
            float G0_approx = (L1 * sin1) / z_upright_approx;
            float theta_res0 = Mathf.Atan(G0_approx);
            float g_eff0 = cfg.g * Mathf.Sqrt(1.0f + G0_approx * G0_approx);

            float etaShiftReserve = Mathf.Clamp(cfg.shiftPriorityReserveRatio, 0.65f, 0.95f);
            float etaVertBudget = (1.0f - etaShiftReserve) * (1.0f - forceOverride);
            float alphaVert = Mathf.Clamp01(cfg.refVerticalKeepRatio) * (1.0f - forceOverride);

            float sinDeltaVert = Mathf.Clamp((etaVertBudget * tauHipMax) / Mathf.Max(1.0f, m2_l2 * g_eff0), 0f, 1.0f);
            float deltaVertMax = Mathf.Asin(sinDeltaVert);

            float desiredLeanOut = Mathf.Clamp(alphaVert * theta_res0, -deltaVertMax, +deltaVertMax);
            float theta2_des = theta_res0 - desiredLeanOut;

            float phiRefLimit = etaVertBudget * phiMax;
            s.theta2_ref = Mathf.Clamp(theta2_des, s.theta1 - phiRefLimit, s.theta1 + phiRefLimit);
            float phi_ref = s.theta1 - s.theta2_ref;

            float sin2_ref = Mathf.Sin(s.theta2_ref);
            float cos2_ref = Mathf.Cos(s.theta2_ref);
            s.y_CoM_ref = L1 * sin1 + L2 * sin2_ref;
            s.z_CoM_ref = Mathf.Max(0.18f, L1 * cos1 + L2 * cos2_ref);
            float G_eq = s.y_CoM_ref / s.z_CoM_ref;

            // =========================================================================
            // 2. 股関節アクチュエーター制御（tau_hip_steer による体幹・重心位置の駆動）
            // =========================================================================
            float g_eff = Mathf.Sqrt(cfg.g * cfg.g + s.a_c * s.a_c);

            float u_stroke_mag = Mathf.Clamp01(Mathf.Abs(s.phi) / phiMax);
            float L0 = Mathf.Max(0.1f, cfg.hipMuscleOptLengthL0);
            float Lmin = Mathf.Clamp(cfg.hipMuscleMinLengthLmin, 0.05f, L0);
            float W = Mathf.Max(0.1f, cfg.hipMuscleGaussianWidthW);
            float muscleL = L0 - (L0 - Lmin) * (0.5f * u_stroke_mag);
            float normStrain = (muscleL - L0) / (W * L0);
            float gaussianFactor = Mathf.Exp(-(normStrain * normStrain));
            float tau_hip_avail = tauHipMax * gaussianFactor;

            float g_eff_for_limit = Mathf.Lerp(g_eff, cfg.g * 0.65f, forceOverride);
            float sinDeltaTorque = Mathf.Clamp(tau_hip_avail / Mathf.Max(1.0f, m2_l2 * g_eff_for_limit), 0f, 1.0f);
            float deltaThetaTorque = Mathf.Asin(sinDeltaTorque);

            float kappaTrunk = Mathf.Clamp(cfg.trunkInPhaseRatio, 0.25f, 0.85f);
            float angulationRatio = 1.0f - kappaTrunk;
            float L_eff = Mathf.Max(0.20f, L1 + kappaTrunk * L2);

            float maxAngShiftRad = Mathf.Min(phiMax, deltaThetaTorque) / Mathf.Max(0.15f, angulationRatio);
            float halfPi = 85.0f * Mathf.Deg2Rad;
            float theta1_shift_min = Mathf.Max(-halfPi, s.theta1 - maxAngShiftRad);
            float theta1_shift_max = Mathf.Min(+halfPi, s.theta1 + maxAngShiftRad);

            s.dy_min_torque = L_eff * (Mathf.Sin(theta1_shift_min) - sin1);
            s.dy_max_torque = L_eff * (Mathf.Sin(theta1_shift_max) - sin1);

            float maxShiftDesign = Mathf.Max(0.05f, cfg.comShiftMaxMeters) * (1.0f + 0.55f * forceOverride);
            float dy_desired = s.u_shift_filtered * maxShiftDesign;
            s.dy_cmd = Mathf.Lerp(
                Mathf.Clamp(dy_desired, s.dy_min_torque, s.dy_max_torque),
                dy_desired,
                forceOverride
            );

            float delta_s1_cmd = s.dy_cmd / L_eff;
            float s1_target = Mathf.Clamp(sin1 + delta_s1_cmd, -0.99f, 0.99f);
            float theta1_shift_target = Mathf.Asin(s1_target);
            float delta_theta1_cmd = theta1_shift_target - s.theta1;

            // 脚伸展 ＋ 脚側(クロスオーバー方向)へのマウス移動による体幹ダイブ目標角
            // ベース比率を引き上げることで、真ん中右や右下(屈曲時)でも素直かつ強力にマウス方向へロール可能にする
            float leanFactor = Mathf.Clamp01(Mathf.Abs(s.theta1) / (15.0f * Mathf.Deg2Rad));
            float extIntent = Mathf.Clamp01(Mathf.Max(input.u_ext_norm, s.dl1 / 0.35f));
            float crossOverFactor = (s.theta1 * s.u_shift_filtered < 0f) ? (1.0f + 0.60f * leanFactor) : 1.0f;
            float headDiveRad = s.u_shift_filtered * (38.0f * Mathf.Deg2Rad) * (0.70f + 0.30f * extIntent * crossOverFactor);

            float effectiveAngRatio = angulationRatio * (1.0f - forceOverride);
            float phi_base_ang = effectiveAngRatio * s.theta1;
            // マウス操作時は第1項の外向傾ロック(体幹を逆方向・左へ曲げる拘束)を解除し、マウス方向へ素直にロールさせる
            float delta_phi_cmd = effectiveAngRatio * delta_theta1_cmd - headDiveRad;

            float minPhiLimit = Mathf.Min(-phiMax, cfg.phi_in_maxRad * 1.35f);
            float maxPhiLimit = Mathf.Max(phiMax, -cfg.phi_in_maxRad * 1.35f);
            float phi_target = Mathf.Clamp(phi_base_ang + delta_phi_cmd, minPhiLimit, maxPhiLimit);
            float theta2_target = s.theta1 - phi_target;

            float theta2_cur = s.theta1 - s.phi;
            float a_c_ff = s.a_c * (1.0f - 0.85f * forceOverride);
            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - s.a_c * Mathf.Cos(theta2_cur));
            float tau_hip_ff   = m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - a_c_ff * Mathf.Cos(theta2_target));

            float kp_base = Mathf.Max(600.0f, cfg.kp_hip * 1.4f);
            float kp_shift_boost = kp_base * (1.0f + 1.25f * shiftActivity);
            float kd_ctrl = Mathf.Max(45.0f, cfg.kd_hip * 1.2f);

            float tau_hip_pd = kp_shift_boost * (phi_target - s.phi) - kd_ctrl * s.dphi;
            s.tau_hip_steer = Mathf.Clamp(tau_hip_ff + tau_hip_pd, -tau_hip_avail, +tau_hip_avail);

            // =========================================================================
            // 3. 下肢アクチュエーター（伸展・屈曲筋力 F_leg）
            // =========================================================================
            EvaluateLegBiomechanics(
                s.l1,
                cfg,
                out s.s_rom,
                out s.knee_flex_deg,
                out s.knee_int_deg,
                out s.posture_force_ratio,
                out float F_iso,
                out float v_max_eff,
                out s.P_leg_peak_avail
            );

            float pHill = Mathf.Max(1.0f, cfg.hillPowerExponent);
            float fvFactor;
            if (s.dl1 > 0f)
            {
                float vNorm = Mathf.Clamp01(s.dl1 / v_max_eff);
                fvFactor = 1.0f - Mathf.Pow(vNorm, pHill);
            }
            else
            {
                float eccBoost = 1.0f + 0.15f * Mathf.Clamp01(-s.dl1 / 1.50f);
                fvFactor = eccBoost;
            }

            float strokeRoomGate = Mathf.Clamp01((cfg.l_max - s.l1) / 0.025f);
            float F_avail = (s.stamina > 0f) ? (F_iso * fvFactor * strokeRoomGate) : 0f;
            float u_norm = Mathf.Clamp01(input.u_ext_norm);
            float u_flex = Mathf.Clamp01(input.u_flex_norm);
            s.u_ext_actual = (u_flex > 0f) ? 0f : (u_norm * F_avail);

            float mg = cfg.M * cfg.g;
            float l_eq_safe = Mathf.Max(0.45f, cfg.l_eq);
            float c_ecc = cfg.c_ecc_factor * cfg.M * Mathf.Sqrt(cfg.g / l_eq_safe);

            float dropHold = (cfg != null) ? cfg.supportDropFromMaxMeters : SupportHoldDropMeters;
            bool hasMouseTargetL1 = input.l1_target > 0.01f;
            float l_hold = hasMouseTargetL1
                ? Mathf.Clamp(input.l1_target, cfg.l_min, cfg.l_max)
                : (cfg.l_max - dropHold);

            float phi_support = ComputeSelfWeightSupportFactor(s.l1, cfg.l_max, cfg);
            float phi_recover = ComputeSquatRecoveryFactor(s.l1, cfg.l_max, cfg);

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex);

            float squatDepth = Mathf.Max(0f, l_hold - s.l1);
            float returnSpring = Mathf.Min(cfg.k_passive * squatDepth, cfg.passiveSpringMaxG * mg);
            float stanceReturnForce = hasMouseTargetL1
                ? (returnSpring * (1.0f - u_flex))
                : (Mathf.Max(mg * phi_recover, returnSpring) * (1.0f - u_flex));

            float dampingMult;
            if (s.dl1 > 0f)
            {
                dampingMult = (u_norm > 0f) ? (1.0f - 0.70f * u_norm) : Mathf.Lerp(1.35f, 0.35f, phi_recover);
            }
            else
            {
                float supportZoneDamp = Mathf.Lerp(0.25f, 1.35f, phi_support);
                dampingMult = supportZoneDamp * (1.0f - 0.65f * u_flex);
            }

            float passiveForce = selfWeightSupport + stanceReturnForce - dampingMult * c_ecc * s.dl1;

            if (u_flex > 0f)
            {
                float activeTuckPull = u_flex * (0.95f + cfg.releaseTuckPullG) * mg;
                passiveForce -= activeTuckPull;
            }

            s.F_leg = Mathf.Max(0f, passiveForce + s.u_ext_actual);

            // =========================================================================
            // 4. 下部セグメント伸縮運動方程式 (l1, dl1)
            // =========================================================================
            float absTheta1 = Mathf.Min(Mathf.Abs(s.theta1), 82f * Mathf.Deg2Rad);
            float cosAbsTheta1 = Mathf.Max(0.15f, Mathf.Cos(absTheta1));
            float sinAbsTheta1 = Mathf.Sin(absTheta1);
            float absPsi = Mathf.Abs(s.Psi_CoM);
            float currentLatG = Mathf.Abs(s.a_c) / cfg.g;

            s.F_compress = mg * (Mathf.Cos(absPsi) + currentLatG * Mathf.Sin(absPsi));
            s.F_compress = Mathf.Max(0.1f * mg, s.F_compress);

            float ddl1 = 0f;
            float l1_prev = s.l1;

            if (u_flex > 0f)
            {
                ddl1 = (passiveForce - s.F_compress) / cfg.M;
                s.dl1 += ddl1 * dt;
                if (s.dl1 > 0f) s.dl1 = 0f;
                float l1_next = l1_prev + s.dl1 * dt;

                if (hasMouseTargetL1 && l_hold > cfg.l_min + 0.005f && l1_prev >= l_hold && l1_next <= l_hold)
                {
                    s.l1 = l_hold;
                    s.dl1 = 0f;
                }
                else
                {
                    s.l1 = l1_next;
                }
            }
            else
            {
                float effectiveLoad = (u_norm <= 0f && s.l1 <= l_hold) ? mg : s.F_compress;
                ddl1 = (s.F_leg - effectiveLoad) / cfg.M;
                s.dl1 += ddl1 * dt;

                if (u_norm <= 0f && s.l1 >= l_hold && s.dl1 > 0f)
                {
                    s.dl1 = 0f;
                }

                float l1_next = l1_prev + s.dl1 * dt;

                bool crossedDownToHold = (u_norm <= 0f && l1_prev >= l_hold && l1_next <= l_hold);
                bool crossedUpToHold   = ((u_norm <= 0f || (hasMouseTargetL1 && l_hold < cfg.l_max - 0.005f)) && l1_prev <= l_hold && l1_next >= l_hold);

                if (crossedDownToHold || crossedUpToHold)
                {
                    s.l1 = l_hold;
                    s.dl1 = 0f;
                }
                else
                {
                    s.l1 = l1_next;
                }
            }

            if (s.l1 <= cfg.l_min)
            {
                s.l1 = cfg.l_min;
                if (s.dl1 < 0f) s.dl1 = 0f;
            }
            else if (s.l1 >= cfg.l_max)
            {
                s.l1 = cfg.l_max;
                if (s.dl1 > 0f) s.dl1 = 0f;
            }

            float v_com_ext = Mathf.Max(0f, s.dl1);

            float v_z_ext = v_com_ext * cosAbsTheta1;
            float strokeRef = Mathf.Max(0.12f, dropHold);
            float G_z_kin = (v_z_ext * v_z_ext) / (2.0f * cfg.g * strokeRef);
            float G_z_inst = (v_com_ext > 0.001f) ? (Mathf.Max(0f, ddl1) * cosAbsTheta1 / cfg.g) : 0f;

            if (v_com_ext > 0.001f && u_norm > 0f)
            {
                float currentUpG = Mathf.Max(G_z_kin, G_z_inst * 0.55f);
                s.G_z_up = Mathf.Max(s.G_z_up, currentUpG);
            }
            else
            {
                s.G_z_up = Mathf.MoveTowards(s.G_z_up, 0f, 3.8f * dt);
            }

            float refInertia = Mathf.Max(1.0f, cfg.M * l_eq_safe * l_eq_safe * 0.65f);
            float inertiaFactor = Mathf.Clamp(s.I_eff / refInertia, 0.4f, 1.6f);

            if (v_com_ext > 0.001f && s.u_ext_actual > 0f)
            {
                s.P_leg_ext = s.u_ext_actual * v_com_ext * inertiaFactor;
                float dW_ext = s.P_leg_ext * dt;
                s.stamina = Mathf.Max(0f, s.stamina - dW_ext);
            }
            else
            {
                s.P_leg_ext = 0f;
                if (u_norm < 0.05f)
                {
                    s.stamina = Mathf.Min(cfg.maxStamina, s.stamina + cfg.staminaRecoveryRate * dt);
                }
            }

            s.P_step_eff = s.P_leg_ext;

            if (v_com_ext <= 0.001f)
            {
                s.u_ext_actual = 0f;
            }

            float xi_ext = 1.0f - phi_support;
            float vertSupportG = (u_norm > 0f || hasMouseTargetL1)
                ? (1.0f - xi_ext * s.G_z_up) * (1.0f - 0.85f * u_flex)
                : (phi_support - xi_ext * s.G_z_up) * (1.0f - 0.85f * u_flex);

            float F_vert_passive = mg * vertSupportG + stanceReturnForce - dampingMult * c_ecc * s.dl1 * cosAbsTheta1;
            if (u_flex > 0f)
            {
                F_vert_passive -= u_flex * (0.95f + cfg.releaseTuckPullG) * mg;
            }

            s.F_vert = F_vert_passive + s.u_ext_actual * cosAbsTheta1;

            float v_y_ext = v_com_ext * sinAbsTheta1;
            float a_y_ext = Mathf.Max(0f, ddl1) * sinAbsTheta1;
            float dy_shift_rate = Mathf.Abs(u_x - s.u_shift_filtered) / tauShift * maxShiftDesign;
            s.F_lat_step = s.u_ext_actual * sinAbsTheta1
                         + cfg.M * (a_y_ext + 2.5f * v_y_ext + 1.5f * dy_shift_rate)
                         + cfg.M * Mathf.Abs(s.a_c) * Mathf.Sin(absPsi);

            float F_vert_pos = Mathf.Max(0f, s.F_vert);
            s.F_leg = F_vert_pos * cosAbsTheta1 + s.F_lat_step * sinAbsTheta1;
            if (F_vert_pos > 0f && s.F_leg < F_vert_pos)
            {
                s.F_leg = F_vert_pos;
            }

            // =========================================================================
            // 5. 台車の水平運動 (x_c, v_c, ddx_c) と 実効遠心加速度 a_c（例外処理なし）
            // =========================================================================
            float playRad = FreePlayDeg * Mathf.Deg2Rad;
            float G_steady = G_eq;
            if (absTheta1 < playRad && Mathf.Abs(u_x) < 0.01f)
            {
                float biteRatio = absTheta1 / playRad;
                G_steady = G_eq * (biteRatio * biteRatio);
            }
            s.G_base = G_steady;

            // 脚アクチュエーターの支持力比のみから lambda_sustain を決定（例外下駄履かせをカット）
            float G_eq_mag = Mathf.Abs(G_eq);
            float F_req_ideal = mg * (Mathf.Cos(absPsi) + G_eq_mag * Mathf.Sin(absPsi));
            float F_req_eff = Mathf.Max(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float loadRatio = s.F_leg / F_req_eff;

            float maxLambda = Mathf.Lerp(1.0f, Mathf.Max(1.2f, cfg.lambda_max), Mathf.Clamp01((s.P_leg_ext + s.F_lat_step * v_y_ext) / 600.0f));
            s.lambda_sustain = Mathf.Clamp(Mathf.Pow(Mathf.Clamp(loadRatio, 0f, 1.8f), cfg.loadSustainPower_q), 0f, maxLambda);

            // 雪面挫折 (Snow Shear Failure) 判定
            float minSkidRad = Mathf.Max(3.0f, cfg.skidMinAngleDeg) * Mathf.Deg2Rad;
            float refSkidRad = Mathf.Max(cfg.skidMinAngleDeg + 5.0f, cfg.skidRefAngleDeg) * Mathf.Deg2Rad;

            float angleGate = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01((absTheta1 - minSkidRad * 0.75f) / Mathf.Max(0.02f, minSkidRad * 0.5f)));
            float sinRef = Mathf.Max(0.2f, Mathf.Sin(refSkidRad));
            float phi_angle = angleGate * Mathf.Pow(sinAbsTheta1 / sinRef, cfg.snowAngleVulnerabilityExp);

            float baseVertSupportG = 0.55f * cosAbsTheta1;
            float currentVertG = s.F_vert / mg;
            float limitVertG = Mathf.Max(0.4f, cfg.snowFailureVerticalForceG) * Mathf.Lerp(1.0f, cosAbsTheta1, 0.55f);
            float phi_vert = Mathf.Max(0f, (currentVertG - baseVertSupportG) / Mathf.Max(0.25f, limitVertG));

            float powerThreshold = Mathf.Max(150.0f, cfg.snowFailurePowerThreshold);
            float phi_power = s.P_step_eff / powerThreshold;

            s.snow_stress_ratio = (s.P_step_eff > 1.0f) ? (phi_angle * phi_vert * phi_power) : 0f;

            if (s.snow_stress_ratio >= 1.0f)
            {
                float targetSkid = Mathf.Clamp01(0.65f + 0.35f * (s.snow_stress_ratio - 1.0f));
                s.skid_intensity = Mathf.MoveTowards(s.skid_intensity, targetSkid, (dt / 0.025f));
            }
            else
            {
                float recTau = (v_com_ext <= 0.01f) ? Mathf.Min(0.06f, cfg.skidRecoveryTau) : Mathf.Max(0.05f, cfg.skidRecoveryTau);
                s.skid_intensity = Mathf.MoveTowards(s.skid_intensity, 0f, (dt / recTau));
            }

            // --- 台車の運動方程式（重心位置 y_CoM / z_CoM とアクチュエーター反力のみで駆動） ---
            float m_c = Mathf.Max(1.0f, cfg.carriageMass);
            float effectiveAngle = Mathf.Max(absTheta1, absPsi);
            const float C_rail = 75.0f;
            float c_glide = Mathf.Lerp(C_rail * 0.35f, C_rail, Mathf.Clamp01(effectiveAngle / (20f * Mathf.Deg2Rad)));

            const float midBankScale = 0.40f;
            const float deepBankBoost = 0.35f;
            const float baseGain5Deg = 1.217f;

            float sinRoll = Mathf.Sin(effectiveAngle);
            float angleShapeFactor = Mathf.Lerp(1.0f, midBankScale, Mathf.Clamp01((effectiveAngle - 5f * Mathf.Deg2Rad) / (25f * Mathf.Deg2Rad)))
                                   + deepBankBoost * (sinRoll * sinRoll);

            // 台車に対する重心の実際の傾き y_CoM / z_CoM から目標速度を決定（高バンクでの速度発散をTanhで飽和）
            float rawTanDrive = s.y_CoM / Mathf.Max(0.18f, s.z_CoM);
            float maxLatDriveG = 1.65f;
            float tanAngleDrive = maxLatDriveG * (float)Math.Tanh(rawTanDrive / maxLatDriveG);

            float angleBaseAccel = tanAngleDrive * cfg.g;
            float dirSign = (Mathf.Abs(s.y_CoM) > 0.001f) ? Mathf.Sign(s.y_CoM) : Mathf.Sign(s.theta1);

            s.v_c_target = angleBaseAccel * baseGain5Deg * angleShapeFactor;

            float carvingRetention = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, Mathf.Clamp01(s.skid_intensity));
            float F_drive = C_rail * s.v_c_target * carvingRetention;

            float R_com_sq = Mathf.Max(0.15f, s.R_CoM * s.R_CoM);
            float M_inertia = s.I_eff / R_com_sq;
            float inertiaMassRatio = M_inertia / (m_c + M_inertia);
            s.v_c_recoil_max = inertiaMassRatio * v_y_ext;

            if (s.skid_intensity > 0.15f && s.P_leg_ext > 1.0f && v_com_ext > 0.001f)
            {
                float dynamicFootForce = (s.P_leg_ext / Mathf.Max(0.3f, v_max_eff)) * sinAbsTheta1;
                s.skid_pushback_force = -dirSign * cfg.carriagePushbackGain * inertiaMassRatio * dynamicFootForce * s.skid_intensity;
            }
            else
            {
                s.skid_pushback_force = 0f;
            }

            // 台車の自由加速度 -> 速度・位置の更新 -> 物理的なレール端ストッパー拘束
            float ddx_c_free = (F_drive + s.skid_pushback_force - c_glide * s.v_c) / m_c;
            float v_c_prev = s.v_c;
            float v_c_next = v_c_prev + ddx_c_free * dt;
            float x_c_next = s.x_c + v_c_next * dt;

            float halfRail = cfg.W_rail * 0.5f;
            if (Mathf.Abs(x_c_next) >= halfRail)
            {
                float railSign = Mathf.Sign(x_c_next);
                x_c_next = railSign * halfRail;
                if (railSign * v_c_next > 0f)
                {
                    v_c_next = 0f;
                }
            }

            // 台車の実加速度 ddx_c（通常加減速およびレール端衝突時の急減速の両方を物理的に内包）
            float ddx_c = (v_c_next - v_c_prev) / dt;
            s.v_c = v_c_next;
            s.x_c = x_c_next;

            bool isAtRailLimit = Mathf.Abs(s.x_c) >= (halfRail - 1e-4f);

            // 台車が重心の要求速度に対して実際に動けている比率 eta_carriage [0..1]
            // （レール端で止まっている時は v_c = 0 となるため、例外分岐なしで自動的に eta_carriage = 0 となる）
            float v_req = Mathf.Abs(s.v_c_target);
            float v_realized = Mathf.Max(0f, Mathf.Sign(s.v_c_target) * s.v_c);
            float eta_carriage = 1.0f - Mathf.Clamp01((v_req - v_realized) / Mathf.Max(0.20f, v_req));

            // テレメトリ・クラッシュ判定用のグリップ状態更新
            float N_min = cfg.N_min_G * mg;
            float gripFactor = 1.0f;

            if (s.z_H < cfg.z_H_min)
            {
                s.gripState = SkiGripState.HipCrash;
                gripFactor = 0f;
            }
            else if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutDeg * Mathf.Deg2Rad)
            {
                s.gripState = SkiGripState.BootOut;
                gripFactor = 0f;
            }
            else if (isAtRailLimit)
            {
                s.gripState = SkiGripState.RailLimitHit;
            }
            else if (s.snow_stress_ratio >= 1.0f || s.skid_intensity > 0.15f)
            {
                s.gripState = SkiGripState.Skidding;
                gripFactor = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, Mathf.Clamp01(s.skid_intensity * 1.25f));
            }
            else if (s.F_vert < N_min && s.F_leg < N_min)
            {
                s.gripState = SkiGripState.Unweighted;
                gripFactor = 0.75f;
            }
            else
            {
                s.gripState = SkiGripState.Carving;
            }

            // 実効遠心加速度 a_c は台車の移動実現度 eta_carriage に比例して発生
            float a_c_target = (s.G_base * cfg.g) * s.lambda_sustain * gripFactor * eta_carriage;
            float fastLag = Mathf.Min(0.015f, cfg.tau_G_lag);
            s.a_c += ((a_c_target - s.a_c) / Mathf.Max(0.005f, fastLag)) * dt;

            s.kappa = s.a_c / Mathf.Max(4.0f, s.V * s.V);
            s.kappa_target = a_c_target / Mathf.Max(4.0f, s.V * s.V);

            // =========================================================================
            // 6. ブーツ支点(台車)まわりのロール運動方程式（アクチュエーター・台車・重心のみ）
            // =========================================================================
            if (input.w_reset)
            {
                s.theta1 = Mathf.MoveTowards(s.theta1, 0f, 3.0f * dt);
                s.omega1 = Mathf.MoveTowards(s.omega1, 0f, 15.0f * dt);
                s.phi = Mathf.MoveTowards(s.phi, 0f, 6.0f * dt);
                s.dphi = 0f;
                s.phi_bend = 0f;
                s.dphi_bend = 0f;
                s.tau_hip_steer = 0f;
                s.P_hip_steer = 0f;
            }
            else
            {
                // ① 重力モーメント（台車支点に対する重心の水平オフセット y_CoM のみで決まる）
                float tau_gravity = mg * s.y_CoM;

                // ② 台車の実加速度 ddx_c とカービング横加速度 a_c が重心高さ z_CoM に作る慣性モーメント
                float tau_carriage_com = -cfg.M * (0.35f * ddx_c + s.a_c) * s.z_CoM;

                // ③ アクチュエーター（脚伸展力 F_leg, u_ext_actual・股関節トルク tau_hip_steer）と台車・重心の結合モーメント
                // 3a. 脚軸(theta1)と合成重心(y_CoM, z_CoM)の幾何学的オフセット (= L2 * sin(theta2 - theta1)) に脚軸力 F_leg が働くモーメント
                float comLegOffsetArm = s.y_CoM * cos1 - s.z_CoM * sin1;
                float tau_leg_com_offset = eta_carriage * s.F_leg * comLegOffsetArm;

                // 3b. 脚伸展アクチュエーターが台車を横方向に蹴り出して重心を起こすモーメント（台車が動ける時のみ発生）
                float skidRightingPenalty = 1.0f - 0.40f * Mathf.Clamp01(s.skid_intensity);
                float forceGate = Mathf.Clamp01(s.u_ext_actual / (1.2f * mg));
                float activeExtMotionGate = Mathf.Clamp01(Mathf.Max(v_com_ext / 0.20f, forceGate * 0.85f));
                float tau_leg_carriage_push = -eta_carriage * activeExtMotionGate * skidRightingPenalty * (
                    cfg.extRightingTorqueGain * (s.u_ext_actual * sin1) * s.l1
                    + cfg.extVelocityRightingGain * mg * v_com_ext * sin1
                );

                // 3c. 股関節アクチュエーターが台車との接地を介して生むモーメント
                float tau_hip_carriage = eta_carriage * cfg.hipReactionGain * s.tau_hip_steer;

                // ④ ロール粘性減衰
                float tau_damping = -Mathf.Min(cfg.gamma_roll, 10.0f) * s.omega1;

                float domega1 = (tau_gravity + tau_carriage_com + tau_leg_com_offset
                               + tau_leg_carriage_push + tau_hip_carriage + tau_damping) / s.I_eff;
                s.omega1 += domega1 * dt;
                s.theta1 += s.omega1 * dt;
                s.theta1 = Mathf.Clamp(s.theta1, -85f * Mathf.Deg2Rad, 85f * Mathf.Deg2Rad);

                // 股関節の相対角運動方程式（dphi_in_phase による強制上書きを廃止）
                float ddphi = (s.tau_hip_steer - tau_hip_load - cfg.kd_hip_steer * s.dphi) / I2_hip;
                float maxHipVel = Mathf.Max(10.0f, cfg.dphi_max * (2.0f + 1.5f * shiftActivity));
                s.dphi = Mathf.Clamp(s.dphi + ddphi * dt, -maxHipVel, +maxHipVel);

                s.phi = Mathf.Clamp(s.phi + s.dphi * dt, minPhiLimit, maxPhiLimit);
                if ((s.phi >= maxPhiLimit && s.dphi > 0f) || (s.phi <= minPhiLimit && s.dphi < 0f))
                {
                    s.dphi = 0f;
                }

                s.P_hip_steer = Mathf.Abs(s.tau_hip_steer * s.dphi);
                s.phi_bend = s.phi - phi_ref;
                s.dphi_bend = s.dphi;
            }

            UpdateKinematics(ref s, cfg);

            float v_y_com_actual = (s.y_CoM - prev_y_CoM) / dt;
            float dirSignPost = (Mathf.Abs(s.theta1) > 0.001f) ? Mathf.Sign(s.theta1) : Mathf.Sign(s.y_CoM);
            float outwardComVel = Mathf.Max(0f, dirSignPost * v_y_com_actual);
            if (outwardComVel > 0.01f)
            {
                s.F_lat_step += cfg.M * 1.8f * outwardComVel;
            }

            float y_CoM_unshifted = L1 * Mathf.Sin(s.theta1 - delta_theta1_cmd) + L2 * sin2_ref;
            s.dy_actual = s.y_CoM - y_CoM_unshifted;

            if (s.z_H < cfg.z_H_min)
            {
                s.gripState = SkiGripState.HipCrash;
                s.a_c = 0f;
            }
            else if (Mathf.Abs(s.theta1) >= cfg.theta_bootOutDeg * Mathf.Deg2Rad)
            {
                s.gripState = SkiGripState.BootOut;
                s.a_c = 0f;
            }

            // =========================================================================
            // 7. 進行方向速度 V(t)
            // =========================================================================
            float alphaRad = cfg.alphaDeg * Mathf.Deg2Rad;
            float gravityDrive = cfg.g * Mathf.Sin(alphaRad) * 0.35f;
            float frictionMult = (s.gripState == SkiGripState.Skidding) ? 3.0f : 1.0f;
            float snowFriction = cfg.mu_snow * frictionMult * (Mathf.Max(0f, s.F_leg) / cfg.M);

            float postureRatio = Mathf.InverseLerp(cfg.l_min, cfg.l_max, s.l1);
            float dragCoeffPerMass = Mathf.Lerp(0.0022f, 0.0036f, postureRatio);
            float airDrag = dragCoeffPerMass * s.V * s.V;

            float pumpPropulsion = (0.045f * s.P_leg_ext) / (cfg.M * Mathf.Max(2.0f, s.V));

            float dV = gravityDrive - snowFriction - airDrag + pumpPropulsion;
            s.V = Mathf.Clamp(s.V + dV * dt, 4.0f, 32.0f);
        }

        public static void UpdateKinematics(ref SimState s, SkyTechSimConfig cfg)
        {
            float theta2 = s.theta1 - s.phi;
            float sin1 = Mathf.Sin(s.theta1);
            float cos1 = Mathf.Cos(s.theta1);
            float sin2 = Mathf.Sin(theta2);
            float cos2 = Mathf.Cos(theta2);

            float totalMass = Mathf.Max(1f, cfg.m1 + cfg.m2);

            s.y_CoM = (cfg.m1 * cfg.c1 * s.l1 * sin1 + cfg.m2 * (s.l1 * sin1 + cfg.l2 * sin2)) / totalMass;
            s.z_CoM = (cfg.m1 * cfg.c1 * s.l1 * cos1 + cfg.m2 * (s.l1 * cos1 + cfg.l2 * cos2)) / totalMass;

            s.Psi_CoM = Mathf.Atan2(s.y_CoM, Mathf.Max(0.01f, s.z_CoM));
            s.R_CoM = Mathf.Sqrt(s.y_CoM * s.y_CoM + s.z_CoM * s.z_CoM);
            s.z_H = s.l1 * cos1;
        }
    }
}