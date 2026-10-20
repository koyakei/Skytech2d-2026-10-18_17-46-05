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
        /// <summary>(N) 鉛直方向の押す力 F_leg * cos(theta1)</summary>
        public float F_vert;
        /// <summary>(W) 雪面挫折判定用の実効踏み込みパワー（重心が動いているときのみ発生）</summary>
        public float P_step_eff;
        /// <summary>[0..2+] 雪面挫折限界に対する現在の負荷率（1.0以上でスキッディング）</summary>
        public float snow_stress_ratio;
        /// <summary>[0..1] スキッディング（雪面挫折）の持続強度</summary>
        public float skid_intensity;
        /// <summary>(m/s) 角度依存の台車目標移動速度</summary>
        public float v_c_target;
        /// <summary>(m/s) 重心の慣性モーメント反作用による台車の反対側への最大許容速度</summary>
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

            float z_upright_approx = Mathf.Max(0.20f, L1 * cos1 + L2);
            float G0_approx = (L1 * sin1) / z_upright_approx;
            float theta_res0 = Mathf.Atan(G0_approx);
            float g_eff0 = cfg.g * Mathf.Sqrt(1.0f + G0_approx * G0_approx);

            float etaShiftReserve = Mathf.Clamp(cfg.shiftPriorityReserveRatio, 0.65f, 0.95f);
            float etaVertBudget = 1.0f - etaShiftReserve;
            float alphaVert = Mathf.Clamp01(cfg.refVerticalKeepRatio);

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
            // 2. 同位相重心移動モデル
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

            float sinDeltaTorque = Mathf.Clamp(tau_hip_avail / Mathf.Max(1.0f, m2_l2 * g_eff), 0f, 1.0f);
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

            float maxShiftDesign = Mathf.Max(0.05f, cfg.comShiftMaxMeters);
            float dy_desired = s.u_shift_filtered * maxShiftDesign;
            s.dy_cmd = Mathf.Clamp(dy_desired, s.dy_min_torque, s.dy_max_torque);

            float delta_s1_cmd = s.dy_cmd / L_eff;
            float s1_target = Mathf.Clamp(sin1 + delta_s1_cmd, -0.99f, 0.99f);
            float theta1_shift_target = Mathf.Asin(s1_target);
            float delta_theta1_cmd = theta1_shift_target - s.theta1;

            float phi_base_ang = angulationRatio * s.theta1;
            float delta_phi_cmd = angulationRatio * delta_theta1_cmd;
            float phi_target = Mathf.Clamp(phi_base_ang + delta_phi_cmd, -phiMax, +phiMax);
            float theta2_target = s.theta1 - phi_target;

            float shiftActivity = Mathf.Clamp01(Mathf.Abs(u_x) + Mathf.Abs(s.u_shift_filtered));
            float theta2_cur = s.theta1 - s.phi;

            float tau_hip_load = m2_l2 * (cfg.g * Mathf.Sin(theta2_cur) - s.a_c * Mathf.Cos(theta2_cur));
            float tau_hip_ff   = m2_l2 * (cfg.g * Mathf.Sin(theta2_target) - s.a_c * Mathf.Cos(theta2_target));

            float kp_base = Mathf.Max(600.0f, cfg.kp_hip * 1.4f);
            float kp_shift_boost = kp_base * (1.0f + 0.85f * shiftActivity);
            float kd_ctrl = Mathf.Max(45.0f, cfg.kd_hip * 1.3f);

            float tau_hip_pd = kp_shift_boost * (phi_target - s.phi) - kd_ctrl * s.dphi;
            s.tau_hip_steer = Mathf.Clamp(tau_hip_ff + tau_hip_pd, -tau_hip_avail, +tau_hip_avail);

            // =========================================================================
            // 3. 下肢アクチュエーター（伸展: Space/↑, 抱え込み: ↓/S）
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
            float l_hold = cfg.l_max - dropHold;

            float phi_support = ComputeSelfWeightSupportFactor(s.l1, cfg.l_max, cfg);
            float phi_recover = ComputeSquatRecoveryFactor(s.l1, cfg.l_max, cfg);

            float selfWeightSupport = mg * phi_support * (1.0f - 0.85f * u_flex);

            float squatDepth = Mathf.Max(0f, l_hold - s.l1);
            float returnSpring = Mathf.Min(cfg.k_passive * squatDepth, cfg.passiveSpringMaxG * mg);
            float stanceReturnForce = Mathf.Max(mg * phi_recover, returnSpring) * (1.0f - u_flex);

            float dampingMult;
            if (s.dl1 > 0f)
            {
                if (u_norm > 0f)
                {
                    dampingMult = 1.0f - 0.70f * u_norm;
                }
                else
                {
                    dampingMult = Mathf.Lerp(1.35f, 0.35f, phi_recover);
                }
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
            // 4. 下部セグメント伸縮運動方程式 (l1, dl1) と 重心運動パワー P_leg_ext
            // =========================================================================
            float absTheta1 = Mathf.Min(Mathf.Abs(s.theta1), 82f * Mathf.Deg2Rad);
            float absPsi = Mathf.Abs(s.Psi_CoM);
            float currentLatG = Mathf.Abs(s.a_c) / cfg.g;

            s.F_compress = mg * (Mathf.Cos(absPsi) + currentLatG * Mathf.Sin(absPsi));
            s.F_compress = Mathf.Max(0.1f * mg, s.F_compress);

            if (u_flex > 0f)
            {
                float ddl1_tuck = (passiveForce - s.F_compress) / cfg.M;
                s.dl1 += ddl1_tuck * dt;
                if (s.dl1 > 0f) s.dl1 = 0f;
                s.l1 += s.dl1 * dt;
            }
            else
            {
                float l1_prev = s.l1;

                float effectiveLoad = (u_norm <= 0f && s.l1 <= l_hold) ? mg : s.F_compress;
                float ddl1 = (s.F_leg - effectiveLoad) / cfg.M;
                s.dl1 += ddl1 * dt;

                if (u_norm <= 0f && s.l1 >= l_hold && s.dl1 > 0f)
                {
                    s.dl1 = 0f;
                }

                float l1_next = l1_prev + s.dl1 * dt;

                bool crossedDownToHold = (u_norm <= 0f && l1_prev >= l_hold && l1_next <= l_hold);
                bool crossedUpToHold   = (u_norm <= 0f && l1_prev <= l_hold && l1_next >= l_hold);

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
                s.F_leg = Mathf.Max(0f, passiveForce);
            }

            // =========================================================================
            // 5. 雪面挫折 (Snow Shear Failure) 判定・キャリッジ車止め衝突・実効遠心加速度 a_c
            // =========================================================================
            float playRad = FreePlayDeg * Mathf.Deg2Rad;
            float G_steady = G_eq;
            if (absTheta1 < playRad && Mathf.Abs(u_x) < 0.01f)
            {
                float biteRatio = absTheta1 / playRad;
                G_steady = G_eq * (biteRatio * biteRatio);
            }
            s.G_base = G_steady;

            float G_eq_mag = Mathf.Abs(G_eq);
            float F_req_ideal = mg * (Mathf.Cos(absPsi) + G_eq_mag * Mathf.Sin(absPsi));
            float F_req_eff = Mathf.Max(0.35f * mg, cfg.loadSustainScale * F_req_ideal);
            float loadRatio = s.F_leg / F_req_eff;

            float maxLambda = Mathf.Lerp(1.0f, Mathf.Max(1.2f, cfg.lambda_max), Mathf.Clamp01(s.P_leg_ext / 600.0f));
            s.lambda_sustain = Mathf.Clamp(Mathf.Pow(Mathf.Clamp(loadRatio, 0f, 1.8f), cfg.loadSustainPower_q), 0f, maxLambda);

            bool isCoMOffCenter = (absPsi > 2.5f * Mathf.Deg2Rad) || (absTheta1 > 2.5f * Mathf.Deg2Rad);
            bool isRetractingLeg = (u_flex > 0.01f) || (s.dl1 < -0.02f);
            if (isCoMOffCenter && isRetractingLeg)
            {
                s.lambda_sustain = Mathf.Max(s.lambda_sustain, 1.0f);
            }
            else if (absTheta1 < 25f * Mathf.Deg2Rad && s.P_leg_ext > 10.0f)
            {
                s.lambda_sustain = Mathf.Max(1.0f, s.lambda_sustain);
            }

            float cosAbsTheta1 = Mathf.Max(0.15f, Mathf.Cos(absTheta1));
            float sinAbsTheta1 = Mathf.Sin(absTheta1);
            s.F_vert = s.F_leg * cosAbsTheta1;

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

            // --- キャリッジ水平軌道 (x_c, v_c) と車止め急停止による慣性撃力の算出 ---
            float m_c = Mathf.Max(1.0f, cfg.carriageMass);
            float rollAngle = Mathf.Abs(s.theta1);
            float comAngle = Mathf.Abs(s.Psi_CoM);
            float effectiveAngle = Mathf.Max(rollAngle, comAngle);
            float deadzoneRad = 0.2f * Mathf.Deg2Rad;
            const float C_rail = 75.0f;

            float uprightGate = Mathf.Exp(-Mathf.Pow(effectiveAngle / (3.5f * Mathf.Deg2Rad), 2f));
            float tuckSpeed = Mathf.Max(0f, -s.dl1) + 0.8f * u_flex;
            float k_return = u_flex * uprightGate * cfg.tuckCenterSpringK * (1.0f + cfg.tuckVelocityReturnBoost * tuckSpeed);
            float F_center_return = -k_return * s.x_c;

            float c_critical = 2.0f * Mathf.Sqrt(m_c * Mathf.Max(1.0f, k_return)) * cfg.tuckCenterDampingRatio;
            float c_effective = Mathf.Lerp(C_rail, c_critical, u_flex * uprightGate);

            float halfRail = cfg.W_rail * 0.5f;
            bool wasAtRailLimit = Mathf.Abs(s.x_c) >= (halfRail - 1e-4f);
            float ddx_c_impact = 0f;

            if (effectiveAngle <= deadzoneRad && Mathf.Abs(u_x) < 0.01f && Mathf.Abs(s.x_c) < 0.02f)
            {
                s.v_c_target = 0f;
                s.v_c_recoil_max = 0f;
                s.skid_pushback_force = 0f;
                float ddx_c_stop = -(C_rail / m_c) * s.v_c;
                s.v_c += ddx_c_stop * dt;
                s.x_c += s.v_c * dt;
                if (Mathf.Abs(s.v_c) < 0.01f) s.v_c = 0f;
            }
            else
            {
                const float midBankScale = 0.35f;
                const float deepBankBoost = 2.20f;
                const float baseGain5Deg = 1.217f;

                float sinRoll = Mathf.Sin(effectiveAngle);
                float angleShapeFactor = Mathf.Lerp(1.0f, midBankScale, Mathf.Clamp01((effectiveAngle - 5f * Mathf.Deg2Rad) / (25f * Mathf.Deg2Rad)))
                                       + deepBankBoost * (sinRoll * sinRoll * sinRoll);

                float tanAngleDrive = (Mathf.Abs(s.theta1) >= Mathf.Abs(s.Psi_CoM))
                    ? G_steady
                    : (s.y_CoM / Mathf.Max(0.18f, s.z_CoM));

                float angleBaseAccel = tanAngleDrive * cfg.g;
                s.v_c_target = angleBaseAccel * baseGain5Deg * angleShapeFactor;

                float carvingRetention = (s.gripState == SkiGripState.HipCrash || s.gripState == SkiGripState.BootOut)
                    ? 0.0f
                    : Mathf.Lerp(1.0f, cfg.skidGripMultiplier, Mathf.Clamp01(s.skid_intensity));

                // 車止めに引っかかって外向きに押されている状態ではカービング駆動力は発生しない
                if (wasAtRailLimit && (s.x_c * s.v_c_target > 0f))
                {
                    carvingRetention = 0f;
                }

                float F_drive = C_rail * s.v_c_target * carvingRetention;

                float R_com_sq = Mathf.Max(0.15f, s.R_CoM * s.R_CoM);
                float M_inertia = s.I_eff / R_com_sq;
                float inertiaMassRatio = M_inertia / (m_c + M_inertia);

                float comHorizExtVel = v_com_ext * Mathf.Sin(absTheta1);
                s.v_c_recoil_max = inertiaMassRatio * comHorizExtVel;

                float dirSign = (Mathf.Abs(s.theta1) > 0.001f) ? Mathf.Sign(s.theta1) : Mathf.Sign(s.y_CoM);

                if (s.gripState == SkiGripState.Skidding && s.P_leg_ext > 1.0f && v_com_ext > 0.001f)
                {
                    float dynamicFootForce = (s.P_leg_ext / Mathf.Max(0.3f, v_max_eff)) * Mathf.Sin(absTheta1);
                    s.skid_pushback_force = -dirSign * cfg.carriagePushbackGain * inertiaMassRatio * dynamicFootForce * s.skid_intensity;
                }
                else
                {
                    s.skid_pushback_force = 0f;
                }

                float ddx_c = (F_drive + s.skid_pushback_force + F_center_return - c_effective * s.v_c) / m_c;

                if (wasAtRailLimit)
                {
                    float railSign = Mathf.Sign(s.x_c);
                    if (railSign * ddx_c >= 0f)
                    {
                        // 車止めに押し付けられている間は外向きへの移動を完全に拘束
                        ddx_c = 0f;
                        s.v_c = 0f;
                        s.x_c = railSign * halfRail;
                    }
                    else
                    {
                        s.v_c += ddx_c * dt;
                        s.x_c += s.v_c * dt;
                    }
                }
                else
                {
                    s.v_c += ddx_c * dt;

                    if (Mathf.Abs(dirSign) > 0.001f)
                    {
                        float v_opp = -dirSign * s.v_c;
                        if (v_opp > s.v_c_recoil_max)
                        {
                            s.v_c = -dirSign * s.v_c_recoil_max;
                        }
                    }

                    s.x_c += s.v_c * dt;
                }
            }

            // 車止め衝突時の急停止処理と慣性減速度 ddx_c_impact の算出
            if (Mathf.Abs(s.x_c) >= halfRail)
            {
                float railSign = Mathf.Sign(s.x_c);
                s.x_c = railSign * halfRail;
                if (railSign * s.v_c > 0f)
                {
                    float v_c_before = s.v_c;
                    s.v_c = 0f;
                    float delta_v_c = s.v_c - v_c_before; // -v_c_before
                    ddx_c_impact = delta_v_c / dt;
                }
            }

            bool isAtRailLimit = Mathf.Abs(s.x_c) >= (halfRail - 1e-4f);

            // --- グリップ状態判定と仮想遠心加速度 a_c ---
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
                gripFactor = 0f;
            }
            else if (s.snow_stress_ratio >= 1.0f || s.skid_intensity > 0.15f)
            {
                s.gripState = SkiGripState.Skidding;
                gripFactor = Mathf.Lerp(1.0f, cfg.skidGripMultiplier, Mathf.Clamp01(s.skid_intensity * 1.25f));
            }
            else if (s.F_leg < N_min && !isRetractingLeg)
            {
                s.gripState = SkiGripState.Unweighted;
                gripFactor = 0.75f;
            }
            else
            {
                s.gripState = SkiGripState.Carving;
                gripFactor = 1.0f;
            }

            if (s.gripState == SkiGripState.RailLimitHit ||
                s.gripState == SkiGripState.BootOut ||
                s.gripState == SkiGripState.HipCrash)
            {
                // 車止め停止時およびクラッシュ時は仮想遠心力を即座に 0 にする
                s.a_c = 0f;
                s.kappa = 0f;
                s.kappa_target = 0f;
            }
            else
            {
                float a_c_target = (s.G_base * cfg.g) * s.lambda_sustain * gripFactor;
                float fastLag = Mathf.Min(0.015f, cfg.tau_G_lag);
                s.a_c += ((a_c_target - s.a_c) / Mathf.Max(0.005f, fastLag)) * dt;

                s.kappa = s.a_c / Mathf.Max(4.0f, s.V * s.V);
                s.kappa_target = a_c_target / Mathf.Max(4.0f, s.V * s.V);
            }

            // =========================================================================
            // 6. ブーツ支点まわりのロール運動方程式 (theta1, omega1)
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
                float tau_gravity = mg * s.y_CoM;
                float tau_centrifugal = -cfg.M * s.a_c * s.z_CoM_ref;

                // 車止め衝突で台車が急停止した慣性力 (-M * ddx_c_impact * z_CoM) によるつんのめり撃力トルク
                // Δω_1 = -(M * z_CoM / I_eff) * Δv_c
                float tau_rail_impact = -cfg.M * ddx_c_impact * s.z_CoM;

                float tau_shift_drive = cfg.comShiftRollDriveGain * mg * s.dy_cmd;
                float tau_hip_reaction = cfg.hipReactionGain * s.tau_hip_steer;

                float skidRightingPenalty = 1.0f - 0.85f * Mathf.Clamp01(s.skid_intensity);
                float activeExtMotionGate = Mathf.Clamp01(v_com_ext / 0.25f);
                float tau_ext_righting = (s.gripState == SkiGripState.RailLimitHit) ? 0f : -u_norm * activeExtMotionGate * skidRightingPenalty * (
                    cfg.extRightingTorqueGain * s.u_ext_actual * s.l1 * Mathf.Sin(s.theta1)
                    + cfg.extVelocityRightingGain * mg * v_com_ext * Mathf.Sin(s.theta1)
                );

                float uprightFactorForTuck = Mathf.Exp(-Mathf.Pow(absPsi / (4.0f * Mathf.Deg2Rad), 2f));
                float tuckVel = Mathf.Max(0f, -s.dl1);
                float tau_tuck_centering = -u_flex * uprightFactorForTuck * cfg.tuckRollCenteringGain * cfg.M * (0.35f + tuckVel) * s.theta1;

                // 角度固定サーボ：エッジ支持を失った状態や、設定角度より深く外側へ倒れ込む動きに対しては働かないようにする
                float tau_lock_servo = 0f;
                if (s.gripState == SkiGripState.Carving && Mathf.Abs(input.phi_lock_cmd) > 0.001f && shiftActivity < 0.05f)
                {
                    float lockError = input.phi_lock_cmd - s.theta1;
                    bool isPullingBackFromDeeperLean = (s.theta1 * lockError) < 0f;
                    if (!isPullingBackFromDeeperLean)
                    {
                        float lockHoldWeight = 1.0f - 0.90f * Mathf.Max(u_norm * activeExtMotionGate * skidRightingPenalty, u_flex * uprightFactorForTuck);
                        tau_lock_servo = 450.0f * lockHoldWeight * lockError;
                    }
                }

                // ロール減衰：二乗ダンピングを廃止し、支持喪失・倒れ込み時は減衰を最小限にして重力で自然に角加速させる
                bool isRighting = (s.theta1 * s.omega1) < 0f;
                float dampingLinear;
                if (s.gripState == SkiGripState.RailLimitHit ||
                    s.gripState == SkiGripState.BootOut ||
                    s.gripState == SkiGripState.HipCrash ||
                    s.gripState == SkiGripState.Unweighted)
                {
                    dampingLinear = 1.5f;
                }
                else if (!isRighting)
                {
                    dampingLinear = Mathf.Min(cfg.gamma_roll, 6.0f);
                }
                else
                {
                    float rightingDampScale = 1.0f - 0.55f * Mathf.Max(u_norm * activeExtMotionGate, u_flex);
                    dampingLinear = Mathf.Min(cfg.gamma_roll, 12.0f) * rightingDampScale;
                }
                float tau_damping = -dampingLinear * s.omega1;

                float domega1 = (tau_gravity + tau_centrifugal + tau_rail_impact + tau_shift_drive + tau_hip_reaction
                               + tau_ext_righting + tau_tuck_centering + tau_lock_servo + tau_damping) / s.I_eff;
                s.omega1 += domega1 * dt;
                s.theta1 += s.omega1 * dt;
                s.theta1 = Mathf.Clamp(s.theta1, -85f * Mathf.Deg2Rad, 85f * Mathf.Deg2Rad);

                float ddphi = (s.tau_hip_steer - tau_hip_load - cfg.kd_hip_steer * s.dphi) / I2_hip;
                float maxHipVel = Mathf.Max(8.0f, cfg.dphi_max * (1.8f + 1.0f * shiftActivity));
                float dphi_raw = Mathf.Clamp(s.dphi + ddphi * dt, -maxHipVel, +maxHipVel);

                float dphi_in_phase = angulationRatio * s.omega1;
                if (shiftActivity > 0.01f)
                {
                    if (s.u_shift_filtered < 0f && s.omega1 < 0f)
                    {
                        dphi_raw = Mathf.Max(dphi_raw, dphi_in_phase);
                    }
                    else if (s.u_shift_filtered > 0f && s.omega1 > 0f)
                    {
                        dphi_raw = Mathf.Min(dphi_raw, dphi_in_phase);
                    }
                    s.dphi = Mathf.Lerp(dphi_raw, dphi_in_phase, 0.65f * shiftActivity);
                }
                else
                {
                    s.dphi = dphi_raw;
                }

                s.phi = Mathf.Clamp(s.phi + s.dphi * dt, -phiMax, +phiMax);
                if ((s.phi >= phiMax && s.dphi > 0f) || (s.phi <= -phiMax && s.dphi < 0f))
                {
                    s.dphi = 0f;
                }

                s.P_hip_steer = Mathf.Abs(s.tau_hip_steer * s.dphi);
                s.phi_bend = s.phi - phi_ref;
                s.dphi_bend = s.dphi;
            }

            UpdateKinematics(ref s, cfg);
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
            float snowFriction = cfg.mu_snow * frictionMult * (s.F_leg / cfg.M);

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