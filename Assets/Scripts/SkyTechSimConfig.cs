using UnityEngine;

namespace SkyTechSimulator
{
    [CreateAssetMenu(fileName = "SkyTechSimConfig", menuName = "SkyTech/SimConfig", order = 1)]
    public class SkyTechSimConfig : ScriptableObject
    {
        [Header("1. Environment & Hardware Constants")]
        public float g = 9.81f;
        public float carriageMass = 6.5f;
        public float W_rail = 17.20f;
        public float railScale = 5.20f;
        public float k_center = 0.15f;
        public float c_center = 4.0f;
        public float R_sc = 12.5f;
        public float alphaDeg = 12.0f;
        public float mu_snow = 0.04f;
        public float theta_deadDeg = 0.5f;
        public float carriageDeadzoneDeg = 0.5f;

        [Header("2. Modified Howe Flex Curve")]
        public float G0_curve = 2.60f;
        public float k_flex = 0.14f;
        public float sinPower_p = 0.82f;
        public float V_ref = 14.5f;

        [Header("3. Unstable Saddle & Active Load Maintenance")]
        public float loadSustainScale = 0.50f;
        public float loadSustainPower_q = 0.80f;
        public float lambda_max = 1.45f;
        [Range(0f, 1f)] public float beta_cuff = 0.45f;
        public float k_ang_boost = 0.25f;
        public float tau_G_lag = 0.018f;

        [Header("4. Two-CoM Body Segment Parameters")]
        public float M = 75.0f;
        public float m1 = 27.0f;
        public float m2 = 48.0f;
        public float c1 = 0.42f;
        public float l2 = 0.44f;
        public float l_min = 0.48f;
        public float l_max = 0.93f;
        public float l_full = 0.96f;
        public float l_eq = 0.84f;

        [Header("5. Biomechanical Leg Actuator")]
        public float F_ext_max_G = 3.80f;
        [Range(0.65f, 0.90f)] public float legOptRomRatio = 0.80f;
        [Range(0.05f, 0.50f)] public float squatMinForceRatio = 0.45f;
        [Range(0.15f, 0.50f)] public float squatForceDropWidth = 0.32f;
        [Range(0.50f, 2.00f)] public float kneeLeverPenaltyExp = 1.10f;
        [Range(0.15f, 0.60f)] public float lockoutMinForceRatio = 0.40f;
        [Range(0.08f, 0.25f)] public float lockoutForceDropWidth = 0.15f;
        public float v_max = 2.20f;
        [Range(0.20f, 0.70f)] public float squatMinVelRatio = 0.42f;
        [Range(0.20f, 0.60f)] public float squatVelDropWidth = 0.38f;
        [Range(0.25f, 0.75f)] public float lockoutMinVelRatio = 0.48f;
        [Range(0.08f, 0.30f)] public float lockoutVelDropWidth = 0.16f;
        [Range(1.00f, 2.00f)] public float hillPowerExponent = 1.35f;
        public float k_passive_factor = 5.2f;
        public float passiveSpringMaxG = 1.20f;
        public float c_ecc_factor = 0.95f;
        public float supportDropFromMaxMeters = 0.11f;
        public float supportFadeWidthMeters = 0.03f;
        public float maxStamina = 1800.0f;
        public float staminaRecoveryRate = 220.0f;

        [Header("6. Hip Angulation Servo & Roll Damping")]
        public float phi_out_maxDeg = 48.0f;
        public float phi_in_maxDeg = -40.0f;
        public float hipSteerMaxIsoTorque = 420.0f;
        public float hipMuscleOptLengthL0 = 1.00f;
        public float hipMuscleMinLengthLmin = 0.58f;
        [Range(0.20f, 0.80f)] public float hipMuscleGaussianWidthW = 0.55f;
        public float kd_hip_steer = 18.0f;
        public float comShiftMaxMeters = 0.28f;
        public float comShiftResponseTau = 0.045f;
        [Range(0.65f, 0.95f)] public float shiftPriorityReserveRatio = 0.82f;
        [Range(0.0f, 1.0f)] public float refVerticalKeepRatio = 0.85f;
        [Range(0.25f, 0.85f)] public float trunkInPhaseRatio = 0.45f;
        public float comShiftRollDriveGain = 1.25f;
        public float kp_hip = 520.0f;
        public float kd_hip = 58.0f;
        public float tau_hip_max = 320.0f;
        public float dphi_max = 4.5f;
        [Range(0.0f, 1.0f)] public float hipReactionGain = 0.85f;
        public float gamma_roll = 125.0f;
        public float gamma_quad = 65.0f;

        [Header("6b. Puppet Apex Control & Turn Transition")]
        public float mouseSteerFullRange = 1.05f;
        [Range(10.0f, 40.0f)] public float apexTiltMaxDeg = 26.0f;
        [Range(8.0f, 25.0f)] public float angulationTransitionDeg = 14.0f;
        [Range(0.0f, 2.0f)] public float centerReturnPushGain = 0.95f;
        [Range(0.1f, 1.5f)] public float transitionReboundGain = 0.60f;
        [Range(0.0f, 1.0f)] public float mouseForceOverrideRatio = 0.85f;
        public float mouseRollMomentGain = 1.80f;
        [Range(0.0f, 0.60f)] public float trunkLeanOutRatio = 0.30f;
        public float comLineNearDist = 0.55f;
        public float comLineFarDist = 1.45f;
        [Range(0.0f, 0.80f)] public float turnEntryLeanOutGain = 0.0f;
        public float extHeadDiveMaxDeg = 0.0f;
        public float headDiveRollDriveGain = 1.15f;

        [Header("7. Snow Shear Failure")]
        public float N_min_G = 0.22f;
        public float kappa_grip = 0.42f;
        public float skidGripMultiplier = 0.55f;
        public float skidMinAngleDeg = 30.0f;
        public float skidRefAngleDeg = 50.0f;
        [Range(0.5f, 2.5f)] public float snowAngleVulnerabilityExp = 1.35f;
        public float snowFailureVerticalForceG = 2.30f;
        public float snowFailurePowerThreshold = 1500.0f;
        public float carriagePushbackGain = 0.30f;
        public float skidRecoveryTau = 0.12f;
        public float z_H_min = 0.12f;
        public float theta_bootOutDeg = 78.0f;

        [Header("8. Extension Righting & Tuck Centering Return")]
        public float extRightingTorqueGain = 0.45f;
        public float extVelocityRightingGain = 0.85f;
        public float releaseTuckPullG = 0.55f;
        public float tuckCenterSpringK = 320.0f;
        public float tuckVelocityReturnBoost = 1.80f;
        public float tuckCenterDampingRatio = 0.85f;
        public float tuckRollCenteringGain = 6.0f;

        [Header("9. Shallow-Angle Capture-Point & Camber Physics")]
        [Range(0.0f, 2.5f)] public float capturePointLeadGain = 0.60f;
        [Range(0.35f, 1.0f)] public float shallowGlideStiffnessMinRatio = 0.80f;
        [Range(0.0f, 1.0f)] public float directCarveRightingFloor = 0.75f;

        [Header("10. Neuromuscular Co-contraction & Adaptive Damping")]
        [Range(0.2f, 5.0f)] public float adaptiveDampingBoost = 1.8f;
        [Range(0.02f, 1.0f)] public float coContractionVelThreshold = 0.15f;

        // 派生物理量・ラジアン変換プロパティ
        public float k_passive => (l_eq > 0.001f) ? (k_passive_factor * M * g / l_eq) : 0f;
        public float c_ecc => (l_eq > 0.001f) ? (c_ecc_factor * M * Mathf.Sqrt(g / l_eq)) : 0f;
        public float F_ext_max => F_ext_max_G * M * g;
        public float N_min => N_min_G * M * g;
        public float alphaRad => alphaDeg * Mathf.Deg2Rad;
        public float theta_deadRad => theta_deadDeg * Mathf.Deg2Rad;
        public float carriageDeadzoneRad => carriageDeadzoneDeg * Mathf.Deg2Rad;
        public float phi_out_maxRad => phi_out_maxDeg * Mathf.Deg2Rad;
        public float phi_in_maxRad => phi_in_maxDeg * Mathf.Deg2Rad;
        public float theta_bootOutRad => theta_bootOutDeg * Mathf.Deg2Rad;
        public float W_rail_half => W_rail * 0.5f;
        public float CoMDistanceDefault => (1f - c1) * l_eq + l2;
    }
}