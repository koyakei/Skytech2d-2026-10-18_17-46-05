using UnityEngine;

namespace SkyTechSimulator
{
    /// <summary>
    /// SkyTechSportスキーシミュレーター（2D前額面モデル）の物理・ハードウェアパラメータ設定クラス。
    /// ScriptableObjectとして管理され、インスペクタからのリアルタイム調整に対応します。
    /// </summary>
    [CreateAssetMenu(fileName = "SkyTechSimConfig", menuName = "SkyTech/SimConfig", order = 1)]
    public class SkyTechSimConfig : ScriptableObject
    {
        [Header("1. Environment & Hardware Constants")]
        [Tooltip("重力加速度 (m/s^2)")]
        public float g = 9.81f;

        [Tooltip("キャリッジ（台車）自体の質量 m_c (kg)")]
        public float carriageMass = 5.0f;

        [Tooltip("SkyTechSportキャリッジ有効レール幅 (m) [-W_rail/2, +W_rail/2: 7.20m]")]
        public float W_rail = 7.20f;

        [Tooltip("仮想横加速度 a_c からキャリッジ実移動速度へのスケーリング係数 (ロール角に対する台車目標速度の全体倍率)")]
        public float railScale = 6.50f;

        [Tooltip("キャリッジの中央復元バネ定数 (1/s^2: 0.0で中央引き戻しをなくし一定速度で横移動)")]
        public float k_center = 0.0f;

        [Tooltip("キャリッジの速度応答減衰係数 (1/s)")]
        public float c_center = 4.0f;

        [Tooltip("仮想スキーの基準サイドカット半径 (m)")]
        public float R_sc = 11.5f;

        [Tooltip("仮想斜面の平均勾配 (deg)")]
        public float alphaDeg = 10.0f;

        [Tooltip("雪面とスキー板の動摩擦係数")]
        public float mu_snow = 0.04f;

        [Tooltip("フラット付近(theta1 ≈ 0)の符号反転緩和デッドゾーン角 (deg)")]
        public float theta_deadDeg = 0.5f;

        [Tooltip("台車の左右移動を開始する最小ロール角デッドゾーン (deg)")]
        public float carriageDeadzoneDeg = 0.5f;

        [Header("2. Modified Howe Flex Curve (30 deg = 2G, 70 deg = 5G)")]
        [Tooltip("基準横G係数 G0")]
        public float G0_curve = 3.20f;

        [Tooltip("深バンクたわみ曲率係数 k_flex")]
        public float k_flex = 0.15f;

        [Tooltip("低バンク領域の立ち上がり指数 p")]
        public float sinPower_p = 0.80f;

        [Tooltip("2G/5G出力の基準となる滑走速度 V_ref (m/s)")]
        public float V_ref = 15.0f;

        [Header("3. Unstable Saddle & Active Load Maintenance (Surf/Bike Feel)")]
        [Tooltip("要求軸方向荷重に対する実効ゲート係数")]
        public float loadSustainScale = 0.48f;

        [Tooltip("踏み込み不足時に遠心力が抜ける非線形べき乗指数 q")]
        public float loadSustainPower_q = 0.85f;

        [Tooltip("強い踏み込み時の最大たわみオーバードライブ上限")]
        public float lambda_max = 1.55f;

        [Tooltip("ブーツカフ剛性による高バンク時のトルク維持率 [0..1] (70度でのcos目減り防止)")]
        [Range(0f, 1f)]
        public float beta_cuff = 0.45f;

        [Tooltip("外向傾(くの字姿勢 phi > 0)による有効モーメントアーム増幅係数")]
        public float k_ang_boost = 0.25f;

        [Tooltip("エッジ噛み込みから遠心力立ち上がりまでの一次遅れ時定数 (s)")]
        public float tau_G_lag = 0.015f;

        [Header("4. Two-CoM Body Segment Parameters")]
        [Tooltip("スキーヤー総質量 M = m1 + m2 (kg)")]
        public float M = 75.0f;

        [Tooltip("下部セグメント質量 m1: 下肢・骨盤系 (kg)")]
        public float m1 = 25.0f;

        [Tooltip("上部セグメント質量 m2: 体幹・頭・両腕 HAT系 (kg)")]
        public float m2 = 50.0f;

        [Tooltip("ブーツ支点から下部重心までの距離比 c1 (0.38)")]
        public float c1 = 0.38f;

        [Tooltip("股関節から上部重心までの固定長 l2 (m: 側屈による重心移動モーメントアーム)")]
        public float l2 = 0.65f;

        [Tooltip("下部セグメント最小屈曲長 l_min (m: 深屈曲フルスクワット位置 = 0% ROM)")]
        public float l_min = 0.40f;

        [Tooltip("下部セグメント最大伸展長 l_max (m: 最大伸展ストローク終端 = 100% ROM)")]
        public float l_max = 0.98f;

        [Tooltip("2リンク下肢モデルの解剖学的完全伸展長 l_full (m: 大腿0.50m + 下腿0.50m = 1.00m)")]
        public float l_full = 1.00f;

        [Tooltip("受動姿勢維持バネの自然平衡長 l_eq (m: 0.864m = 80% ROM パワーポジション)")]
        public float l_eq = 0.864f;

        [Header("5. Athlete Jump / Deadlift vs Squat Biomechanical Leg Actuator")]
        [Tooltip("可動域80%(パワーポジション/膝伸展位デッドリフト)における最大等尺性伸展筋力 (体重比 G単位)")]
        public float F_ext_max_G = 7.20f;

        [Tooltip("最大筋力・最大パワーを発揮する最適正規化可動域 s_opt [0..1]")]
        [Range(0.65f, 0.90f)]
        public float legOptRomRatio = 0.80f;

        [Tooltip("深屈曲フルスクワット(0% ROM)での最小等尺性筋力維持比")]
        [Range(0.05f, 0.40f)]
        public float squatMinForceRatio = 0.42f;

        [Tooltip("80% ROMからしゃがみ込んだ際の筋力減衰ガウス幅係数 sigma_squat")]
        [Range(0.15f, 0.50f)]
        public float squatForceDropWidth = 0.30f;

        [Tooltip("2リンク膝関節モーメントアーム悪化による軸力減衰べき乗指数 p_lever")]
        [Range(0.50f, 2.00f)]
        public float kneeLeverPenaltyExp = 1.15f;

        [Tooltip("完全伸展終端(100% ROM)での等尺性筋力維持比")]
        [Range(0.15f, 0.60f)]
        public float lockoutMinForceRatio = 0.35f;

        [Tooltip("80% ROMから100%完全伸展へ向かう際の筋力減衰ガウス幅係数 sigma_lock")]
        [Range(0.08f, 0.25f)]
        public float lockoutForceDropWidth = 0.14f;

        [Tooltip("80% ROMパワーポジションにおける最大無負荷伸展速度 v_max (m/s)")]
        public float v_max = 3.60f;

        [Tooltip("深屈曲フルスクワット(0% ROM)における最大伸展速度の低下比")]
        [Range(0.20f, 0.70f)]
        public float squatMinVelRatio = 0.38f;

        [Tooltip("80% ROMからしゃがみ込んだ際の最大伸展速度ガウス幅係数")]
        [Range(0.20f, 0.60f)]
        public float squatVelDropWidth = 0.38f;

        [Tooltip("完全伸展終端(100% ROM)における最大伸展速度の低下比")]
        [Range(0.25f, 0.75f)]
        public float lockoutMinVelRatio = 0.45f;

        [Tooltip("80% ROMから100%完全伸展へ向かう際の最大伸展速度ガウス幅係数")]
        [Range(0.08f, 0.30f)]
        public float lockoutVelDropWidth = 0.16f;

        [Tooltip("パワー保持型Hill力-速度関係の速度べき乗指数")]
        [Range(1.00f, 2.00f)]
        public float hillPowerExponent = 1.35f;

        [Tooltip("下肢の片方向受動姿勢維持バネ無次元化係数")]
        public float k_passive_factor = 6.5f;

        [Tooltip("深屈曲時に受動バネが過大な反発力を出さないための軟飽和上限 (体重比 G単位)")]
        public float passiveSpringMaxG = 1.25f;

        [Tooltip("伸張性収縮(Eccentric)ダンピング無次元化係数")]
        public float c_ecc_factor = 0.89f;

        [Tooltip("スペースキーを離したときに戻る、最高位(l_max)からの引き下げ距離 (m)")]
        public float supportDropFromMaxMeters = 0.15f;

        [Tooltip("自重を支える力が徐々にゼロになる遷移幅 (m)")]
        public float supportFadeWidthMeters = 0.03f;

        [Tooltip("無酸素パワーゲージ(ATP-CP系スタミナ)最大容量 (J)")]
        public float maxStamina = 2200.0f;

        [Tooltip("非伸展時のスタミナ自然回復レート (J/s)")]
        public float staminaRecoveryRate = 260.0f;

        [Header("6. Hip Angulation Servo & Roll Damping")]
        [Tooltip("外向傾(ターン外側へ上体を起こすくの字姿勢)の最大角度 (deg)")]
        public float phi_out_maxDeg = 55.0f;

        [Tooltip("内倒(ターン内側へ上体が倒れる姿勢)の最大角度 (deg)")]
        public float phi_in_maxDeg = -45.0f;

        [Tooltip("最適長 L_0 において発揮される最大等尺性側屈トルク (N*m)")]
        public float hipSteerMaxIsoTorque = 950.0f;

        [Tooltip("筋肉が最も力を発揮できる最適長 L_0")]
        public float hipMuscleOptLengthL0 = 1.00f;

        [Tooltip("同方向の最大曲げ角(+phi_max)まで収縮したときの最小筋肉長 L_min")]
        public float hipMuscleMinLengthLmin = 0.55f;

        [Tooltip("ガウス関数モデルの可動域形状幅係数 W")]
        [Range(0.20f, 0.80f)]
        public float hipMuscleGaussianWidthW = 0.55f;

        [Tooltip("体幹側屈の関節粘性減衰係数 (N*m*s/rad)")]
        public float kd_hip_steer = 28.0f;

        [Tooltip("矢印キー/マウスによる目標重心シフト量の設計上限 Δy_design (m)")]
        public float comShiftMaxMeters = 0.30f;

        [Tooltip("重心シフト入力の1次遅れ時定数 τ_shift (s)")]
        public float comShiftResponseTau = 0.055f;

        [Tooltip("股関節の最大トルク・可動域のうち『重心移動』に最優先で確保する割合 η_shift [0.65..0.95]")]
        [Range(0.65f, 0.95f)]
        public float shiftPriorityReserveRatio = 0.82f;

        [Tooltip("キー無入力時の余力範囲内で、合成力に対してリーンアウト(垂直方向へ補正)する係数 α_vert [0..1]")]
        [Range(0.0f, 1.0f)]
        public float refVerticalKeepRatio = 0.85f;

        [Tooltip("重心移動時の下部リンク傾きに対する体幹の同位相傾き配分比 κ_trunk [0.25..0.85]")]
        [Range(0.25f, 0.85f)]
        public float trunkInPhaseRatio = 0.35f;

        [Tooltip("重心シフト指令による下部リンク(腰関節移動)の直接ロール駆動ゲイン")]
        public float comShiftRollDriveGain = 1.45f;

        [Tooltip("股関節PDサーボ比例ゲイン (N*m/rad)")]
        public float kp_hip = 850.0f;

        [Tooltip("股関節PDサーボ微分ゲイン (N*m*s/rad)")]
        public float kd_hip = 90.0f;

        [Tooltip("股関節の最大発揮トルク上限 (N*m)")]
        public float tau_hip_max = 350.0f;

        [Tooltip("股関節の相対角速度上限 (rad/s)")]
        public float dphi_max = 5.0f;

        [Tooltip("股関節角加速度による下部リンクへの反作用トルク結合ゲイン")]
        [Range(0.0f, 1.0f)]
        public float hipReactionGain = 0.30f;

        [Tooltip("ロール回転の線形粘性減衰係数 (N*m*s/rad)")]
        public float gamma_roll = 180.0f;

        [Tooltip("ロール回転の二乗抗力減衰係数 (N*m*s^2/rad^2)")]
        public float gamma_quad = 85.0f;

        [Header("6b. Hip-First & Lean-Out Turn Entry (上体垂直復帰＆腰先行リーンアウト導入制御)")]
        [Tooltip("画面中央から横方向へこの距離(m)マウスを動かすと最大ローリング指令(100%)になるアナログ操作幅")]
        public float mouseSteerFullRange = 1.15f;

        [Tooltip("マウス横入力時にマウス方向への腰移動を優先する割合 [0..1]")]
        [Range(0.0f, 1.0f)]
        public float mouseForceOverrideRatio = 0.88f;

        [Tooltip("マウス位置方向へ直接発生させる優先ローリングモーメントゲイン")]
        public float mouseRollMomentGain = 2.30f;

        [Tooltip("ターン中およびターン導入時に、上体(腰→頭)を鉛直(0度)よりもさらにターン外側へリーンアウトさせる比率 (0.0=鉛直キープ, 0.28=脚角の28%分だけ鉛直より外側へリーンアウト)")]
        [Range(0.0f, 0.60f)]
        public float trunkLeanOutRatio = 0.28f;

        [Tooltip("マウスでターンに入る瞬間、腰を先行させて頭を鉛直より外側へリーンアウトさせる導入ゲイン")]
        [Range(0.0f, 0.80f)]
        public float turnEntryLeanOutGain = 0.38f;

        [Tooltip("旧互換用（頭突っ込み角: 0で無効化）")]
        public float extHeadDiveMaxDeg = 0.0f;

        [Tooltip("切り替え時に腰(下部リンク)を先行して新しいターン内側へ送り出すブーストゲイン")]
        public float headDiveRollDriveGain = 1.35f;

        [Header("7. Snow Shear Failure (雪面挫折スキッディング & 台車押し負け)")]
        [Tooltip("最小接地圧閾値 N_min (体重比 G単位)")]
        public float N_min_G = 0.25f;

        [Tooltip("高ロール角領域でスキッドを防止するために必要な最小支持力比（互換用）")]
        public float kappa_grip = 0.42f;

        [Tooltip("スキッド(雪面挫折)発生時の横G伝達減衰率")]
        public float skidGripMultiplier = 0.55f;

        [Tooltip("雪面挫折スキッディングが発生し始める最小エッジ角 (deg)")]
        public float skidMinAngleDeg = 28.0f;

        [Tooltip("雪面挫折の基準となるエッジ角 (deg)")]
        public float skidRefAngleDeg = 48.0f;

        [Tooltip("角度が深くなるにつれて雪面が挫折しやすくなる角度依存指数")]
        [Range(0.5f, 2.5f)]
        public float snowAngleVulnerabilityExp = 1.35f;

        [Tooltip("重心の慣性平衡荷重を超えて雪面を押し潰す超過鉛直押下力閾値 (体重比 G単位)")]
        public float snowFailureVerticalForceG = 2.20f;

        [Tooltip("雪面が挫折し始める実効踏み込みパワー閾値 (W)")]
        public float snowFailurePowerThreshold = 1600.0f;

        [Tooltip("雪面挫折時に台車が超過踏力に押し負けて失速する減衰ゲイン")]
        public float carriagePushbackGain = 0.35f;

        [Tooltip("一度雪面が挫折してスキッディングした際、エッジが再グリップするまでの回復時定数 (s)")]
        public float skidRecoveryTau = 0.12f;

        [Tooltip("股関節の雪面クリアランス最低高さ z_H_min (m) ※これ未満で内倒クラッシュ")]
        public float z_H_min = 0.10f;

        [Tooltip("ブーツアウト強制スリップ限界エッジ角 (deg)")]
        public float theta_bootOutDeg = 78.0f;

        [Header("8. Extension Righting & Tuck Centering Return (起き上がり＆抱え込み台車復帰)")]
        [Tooltip("スペースキー伸展時にロール角を0度(直立)へ押し戻す直接起き上がりトルクゲイン")]
        public float extRightingTorqueGain = 0.65f;

        [Tooltip("脚が伸びる速度(dl1 > 0)に応じて発生する動的起き上がりトルクゲイン")]
        public float extVelocityRightingGain = 1.15f;

        [Tooltip("スペースキーを離したときに自動で膝を抱え込む(抜重する)引き込み力 (体重比 G単位)")]
        public float releaseTuckPullG = 0.65f;

        [Tooltip("抱え込み(スペースキー解放)時に台車を中央(x_c = 0)へ引き戻すバネ定数 (N/m)")]
        public float tuckCenterSpringK = 380.0f;

        [Tooltip("脚を抱え込む速度(dl1 < 0)に応じて台車の中央復帰力を増幅するブースト係数")]
        public float tuckVelocityReturnBoost = 2.20f;

        [Tooltip("抱え込み中央復帰時の台車ダンピング比")]
        public float tuckCenterDampingRatio = 0.85f;

        [Tooltip("抱え込みによって台車が中央へ戻る際、脚リンクも中央(0度)へ引き戻す連動ゲイン")]
        public float tuckRollCenteringGain = 8.0f;

        [Header("9. Shallow-Angle Capture-Point & Camber Physics (①・③・④ 浅角引き起こし物理)")]
        [Tooltip("① 倒れ込み角速度 ω1 に対して台車が重心落下予測点(Capture Point)へ先行して潜り込むゲイン")]
        [Range(0.0f, 2.5f)]
        public float capturePointLeadGain = 0.65f;

        [Tooltip("③ 浅い角度(0〜15度)におけるキャンバー初期接地の台車追従スティフネス下限比")]
        [Range(0.35f, 1.0f)]
        public float shallowGlideStiffnessMinRatio = 0.82f;

        [Tooltip("④ 台車加速途中でもエッジ角に応じて直接立ち上がるカービング遠心力の最低保証比")]
        [Range(0.0f, 1.0f)]
        public float directCarveRightingFloor = 0.80f;

        [Header("10. Neuromuscular Co-contraction & Adaptive Damping (神経筋の同時収縮と振動抑制)")]
        [Tooltip("中央復帰時や姿勢保持時に臨界減衰(振動ゼロ)へ収束させる生体インピーダンス減衰ゲイン (1.0〜4.0)")]
        [Range(0.2f, 5.0f)]
        public float adaptiveDampingBoost = 2.4f;

        [Tooltip("筋肉の同時収縮(Co-contraction)が効き始める角速度の閾値 (rad/s)")]
        [Range(0.02f, 1.0f)]
        public float coContractionVelThreshold = 0.12f;

        // --- 計算プロパティ（派生物理量・ラジアン変換キャッシュ） ---

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