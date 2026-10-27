using UnityEngine;
using SkyTechSimulator;
#if UNITY_IOS && !UNITY_EDITOR
using System.Runtime.InteropServices;
#endif
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
#endif

public class SkyTechGameController : MonoBehaviour
{
    public enum MouseHeightControlMode
    {
        [Tooltip("中立基準線を標準スタンスとし、上へ動かすと伸長(l_max)、下へ動かすと脚を曲げずにリーンアウト")]
        VerticalSignedFromCenter,
        [Tooltip("画面の中心座標(X, Y)からの2D距離で制御（下側は脚を屈曲させず中立長を維持）")]
        DistanceFromScreenCenter,
        [Tooltip("画面の上下中央(Y)からの垂直距離 |dy| で制御（下側は脚を屈曲させず中立長を維持）")]
        VerticalDistanceFromCenter
    }

    [Header("=== 設定アセット ===")]
    public SkyTechSimConfig Config;

    [Header("=== アシスト・マウス操作設定 ===")]
    public bool AutoAngulationAssist = false;
    [Tooltip("PC/エディタ用のマウス姿勢制御・頂点誘導。iPhone実機では画面タップによる誤動作を防ぐため無効化されます")]
    public bool UseMouseControl = true;
    public MouseHeightControlMode HeightControlMode = MouseHeightControlMode.VerticalSignedFromCenter;
    public float MouseCenterOffsetY = 0.45f;

    [Header("=== AirPods (頭部傾き) × iPhone (画面傾き) デュアル制御設定 ===")]
    [Tooltip("AirPodsの傾きセンサー(CMHeadphoneMotionManager)による上体・頭部の独立制御を有効にする")]
    public bool UseAirPodsHeadControl = true;
    [Tooltip("AirPodsの頭のかしげ角(度)に対するゲーム内の体幹傾き角(θ2)の感度倍率")]
    [Range(0.8f, 2.5f)] public float AirPodsRollSensitivity = 1.45f;
    [Tooltip("AirPodsで最大過剰リーンアウト(mouse_lean_out_norm=1)に達するターン外側への首かしげ角 [度]")]
    public float AirPodsMaxLeanOutDeg = 22.0f;
    [Tooltip("AirPods傾きの不感帯 [度]")]
    public float AirPodsDeadzoneDeg = 1.5f;
    [Tooltip("AirPods傾きの平滑化時定数 [秒]")]
    public float AirPodsSmoothingTau = 0.06f;

    [Header("=== ストレート内倒超え (オーバーリーンイン) 警告設定 ===")]
    public bool EnableOverLeanInWarning = true;
    public float OverLeanInWarnThresholdDeg = 1.5f;

    [Header("=== iPhone 端末チルト (脚ロール θ1) 設定 ===")]
    [Tooltip("iPhoneの画面傾き(加速度/ジャイロ)による脚ロール角(エッジ角)の操作を有効にする")]
    public bool UseDeviceTilt = true;
    public bool UseIosSensorControl { get => UseDeviceTilt; set => UseDeviceTilt = value; }
    [Range(0.5f, 3.0f)] public float DeviceTiltSensitivity = 1.5f;
    public float DeviceTiltDeadzoneDeg = 1.5f;
    public float DeviceTiltSmoothingTau = 0.08f;
    [Tooltip("AirPodsのピッチ角（うなずき）で脚の伸縮を補助する")]
    public bool UseAirPodsPitchForLeg = false;

    // UI表示用テレメトリプロパティ
    public bool IsAirPodsConnected { get; private set; }
    public float CurrentHeadRollDeg => _smoothedHeadRollDeg;
    public float CurrentDesiredTrunkDeg { get; private set; }
    public bool IsExcessLeanOutActive { get; private set; }
    public bool IsOverLeanInWarning { get; private set; }
    public float OverLeanInExcessDeg { get; private set; }
    public float OverLeanInIntensity { get; private set; }
    public float CurrentDeviceRollDeg => _smoothedRollDeg;

    private float _neutralHeadRollOffsetDeg;
    private float _smoothedHeadRollDeg;
    private float _hapticCooldownTimer;
    private float _neutralRollOffsetDeg;
    private float _smoothedRollDeg;

#if UNITY_IOS && !UNITY_EDITOR
    [DllImport("__Internal")] private static extern void HeadphoneMotion_Start();
    [DllImport("__Internal")] private static extern void HeadphoneMotion_Stop();
    [DllImport("__Internal")] private static extern bool HeadphoneMotion_IsActive();
    [DllImport("__Internal")] private static extern void HeadphoneMotion_GetAttitude(out float roll, out float pitch, out float yaw);
#endif

    public float MouseLeanInLineOffsetY = 2.20f;
    public float MouseLeanInUpRange = 1.75f;
    public float MouseExtendUpRange = 2.50f;

    public float MouseFlexDownRange = 0.75f;
    public float MouseFullExtendDist = 0.35f;
    public float MouseFullFlexDist = 2.50f;
    public float MouseLineDeadzone = 0.005f;

    public SimState CurrentState => _state;
    public SimInput CurrentInput => _input;
    public int TurnCount { get; private set; }
    public int ComboCount { get; private set; }
    public float TotalScore { get; private set; }
    public float CurrentTurnPeakEdgeDeg { get; private set; }
    public float CurrentTurnPeakCoMDeg { get; private set; }
    public float LastTurnPeakEdgeDeg { get; private set; }
    public float LastTurnPeakCoMDeg { get; private set; }
    public string LastTurnEvaluation { get; private set; } = "READY";
    public bool IsCrashFrozen { get; private set; }
    public float TargetLegRollDeg => UseDeviceTilt ? _smoothedRollDeg : _lockedHipDeg;

    private SimState _state;
    private SimInput _input;
    private float _lockedHipDeg;
    private float _crashFrozenTimer;
    private float _timeSinceLastSwitch;
    private float _lastEdgeSign;
    private float _skidDurationInTurn;
    private bool _isSteerNeutralActive;

    private float _prevAbsDx;
    private bool _hasPrevMouseSample;
    private float _centerReturnPushBoost;
    private float _prevSteer;

    private float _leftRollHoldTimer, _leftRollRepeatTimer;
    private float _rightRollHoldTimer, _rightRollRepeatTimer;

    private const float RollRepeatInitialDelay = 0.28f;
    private const float RollRepeatInterval = 0.08f;
    private const float RollDegreeStep = 2.0f;
    private const float TurnSwitchMinTime = 0.45f;
    private const float TurnSwitchMinPeakDeg = 15.0f;
    private const float CrashFreezeDuration = 1.5f;

    private void EnforceUpdatedRanges()
    {
        if (MouseLeanInLineOffsetY < 2.2f) MouseLeanInLineOffsetY = 2.40f;
        if (MouseLeanInUpRange < 1.5f)     MouseLeanInUpRange = 1.75f;
        if (MouseExtendUpRange < 2.2f)     MouseExtendUpRange = 2.60f;
        if (MouseFullFlexDist < 2.0f)      MouseFullFlexDist = 2.50f;
        if (Config != null && Config.mouseSteerFullRange < 2.50f)
        {
            Config.mouseSteerFullRange = 3.20f;
        }
    }

    private void OnValidate() => EnforceUpdatedRanges();

    private void Awake()
    {
        Input.simulateMouseWithTouches = false;
        EnforceUpdatedRanges();
        if (Config == null) Config = ScriptableObject.CreateInstance<SkyTechSimConfig>();
    }

    public void EnableDeviceSensors()
    {
        Input.simulateMouseWithTouches = false;
        Input.gyro.enabled = true;
        StartAirPodsSensor();
    }

    private void StartAirPodsSensor()
    {
#if UNITY_IOS && !UNITY_EDITOR
        if (UseAirPodsHeadControl) HeadphoneMotion_Start();
#else
        AirPodsMotionManager.Start();
#endif
    }

    private void StopAirPodsSensor()
    {
#if UNITY_IOS && !UNITY_EDITOR
        HeadphoneMotion_Stop();
#else
        AirPodsMotionManager.Stop();
#endif
    }

    private bool TryGetRawAirPodsRollDeg(out float headRollDeg)
    {
#if UNITY_IOS && !UNITY_EDITOR
        if (UseAirPodsHeadControl && HeadphoneMotion_IsActive())
        {
            HeadphoneMotion_GetAttitude(out float rollRad, out _, out _);
            // 右へ首をかしげると正(+)、左へかしげると負(-)
            headRollDeg = rollRad * Mathf.Rad2Deg;
            return true;
        }
#else
        if (AirPodsMotionManager.IsActive)
        {
            headRollDeg = AirPodsMotionManager.GetRollDeg();
            return true;
        }
#endif
        headRollDeg = 0f;
        return false;
    }

    private float ComputeRawDeviceZRollDeg()
    {
#if UNITY_IOS && !UNITY_EDITOR
        Vector3 acc = Input.acceleration;
        if (acc.sqrMagnitude > 0.01f)
        {
            if (Screen.orientation == ScreenOrientation.LandscapeRight)
            {
                return Mathf.Atan2(-acc.x, acc.y) * Mathf.Rad2Deg;
            }
            else if (Screen.orientation == ScreenOrientation.LandscapeLeft)
            {
                return Mathf.Atan2(acc.x, -acc.y) * Mathf.Rad2Deg;
            }
            else
            {
                return Mathf.Atan2(-acc.x, -acc.y) * Mathf.Rad2Deg;
            }
        }
#endif
        return 0f;
    }

    // CalibrateNeutralTilt() を拡張：iPhoneとAirPodsのゼロ点を同時キャリブレーション
    public void CalibrateDualSensors()
    {
        _neutralRollOffsetDeg = ComputeRawDeviceZRollDeg();
        _smoothedRollDeg = 0f;
        if (TryGetRawAirPodsRollDeg(out float rawHead))
        {
            _neutralHeadRollOffsetDeg = rawHead;
        }
        _smoothedHeadRollDeg = 0f;
#if UNITY_EDITOR
        AirPodsMotionManager.Calibrate();
#endif
        Debug.Log("<color=#58D68D>[SkyTechGameController] Calibrated Dual Sensors (iPhone & AirPods).</color>");
    }

    private void OnEnable()
    {
        EnableDeviceSensors();
    }

    private void OnDisable()
    {
        StopAirPodsSensor();
    }

    private void Start() => ResetSimulation();

    private void Update() => HandleKeyboardInput();

    private void FixedUpdate()
    {
        if (IsCrashFrozen)
        {
            if ((_crashFrozenTimer -= Time.fixedDeltaTime) <= 0f)
            {
                IsCrashFrozen = false;
                ResetSimulation();
            }
            return;
        }

        SkyTechPhysicsSolver.Step(ref _state, in _input, Config, Time.fixedDeltaTime);

        if (_state.gripState == SkiGripState.RailLimitHit)
        {
            ComboCount = 0;
            LastTurnEvaluation = "RAIL HIT! RESET";
            ResetSimulation();
            return;
        }

        if (_state.gripState is SkiGripState.HipCrash or SkiGripState.BootOut)
        {
            TriggerCrash();
            return;
        }

        UpdateTurnTelemetry();
        CheckTurnSwitch();
    }

    private void HandleKeyboardInput()
    {
        if (IsKeyDown(KeyCode.R))
        {
            ResetSimulation();
            return;
        }

        if (IsKeyDown(KeyCode.Tab)) AutoAngulationAssist = !AutoAngulationAssist;

        if (IsCrashFrozen)
        {
            _input = default;
            _isSteerNeutralActive = false;
            _centerReturnPushBoost = 0f;
            return;
        }

        if (IsKeyDown(KeyCode.C))
        {
            CalibrateDualSensors();
        }

        bool qDown = IsKeyDown(KeyCode.Q), qHeld = IsKeyHeld(KeyCode.Q);
        bool eDown = IsKeyDown(KeyCode.E), eHeld = IsKeyHeld(KeyCode.E);
        bool resetActive = IsKeyDown(KeyCode.W) || IsKeyHeld(KeyCode.W)
                        || IsKeyDown(KeyCode.X) || IsKeyHeld(KeyCode.X)
                        || (qHeld && eHeld);

        float safeDt = Mathf.Max(1e-4f, Time.deltaTime);

#if UNITY_EDITOR
        float mouseScrollY = 0f;
        try { mouseScrollY = Input.mouseScrollDelta.y; } catch { }
        AirPodsMotionManager.UpdateEditorSimulation(Time.deltaTime, qHeld, eHeld, mouseScrollY);
#endif

        // 1. iPhone 画面傾き（Z軸ロール）による脚ロール操作 -> マウス互換のステア入力
        float steerFromPhone = 0f;
#if UNITY_IOS && !UNITY_EDITOR
        if (UseDeviceTilt)
        {
            float rawDevRoll = ComputeRawDeviceZRollDeg() - _neutralRollOffsetDeg;
            float absDev = Mathf.Abs(rawDevRoll);
            float filteredDev = (absDev > DeviceTiltDeadzoneDeg)
                ? Mathf.Sign(rawDevRoll) * (absDev - DeviceTiltDeadzoneDeg) * DeviceTiltSensitivity
                : 0f;
            float alphaDev = safeDt / (Mathf.Max(0.005f, DeviceTiltSmoothingTau) + safeDt);
            _smoothedRollDeg = Mathf.Lerp(_smoothedRollDeg, filteredDev, alphaDev);

            // iPhone画面の傾き (最大45°) を正規化ステア入力 [-1, 1] へ滑らかにマッピング
            float absSmoothDev = Mathf.Abs(_smoothedRollDeg);
            const float fullTiltDeg = 45.0f;
            float normTilt = Mathf.Clamp01(absSmoothDev / fullTiltDeg);
            steerFromPhone = (absSmoothDev > 0.05f) ? Mathf.Sign(_smoothedRollDeg) * Mathf.SmoothStep(0f, 1f, normTilt) : 0f;
        }
#endif

        // 2. AirPods 頭部傾きセンサーによる体幹目標角 θ2,ref 取得
        bool hasRawHead = TryGetRawAirPodsRollDeg(out float rawHeadRoll);
        IsAirPodsConnected = hasRawHead || (Application.isEditor && AirPodsMotionManager.IsActive);

        if (IsAirPodsConnected)
        {
            float headRollDiff = rawHeadRoll - _neutralHeadRollOffsetDeg;
            float absHead = Mathf.Abs(headRollDiff);
            float filteredHead = (absHead > AirPodsDeadzoneDeg)
                ? Mathf.Sign(headRollDiff) * (absHead - AirPodsDeadzoneDeg)
                : 0f;
            float alphaHead = safeDt / (Mathf.Max(0.005f, AirPodsSmoothingTau) + safeDt);
            _smoothedHeadRollDeg = Mathf.Lerp(_smoothedHeadRollDeg, filteredHead, alphaHead);
            CurrentDesiredTrunkDeg = _smoothedHeadRollDeg * AirPodsRollSensitivity;
        }
        else
        {
            _smoothedHeadRollDeg = Mathf.Lerp(_smoothedHeadRollDeg, 0f, 10.0f * safeDt);
            CurrentDesiredTrunkDeg = 0f;
            IsExcessLeanOutActive = false;
            IsOverLeanInWarning = false;
            OverLeanInExcessDeg = 0f;
            OverLeanInIntensity = 0f;
        }

        if (resetActive)
        {
            _lockedHipDeg = 0f;
            _input.phi_steer_cmd = 0f;
            _input.mouse_lean_out_norm = 0f;
            _input.mouse_mid_norm = 0f;
            _input.mouse_lean_in_norm = 0f;
            _input.steer_neutral_cmd = false;
            _input.w_reset = true;
            _isSteerNeutralActive = false;
            _leftRollHoldTimer = _leftRollRepeatTimer = _rightRollHoldTimer = _rightRollRepeatTimer = 0f;
            _centerReturnPushBoost = 0f;
        }
        else
        {
            _input.w_reset = false;
            bool deviceTiltDriving = false;
#if UNITY_IOS && !UNITY_EDITOR
            deviceTiltDriving = UseDeviceTilt && Mathf.Abs(_smoothedRollDeg) > 0.1f;
#endif
            if (!deviceTiltDriving)
            {
                UpdateRollRepeat(qDown, qHeld, -1f, ref _leftRollHoldTimer, ref _leftRollRepeatTimer);
                UpdateRollRepeat(eDown, eHeld,  1f, ref _rightRollHoldTimer, ref _rightRollRepeatTimer);
            }
        }

        _input.phi_lock_cmd = _lockedHipDeg * Mathf.Deg2Rad;

        bool extPressed = IsKeyHeld(KeyCode.Space) || IsKeyHeld(KeyCode.UpArrow);
        bool flexPressed = IsKeyHeld(KeyCode.S);

        if (UseAirPodsPitchForLeg && IsAirPodsConnected)
        {
            float pitch = AirPodsMotionManager.GetPitchDeg();
            if (pitch > 6.0f) extPressed = true;
            else if (pitch < -6.0f) flexPressed = true;
        }

#if ENABLE_INPUT_SYSTEM
        if (Gamepad.current != null)
        {
            extPressed |= Gamepad.current.buttonSouth.isPressed;
            flexPressed |= Gamepad.current.buttonEast.isPressed;
        }
#endif

        Vector2 mouseWorld = Vector2.zero;
        Vector2 screenCenterWorld = Vector2.zero;
#if !UNITY_EDITOR && (UNITY_IOS || UNITY_ANDROID)
        // iPhone実機上ではタップ・タッチによる姿勢制御を完全に無効化（PC/エディタのみ有効）
        bool hasMouseWorld = false;
#else
        bool hasMouseWorld = !Application.isMobilePlatform && UseMouseControl && TryGetMouseWorldPosition(out mouseWorld, out screenCenterWorld);
#endif
        _input.mouse_world = mouseWorld;
        _input.has_mouse_world = hasMouseWorld;

        float baseCenterY = screenCenterWorld.y + MouseCenterOffsetY;
        float dy = mouseWorld.y - baseCenterY;

        // 3. デュアルチルトによる姿勢・警告計算（AirPods 体幹角 θ2,ref × 脚ロール θ1）
        if (UseAirPodsHeadControl && IsAirPodsConnected && Config != null)
        {
            float legRollDeg = (Mathf.Abs(steerFromPhone) > 0.05f) ? steerFromPhone * 45.0f : _state.theta1 * Mathf.Rad2Deg;
            float activeTurnSign = (Mathf.Abs(legRollDeg) > 1.0f)
                ? Mathf.Sign(legRollDeg)
                : ((Mathf.Abs(steerFromPhone) > 0.01f) ? Mathf.Sign(steerFromPhone) : 0f);

            if (activeTurnSign != 0f)
            {
                float turnBankDeg = Mathf.Abs(legRollDeg);
                float trunkInTurnDeg = CurrentDesiredTrunkDeg * activeTurnSign;

                if (trunkInTurnDeg < 0f)
                {
                    // (A) ターン外側への首かしげ ＝ 過剰リーンアウト (Excess Lean-Out)
                    float outwardDeg = -trunkInTurnDeg;
                    IsExcessLeanOutActive = outwardDeg > 1.0f;
                    float normOut = Mathf.Clamp01(outwardDeg / Mathf.Max(5.0f, AirPodsMaxLeanOutDeg));
                    _input.mouse_lean_out_norm = normOut * normOut;
                    _input.mouse_mid_norm = 0f;
                    _input.mouse_lean_in_norm = 0f;

                    IsOverLeanInWarning = false;
                    OverLeanInExcessDeg = 0f;
                    OverLeanInIntensity = 0f;
                }
                else if (trunkInTurnDeg <= turnBankDeg)
                {
                    // (B) 鉛直 ～ ストレート内倒 (Mid Zone)
                    IsExcessLeanOutActive = false;
                    _input.mouse_lean_out_norm = 0f;
                    float midRatio = (turnBankDeg > 0.1f) ? Mathf.Clamp01(trunkInTurnDeg / turnBankDeg) : 0f;
                    float centeredMid = (midRatio - 0.5f) * 2.0f;
                    float plateauMid = 0.5f + 0.5f * (0.30f * centeredMid + 0.70f * centeredMid * centeredMid * centeredMid);
                    _input.mouse_mid_norm = Mathf.Clamp01(plateauMid);
                    _input.mouse_lean_in_norm = 0f;

                    IsOverLeanInWarning = false;
                    OverLeanInExcessDeg = 0f;
                    OverLeanInIntensity = 0f;
                }
                else
                {
                    // (C) ストレート内倒超え ＝ オーバーリーンイン (Over Lean-In)
                    IsExcessLeanOutActive = false;
                    _input.mouse_lean_out_norm = 0f;
                    _input.mouse_mid_norm = 1.0f;

                    float excessDeg = trunkInTurnDeg - turnBankDeg;
                    OverLeanInExcessDeg = excessDeg;
                    float rawIn = Mathf.Clamp01(excessDeg / 15.0f);
                    _input.mouse_lean_in_norm = rawIn * rawIn;

                    if (EnableOverLeanInWarning && excessDeg > OverLeanInWarnThresholdDeg)
                    {
                        IsOverLeanInWarning = true;
                        OverLeanInIntensity = Mathf.Clamp01((excessDeg - OverLeanInWarnThresholdDeg) / 10.0f);

                        _hapticCooldownTimer -= safeDt;
                        if (_hapticCooldownTimer <= 0f)
                        {
                            _hapticCooldownTimer = 0.28f;
#if UNITY_IOS && !UNITY_EDITOR
                            Handheld.Vibrate();
#endif
                        }
                    }
                    else
                    {
                        IsOverLeanInWarning = false;
                        OverLeanInIntensity = 0f;
                    }
                }
            }
            else
            {
                IsExcessLeanOutActive = false;
                IsOverLeanInWarning = false;
                OverLeanInExcessDeg = 0f;
                OverLeanInIntensity = 0f;
                _input.mouse_lean_out_norm = 0f;
                _input.mouse_mid_norm = 0f;
                _input.mouse_lean_in_norm = 0f;
            }

            if (hasMouseWorld)
            {
                float absDx = Mathf.Abs(mouseWorld.x - screenCenterWorld.x);
                if (_hasPrevMouseSample)
                {
                    float inwardSpeed = Mathf.Max(0f, (_prevAbsDx - absDx) / safeDt);
                    float turnBankGate = Mathf.Clamp01((Mathf.Abs(_state.theta1) * Mathf.Rad2Deg - 8.0f) / 14.0f);
                    float returnTarget = Mathf.Clamp01((inwardSpeed / 2.2f) * Config.centerReturnPushGain) * turnBankGate;
                    float rate = (returnTarget > _centerReturnPushBoost) ? 10.0f : 3.2f;
                    float target = (returnTarget > _centerReturnPushBoost) ? returnTarget : 0f;
                    _centerReturnPushBoost = Mathf.MoveTowards(_centerReturnPushBoost, target, rate * safeDt);
                }
                _prevAbsDx = absDx;
                _hasPrevMouseSample = true;
            }
        }
        else if (hasMouseWorld && Config != null)
        {
            IsExcessLeanOutActive = false;
            IsOverLeanInWarning = false;
            OverLeanInExcessDeg = 0f;
            OverLeanInIntensity = 0f;

            float leanInOffset = Mathf.Max(0.50f, MouseLeanInLineOffsetY);

            // 中立基準線より下 (dy < 0): 0 -> 1 (鉛直を超えてさらにリーンアウト)
            float rawOut = (dy < 0f) ? Mathf.Clamp01(-dy / Mathf.Max(0.20f, MouseFlexDownRange)) : 0f;
            _input.mouse_lean_out_norm = rawOut * rawOut;

            float rawMid = (dy > 0f) ? Mathf.Clamp01(dy / leanInOffset) : 0f;
            float centeredMid = (rawMid - 0.5f) * 2.0f;
            float plateauMid = 0.5f + 0.5f * (0.30f * centeredMid + 0.70f * centeredMid * centeredMid * centeredMid);
            _input.mouse_mid_norm = Mathf.Clamp01(plateauMid);

            float rawIn = (dy > leanInOffset)
                ? Mathf.Clamp01((dy - leanInOffset) / Mathf.Max(0.20f, MouseLeanInUpRange))
                : 0f;
            _input.mouse_lean_in_norm = rawIn * rawIn;

            float absDx = Mathf.Abs(mouseWorld.x - screenCenterWorld.x);
            if (_hasPrevMouseSample)
            {
                float inwardSpeed = Mathf.Max(0f, (_prevAbsDx - absDx) / safeDt);
                float turnBankGate = Mathf.Clamp01((Mathf.Abs(_state.theta1) * Mathf.Rad2Deg - 8.0f) / 14.0f);
                float returnTarget = Mathf.Clamp01((inwardSpeed / 2.2f) * Config.centerReturnPushGain) * turnBankGate;
                float rate = (returnTarget > _centerReturnPushBoost) ? 10.0f : 3.2f;
                float target = (returnTarget > _centerReturnPushBoost) ? returnTarget : 0f;
                _centerReturnPushBoost = Mathf.MoveTowards(_centerReturnPushBoost, target, rate * safeDt);
            }
            _prevAbsDx = absDx;
            _hasPrevMouseSample = true;
        }

        else
        {
            _input.mouse_lean_out_norm = 0f;
            _input.mouse_mid_norm = 0f;
            _input.mouse_lean_in_norm = 0f;
            _hasPrevMouseSample = false;
            _centerReturnPushBoost = Mathf.MoveTowards(_centerReturnPushBoost, 0f, 4.0f * safeDt);
        }

        // 1. 脚の伸縮入力
        if (extPressed || flexPressed)
        {
            _input.l1_target = 0f;
            _input.u_ext_norm = extPressed ? Mathf.MoveTowards(_input.u_ext_norm, 1.0f, 45.0f * Time.deltaTime) : 0f;
            _input.u_flex_norm = flexPressed ? Mathf.MoveTowards(_input.u_flex_norm, 1.0f, 24.0f * Time.deltaTime) : 0f;

            if (extPressed && !qHeld && !eHeld && _state.gripState != SkiGripState.Skidding && _state.snow_stress_ratio < 0.9f)
            {
                _lockedHipDeg = Mathf.MoveTowards(_lockedHipDeg, 0f, 35.0f * Time.deltaTime);
            }
        }
        else if (hasMouseWorld && Config != null)
        {
            float neutralL1 = Mathf.Clamp(Config.l_max - Config.supportDropFromMaxMeters, Config.l_min, Config.l_max);
            float baseTargetL1 = (dy < 0f) ? neutralL1 : HeightControlMode switch
            {
                MouseHeightControlMode.VerticalSignedFromCenter =>
                    neutralL1 + Mathf.Clamp01(dy / Mathf.Max(0.05f, MouseExtendUpRange)) * (Config.l_max - neutralL1),
                MouseHeightControlMode.VerticalDistanceFromCenter =>
                    Mathf.Lerp(Config.l_max, neutralL1, Mathf.InverseLerp(MouseFullExtendDist, MouseFullFlexDist, dy)),
                _ =>
                    Mathf.Lerp(Config.l_max, neutralL1, Mathf.InverseLerp(MouseFullExtendDist, MouseFullFlexDist, Vector2.Distance(mouseWorld, new Vector2(screenCenterWorld.x, baseCenterY))))
            };

            float targetL1 = Mathf.Lerp(baseTargetL1, Config.l_max, _centerReturnPushBoost * 0.85f);
            _input.l1_target = targetL1;

            float linDrive = Mathf.Clamp(((targetL1 - _state.l1) - 0.08f * _state.dl1) / 0.12f, -1.0f, 1.0f);
            float smoothDrive = linDrive * Mathf.Abs(linDrive);
            float topBoost = Mathf.SmoothStep(0f, 1f, (targetL1 - (Config.l_max - 0.03f)) / 0.025f);
            float extTarget = Mathf.Max(Mathf.Max(0f, smoothDrive), Mathf.Max(topBoost, _centerReturnPushBoost));

            _input.u_ext_norm = Mathf.MoveTowards(_input.u_ext_norm, extTarget, 35.0f * Time.deltaTime);
            _input.u_flex_norm = Mathf.MoveTowards(_input.u_flex_norm, 0f, 24.0f * Time.deltaTime);
        }
        else
        {
            _input.l1_target = 0f;
            _input.u_ext_norm = 0f;
            _input.u_flex_norm = 0f;
        }

                // 2. 左右の頂点誘導入力（広いストローク＆高ロール角での感度緩和）
        float steer = 0f;
        bool kbLeft = IsKeyHeld(KeyCode.A) || IsKeyHeld(KeyCode.LeftArrow);
        bool kbRight = IsKeyHeld(KeyCode.D) || IsKeyHeld(KeyCode.RightArrow);

        if (kbLeft || kbRight)
        {
            steer = (kbRight ? 1f : 0f) - (kbLeft ? 1f : 0f);
        }
        else if (Mathf.Abs(steerFromPhone) > 0.001f)
        {
            steer = steerFromPhone;
        }
        else if (hasMouseWorld)
        {
            float dx = mouseWorld.x - screenCenterWorld.x;
            float absDx = Mathf.Abs(dx);
            if (absDx > MouseLineDeadzone)
            {
                float fullRange = (Config != null) ? Mathf.Max(2.50f, Config.mouseSteerFullRange) : 3.20f;
                float normSteer = Mathf.Clamp01((absDx - MouseLineDeadzone) / fullRange);
                // 中間〜深いロール角にかけて感度が穏やかになるよう、緩やかなS字（SmoothStepブレンド）を適用
                float gentleSteer = Mathf.Lerp(normSteer, Mathf.SmoothStep(0f, 1f, normSteer), 0.45f);
                steer = Mathf.Sign(dx) * gentleSteer;
            }
        }

        // ターン切り返し（エッジ反転）時の屈曲抜重 (Flexion Unweighting)
        float steerRate = Mathf.Abs(steer - _prevSteer) / safeDt;
        _prevSteer = steer;
        float unweightingFlex = Mathf.Clamp01((steerRate - 1.8f) / 2.5f) * Mathf.Clamp01(1.0f - Mathf.Abs(steer) / 0.35f);
        if (unweightingFlex > 0.05f && !extPressed && !hasMouseWorld)
        {
            _input.u_flex_norm = Mathf.Max(_input.u_flex_norm, unweightingFlex * 0.65f);
        }

        bool downHeld = IsKeyHeld(KeyCode.DownArrow);
        if (IsKeyDown(KeyCode.DownArrow) || downHeld) _isSteerNeutralActive = true;
        if (Mathf.Abs(steer) > 0.01f) _isSteerNeutralActive = false;
        if (_isSteerNeutralActive && !downHeld && Mathf.Abs(_state.u_shift_filtered) < 0.01f && Mathf.Abs(_state.phi_bend) < 0.01f)
        {
            _isSteerNeutralActive = false;
        }

        _input.phi_steer_cmd = -steer;
        _input.steer_neutral_cmd = _isSteerNeutralActive;
    }

    private void UpdateRollRepeat(bool down, bool held, float dir, ref float holdTimer, ref float repeatTimer)
    {
        if (down)
        {
            StepLockedHip(dir);
            holdTimer = repeatTimer = 0f;
        }
        else if (held)
        {
            holdTimer += Time.deltaTime;
            if (holdTimer >= RollRepeatInitialDelay)
            {
                repeatTimer += Time.deltaTime;
                if (repeatTimer >= RollRepeatInterval)
                {
                    StepLockedHip(dir);
                    repeatTimer = 0f;
                }
            }
        }
        else
        {
            holdTimer = repeatTimer = 0f;
        }
    }

    private void StepLockedHip(float dir)
    {
        bool isOpposite = (_lockedHipDeg * dir) < -0.001f;
        float step = isOpposite ? (RollDegreeStep * 2.0f) : RollDegreeStep;
        float nextDeg = _lockedHipDeg + dir * step;
        if (isOpposite && (nextDeg * dir) > 0.0f) nextDeg = 0.0f;
        _lockedHipDeg = Mathf.Clamp(nextDeg, -75.0f, 75.0f);
    }

    public static bool TryGetMouseWorldPosition(out Vector2 worldPos, out Vector2 screenCenterWorld)
    {
        worldPos = screenCenterWorld = Vector2.zero;
#if !UNITY_EDITOR && (UNITY_IOS || UNITY_ANDROID)
        // モバイル実機ではタップによるマウス姿勢制御を行わない（PC/Editor限定）
        return false;
#else
        if (Application.isMobilePlatform) return false;
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (cam == null) return false;

        Vector3 screenPos = Vector3.zero;
        bool hasPos = false;

        try
        {
            Vector3 legacyPos = Input.mousePosition;
            if (!float.IsNaN(legacyPos.x) && !float.IsNaN(legacyPos.y))
            {
                screenPos = legacyPos;
                hasPos = true;
            }
        }
        catch { hasPos = false; }

#if ENABLE_INPUT_SYSTEM
        if (!hasPos)
        {
            var pointer = Pointer.current ?? (Pointer)Mouse.current;
            if (pointer != null)
            {
                Vector2 mp = pointer.position.ReadValue();
                if (!float.IsNaN(mp.x) && !float.IsNaN(mp.y))
                {
                    screenPos = new Vector3(mp.x, mp.y, 0f);
                    hasPos = true;
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
#if UNITY_EDITOR
                    float retinaFactor = UnityEditor.EditorGUIUtility.pixelsPerPoint;
#else
                    float retinaFactor = (Display.main.systemWidth > 0 && Display.main.renderingWidth > 0)
                        ? (float)Display.main.renderingWidth / Display.main.systemWidth
                        : (Screen.dpi > 160f ? 2.0f : 1.0f);
#endif
                    if (retinaFactor > 1.05f &&
                        screenPos.x * retinaFactor <= cam.pixelWidth * 1.05f &&
                        screenPos.y * retinaFactor <= cam.pixelHeight * 1.05f)
                    {
                        screenPos.x *= retinaFactor;
                        screenPos.y *= retinaFactor;
                    }
#endif
                }
            }
        }
#endif
        if (!hasPos) return false;

        float depth = -cam.transform.position.z;
        screenPos.z = depth;
        worldPos = cam.ScreenToWorldPoint(screenPos);
        screenCenterWorld = cam.ScreenToWorldPoint(new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, depth));
        return true;
#endif
    }

#if ENABLE_INPUT_SYSTEM
    private static KeyControl GetKeyControl(Keyboard kb, KeyCode kc) => kc switch
    {
        KeyCode.Space => kb.spaceKey,
        KeyCode.W => kb.wKey,
        KeyCode.S => kb.sKey,
        KeyCode.A => kb.aKey,
        KeyCode.D => kb.dKey,
        KeyCode.Q => kb.qKey,
        KeyCode.E => kb.eKey,
        KeyCode.R => kb.rKey,
        KeyCode.X => kb.xKey,
        KeyCode.C => kb.cKey,
        KeyCode.Tab => kb.tabKey,
        KeyCode.UpArrow => kb.upArrowKey,
        KeyCode.DownArrow => kb.downArrowKey,
        KeyCode.LeftArrow => kb.leftArrowKey,
        KeyCode.RightArrow => kb.rightArrowKey,
        _ => null
    };
#endif

    private static bool IsKeyHeld(KeyCode key)
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null && GetKeyControl(Keyboard.current, key) is { isPressed: true }) return true;
#endif
        try { return Input.GetKey(key); } catch { return false; }
    }

    private static bool IsKeyDown(KeyCode key)
    {
#if ENABLE_INPUT_SYSTEM
        if (Keyboard.current != null && GetKeyControl(Keyboard.current, key) is { wasPressedThisFrame: true }) return true;
#endif
        try { return Input.GetKeyDown(key); } catch { return false; }
    }

    private void UpdateTurnTelemetry()
    {
        _timeSinceLastSwitch += Time.fixedDeltaTime;
        CurrentTurnPeakEdgeDeg = Mathf.Max(CurrentTurnPeakEdgeDeg, Mathf.Abs(_state.theta1) * Mathf.Rad2Deg);
        CurrentTurnPeakCoMDeg = Mathf.Max(CurrentTurnPeakCoMDeg, Mathf.Abs(_state.Psi_CoM) * Mathf.Rad2Deg);
        if (_state.gripState == SkiGripState.Skidding) _skidDurationInTurn += Time.fixedDeltaTime;
    }

    private void CheckTurnSwitch()
    {
        float currentSign = Mathf.Sign(_state.theta1);
        if (_lastEdgeSign == 0f)
        {
            if (Mathf.Abs(_state.theta1) > 0.01f) _lastEdgeSign = currentSign;
            return;
        }

        if (currentSign != _lastEdgeSign && Mathf.Abs(_state.theta1) > 0.005f)
        {
            if (_timeSinceLastSwitch >= TurnSwitchMinTime && CurrentTurnPeakEdgeDeg >= TurnSwitchMinPeakDeg)
            {
                EvaluateCompletedTurn();
            }
            _lastEdgeSign = currentSign;
        }
    }

    private void EvaluateCompletedTurn()
    {
        TurnCount++;
        ComboCount++;

        bool hasAngulationBonus = CurrentTurnPeakEdgeDeg > (CurrentTurnPeakCoMDeg + 3.0f);
        float baseScore = CurrentTurnPeakEdgeDeg * (hasAngulationBonus ? 1.25f : 1.0f);
        baseScore *= Mathf.Clamp01(1.0f - (_skidDurationInTurn * 0.20f));
        TotalScore += baseScore * (1.0f + (ComboCount - 1) * 0.10f);

        LastTurnEvaluation = CurrentTurnPeakEdgeDeg switch
        {
            >= 65.0f => hasAngulationBonus ? "WORLD CUP CARVE!!" : "WORLD CUP SPEED!",
            >= 50.0f => hasAngulationBonus ? "DEEP CARVE!" : "SOLID EDGE!",
            >= 35.0f => "GOOD TURN",
            _ => "SHALLOW TURN"
        };

        LastTurnPeakEdgeDeg = CurrentTurnPeakEdgeDeg;
        LastTurnPeakCoMDeg = CurrentTurnPeakCoMDeg;
        CurrentTurnPeakEdgeDeg = CurrentTurnPeakCoMDeg = _skidDurationInTurn = _timeSinceLastSwitch = 0f;
    }

    private void TriggerCrash()
    {
        ComboCount = 0;
        IsCrashFrozen = true;
        _crashFrozenTimer = CrashFreezeDuration;
        LastTurnEvaluation = (_state.gripState == SkiGripState.BootOut) ? "BOOT OUT CRASH!" : "HIP CRASH!";
    }

    public void ResetSimulation()
    {
        _state = SkyTechPhysicsSolver.CreateInitialState(Config);
        _input = default;
        _lockedHipDeg = _prevAbsDx = _centerReturnPushBoost = 0f;
        _isSteerNeutralActive = _hasPrevMouseSample = IsCrashFrozen = false;
        CurrentTurnPeakEdgeDeg = CurrentTurnPeakCoMDeg = _skidDurationInTurn = _timeSinceLastSwitch = _lastEdgeSign = _crashFrozenTimer = 0f;
    }
}