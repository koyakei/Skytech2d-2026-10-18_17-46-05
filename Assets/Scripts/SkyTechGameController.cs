using UnityEngine;
using SkyTechSimulator;
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
    public bool UseMouseControl = true;
    public MouseHeightControlMode HeightControlMode = MouseHeightControlMode.VerticalSignedFromCenter;
    public float MouseCenterOffsetY = 0.45f;

    [Header("=== AirPods モーション連携設定 ===")]
    [Tooltip("AirPodsの傾きモーション（頭のロール角）で角付け角(_lockedHipDeg)を操作する")]
    public bool UseAirPodsMotion = true;
    [Tooltip("AirPodsのロール角感度倍率")]
    [Range(0.5f, 3.0f)] public float AirPodsRollSensitivity = 1.2f;
    [Tooltip("AirPodsロール角の不感帯（度）")]
    [Range(0.0f, 5.0f)] public float AirPodsRollDeadzone = 1.0f;
    [Tooltip("AirPodsのピッチ角（うなずき）で脚の伸縮を補助する")]
    public bool UseAirPodsPitchForLeg = false;

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
    public float TargetLegRollDeg => _lockedHipDeg;

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
        EnforceUpdatedRanges();
        if (Config == null) Config = ScriptableObject.CreateInstance<SkyTechSimConfig>();
    }

    private void OnEnable()
    {
        AirPodsMotionManager.Start();
    }

    private void OnDisable()
    {
        AirPodsMotionManager.Stop();
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
            AirPodsMotionManager.Calibrate();
        }

        bool qDown = IsKeyDown(KeyCode.Q), qHeld = IsKeyHeld(KeyCode.Q);
        bool eDown = IsKeyDown(KeyCode.E), eHeld = IsKeyHeld(KeyCode.E);
        bool resetActive = IsKeyDown(KeyCode.W) || IsKeyHeld(KeyCode.W)
                        || IsKeyDown(KeyCode.X) || IsKeyHeld(KeyCode.X)
                        || (qHeld && eHeld);

#if UNITY_EDITOR
        float mouseScrollY = 0f;
        try { mouseScrollY = Input.mouseScrollDelta.y; } catch { }
        AirPodsMotionManager.UpdateEditorSimulation(Time.deltaTime, qHeld, eHeld, mouseScrollY);
#endif

        float airPodsRoll = 0f;
        float airPodsPitch = 0f;
        bool hasAirPodsMotion = UseAirPodsMotion && AirPodsMotionManager.IsActive;

        if (hasAirPodsMotion)
        {
            float rawRoll = AirPodsMotionManager.UpdateAndGetSmoothedRollDeg(Time.deltaTime, 0.06f);
            airPodsPitch = AirPodsMotionManager.UpdateAndGetSmoothedPitchDeg(Time.deltaTime, 0.08f);

            float absRoll = Mathf.Abs(rawRoll);
            if (absRoll > AirPodsRollDeadzone)
            {
                float effectiveRoll = Mathf.Sign(rawRoll) * (absRoll - AirPodsRollDeadzone) * AirPodsRollSensitivity;
                airPodsRoll = Mathf.Clamp(effectiveRoll, -75.0f, 75.0f);
            }
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
            if (hasAirPodsMotion && (Mathf.Abs(airPodsRoll) > 0.01f || (!qHeld && !eHeld && !qDown && !eDown)))
            {
                _lockedHipDeg = Mathf.Lerp(_lockedHipDeg, airPodsRoll, 16.0f * Time.deltaTime);
            }
            else
            {
                UpdateRollRepeat(qDown, qHeld, -1f, ref _leftRollHoldTimer, ref _leftRollRepeatTimer);
                UpdateRollRepeat(eDown, eHeld,  1f, ref _rightRollHoldTimer, ref _rightRollRepeatTimer);
            }
        }

        _input.phi_lock_cmd = _lockedHipDeg * Mathf.Deg2Rad;

        bool extPressed = IsKeyHeld(KeyCode.Space) || IsKeyHeld(KeyCode.UpArrow);
        bool flexPressed = IsKeyHeld(KeyCode.S);

        if (hasAirPodsMotion && UseAirPodsPitchForLeg)
        {
            if (airPodsPitch > 6.0f) extPressed = true;
            else if (airPodsPitch < -6.0f) flexPressed = true;
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
        bool hasMouseWorld = UseMouseControl && TryGetMouseWorldPosition(out mouseWorld, out screenCenterWorld);
        _input.mouse_world = mouseWorld;
        _input.has_mouse_world = hasMouseWorld;

        float safeDt = Mathf.Max(1e-4f, Time.deltaTime);
        float baseCenterY = screenCenterWorld.y + MouseCenterOffsetY;
        float dy = mouseWorld.y - baseCenterY;

                // 0. 上下基準線からのリーンアウト / 中間(スイートスポット付き) / リーンイン算出
        if (hasMouseWorld && Config != null)
        {
            float leanInOffset = Mathf.Max(0.50f, MouseLeanInLineOffsetY);

            // 中立基準線より下 (dy < 0): 0 -> 1 (鉛直を超えてさらにリーンアウト)
            float rawOut = (dy < 0f) ? Mathf.Clamp01(-dy / Mathf.Max(0.20f, MouseFlexDownRange)) : 0f;
            _input.mouse_lean_out_norm = rawOut * rawOut; // 境界付近での急変を防ぐ2乗カーブ

            // 中立基準線 (dy = 0) ～ Lean-in line (dy = leanInOffset) の補間比率 (0..1)
            // 【改善】中間部分 (0.25 ～ 0.75) の感度を緩やかにするS字プラトー変換
            // これにより、中間付近でマウスが上下にブレても姿勢がシビアに変動しない
            float rawMid = (dy > 0f) ? Mathf.Clamp01(dy / leanInOffset) : 0f;
            float centeredMid = (rawMid - 0.5f) * 2.0f; // -1.0 .. +1.0
            // 中心(0)付近の傾きを30%に抑え、両端近くでのみ100%へ遷移する不感帯プラトー
            float plateauMid = 0.5f + 0.5f * (0.30f * centeredMid + 0.70f * centeredMid * centeredMid * centeredMid);
            _input.mouse_mid_norm = Mathf.Clamp01(plateauMid);

            // Lean-in line より上 (dy > leanInOffset): 0 -> 1
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