using UnityEngine;
using SkyTechSimulator;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class SkyTechGameController : MonoBehaviour
{
    public enum MouseHeightControlMode
    {
        [Tooltip("画面の上下中央を基準高さ(標準スタンス)とし、上へ動かすと伸長(l_max)、下へ動かすと短縮(l_min)")]
        VerticalSignedFromCenter,

        [Tooltip("画面の中心座標(X, Y)からの2D距離で制御（中心に近いほど伸長 l_max、離れるほど短縮 l_min）")]
        DistanceFromScreenCenter,

        [Tooltip("画面の上下中央(Y)からの垂直距離 |dy| で制御（中央の高さに近いほど伸長 l_max、上下に離れるほど短縮 l_min）")]
        VerticalDistanceFromCenter
    }

    [Header("=== 設定アセット ===")]
    public SkyTechSimConfig Config;

    [Header("=== アシスト・マウス操作設定 ===")]
    public bool AutoAngulationAssist = false;
    public bool UseMouseControl = true;

    [Tooltip("画面中央基準での高さ（脚の伸縮）制御モード")]
    public MouseHeightControlMode HeightControlMode = MouseHeightControlMode.VerticalSignedFromCenter;

    [Tooltip("[VerticalSignedFromCenter用] 画面中央から上方向へこの距離(m)動かすと最大伸長(l_max)")]
    public float MouseExtendUpRange = 0.55f;

    [Tooltip("[VerticalSignedFromCenter用] 画面中央から下方向へこの距離(m)動かすと最小屈曲(l_min)")]
    public float MouseFlexDownRange = 0.85f;

    [Tooltip("[Distance系モード用] 画面中央からこの距離(m)以下に近づくと脚を最大長(l_max)まで伸長")]
    public float MouseFullExtendDist = 0.40f;

    [Tooltip("[Distance系モード用] 画面中央からこの距離(m)以上に離れると脚を最小長(l_min)まで短縮")]
    public float MouseFullFlexDist = 1.45f;

    [Tooltip("画面の左右中央線上でのチャタリング防止用微小デッドゾーン(m)")]
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

    /// <summary>Q/Eキーで増減して固定されている股関節角度 (deg)</summary>
    public float TargetLegRollDeg => _lockedHipDeg;

    private SimState _state;
    private SimInput _input;
    private float _lockedHipDeg = 0f;
    private float _crashFrozenTimer = 0f;
    private float _timeSinceLastSwitch = 0f;
    private float _lastEdgeSign = 0f;
    private bool _turnInvalidatedByRail = false;
    private float _skidDurationInTurn = 0f;
    private bool _isSteerNeutralActive = false;

    private float _leftRollHoldTimer = 0f;
    private float _leftRollRepeatTimer = 0f;
    private float _rightRollHoldTimer = 0f;
    private float _rightRollRepeatTimer = 0f;

    private const float RollRepeatInitialDelay = 0.28f;
    private const float RollRepeatInterval = 0.08f;
    private const float RollDegreeStep = 2.0f;

    private const float TurnSwitchMinTime = 0.45f;
    private const float TurnSwitchMinPeakDeg = 15.0f;
    private const float CrashFreezeDuration = 1.5f;

    private void Awake()
    {
        if (Config == null)
        {
            Config = ScriptableObject.CreateInstance<SkyTechSimConfig>();
        }
    }

    private void Start()
    {
        ResetSimulation();
    }

    private void Update()
    {
        HandleKeyboardInput();
    }

    private void FixedUpdate()
    {
        if (IsCrashFrozen)
        {
            _crashFrozenTimer -= Time.fixedDeltaTime;
            if (_crashFrozenTimer <= 0f)
            {
                IsCrashFrozen = false;
                ResetSimulation();
            }
            return;
        }

        SkyTechPhysicsSolver.Step(ref _state, in _input, Config, Time.fixedDeltaTime);

        if (_state.gripState == SkiGripState.HipCrash || _state.gripState == SkiGripState.BootOut)
        {
            TriggerCrash();
            return;
        }

        UpdateTurnTelemetry();
        CheckTurnSwitch();
    }

    private void HandleKeyboardInput()
    {
        if (IsKeyPressedThisFrame(KeyCode.R, KeyCodeToKey(KeyCode.R)))
        {
            ResetSimulation();
            return;
        }

        if (IsKeyPressedThisFrame(KeyCode.Tab, KeyCodeToKey(KeyCode.Tab)))
        {
            AutoAngulationAssist = !AutoAngulationAssist;
        }

        if (IsCrashFrozen)
        {
            _input.u_ext_norm = 0f;
            _input.u_flex_norm = 0f;
            _input.l1_target = 0f;
            _input.phi_lock_cmd = 0f;
            _input.phi_steer_cmd = 0f;
            _input.steer_neutral_cmd = false;
            _input.w_reset = false;
            _isSteerNeutralActive = false;
            return;
        }

        bool qDown = IsKeyPressedThisFrame(KeyCode.Q, KeyCodeToKey(KeyCode.Q));
        bool qHeld = IsKeyHeld(KeyCode.Q, KeyCodeToKey(KeyCode.Q));

        bool eDown = IsKeyPressedThisFrame(KeyCode.E, KeyCodeToKey(KeyCode.E));
        bool eHeld = IsKeyHeld(KeyCode.E, KeyCodeToKey(KeyCode.E));

        bool wDown = IsKeyPressedThisFrame(KeyCode.W, KeyCodeToKey(KeyCode.W));
        bool wHeld = IsKeyHeld(KeyCode.W, KeyCodeToKey(KeyCode.W));

        bool resetDown = wDown
                      || IsKeyPressedThisFrame(KeyCode.X, KeyCodeToKey(KeyCode.X))
                      || IsKeyPressedThisFrame(KeyCode.C, KeyCodeToKey(KeyCode.C))
                      || (qDown && eHeld) || (eDown && qHeld);
        bool resetHeld = wHeld
                      || IsKeyHeld(KeyCode.X, KeyCodeToKey(KeyCode.X))
                      || IsKeyHeld(KeyCode.C, KeyCodeToKey(KeyCode.C))
                      || (qHeld && eHeld);

        if (resetDown || resetHeld)
        {
            _lockedHipDeg = 0.0f;
            _input.phi_steer_cmd = 0.0f;
            _input.steer_neutral_cmd = false;
            _input.w_reset = true;
            _isSteerNeutralActive = false;
            _leftRollHoldTimer = 0f;
            _leftRollRepeatTimer = 0f;
            _rightRollHoldTimer = 0f;
            _rightRollRepeatTimer = 0f;
        }
        else
        {
            _input.w_reset = false;

            if (qDown)
            {
                float step = (_lockedHipDeg > 0.001f) ? (RollDegreeStep * 2.0f) : RollDegreeStep;
                float nextDeg = _lockedHipDeg - step;
                if (_lockedHipDeg > 0.001f && nextDeg < 0.0f)
                {
                    nextDeg = 0.0f;
                }
                _lockedHipDeg = Mathf.Clamp(nextDeg, -75.0f, 75.0f);
                _leftRollHoldTimer = 0f;
                _leftRollRepeatTimer = 0f;
            }
            else if (qHeld)
            {
                _leftRollHoldTimer += Time.deltaTime;
                if (_leftRollHoldTimer >= RollRepeatInitialDelay)
                {
                    _leftRollRepeatTimer += Time.deltaTime;
                    if (_leftRollRepeatTimer >= RollRepeatInterval)
                    {
                        float step = (_lockedHipDeg > 0.001f) ? (RollDegreeStep * 2.0f) : RollDegreeStep;
                        float nextDeg = _lockedHipDeg - step;
                        if (_lockedHipDeg > 0.001f && nextDeg < 0.0f)
                        {
                            nextDeg = 0.0f;
                        }
                        _lockedHipDeg = Mathf.Clamp(nextDeg, -75.0f, 75.0f);
                        _leftRollRepeatTimer = 0f;
                    }
                }
            }
            else
            {
                _leftRollHoldTimer = 0f;
                _leftRollRepeatTimer = 0f;
            }

            if (eDown)
            {
                float step = (_lockedHipDeg < -0.001f) ? (RollDegreeStep * 2.0f) : RollDegreeStep;
                float nextDeg = _lockedHipDeg + step;
                if (_lockedHipDeg < -0.001f && nextDeg > 0.0f)
                {
                    nextDeg = 0.0f;
                }
                _lockedHipDeg = Mathf.Clamp(nextDeg, -75.0f, 75.0f);
                _rightRollHoldTimer = 0f;
                _rightRollRepeatTimer = 0f;
            }
            else if (eHeld)
            {
                _rightRollHoldTimer += Time.deltaTime;
                if (_rightRollHoldTimer >= RollRepeatInitialDelay)
                {
                    _rightRollRepeatTimer += Time.deltaTime;
                    if (_rightRollRepeatTimer >= RollRepeatInterval)
                    {
                        float step = (_lockedHipDeg < -0.001f) ? (RollDegreeStep * 2.0f) : RollDegreeStep;
                        float nextDeg = _lockedHipDeg + step;
                        if (_lockedHipDeg < -0.001f && nextDeg > 0.0f)
                        {
                            nextDeg = 0.0f;
                        }
                        _lockedHipDeg = Mathf.Clamp(nextDeg, -75.0f, 75.0f);
                        _rightRollRepeatTimer = 0f;
                    }
                }
            }
            else
            {
                _rightRollHoldTimer = 0f;
                _rightRollRepeatTimer = 0f;
            }
        }

        _input.phi_lock_cmd = _lockedHipDeg * Mathf.Deg2Rad;

        bool extPressed = IsKeyHeld(KeyCode.Space, KeyCodeToKey(KeyCode.Space))
                       || IsKeyHeld(KeyCode.UpArrow, KeyCodeToKey(KeyCode.UpArrow));

        bool flexPressed = IsKeyHeld(KeyCode.S, KeyCodeToKey(KeyCode.S));

#if ENABLE_INPUT_SYSTEM
        if (Gamepad.current != null)
        {
            if (!extPressed) extPressed = Gamepad.current.buttonSouth.isPressed;
            if (!flexPressed) flexPressed = Gamepad.current.buttonEast.isPressed;
        }
#endif

        Vector2 mouseWorld = Vector2.zero;
        Vector2 screenCenterWorld = Vector2.zero;
        bool hasMouseWorld = UseMouseControl && TryGetMouseWorldPosition(out mouseWorld, out screenCenterWorld);

        // --- 1. 脚の伸縮入力（キーボード優先、未押下時は画面中央基準で高さを制御） ---
        if (extPressed || flexPressed)
        {
            _input.l1_target = 0f;

            if (extPressed)
            {
                _input.u_ext_norm = Mathf.MoveTowards(_input.u_ext_norm, 1.0f, 45.0f * Time.deltaTime);

                if (!qHeld && !eHeld && _state.gripState != SkiGripState.Skidding && _state.snow_stress_ratio < 0.9f)
                {
                    _lockedHipDeg = Mathf.MoveTowards(_lockedHipDeg, 0f, 35.0f * Time.deltaTime);
                }
            }
            else
            {
                _input.u_ext_norm = 0f;
            }

            if (flexPressed)
            {
                _input.u_flex_norm = Mathf.MoveTowards(_input.u_flex_norm, 1.0f, 24.0f * Time.deltaTime);
            }
            else
            {
                _input.u_flex_norm = 0f;
            }
        }
        else if (hasMouseWorld && Config != null)
        {
            float dy = mouseWorld.y - screenCenterWorld.y;
            float neutralL1 = Mathf.Clamp(Config.l_max - Config.supportDropFromMaxMeters, Config.l_min, Config.l_max);

            float targetL1 = HeightControlMode switch
            {
                MouseHeightControlMode.VerticalSignedFromCenter =>
                    neutralL1 + Mathf.Clamp(dy / Mathf.Max(0.05f, dy >= 0f ? MouseExtendUpRange : MouseFlexDownRange), -1f, 1f)
                              * (dy >= 0f ? (Config.l_max - neutralL1) : (neutralL1 - Config.l_min)),

                MouseHeightControlMode.VerticalDistanceFromCenter =>
                    Mathf.Lerp(Config.l_max, Config.l_min, Mathf.InverseLerp(MouseFullExtendDist, MouseFullFlexDist, Mathf.Abs(dy))),

                _ =>
                    Mathf.Lerp(Config.l_max, Config.l_min, Mathf.InverseLerp(MouseFullExtendDist, MouseFullFlexDist, Vector2.Distance(mouseWorld, screenCenterWorld)))
            };

            _input.l1_target = targetL1;

            // 位置誤差から現在の速度(dl1)を差し引いてオーバーシュートを防ぎ、二乗特性(drive * |drive|)で中央付近を滑らかに鎮静化
            float dampedError = (targetL1 - _state.l1) - 0.08f * _state.dl1;
            float linDrive = Mathf.Clamp(dampedError / 0.12f, -1.0f, 1.0f);
            float smoothDrive = linDrive * Mathf.Abs(linDrive);

            // 上端(l_max)・下端(l_min)付近へ大きく動かしたときのみフル出力へ滑らかにブースト
            float topBoost    = Mathf.SmoothStep(0f, 1f, (targetL1 - (Config.l_max - 0.03f)) / 0.025f);
            float bottomBoost = Mathf.SmoothStep(0f, 1f, ((Config.l_min + 0.03f) - targetL1) / 0.025f);

            float extTarget  = Mathf.Max(Mathf.Max(0f, smoothDrive), topBoost);
            float flexTarget = Mathf.Max(Mathf.Max(0f, -smoothDrive), bottomBoost);

            _input.u_ext_norm  = Mathf.MoveTowards(_input.u_ext_norm,  extTarget,  35.0f * Time.deltaTime);
            _input.u_flex_norm = Mathf.MoveTowards(_input.u_flex_norm, flexTarget, 24.0f * Time.deltaTime);

            if (_input.u_ext_norm > 0.15f && !qHeld && !eHeld && _state.snow_stress_ratio < 0.9f)
            {
                _lockedHipDeg = Mathf.MoveTowards(_lockedHipDeg, 0f, 35.0f * Time.deltaTime);
            }
        }
        else
        {
            _input.l1_target = 0f;
            _input.u_ext_norm = 0f;
            _input.u_flex_norm = 0f;
        }

        // --- 2. 左右ステアリング入力（マウス横移動距離に応じたアナログ比例制御） ---
        float steer = 0f;
        bool kbLeft = IsKeyHeld(KeyCode.A, KeyCodeToKey(KeyCode.A)) || IsKeyHeld(KeyCode.LeftArrow, KeyCodeToKey(KeyCode.LeftArrow));
        bool kbRight = IsKeyHeld(KeyCode.D, KeyCodeToKey(KeyCode.D)) || IsKeyHeld(KeyCode.RightArrow, KeyCodeToKey(KeyCode.RightArrow));

        if (kbLeft || kbRight)
        {
            if (kbLeft) steer -= 1f;
            if (kbRight) steer += 1f;
        }
        else if (hasMouseWorld)
        {
            float dx = mouseWorld.x - screenCenterWorld.x;
            float absDx = Mathf.Abs(dx);

            if (absDx > MouseLineDeadzone)
            {
                float fullRange = (Config != null) ? Mathf.Max(0.15f, Config.mouseSteerFullRange) : 1.15f;
                float normSteer = Mathf.Clamp01((absDx - MouseLineDeadzone) / fullRange);
                // 少しの動きではマイルドに、大きく動かしたときに強く効くカーブ
                steer = Mathf.Sign(dx) * Mathf.Lerp(normSteer, normSteer * normSteer, 0.35f);
            }
        }

        bool downPressed = IsKeyPressedThisFrame(KeyCode.DownArrow, KeyCodeToKey(KeyCode.DownArrow));
        bool downHeld = IsKeyHeld(KeyCode.DownArrow, KeyCodeToKey(KeyCode.DownArrow));

        if (downPressed || downHeld)
        {
            _isSteerNeutralActive = true;
        }

        if (Mathf.Abs(steer) > 0.01f)
        {
            _isSteerNeutralActive = false;
        }

        if (_isSteerNeutralActive && !downHeld && Mathf.Abs(_state.u_shift_filtered) < 0.01f && Mathf.Abs(_state.phi_bend) < 0.01f)
        {
            _isSteerNeutralActive = false;
        }

        _input.phi_steer_cmd = -steer;
        _input.steer_neutral_cmd = _isSteerNeutralActive;
    }

    private static bool TryGetMouseWorldPosition(out Vector2 worldPos, out Vector2 screenCenterWorld)
    {
        worldPos = Vector2.zero;
        screenCenterWorld = Vector2.zero;

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
        catch
        {
            hasPos = false;
        }

#if ENABLE_INPUT_SYSTEM
        if (!hasPos)
        {
            var pointer = (Pointer.current != null) ? Pointer.current : (Pointer)Mouse.current;
            if (pointer != null)
            {
                Vector2 mp = pointer.position.ReadValue();
                if (!float.IsNaN(mp.x) && !float.IsNaN(mp.y))
                {
                    screenPos = new Vector3(mp.x, mp.y, 0f);
                    hasPos = true;

#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
                    float retinaFactor = 1.0f;
#if UNITY_EDITOR
                    retinaFactor = UnityEditor.EditorGUIUtility.pixelsPerPoint;
#else
                    if (Display.main.systemWidth > 0 && Display.main.renderingWidth > 0)
                    {
                        retinaFactor = (float)Display.main.renderingWidth / Display.main.systemWidth;
                    }
                    else if (Screen.dpi > 160f)
                    {
                        retinaFactor = 2.0f;
                    }
#endif
                    if (retinaFactor > 1.05f)
                    {
                        if (screenPos.x * retinaFactor <= cam.pixelWidth * 1.05f &&
                            screenPos.y * retinaFactor <= cam.pixelHeight * 1.05f)
                        {
                            screenPos.x *= retinaFactor;
                            screenPos.y *= retinaFactor;
                        }
                    }
#endif
                }
            }
        }
#endif

        if (!hasPos) return false;

        float depth = -cam.transform.position.z;
        screenPos.z = depth;
        Vector3 wp = cam.ScreenToWorldPoint(screenPos);
        worldPos = new Vector2(wp.x, wp.y);

        Vector3 centerScreen = new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, depth);
        Vector3 cp = cam.ScreenToWorldPoint(centerScreen);
        screenCenterWorld = new Vector2(cp.x, cp.y);

        return true;
    }

#if ENABLE_INPUT_SYSTEM
    private static Key KeyCodeToKey(KeyCode kc)
    {
        switch (kc)
        {
            case KeyCode.Space: return Key.Space;
            case KeyCode.W: return Key.W;
            case KeyCode.S: return Key.S;
            case KeyCode.UpArrow: return Key.UpArrow;
            case KeyCode.DownArrow: return Key.DownArrow;
            case KeyCode.A: return Key.A;
            case KeyCode.D: return Key.D;
            case KeyCode.LeftArrow: return Key.LeftArrow;
            case KeyCode.RightArrow: return Key.RightArrow;
            case KeyCode.Q: return Key.Q;
            case KeyCode.E: return Key.E;
            case KeyCode.R: return Key.R;
            case KeyCode.Tab: return Key.Tab;
            case KeyCode.X: return Key.X;
            case KeyCode.C: return Key.C;
            default: return Key.None;
        }
    }
#else
    private static int KeyCodeToKey(KeyCode kc) => 0;
#endif

    private static bool IsKeyHeld(KeyCode legacyKey,
#if ENABLE_INPUT_SYSTEM
        Key newKey
#else
        int dummy = 0
#endif
    )
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb != null && newKey != Key.None)
        {
            switch (newKey)
            {
                case Key.Space: if (kb.spaceKey.isPressed) return true; break;
                case Key.W: if (kb.wKey.isPressed) return true; break;
                case Key.S: if (kb.sKey.isPressed) return true; break;
                case Key.UpArrow: if (kb.upArrowKey.isPressed) return true; break;
                case Key.DownArrow: if (kb.downArrowKey.isPressed) return true; break;
                case Key.A: if (kb.aKey.isPressed) return true; break;
                case Key.D: if (kb.dKey.isPressed) return true; break;
                case Key.LeftArrow: if (kb.leftArrowKey.isPressed) return true; break;
                case Key.RightArrow: if (kb.rightArrowKey.isPressed) return true; break;
                case Key.Q: if (kb.qKey.isPressed) return true; break;
                case Key.E: if (kb.eKey.isPressed) return true; break;
                case Key.R: if (kb.rKey.isPressed) return true; break;
                case Key.Tab: if (kb.tabKey.isPressed) return true; break;
                case Key.X: if (kb.xKey.isPressed) return true; break;
                case Key.C: if (kb.cKey.isPressed) return true; break;
            }
        }
#endif
        try { return Input.GetKey(legacyKey); }
        catch { return false; }
    }

    private static bool IsKeyPressedThisFrame(KeyCode legacyKey,
#if ENABLE_INPUT_SYSTEM
        Key newKey
#else
        int dummy = 0
#endif
    )
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb != null && newKey != Key.None)
        {
            switch (newKey)
            {
                case Key.Space: if (kb.spaceKey.wasPressedThisFrame) return true; break;
                case Key.W: if (kb.wKey.wasPressedThisFrame) return true; break;
                case Key.S: if (kb.sKey.wasPressedThisFrame) return true; break;
                case Key.UpArrow: if (kb.upArrowKey.wasPressedThisFrame) return true; break;
                case Key.DownArrow: if (kb.downArrowKey.wasPressedThisFrame) return true; break;
                case Key.A: if (kb.aKey.wasPressedThisFrame) return true; break;
                case Key.D: if (kb.dKey.wasPressedThisFrame) return true; break;
                case Key.LeftArrow: if (kb.leftArrowKey.wasPressedThisFrame) return true; break;
                case Key.RightArrow: if (kb.rightArrowKey.wasPressedThisFrame) return true; break;
                case Key.Q: if (kb.qKey.wasPressedThisFrame) return true; break;
                case Key.E: if (kb.eKey.wasPressedThisFrame) return true; break;
                case Key.R: if (kb.rKey.wasPressedThisFrame) return true; break;
                case Key.Tab: if (kb.tabKey.wasPressedThisFrame) return true; break;
                case Key.X: if (kb.xKey.wasPressedThisFrame) return true; break;
                case Key.C: if (kb.cKey.wasPressedThisFrame) return true; break;
            }
        }
#endif
        try { return Input.GetKeyDown(legacyKey); }
        catch { return false; }
    }

    private void UpdateTurnTelemetry()
    {
        _timeSinceLastSwitch += Time.fixedDeltaTime;

        float currentEdgeDeg = Mathf.Abs(_state.theta1) * Mathf.Rad2Deg;
        float currentCoMDeg = Mathf.Abs(_state.Psi_CoM) * Mathf.Rad2Deg;

        CurrentTurnPeakEdgeDeg = Mathf.Max(CurrentTurnPeakEdgeDeg, currentEdgeDeg);
        CurrentTurnPeakCoMDeg = Mathf.Max(CurrentTurnPeakCoMDeg, currentCoMDeg);

        if (_state.gripState == SkiGripState.RailLimitHit)
        {
            _turnInvalidatedByRail = true;
        }

        if (_state.gripState == SkiGripState.Skidding)
        {
            _skidDurationInTurn += Time.fixedDeltaTime;
        }
    }

    private void CheckTurnSwitch()
    {
        float currentSign = Mathf.Sign(_state.theta1);

        if (_lastEdgeSign == 0f)
        {
            if (Mathf.Abs(_state.theta1) > 0.01f)
            {
                _lastEdgeSign = currentSign;
            }
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
        if (!_turnInvalidatedByRail)
        {
            TurnCount++;
            ComboCount++;

            float baseScore = CurrentTurnPeakEdgeDeg;
            bool hasAngulationBonus = CurrentTurnPeakEdgeDeg > (CurrentTurnPeakCoMDeg + 3.0f);
            if (hasAngulationBonus)
            {
                baseScore *= 1.25f;
            }

            float skidMultiplier = Mathf.Clamp01(1.0f - (_skidDurationInTurn * 0.20f));
            baseScore *= skidMultiplier;

            float comboMultiplier = 1.0f + (ComboCount - 1) * 0.10f;
            TotalScore += baseScore * comboMultiplier;

            if (CurrentTurnPeakEdgeDeg >= 65.0f)
                LastTurnEvaluation = hasAngulationBonus ? "WORLD CUP CARVE!!" : "WORLD CUP SPEED!";
            else if (CurrentTurnPeakEdgeDeg >= 50.0f)
                LastTurnEvaluation = hasAngulationBonus ? "DEEP CARVE!" : "SOLID EDGE!";
            else if (CurrentTurnPeakEdgeDeg >= 35.0f)
                LastTurnEvaluation = "GOOD TURN";
            else
                LastTurnEvaluation = "SHALLOW TURN";
        }
        else
        {
            ComboCount = 0;
            LastTurnEvaluation = "RAIL HIT! (0 PT)";
        }

        LastTurnPeakEdgeDeg = CurrentTurnPeakEdgeDeg;
        LastTurnPeakCoMDeg = CurrentTurnPeakCoMDeg;
        CurrentTurnPeakEdgeDeg = 0f;
        CurrentTurnPeakCoMDeg = 0f;
        _turnInvalidatedByRail = false;
        _skidDurationInTurn = 0f;
        _timeSinceLastSwitch = 0f;
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
        _lockedHipDeg = 0f;
        _isSteerNeutralActive = false;
        _input = new SimInput
        {
            u_ext_norm = 0f,
            u_flex_norm = 0f,
            l1_target = 0f,
            phi_lock_cmd = 0f,
            phi_steer_cmd = 0f,
            steer_neutral_cmd = false,
            w_reset = false
        };

        CurrentTurnPeakEdgeDeg = 0f;
        CurrentTurnPeakCoMDeg = 0f;
        _turnInvalidatedByRail = false;
        _skidDurationInTurn = 0f;
        _timeSinceLastSwitch = 0f;
        _lastEdgeSign = 0f;
        IsCrashFrozen = false;
        _crashFrozenTimer = 0f;
    }
}