using UnityEngine;
using SkyTechSimulator;

[RequireComponent(typeof(SkyTechGameController))]
public class SkyTechVisualizer : MonoBehaviour
{
    [Header("=== 画面中央基準・頂点誘導ガイド表示設定 ===")]
    public bool ShowCenterCrosshair = true;
    public bool ShowPuppetApexGuide = true;
    public float GuideExtendUpRange = 2.80f;
    [Tooltip("一番下の基準線（リーンアウト下限）の深さ [m]")]
    public float GuideFlexDownRange = 1.75f;

    [Header("=== 理想のマウスリード目印（ペースメーカー）設定 ===")]
    public bool ShowIdealMouseLead = true;
    [Tooltip("目標とするレール幅の使用率（0.70 = レール全幅の70%を使ってターン）")]
    [Range(0.40f, 0.90f)] public float TargetRailUsageRatio = 0.70f;
    [Tooltip("極限ディープターンの基準目標ロール角（度） -> 70度")]
    [Range(60.0f, 72.0f)] public float IdealDeepTurnEdgeDeg = 70.0f;
    [Tooltip("マウスの動きをどれだけゆっくり滑らかにリードするか（周期スケール倍率）")]
    [Range(1.00f, 1.80f)] public float SlowCarvePeriodScale = 1.35f;
    [Tooltip("位相をどれだけ先回りしてリードするか（ラジアン）")]
    public float LeadPhaseAdvanceRad = 0.30f;

    [Header("=== カメラ水平ロック（逆回転補正）設定 ===")]
    [Tooltip("iPhoneを傾けた際にカメラを逆回転させ、画面内の鉛直線を実空間の重力鉛直と一致させる（※falseで画面をロールさせず水平固定）")]
    public bool EnableCameraHorizonLock = false;
    [Tooltip("iPhone傾きに対するカメラ逆回転の追従比率（0.70〜1.00推奨）")]
    [Range(0.0f, 1.0f)] public float CameraHorizonLockRatio = 0.75f;

    [Header("=== 下肢・上体ターゲット角度線表示設定 ===")]
    [Tooltip("iPhoneチルト(下肢目標ロール角)およびAirPods/体幹制御(上体目標角)のターゲット線を画面内に表示する")]
    public bool ShowTargetAngleGuides = true;
    [Tooltip("下肢ターゲット線の太さ")]
    public float TargetLegLineWidth = 0.038f;
    [Tooltip("上体ターゲット線の太さ")]
    public float TargetTrunkLineWidth = 0.038f;
    [Tooltip("目標姿勢の頭部・股関節リングを表示する")]
    public bool ShowTargetJointMarkers = true;

    [Header("=== レール画面表示・カメラフィッティング設定 ===")]
    [Tooltip("画面端とレールの間の安全余白（メートル: 0.15〜0.30推奨。横幅いっぱいにフィットさせます）")]
    [Range(0.05f, 0.60f)] public float RailSideMargin = 0.20f;
    [Tooltip("レール(y=0)の画面下端からの垂直マージン比率（画面高さに対する比率: 0.15〜0.25推奨）")]
    [Range(0.10f, 0.35f)] public float RailBottomMarginRatio = 0.18f;

    private Camera _mainCamera;

    private float _pacemakerPhase = 0f;
    private float _smoothedTargetDeg = 70.0f;
    private float _currentTurnMaxAbsX = 0f;
    private float _lastTurnPeakAbsX = 0f;
    private float _prevVelSign = 0f;

    private void EnforceVisualizerRanges()
    {
        if (GuideExtendUpRange < 2.0f) GuideExtendUpRange = 2.80f;
        if (GuideFlexDownRange < 1.2f) GuideFlexDownRange = 1.75f;
    }

    private void OnValidate() => EnforceVisualizerRanges();

    private SkyTechGameController _controller;
    private Material _lineMaterial;

    private LineRenderer _centerHorizLine, _leanInThresholdLine, _straightThresholdLine, _centerVertLine, _centerReticleCircle;
    private LineRenderer _heightMaxTickLine, _heightMinTickLine, _mouseGuideLine, _mouseCursorCircle;
    private LineRenderer _puppetStringLine, _apexTargetCircle;
    private LineRenderer _idealLeadOuterRing, _idealLeadInnerCore, _idealLeadOrbitLine, _idealLeadTetherLine;
    private LineRenderer _railLine, _absorberLeft, _absorberRight, _carriageLine, _skiBoardLine;
    private LineRenderer _railTargetLeftTick, _railTargetRightTick; // 追加: レール幅70%の目標ライン目印
    private LineRenderer _legLine, _trunkLine, _hipJointCircle, _headCircle;
    private LineRenderer _targetLegLine, _targetTrunkLine, _currentToTargetTrunkLine, _targetHipCircle, _targetHeadCircle; // 下肢・上体のターゲット角度指示線
    private LineRenderer _comCircle, _comLine, _lowerComCircle, _upperComCircle, _twoComDistLine;
    private LineRenderer _forceLine, _ghostPeakLine;

    private Texture2D _panelBgTex, _barBgTex, _barFillTex, _badgeTex;
    private GUIStyle _panelStyle, _titleStyle, _labelStyle, _valueStyle, _badgeStyle, _guideStyle, _centerTagStyle, _btnStyle;
    private bool _guiInitialized;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void AutoInitializeIfMissing()
    {
        if (Object.FindAnyObjectByType<SkyTechGameController>() == null)
        {
            new GameObject("SkyTechSimulator").AddComponent<SkyTechVisualizer>();
            Debug.Log("<color=#58D68D>[SkyTechVisualizer] シーン内にSkyTechSimulatorを自動生成しました。Playを開始します！</color>");
        }
    }

    private void Awake()
    {
        EnforceVisualizerRanges();
        _controller = GetComponent<SkyTechGameController>();
        EnsureCameraSetup();

        Shader shader = Shader.Find("Universal Render Pipeline/2D/Sprite-Unlit-Default")
                     ?? Shader.Find("Universal Render Pipeline/Unlit")
                     ?? Shader.Find("Sprites/Default")
                     ?? Shader.Find("Unlit/Color")
                     ?? Shader.Find("Hidden/Internal-Colored");

        _lineMaterial = new Material(shader) { hideFlags = HideFlags.DontSave };
        SetupLineRenderers();
    }

    private void EnsureCameraSetup()
    {
        _mainCamera = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (_mainCamera == null)
        {
            _mainCamera = new GameObject("Main Camera") { tag = "MainCamera" }.AddComponent<Camera>();
        }

        _mainCamera.orthographic = true;
        _mainCamera.backgroundColor = new Color(0.063f, 0.082f, 0.125f);
        _mainCamera.clearFlags = CameraClearFlags.SolidColor;
        UpdateCameraFraming();
    }

    private void UpdateCamera()
    {
        if (_mainCamera == null) EnsureCameraSetup();
        if (_mainCamera == null) return;

        UpdateCameraFraming();

        // iPhoneの傾き角の70%〜100%分だけカメラを逆回転させ、ゲーム内の重力鉛直を実空間の鉛直と一致させる
        float ratio = EnableCameraHorizonLock ? CameraHorizonLockRatio : 0f;
        float camCounterRoll = (_controller != null && _controller.UseIosSensorControl) ? -_controller.CurrentDeviceRollDeg * ratio : 0f;
        _mainCamera.transform.rotation = Quaternion.Euler(0f, 0f, camCounterRoll);
    }

    private void UpdateCameraFraming()
    {
        if (_mainCamera == null) return;
        _mainCamera.orthographic = true;

        float halfRail = (_controller != null && _controller.Config != null) ? _controller.Config.W_rail * 0.5f : 3.60f;
        float aspect = (_mainCamera.pixelWidth > 0 && _mainCamera.pixelHeight > 0)
            ? (float)_mainCamera.pixelWidth / _mainCamera.pixelHeight
            : ((Screen.width > 0 && Screen.height > 0) ? (float)Screen.width / Screen.height : (16f / 9f));

        // iPhoneのLandscape(横画面)時にレールが画面の横幅いっぱいにフィットするよう、
        // レール半幅＋ダンパー・端余白から orthographicSize を動的に算出
        float visibleHalfWidth = halfRail + Mathf.Max(0.05f, RailSideMargin);
        float targetOrthoSize = visibleHalfWidth / Mathf.Max(0.1f, aspect);

        _mainCamera.orthographicSize = targetOrthoSize;

        // レール(y=0)が画面下端寄りの安定した位置に収まるようカメラY位置を設定
        float bottomMargin = Mathf.Max(0.20f, targetOrthoSize * Mathf.Clamp(RailBottomMarginRatio, 0.10f, 0.35f));
        float targetCamY = targetOrthoSize - bottomMargin;

        Vector3 curPos = _mainCamera.transform.position;
        _mainCamera.transform.position = new Vector3(0f, targetCamY, curPos.z != 0f ? curPos.z : -10f);
    }

    private void Start()
    {
        EnsureCameraSetup();
        UpdateRailGeometry();
    }

    private void LateUpdate()
    {
        if (_controller == null || _controller.Config == null) return;
        SimState state = _controller.CurrentState;
        SimInput input = _controller.CurrentInput;
        UpdateCamera();
        UpdateRailGeometry();
        UpdateCenterCrosshairGeometry(input);
        UpdateDynamicGeometry(state, input, _controller.Config);
        UpdateIdealMouseLeadMarker(state, input, _controller.Config);
    }

    private void UpdateIdealMouseLeadMarker(in SimState state, in SimInput input, SkyTechSimConfig cfg)
    {
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (!ShowIdealMouseLead || cam == null || _controller.IsCrashFrozen)
        {
            _idealLeadOuterRing.positionCount = _idealLeadInnerCore.positionCount = 0;
            _idealLeadOrbitLine.positionCount = _idealLeadTetherLine.positionCount = 0;
            return;
        }

        float dt = Mathf.Max(1e-4f, Time.deltaTime);

        // 1. レールの片側幅の70%を目標ストローク振幅 X_target とする
        float halfRail = cfg.W_rail * 0.5f;
        float usageRatio = Mathf.Clamp(TargetRailUsageRatio, 0.40f, 0.90f);
        float targetAmpRailX = halfRail * usageRatio;

        _currentTurnMaxAbsX = Mathf.Max(_currentTurnMaxAbsX, Mathf.Abs(state.x_c));
        float velSign = Mathf.Abs(state.v_c) > 0.15f ? Mathf.Sign(state.v_c) : _prevVelSign;
        if (_prevVelSign != 0f && velSign != _prevVelSign)
        {
            if (_currentTurnMaxAbsX > 0.5f)
            {
                _lastTurnPeakAbsX = Mathf.Lerp(
                    _lastTurnPeakAbsX > 0.1f ? _lastTurnPeakAbsX : _currentTurnMaxAbsX,
                    _currentTurnMaxAbsX,
                    0.5f
                );
            }
            _currentTurnMaxAbsX = Mathf.Abs(state.x_c);
        }
        if (velSign != 0f) _prevVelSign = velSign;

        // 2. 目標ロール角 70度 に向けたスロー＆ディープ周期の物理計算
        float baseDeepDeg = Mathf.Clamp(IdealDeepTurnEdgeDeg, 60.0f, 72.0f);
        float amplitudeErrorRatio = (_lastTurnPeakAbsX > 0.2f)
            ? ((targetAmpRailX - _lastTurnPeakAbsX) / Mathf.Max(0.5f, targetAmpRailX))
            : 0f;

        float desiredDeepDeg = Mathf.Clamp(baseDeepDeg + amplitudeErrorRatio * 6.0f, 65.0f, 72.0f);
        _smoothedTargetDeg = Mathf.Lerp(_smoothedTargetDeg, desiredDeepDeg, 2.0f * dt);

        // ロール角70度での強大な遠心力（tan(70°) ≒ 2.75G）に基づくダイナミック周期
        float deepPeakG = Mathf.Clamp(Mathf.Tan(_smoothedTargetDeg * Mathf.Deg2Rad) * 0.92f, 1.8f, 3.2f);
        float omegaIdeal = Mathf.Sqrt((deepPeakG * cfg.g) / Mathf.Max(1.0f, targetAmpRailX)) / Mathf.Max(1.0f, SlowCarvePeriodScale);
        float idealCyclePeriod = Mathf.Clamp((Mathf.PI * 2.0f) / omegaIdeal, 3.2f, 5.8f);
        float omegaRhythm = (Mathf.PI * 2.0f) / idealCyclePeriod;

        _pacemakerPhase += omegaRhythm * dt;

        float normX = Mathf.Clamp(-state.x_c / Mathf.Max(0.5f, targetAmpRailX), -1.25f, 1.25f);
        float normV = Mathf.Clamp(-state.v_c / Mathf.Max(0.5f, targetAmpRailX * omegaRhythm), -1.25f, 1.25f);
        if (Mathf.Abs(state.x_c) > 0.20f || Mathf.Abs(state.v_c) > 0.25f)
        {
            float carriagePhase = Mathf.Atan2(normX, normV);
            float phaseDiff = Mathf.DeltaAngle(_pacemakerPhase * Mathf.Rad2Deg, carriagePhase * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            _pacemakerPhase += phaseDiff * Mathf.Clamp01(1.2f * dt);
        }

        float leadPhase = _pacemakerPhase + LeadPhaseAdvanceRad;

        // 3. 70度ターンを実現するマウス座標の算出（最大振幅へフル活用）
        Vector3 centerWorld3 = cam.ScreenToWorldPoint(new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, -cam.transform.position.z));
        Vector2 centerWorld = new Vector2(centerWorld3.x, centerWorld3.y + _controller.MouseCenterOffsetY);

        float maxRollDeg = Mathf.Min(72.0f, cfg.theta_bootOutDeg - 2.0f);
        float targetSteerNorm = Mathf.Clamp01(_smoothedTargetDeg / Mathf.Max(20.0f, maxRollDeg));
        float fullRangeX = Mathf.Max(2.80f, cfg.mouseSteerFullRange);
        float idealAmpX = _controller.MouseLineDeadzone + targetSteerNorm * fullRangeX;

        // X座標: 端でしっかりと70度のタメを作るワイドホールド波形
        float rawSin = Mathf.Sin(leadPhase);
        float holdSin = rawSin * (1.18f - 0.18f * rawSin * rawSin);
        float idealX = centerWorld.x + idealAmpX * holdSin;

        // Y座標: 70度の深い内倒・外傾を支えるワイド上下スパン
        float leanInSpan   = Mathf.Max(2.00f, _controller.MouseStraightLineOffsetY);
        float bottomMargin = leanInSpan * 0.12f;
        float midBaseY     = leanInSpan * 0.48f;
        float verticalAmp  = leanInSpan * 0.38f;

        float idealOffsetY = Mathf.Max(bottomMargin, midBaseY + verticalAmp * Mathf.Cos(leadPhase * 2.0f));
        Vector3 leadPos = new Vector3(idealX, centerWorld.y + idealOffsetY, 0f);

        // 4. ガイド軌道およびパルスリングの描画
        const int orbitSegs = 56;
        _idealLeadOrbitLine.positionCount = orbitSegs + 1;
        for (int i = 0; i <= orbitSegs; i++)
        {
            float th = (i / (float)orbitSegs) * Mathf.PI * 2.0f;
            float sTh = Mathf.Sin(th);
            float hSin = sTh * (1.18f - 0.18f * sTh * sTh);
            float ox = centerWorld.x + idealAmpX * hSin;
            float oy = centerWorld.y + Mathf.Max(bottomMargin, midBaseY + verticalAmp * Mathf.Cos(th * 2.0f));
            _idealLeadOrbitLine.SetPosition(i, new Vector3(ox, oy, 0f));
        }

        float slowPulse = 1.0f + 0.14f * Mathf.Sin(Time.time * 4.0f);
        SetCirclePositions(_idealLeadOuterRing, leadPos, 0.076f * slowPulse, 24);
        SetCirclePositions(_idealLeadInnerCore, leadPos, 0.025f, 16);

        if (input.has_mouse_world)
        {
            Vector3 mousePos = new Vector3(input.mouse_world.x, input.mouse_world.y, 0f);
            SetLine(_idealLeadTetherLine, mousePos, leadPos);

            float dist = Vector2.Distance(input.mouse_world, new Vector2(leadPos.x, leadPos.y));
            float proximity = Mathf.Clamp01(1.0f - dist / 0.95f);
            Color ringCol = Color.Lerp(new Color(0.20f, 0.90f, 1.00f, 0.85f), new Color(0.10f, 1.00f, 0.45f, 0.95f), proximity);
            SetColor(_idealLeadOuterRing, ringCol);
        }
        else
        {
            _idealLeadTetherLine.positionCount = 0;
        }
    }

    private void SetupLineRenderers()
    {
        _centerHorizLine       = CreateLine("CenterHorizGuide",  0.014f, new Color(0.55f, 0.75f, 0.95f, 0.25f), -4);
        _leanInThresholdLine   = CreateLine("LeanInGuideLine",   0.014f, new Color(1.00f, 0.45f, 0.75f, 0.32f), -4);
        _straightThresholdLine = CreateLine("StraightGuideLine", 0.016f, new Color(0.30f, 0.95f, 1.00f, 0.38f), -4);
        _centerVertLine        = CreateLine("CenterVertGuide",   0.014f, new Color(0.55f, 0.75f, 0.95f, 0.28f), -4);
        _heightMaxTickLine     = CreateLine("HeightMaxTick",     0.016f, new Color(0.35f, 0.95f, 0.60f, 0.35f), -3);
        _heightMinTickLine     = CreateLine("HeightMinTick",     0.018f, new Color(1.00f, 0.60f, 0.25f, 0.45f), -3);
        _centerReticleCircle   = CreateLine("CenterReticle",     0.015f, new Color(0.75f, 0.88f, 1.00f, 0.40f), -2);
        _mouseGuideLine        = CreateLine("MouseGuideLine",    0.012f, new Color(1.00f, 0.90f, 0.40f, 0.30f), -1);

        _railLine            = CreateLine("Rail",           0.035f, new Color(0.40f, 0.45f, 0.55f, 0.8f), 0);
        _railTargetLeftTick  = CreateLine("RailTarget70L",  0.022f, new Color(0.20f, 0.95f, 0.85f, 0.65f), 1);
        _railTargetRightTick = CreateLine("RailTarget70R",  0.022f, new Color(0.20f, 0.95f, 0.85f, 0.65f), 1);
        _absorberLeft        = CreateLine("AbsorberL",      0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _absorberRight       = CreateLine("AbsorberR",      0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _ghostPeakLine   = CreateLine("GhostPeak",      0.020f, new Color(0.3f, 0.8f, 1.0f, 0.25f),   2);
        _carriageLine    = CreateLine("Carriage",       0.030f, new Color(0.75f, 0.80f, 0.90f, 0.9f), 3);
        _skiBoardLine    = CreateLine("SkiBoard",       0.045f, new Color(0.90f, 0.95f, 1.00f, 1.0f), 4);
        _legLine         = CreateLine("LegSegment",     0.065f, Color.cyan,                           5);
        _trunkLine       = CreateLine("TrunkSegment",   0.075f, new Color(0.9f, 0.9f, 0.95f, 1.0f),   6);
        _hipJointCircle  = CreateLine("HipJoint",       0.025f, new Color(1.0f, 0.8f, 0.2f, 1.0f),    7);
        _headCircle      = CreateLine("HeadCircle",     0.030f, new Color(1.0f, 0.85f, 0.7f, 1.0f),   7);

        // 下肢・上体のターゲット角度指示線（iPhoneチルト / AirPods首かしげ 目標姿勢）
        _targetLegLine            = CreateLine("TargetLegLine",            TargetLegLineWidth,   new Color(0.15f, 0.95f, 0.55f, 0.85f), 5);
        _targetTrunkLine          = CreateLine("TargetTrunkLine",          TargetTrunkLineWidth, new Color(1.00f, 0.85f, 0.20f, 0.85f), 6);
        _currentToTargetTrunkLine = CreateLine("CurrentToTargetTrunkLine", 0.022f,               new Color(1.00f, 0.85f, 0.20f, 0.50f), 6);
        _targetHipCircle          = CreateLine("TargetHipCircle",          0.022f,               new Color(0.15f, 0.95f, 0.55f, 0.75f), 7);
        _targetHeadCircle         = CreateLine("TargetHeadCircle",         0.026f,               new Color(1.00f, 0.85f, 0.20f, 0.80f), 8);

        _twoComDistLine  = CreateLine("TwoCoMLine",     0.015f, new Color(1.0f, 1.0f, 1.0f, 0.45f),   8);
        _comLine         = CreateLine("CoMLine",        0.015f, new Color(1.0f, 0.95f, 0.2f, 0.45f),  8);
        _lowerComCircle  = CreateLine("LowerCoMCircle", 0.025f, new Color(0.0f, 0.90f, 1.0f, 1.0f),   9);
        _upperComCircle  = CreateLine("UpperCoMCircle", 0.042f, new Color(1.0f, 0.20f, 0.50f, 1.0f),  9);
        _comCircle       = CreateLine("CoMCircle",      0.025f, new Color(1.0f, 0.92f, 0.15f, 1.0f),  10);
        _forceLine       = CreateLine("ForceVector",    0.040f, Color.green,                          11);

        _puppetStringLine  = CreateLine("PuppetStringLine", 0.014f, new Color(1.0f, 0.85f, 0.45f, 0.55f), 12);
        _apexTargetCircle  = CreateLine("ApexTargetCircle", 0.016f, new Color(0.45f, 1.0f, 0.75f, 0.65f), 12);
        _mouseCursorCircle = CreateLine("MouseCursor",      0.018f, new Color(1.0f, 0.92f, 0.35f, 0.75f), 13);

        _idealLeadOrbitLine = CreateLine("IdealLeadOrbit",  0.010f, new Color(0.20f, 0.95f, 1.00f, 0.18f), 11);
        _idealLeadTetherLine= CreateLine("IdealLeadTether", 0.012f, new Color(0.30f, 1.00f, 0.85f, 0.40f), 12);
        _idealLeadOuterRing = CreateLine("IdealLeadOuter",  0.022f, new Color(0.20f, 1.00f, 0.80f, 0.90f), 14);
        _idealLeadInnerCore = CreateLine("IdealLeadCore",   0.016f, new Color(1.00f, 1.00f, 1.00f, 0.95f), 15);
    }

    private LineRenderer CreateLine(string objName, float width, Color color, int sortingOrder)
    {
        GameObject child = new GameObject($"_Vis_{objName}");
        child.transform.SetParent(transform, false);

        LineRenderer lr = child.AddComponent<LineRenderer>();
        lr.material = _lineMaterial;
        lr.useWorldSpace = true;
        lr.startWidth = lr.endWidth = width;
        lr.startColor = lr.endColor = color;
        lr.sortingLayerName = "Default";
        lr.sortingOrder = 100 + sortingOrder;
        lr.numCapVertices = lr.numCornerVertices = 4;
        return lr;
    }

    private static void SetLine(LineRenderer lr, Vector3 a, Vector3 b)
    {
        lr.positionCount = 2;
        lr.SetPosition(0, a);
        lr.SetPosition(1, b);
    }

    private static void SetColor(LineRenderer lr, Color c) => lr.startColor = lr.endColor = c;

    private void UpdateRailGeometry()
    {
        float halfRail = _controller.Config != null ? _controller.Config.W_rail * 0.5f : 3.60f;
        SetLine(_railLine, new Vector3(-halfRail, 0f, 0f), new Vector3(halfRail, 0f, 0f));
        SetBoxPositions(_absorberLeft, new Vector3(-halfRail - 0.05f, 0f, 0f), new Vector2(0.10f, 0.14f));
        SetBoxPositions(_absorberRight, new Vector3(halfRail + 0.05f, 0f, 0f), new Vector2(0.10f, 0.14f));

        if (_railTargetLeftTick != null && _railTargetRightTick != null)
        {
            if (ShowIdealMouseLead)
            {
                float targetX = halfRail * TargetRailUsageRatio;
                float tickHalfH = 0.08f;
                SetLine(_railTargetLeftTick, new Vector3(-targetX, -tickHalfH, 0f), new Vector3(-targetX, tickHalfH, 0f));
                SetLine(_railTargetRightTick, new Vector3(targetX, -tickHalfH, 0f), new Vector3(targetX, tickHalfH, 0f));
            }
            else
            {
                _railTargetLeftTick.positionCount = _railTargetRightTick.positionCount = 0;
            }
        }
    }

    private void UpdateCenterCrosshairGeometry(in SimInput input)
    {
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (!ShowCenterCrosshair || cam == null)
        {
            _centerHorizLine.positionCount = _leanInThresholdLine.positionCount = _straightThresholdLine.positionCount = _centerVertLine.positionCount = 0;
            _centerReticleCircle.positionCount = _heightMaxTickLine.positionCount = _heightMinTickLine.positionCount = 0;
            _mouseGuideLine.positionCount = _mouseCursorCircle.positionCount = 0;
            return;
        }

        Vector3 centerWorld3 = cam.ScreenToWorldPoint(new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, -cam.transform.position.z));
        Vector2 centerWorld = new Vector2(centerWorld3.x, centerWorld3.y + _controller.MouseCenterOffsetY);

        float orthoH = cam.orthographic ? cam.orthographicSize : 2.2f;
        float aspect = (cam.pixelWidth > 0 && cam.pixelHeight > 0) ? (float)cam.pixelWidth / cam.pixelHeight : (16f / 9f);
        float orthoW = orthoH * aspect;

        SetLine(_centerHorizLine, new Vector3(centerWorld.x - orthoW * 0.95f, centerWorld.y, 0f), new Vector3(centerWorld.x + orthoW * 0.95f, centerWorld.y, 0f));

        float leanInY = centerWorld.y + Mathf.Max(0.02f, _controller.MouseLeanInLineOffsetY);
        SetLine(_leanInThresholdLine, new Vector3(centerWorld.x - orthoW * 0.95f, leanInY, 0f), new Vector3(centerWorld.x + orthoW * 0.95f, leanInY, 0f));
        SetColor(_leanInThresholdLine, Color.Lerp(new Color(1.00f, 0.45f, 0.75f, 0.28f), new Color(1.00f, 0.30f, 0.65f, 0.85f), Mathf.Clamp01((input.mouse_mid_norm - 0.35f) / 0.35f)));

        // ★ ストレート内倒ライン（LEAN-IN 基準線の上方に新設描画）
        float straightOffset = Mathf.Max(_controller.MouseLeanInLineOffsetY + 0.1f, _controller.MouseStraightLineOffsetY);
        float straightY = centerWorld.y + straightOffset;
        SetLine(_straightThresholdLine, new Vector3(centerWorld.x - orthoW * 0.95f, straightY, 0f), new Vector3(centerWorld.x + orthoW * 0.95f, straightY, 0f));
        Color straightBaseCol = Color.Lerp(new Color(0.25f, 0.90f, 1.00f, 0.32f), new Color(0.35f, 1.00f, 0.60f, 0.90f), input.mouse_mid_norm >= 0.98f ? 1f : input.mouse_mid_norm * 0.7f);
        Color overLeanCol = new Color(1.00f, 0.35f, 0.25f, 0.95f);
        SetColor(_straightThresholdLine, Color.Lerp(straightBaseCol, overLeanCol, input.mouse_lean_in_norm));

        SetLine(_centerVertLine, new Vector3(centerWorld.x, centerWorld.y - orthoH * 0.95f, 0f), new Vector3(centerWorld.x, centerWorld.y + orthoH * 0.95f, 0f));
        SetColor(_centerVertLine, Color.Lerp(new Color(0.55f, 0.75f, 0.95f, 0.28f), new Color(0.40f, 0.90f, 1.00f, 0.50f), Mathf.Abs(input.phi_steer_cmd)));

        SetCirclePositions(_centerReticleCircle, new Vector3(centerWorld.x, centerWorld.y, 0f), 0.055f, 20);

        float tickHalfW = 0.45f;
        float upY = centerWorld.y + Mathf.Max(0.05f, GuideExtendUpRange);
        float downY = centerWorld.y - Mathf.Max(0.05f, GuideFlexDownRange);

        SetLine(_heightMaxTickLine, new Vector3(centerWorld.x - tickHalfW, upY, 0f), new Vector3(centerWorld.x + tickHalfW, upY, 0f));
        SetColor(_heightMaxTickLine, Color.Lerp(new Color(0.35f, 0.95f, 0.60f, 0.30f), new Color(0.35f, 1.00f, 0.55f, 0.85f), input.u_ext_norm));

        // 一番下の基準線（横幅を広げ、下限位置へ描画）
        SetLine(_heightMinTickLine, new Vector3(centerWorld.x - tickHalfW, downY, 0f), new Vector3(centerWorld.x + tickHalfW, downY, 0f));
        SetColor(_heightMinTickLine, Color.Lerp(new Color(1.00f, 0.60f, 0.25f, 0.30f), new Color(1.00f, 0.55f, 0.15f, 0.90f), input.mouse_lean_out_norm));

        if (input.has_mouse_world)
        {
            Vector2 mw = input.mouse_world;
            Vector3 mPos3 = new Vector3(mw.x, mw.y, 0f);
            SetCirclePositions(_mouseCursorCircle, mPos3, 0.042f, 16);

            _mouseGuideLine.positionCount = 3;
            _mouseGuideLine.SetPosition(0, new Vector3(centerWorld.x, centerWorld.y, 0f));
            _mouseGuideLine.SetPosition(1, new Vector3(mw.x, centerWorld.y, 0f));
            _mouseGuideLine.SetPosition(2, mPos3);

            float dx = mw.x - centerWorld.x;
            Color cursorCol = dx > _controller.MouseLineDeadzone
                ? new Color(0.35f, 0.90f, 1.00f, 0.80f)
                : (dx < -_controller.MouseLineDeadzone ? new Color(1.00f, 0.65f, 0.25f, 0.80f) : new Color(0.85f, 0.90f, 0.95f, 0.65f));
            SetColor(_mouseCursorCircle, cursorCol);
        }
        else
        {
            _mouseGuideLine.positionCount = _mouseCursorCircle.positionCount = 0;
        }
    }

    private void UpdateDynamicGeometry(in SimState state, in SimInput input, SkyTechSimConfig cfg)
    {
        Vector3 P = new Vector3(state.x_c, 0f, 0f);
        SetBoxPositions(_carriageLine, P + new Vector3(0f, -0.03f, 0f), new Vector2(0.24f, 0.06f));

        Vector3 skiTangent = new Vector3(Mathf.Cos(state.theta1), -Mathf.Sin(state.theta1), 0f);
        SetLine(_skiBoardLine, P - skiTangent * 0.16f, P + skiTangent * 0.16f);

        float sin1 = Mathf.Sin(state.theta1), cos1 = Mathf.Cos(state.theta1);
        Vector3 H = P + new Vector3(state.l1 * sin1, state.l1 * cos1, 0f);

        SetLine(_legLine, P, H);
        SetColor(_legLine, Color.Lerp(new Color(0.2f, 0.85f, 1.0f, 1f), new Color(1.0f, 0.45f, 0.1f, 1f), input.u_ext_norm));
        SetCirclePositions(_hipJointCircle, H, 0.040f, 16);

        float theta2 = state.theta1 - state.phi;
        Vector3 trunkVec = new Vector3(Mathf.Sin(theta2), Mathf.Cos(theta2), 0f);
        Vector3 U = H + trunkVec * cfg.l2;
        Vector3 HeadPos = H + trunkVec * (cfg.l2 * 1.55f);

        SetLine(_trunkLine, H, U);
        SetCirclePositions(_headCircle, HeadPos, 0.075f, 20);
        if (AirPodsMotionManager.IsActive)
        {
            SetColor(_headCircle, new Color(0.35f, 0.95f, 0.85f, 1.0f));
        }
        else
        {
            SetColor(_headCircle, new Color(1.0f, 0.85f, 0.7f, 1.0f));
        }

        UpdatePuppetApexGuide(H, HeadPos, state, input, cfg);

        Vector3 CoM1 = P + new Vector3(cfg.c1 * state.l1 * sin1, cfg.c1 * state.l1 * cos1, 0f);
        SetCirclePositions(_lowerComCircle, CoM1, 0.040f, 16);
        SetCirclePositions(_upperComCircle, U, 0.042f, 16);
        SetLine(_twoComDistLine, CoM1, U);

        Vector3 C = P + new Vector3(state.y_CoM, state.z_CoM, 0f);
        SetCirclePositions(_comCircle, C, 0.048f, 18);
        SetLine(_comLine, P, C);

        float mg = cfg.M * cfg.g;
        float forceLen = (mg > 0.001f) ? (state.F_leg / mg) * 0.35f : 0f;
        SetLine(_forceLine, P, P + new Vector3(sin1, cos1, 0f) * forceLen);

        SetColor(_forceLine, state.gripState switch
        {
            SkiGripState.Carving => new Color(0.2f, 0.95f, 0.3f, 1f),
            SkiGripState.Unweighted => new Color(1.0f, 0.85f, 0.2f, 1f),
            SkiGripState.Skidding => new Color(1.0f, 0.25f, 0.25f, 1f),
            SkiGripState.RailLimitHit => new Color(1.0f, 0.55f, 0.1f, 1f),
            _ => new Color(0.9f, 0.2f, 0.95f, 1f)
        });

        float peakRad = _controller.CurrentTurnPeakEdgeDeg * Mathf.Deg2Rad;
        if (peakRad > 0.05f)
        {
            float sign = Mathf.Sign(state.theta1);
            if (sign == 0f) sign = 1f;
            SetLine(_ghostPeakLine, P, P + new Vector3(Mathf.Sin(peakRad * sign), Mathf.Cos(peakRad * sign), 0f) * 0.85f);
        }
        else
        {
            _ghostPeakLine.positionCount = 0;
        }

        UpdateTargetAngleGuides(P, H, state, input, cfg);
    }

    private void UpdateTargetAngleGuides(Vector3 P, Vector3 H, in SimState state, in SimInput input, SkyTechSimConfig cfg)
    {
        if (!ShowTargetAngleGuides)
        {
            _targetLegLine.positionCount = _targetTrunkLine.positionCount = 0;
            _currentToTargetTrunkLine.positionCount = 0;
            _targetHipCircle.positionCount = _targetHeadCircle.positionCount = 0;
            return;
        }

        // 1. 下肢ターゲット角度 θ1,target の算出 (iPhone端末ロール角 または ステア指令)
        float maxRollRad = Mathf.Min(75.0f * Mathf.Deg2Rad, cfg.theta_bootOutRad - 2.0f * Mathf.Deg2Rad);
        float targetLegRad;
        if (_controller != null && _controller.UseDeviceTilt && Mathf.Abs(_controller.TargetLegRollDeg) > 0.05f)
        {
            targetLegRad = Mathf.Clamp(_controller.TargetLegRollDeg * Mathf.Deg2Rad, -maxRollRad, maxRollRad);
        }
        else if (Mathf.Abs(input.phi_steer_cmd) > 0.001f)
        {
            targetLegRad = Mathf.Clamp(-input.phi_steer_cmd * maxRollRad + input.phi_lock_cmd, -maxRollRad, maxRollRad);
        }
        else
        {
            targetLegRad = state.theta1;
        }

        // 目標下肢方向ベクトル
        Vector3 targetLegDir = new Vector3(Mathf.Sin(targetLegRad), Mathf.Cos(targetLegRad), 0f);
        float legLength = (state.l1 > 0.1f) ? state.l1 : cfg.l_eq;
        Vector3 targetH = P + targetLegDir * legLength;

        // 下肢ターゲット線の描画 (足元 P -> 目標股関節 targetH)
        _targetLegLine.startWidth = _targetLegLine.endWidth = TargetLegLineWidth;
        SetLine(_targetLegLine, P, targetH);

        if (ShowTargetJointMarkers)
        {
            SetCirclePositions(_targetHipCircle, targetH, 0.038f, 16);
        }
        else
        {
            _targetHipCircle.positionCount = 0;
        }

        // 2. 上体ターゲット角度 θ2,target の算出 (AirPods首かしげ角 または 物理ソルバーの体幹目標角)
        float targetTrunkRad = state.theta2_ref;
        Vector3 targetTrunkDir = new Vector3(Mathf.Sin(targetTrunkRad), Mathf.Cos(targetTrunkRad), 0f);
        Vector3 targetHead = targetH + targetTrunkDir * (cfg.l2 * 1.55f);

        // 上体ターゲット線の描画 (目標股関節 targetH -> 目標頭部 targetHead)
        _targetTrunkLine.startWidth = _targetTrunkLine.endWidth = TargetTrunkLineWidth;
        SetLine(_targetTrunkLine, targetH, targetHead);

        // 現在の股関節 H からも上体目標角を示す補助線を描画 (今の体幹との角度差を直接比較可能)
        Vector3 curTargetHead = H + targetTrunkDir * (cfg.l2 * 1.45f);
        SetLine(_currentToTargetTrunkLine, H, curTargetHead);

        if (ShowTargetJointMarkers)
        {
            SetCirclePositions(_targetHeadCircle, targetHead, 0.055f, 18);
        }
        else
        {
            _targetHeadCircle.positionCount = 0;
        }

        // 内倒警告または過剰リーンアウトに応じたカラーフィードバック
        if (_controller != null && _controller.IsOverLeanInWarning)
        {
            SetColor(_targetTrunkLine, new Color(1.0f, 0.25f, 0.25f, 0.90f));
            SetColor(_currentToTargetTrunkLine, new Color(1.0f, 0.25f, 0.25f, 0.60f));
            SetColor(_targetHeadCircle, new Color(1.0f, 0.25f, 0.25f, 0.90f));
        }
        else if (_controller != null && _controller.IsExcessLeanOutActive)
        {
            SetColor(_targetTrunkLine, new Color(0.20f, 1.0f, 0.70f, 0.90f));
            SetColor(_currentToTargetTrunkLine, new Color(0.20f, 1.0f, 0.70f, 0.60f));
            SetColor(_targetHeadCircle, new Color(0.20f, 1.0f, 0.70f, 0.90f));
        }
        else
        {
            SetColor(_targetTrunkLine, new Color(1.00f, 0.85f, 0.20f, 0.85f));
            SetColor(_currentToTargetTrunkLine, new Color(1.00f, 0.85f, 0.20f, 0.45f));
            SetColor(_targetHeadCircle, new Color(1.00f, 0.85f, 0.20f, 0.80f));
        }
    }

    private void UpdatePuppetApexGuide(Vector3 hipPos, Vector3 headPos, in SimState state, in SimInput input, SkyTechSimConfig cfg)
    {
        if (!ShowPuppetApexGuide || !input.has_mouse_world)
        {
            _puppetStringLine.positionCount = _apexTargetCircle.positionCount = 0;
            return;
        }

        Vector3 targetTrunkDir = new Vector3(Mathf.Sin(state.theta2_ref), Mathf.Cos(state.theta2_ref), 0f);
        Vector3 apexTargetPos = hipPos + targetTrunkDir * (cfg.l2 * 1.55f);
        SetCirclePositions(_apexTargetCircle, apexTargetPos, 0.052f, 18);

        _puppetStringLine.positionCount = 3;
        _puppetStringLine.SetPosition(0, headPos);
        _puppetStringLine.SetPosition(1, apexTargetPos);
        _puppetStringLine.SetPosition(2, new Vector3(input.mouse_world.x, input.mouse_world.y, 0f));

        Color stringCol = Color.Lerp(new Color(1.0f, 0.85f, 0.40f, 0.50f), new Color(0.30f, 1.0f, 0.65f, 0.80f), input.u_ext_norm);
        _puppetStringLine.startColor = stringCol;
        _puppetStringLine.endColor = new Color(stringCol.r, stringCol.g, stringCol.b, 0.25f);
    }

    private static void SetCirclePositions(LineRenderer lr, Vector3 center, float radius, int segments)
    {
        lr.positionCount = segments + 1;
        float deltaAngle = (Mathf.PI * 2f) / segments;
        for (int i = 0; i <= segments; i++)
        {
            float angle = i * deltaAngle;
            lr.SetPosition(i, center + new Vector3(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius, 0f));
        }
    }

    private static void SetBoxPositions(LineRenderer lr, Vector3 center, Vector2 size)
    {
        lr.positionCount = 5;
        float hx = size.x * 0.5f, hy = size.y * 0.5f;
        lr.SetPosition(0, center + new Vector3(-hx, -hy, 0f));
        lr.SetPosition(1, center + new Vector3( hx, -hy, 0f));
        lr.SetPosition(2, center + new Vector3( hx,  hy, 0f));
        lr.SetPosition(3, center + new Vector3(-hx,  hy, 0f));
        lr.SetPosition(4, center + new Vector3(-hx, -hy, 0f));
    }

    private void InitGUIStyles()
    {
        if (_guiInitialized) return;

        _panelBgTex = CreateSolidTexture(new Color(0.08f, 0.11f, 0.16f, 0.88f));
        _barBgTex   = CreateSolidTexture(new Color(0.15f, 0.18f, 0.25f, 0.90f));
        _barFillTex = CreateSolidTexture(new Color(0.20f, 0.75f, 0.95f, 1.00f));
        _badgeTex   = CreateSolidTexture(Color.white);

        _panelStyle     = new GUIStyle(GUI.skin.box) { normal = { background = _panelBgTex }, padding = new RectOffset(12, 12, 10, 10) };
        _titleStyle     = new GUIStyle(GUI.skin.label) { fontSize = 13, fontStyle = FontStyle.Bold, normal = { textColor = new Color(0.4f, 0.85f, 1.0f) }, alignment = TextAnchor.MiddleLeft };
        _labelStyle     = new GUIStyle(GUI.skin.label) { fontSize = 11, normal = { textColor = new Color(0.80f, 0.85f, 0.92f) }, alignment = TextAnchor.MiddleLeft, richText = true };
        _valueStyle     = new GUIStyle(GUI.skin.label) { fontSize = 11, fontStyle = FontStyle.Bold, normal = { textColor = Color.white }, alignment = TextAnchor.MiddleRight, richText = true };
        _badgeStyle     = new GUIStyle(GUI.skin.box) { fontSize = 12, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter, normal = { background = _badgeTex, textColor = Color.white } };
        _guideStyle     = new GUIStyle(GUI.skin.label) { fontSize = 11, normal = { textColor = new Color(0.75f, 0.82f, 0.90f) }, alignment = TextAnchor.MiddleCenter, richText = true };
        _centerTagStyle = new GUIStyle(GUI.skin.label) { fontSize = 10, normal = { textColor = new Color(0.65f, 0.82f, 0.98f, 0.65f) }, alignment = TextAnchor.MiddleLeft, richText = true };
        _btnStyle       = new GUIStyle(GUI.skin.button) { fontSize = 10, fontStyle = FontStyle.Bold };

        _guiInitialized = true;
    }

    private void OnGUI()
    {
        if (_controller == null || _controller.Config == null) return;
        InitGUIStyles();

        DrawCenterGuideLabels();
        DrawBiomechanicsPanel(_controller.CurrentState, _controller.Config);
        DrawScorePanel();
        DrawControlsGuidePanel();
        DrawDualTiltHorizonGauge(_controller.CurrentState);
        DrawOverLeanInWarningBanner();
    }

    // SkyTechVisualizer.cs の OnGUI() 内から呼び出すデュアルチルトUIバー
    private void DrawDualTiltHorizonGauge(in SimState state)
    {
        float gaugeW = Mathf.Min(460f, Screen.width - 40f);
        float gaugeH = 44f;
        float gx = (Screen.width - gaugeW) * 0.5f;
        float gy = 52f;

        Rect boxRect = new Rect(gx, gy, gaugeW, gaugeH);
        GUI.Box(boxRect, GUIContent.none, _panelStyle);

        float centerX = gx + gaugeW * 0.5f;
        float barY = gy + 22f;
        float barW = gaugeW - 36f;
        float barLeft = centerX - barW * 0.5f;
        float halfBarW = barW * 0.5f;
        const float maxDisplayDeg = 75.0f; // 左右±75°フルスケール

        Color oldCol = GUI.color;

        // 1. ベースレール背景
        GUI.color = new Color(0.14f, 0.18f, 0.25f, 0.95f);
        GUI.DrawTexture(new Rect(barLeft, barY - 4f, barW, 8f), _barBgTex);

        // 2. 現在の脚ロール角 θ1 (= ストレート内倒の境界ライン) と 実際の体幹角 θ2
        float legDeg = state.theta1 * Mathf.Rad2Deg;
        float trunkDeg = (state.theta1 - state.phi) * Mathf.Rad2Deg;

        float legScreenX = centerX + Mathf.Clamp(legDeg / maxDisplayDeg, -1f, 1f) * halfBarW;
        float trunkScreenX = centerX + Mathf.Clamp(trunkDeg / maxDisplayDeg, -1f, 1f) * halfBarW;

        // 3. 股関節外傾角 (φ = θ1 - θ2) のブリッジ帯を描画
        float bridgeLeft = Mathf.Min(legScreenX, trunkScreenX);
        float bridgeWidth = Mathf.Abs(legScreenX - trunkScreenX);
        Color bridgeCol = _controller.IsOverLeanInWarning
            ? new Color(1.0f, 0.20f, 0.20f, 0.85f) // 内倒超過時は赤
            : (_controller.IsExcessLeanOutActive
                ? new Color(0.15f, 1.0f, 0.75f, 0.75f) // 過剰リーンアウト時はエメラルド
                : new Color(0.30f, 0.75f, 1.0f, 0.55f));
        GUI.color = bridgeCol;
        GUI.DrawTexture(new Rect(bridgeLeft, barY - 3f, Mathf.Max(2f, bridgeWidth), 6f), _barFillTex);

        // 4. 鉛直 0° センターライン (これよりターン外側なら過剰リーンアウト)
        GUI.color = new Color(1f, 1f, 1f, 0.85f);
        GUI.DrawTexture(new Rect(centerX - 1f, barY - 10f, 2f, 20f), _barFillTex);

        // 5. 📱 ストレート内倒リミットバー (脚角 θ1 / iPhone傾き)
        GUI.color = new Color(0.20f, 0.85f, 1.0f, 1.0f);
        GUI.DrawTexture(new Rect(legScreenX - 2f, barY - 9f, 4f, 18f), _barFillTex);

        // 6. 🎧 頭部・体幹角マーカー (θ2 / AirPods傾き)
        Color headCol = _controller.IsOverLeanInWarning
            ? new Color(1.0f, 0.25f, 0.25f, 1f)
            : (_controller.IsExcessLeanOutActive ? new Color(0.20f, 1.0f, 0.65f, 1f) : new Color(1.0f, 0.90f, 0.25f, 1f));
        GUI.color = headCol;
        GUI.DrawTexture(new Rect(trunkScreenX - 5f, barY - 7f, 10f, 14f), _barFillTex);

        GUI.color = oldCol;

        // 7. ステータス＆角度ラベル表示
        string sensorTag = _controller.IsAirPodsConnected ? "<color=#58D68D>🎧 AirPods+📱iPhone</color>" : "<color=#85C1E9>📱 iPhone Tilt</color>";
        string postureState = _controller.IsOverLeanInWarning
            ? $"<color=#FF4444>⚠ OVER LEAN-IN! (+{_controller.OverLeanInExcessDeg:F1}°)</color>"
            : (_controller.IsExcessLeanOutActive
                ? $"<color=#2ECC71>★ EXCESS LEAN-OUT (外側 {Mathf.Abs(trunkDeg):F1}°)</color>"
                : $"鉛直0°基準 | 頭θ2:{trunkDeg:+0.0;-0.0;0.0}° / 脚θ1:{legDeg:+0.0;-0.0;0.0}°");

        float btnW = 76f;
        float btnH = 20f;
        float labelW = gaugeW - btnW - 24f;
        GUI.Label(new Rect(gx + 10f, gy + 2f, labelW, 18f), $"{sensorTag}   {postureState}", _guideStyle);

        if (GUI.Button(new Rect(gx + gaugeW - btnW - 6f, gy + 2f, btnW, btnH), "ゼロ点補正", _btnStyle))
        {
            _controller.CalibrateDualSensors();
        }
    }

    private void DrawOverLeanInWarningBanner()
    {
        if (!_controller.IsOverLeanInWarning) return;

        float bannerW = 380f;
        float bannerH = 30f;
        Rect bannerRect = new Rect((Screen.width - bannerW) * 0.5f, 14f, bannerW, bannerH);

        float flash = (Mathf.Sin(Time.time * 14.0f) + 1.0f) * 0.5f;
        Color warnColor = Color.Lerp(new Color(0.85f, 0.15f, 0.15f, 0.95f), new Color(1.0f, 0.40f, 0.10f, 0.95f), flash);

        DrawBadge(bannerRect, $"▲ OVER LEAN-IN WARNING! (+{_controller.OverLeanInExcessDeg:F1}°)", warnColor, Color.white);
    }

    private void DrawCenterGuideLabels()
    {
        if (!ShowCenterCrosshair) return;

        Camera cam = Camera.main;
        float cx = Screen.width * 0.5f, cy = Screen.height * 0.5f;
        float leanInScreenY = cy - 45f;
        float straightScreenY = cy - 85f;
        float downTickScreenY = cy + 55f;

        if (cam != null)
        {
            Vector3 wp = cam.ScreenToWorldPoint(new Vector3(cx, cy, -cam.transform.position.z));
            wp.y += _controller.MouseCenterOffsetY;
            cy = Screen.height - cam.WorldToScreenPoint(wp).y;

            Vector3 wpLean = wp;
            wpLean.y += Mathf.Max(0.02f, _controller.MouseLeanInLineOffsetY);
            leanInScreenY = Screen.height - cam.WorldToScreenPoint(wpLean).y;

            Vector3 wpStraight = wp;
            wpStraight.y += Mathf.Max(_controller.MouseLeanInLineOffsetY + 0.1f, _controller.MouseStraightLineOffsetY);
            straightScreenY = Screen.height - cam.WorldToScreenPoint(wpStraight).y;

            Vector3 wpDown = wp;
            wpDown.y -= Mathf.Max(0.05f, GuideFlexDownRange);
            downTickScreenY = Screen.height - cam.WorldToScreenPoint(wpDown).y;
        }

        GUI.Label(new Rect(cx + 10f, straightScreenY - 18f, 360f, 18f), "<color=#4DFFA0>★ STRAIGHT LINE (ストレート内倒 / これより上でさらに深くリーンイン)</color>", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, leanInScreenY - 18f, 320f, 18f), "<color=#FF73BF>▲ LEAN-IN LINE (これより上で激しくストレート内倒へ移行)</color>", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, cy - 18f, 260f, 18f), "CENTER (中立基準線 / 基準脚長)", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, downTickScreenY - 18f, 280f, 18f), "<color=#FFA040>▼ FLEX / LEAN-OUT (下限基準線 / リーンアウト)</color>", _centerTagStyle);
        GUI.Label(new Rect(cx - 140f, cy + 4f, 130f, 18f), "← 左へ頭頂誘導", _centerTagStyle);
    }

    private void DrawBiomechanicsPanel(in SimState state, SkyTechSimConfig cfg)
    {
        Rect panelRect = new Rect(15, 15, 355, 540);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        float curY = panelRect.y + 10, leftX = panelRect.x + 12, contentW = panelRect.width - 24;
        GUI.Label(new Rect(leftX, curY, contentW, 20), "BIOMECHANICS & TELEMETRY", _titleStyle);
        curY += 24;

        (string badgeText, Color badgeBg) = state.gripState switch
        {
            SkiGripState.Carving      => ("GRIP: CARVING",            new Color(0.15f, 0.70f, 0.35f)),
            SkiGripState.Unweighted   => ("GRIP: UNWEIGHTED",         new Color(0.85f, 0.65f, 0.10f)),
            SkiGripState.Skidding     => ("SKIDDING (SNOW FAILURE!)", new Color(0.85f, 0.20f, 0.20f)),
            SkiGripState.RailLimitHit => ("STATUS: RAIL LIMIT HIT",   new Color(0.90f, 0.45f, 0.10f)),
            SkiGripState.BootOut      => ("CRASH: BOOT OUT!",         new Color(0.65f, 0.15f, 0.75f)),
            SkiGripState.HipCrash     => ("CRASH: HIP DOWN!",         new Color(0.75f, 0.10f, 0.20f)),
            _                         => (state.gripState.ToString(), Color.gray)
        };

        DrawBadge(new Rect(leftX, curY, contentW, 22), badgeText, badgeBg, Color.white);
        curY += 28;

        float lowHipDist = (1f - cfg.c1) * state.l1;
        float twoComGap = Mathf.Sqrt(lowHipDist * lowHipDist + cfg.l2 * cfg.l2 + 2f * lowHipDist * cfg.l2 * Mathf.Cos(state.phi));
        DrawMetricRow(leftX, ref curY, contentW, "CoM Gap <color=#FF3366>●上</color>-<color=#00E5FF>●下</color>", $"{twoComGap * 100f:F1} cm (基準: {cfg.CoMDistanceDefault * 100f:F0}cm)");
        curY += 2;

        float edgeDeg = Mathf.Abs(state.theta1) * Mathf.Rad2Deg;
        float comDeg = Mathf.Abs(state.Psi_CoM) * Mathf.Rad2Deg;
        DrawMetricRow(leftX, ref curY, contentW, "Edge θ1 vs <color=#FFEB3B>CoM</color> Ψ", $"{state.theta1 * Mathf.Rad2Deg:+0.0;-0.0;0.0}° [目標: {_controller.TargetLegRollDeg:+0;-0;0}°] / {comDeg:F1}°");
        DrawProgressBar(new Rect(leftX, curY, contentW, 10), edgeDeg / 78.0f, new Color(0.2f, 0.8f, 1f));
        curY += 16;

        string shiftInfo = (Mathf.Abs(state.dy_actual) > 0.005f) ? $" [Δy:{state.dy_actual * 100f:+0.0;-0.0;0.0}cm]" : "";
        DrawMetricRow(leftX, ref curY, contentW, "Hip φ / CoM Shift", $"{state.phi * Mathf.Rad2Deg:F1}° (±{cfg.phi_out_maxDeg:F0}°){shiftInfo}");
        curY += 2;

        string zoneLabel = (state.s_rom >= 0.68f && state.s_rom <= 0.88f)
            ? "<color=#58D68D>POWER POS (DL)</color>"
            : (state.s_rom < 0.68f ? "<color=#F5B041>SQUAT</color>" : "<color=#85C1E9>LOCKOUT</color>");
        DrawMetricRow(leftX, ref curY, contentW, "Leg ROM / Knee Flex", $"{state.s_rom * 100f:F0}% ({state.l1:F2}m) | 膝:{state.knee_flex_deg:F0}° [{zoneLabel}]");
        DrawProgressBar(new Rect(leftX, curY, contentW, 10), state.s_rom, Color.Lerp(new Color(0.95f, 0.35f, 0.2f), new Color(0.2f, 0.95f, 0.5f), state.posture_force_ratio));
        curY += 14;

        DrawMetricRow(leftX, ref curY, contentW, "Leg Power (Cur / Max)", $"{state.P_leg_ext:F0} W / {state.P_leg_peak_avail:F0} W ({state.posture_force_ratio * 100f:F0}%)");
        curY += 2;

        float mg = cfg.M * cfg.g;
        float invMg = (mg > 0.001f) ? (1f / mg) : 0f;
        DrawMetricRow(leftX, ref curY, contentW, "Load: F_leg / F_vert", $"{state.F_leg * invMg:F2} G / 鉛直:{state.F_vert * invMg:F2} G (ストレス:{state.snow_stress_ratio * 100f:F0}%)");
        curY += 2;

        float realLatG = (cfg.g > 0.001f) ? (Mathf.Abs(state.a_c) / cfg.g) : 0f;
        DrawMetricRow(leftX, ref curY, contentW, "Lateral G (Base / Real)", $"{Mathf.Abs(state.G_base):F2} G / {realLatG:F2} G (λ:{state.lambda_sustain * 100f:F0}%)");
        curY += 2;

        DrawMetricRow(leftX, ref curY, contentW, "Speed V", $"{state.V * 3.6f:F1} km/h ({state.V:F1} m/s)");
        curY += 2;

        DrawMetricRow(leftX, ref curY, contentW, "Carriage X / Speed", $"{state.x_c:+0.00;-0.00;0.00}m ({state.v_c * 3.6f:+0.0;-0.0;0.0} / 目標:{state.v_c_target * 3.6f:+0.0;-0.0;0.0}km/h)");
        curY += 2;

        float currentR = (Mathf.Abs(state.kappa) > 0.005f) ? (1.0f / Mathf.Abs(state.kappa)) : 99.9f;
        DrawMetricRow(leftX, ref curY, contentW, "Flex Curvature κ", $"{state.kappa:+0.000;-0.000;0.000} /m (R={currentR:F1}m)");
        curY += 2;

        float staminaNorm = (cfg.maxStamina > 0.001f) ? Mathf.Clamp01(state.stamina / cfg.maxStamina) : 1f;
        DrawMetricRow(leftX, ref curY, contentW, "Anaerobic Stamina", $"{state.stamina:F0} J ({staminaNorm * 100f:F0}%)");
        DrawProgressBar(new Rect(leftX, curY, contentW, 10), staminaNorm, staminaNorm > 0.35f ? new Color(0.3f, 0.9f, 0.4f) : new Color(0.95f, 0.3f, 0.2f));
        curY += 14;

        bool airPodsActive = _controller.IsAirPodsConnected;
        string apStatus;
#if UNITY_EDITOR
        apStatus = airPodsActive ? "<color=#F4D03F>EDITOR SIM</color>" : "<color=#7F8C8D>OFFLINE</color>";
#else
        apStatus = airPodsActive ? "<color=#58D68D>CONNECTED</color>" : "<color=#7F8C8D>OFFLINE</color>";
#endif
        DrawMetricRow(leftX, ref curY, contentW, "Dual Sensor Mode", apStatus);
        if (airPodsActive)
        {
            float headRoll = _controller.CurrentHeadRollDeg;
            float trunkDeg = _controller.CurrentDesiredTrunkDeg;
            DrawMetricRow(leftX, ref curY, contentW, "  Head Roll (AirPods)", $"{headRoll:+0.0;-0.0;0.0}° [C:初期化]");
            DrawMetricRow(leftX, ref curY, contentW, "  Trunk Target θ2", $"{trunkDeg:+0.0;-0.0;0.0}°");

            if (_controller.IsOverLeanInWarning)
            {
                DrawMetricRow(leftX, ref curY, contentW, "  Posture Status", $"<color=#FF3344>▲ OVER LEAN-IN! (+{_controller.OverLeanInExcessDeg:F1}°)</color>");
            }
            else if (_controller.IsExcessLeanOutActive)
            {
                DrawMetricRow(leftX, ref curY, contentW, "  Posture Status", "<color=#FFA040>▼ EXCESS LEAN-OUT (外傾)</color>");
            }
            else
            {
                DrawMetricRow(leftX, ref curY, contentW, "  Posture Status", "<color=#58D68D>● BALANCED (適正内倒)</color>");
            }
        }
        curY += 4;

        string assistStr = _controller.AutoAngulationAssist ? "<color=#2ECC71>AUTO-ASSIST: ON</color>" : "<color=#BDC3C7>MANUAL (Tab)</color>";
        GUI.Label(new Rect(leftX, curY, contentW, 18), $"Control: {assistStr}", _labelStyle);
    }

    private void DrawScorePanel()
    {
        Rect panelRect = new Rect(Screen.width - 325, 15, 310, 205);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        float curY = panelRect.y + 10, leftX = panelRect.x + 12, contentW = panelRect.width - 24;
        GUI.Label(new Rect(leftX, curY, contentW, 20), "TURN SCORE & COMBO", _titleStyle);
        curY += 24;

        DrawMetricRow(leftX, ref curY, contentW, "Total Score", $"{_controller.TotalScore:F0}");
        curY += 2;
        DrawMetricRow(leftX, ref curY, contentW, "Turns / Combo", $"{_controller.TurnCount} / {_controller.ComboCount}x");
        curY += 4;
        DrawMetricRow(leftX, ref curY, contentW, "Current Peak Edge", $"{_controller.CurrentTurnPeakEdgeDeg:F1}°");
        DrawProgressBar(new Rect(leftX, curY, contentW, 8), _controller.CurrentTurnPeakEdgeDeg / 78.0f, new Color(0.3f, 0.75f, 1f));
        curY += 14;
        DrawMetricRow(leftX, ref curY, contentW, "Last Turn Peak", $"{_controller.LastTurnPeakEdgeDeg:F1}°");
        curY += 4;

        string eval = _controller.LastTurnEvaluation;
        Color evalBg = eval.Contains("WORLD CUP") ? new Color(0.85f, 0.55f, 0.05f)
                     : eval.Contains("DEEP")      ? new Color(0.15f, 0.65f, 0.40f)
                     : (eval.Contains("CRASH") || eval.Contains("RAIL")) ? new Color(0.80f, 0.20f, 0.20f)
                     : new Color(0.20f, 0.45f, 0.75f);

        DrawBadge(new Rect(leftX, curY, contentW, 26), eval, evalBg, Color.white);
    }

    private void DrawControlsGuidePanel()
    {
        float panelW = Mathf.Min(820, Screen.width - 30);
        Rect panelRect = new Rect((Screen.width - panelW) * 0.5f, Screen.height - 62, panelW, 50);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        GUI.Label(new Rect(panelRect.x, panelRect.y + 6, panelW, 18),
            "<color=#FF73BF>[リーンイン線より上]</color> 手動リーンイン   |   <color=#FFA040>[中立線より下]</color> リーンアウト(脚屈曲なし)   |   <color=#58D68D>[Space / ↑ / 中立線より上]</color> 脚伸展   |   <color=#85C1E9>[S]</color> 脚屈曲",
            _guideStyle);
        GUI.Label(new Rect(panelRect.x, panelRect.y + 24, panelW, 18),
            $"<color=#5DADE2>[iPhone傾き/マウス左右/A・D]</color> 脚ロール(θ1)   |   <color=#F4D03F>[AirPods首かしげ/Q・E]</color> 体幹目標(θ2)   |   <color=#58D68D>[C]</color> デュアルゼロ点補正   |   <color=#2ECC71>[W]</color> 直立   |   <color=#E74C3C>[R]</color> リセット",
            _guideStyle);
    }

    private void DrawMetricRow(float x, ref float y, float width, string label, string value)
    {
        float labelW = width * 0.46f;
        GUI.Label(new Rect(x, y, labelW, 18), label, _labelStyle);
        GUI.Label(new Rect(x + labelW, y, width * 0.54f, 18), value, _valueStyle);
        y += 18;
    }

    private void DrawProgressBar(Rect rect, float fill01, Color fillColor)
    {
        fill01 = Mathf.Clamp01(fill01);
        Color oldColor = GUI.color;
        GUI.color = Color.white;
        GUI.DrawTexture(rect, _barBgTex);

        if (fill01 > 0.005f)
        {
            GUI.color = fillColor;
            GUI.DrawTexture(new Rect(rect.x + 1, rect.y + 1, (rect.width - 2) * fill01, rect.height - 2), _barFillTex);
        }
        GUI.color = oldColor;
    }

    private void DrawBadge(Rect rect, string text, Color bgColor, Color textColor)
    {
        Color oldColor = GUI.color;
        GUI.color = bgColor;
        GUI.DrawTexture(rect, _badgeTex);
        GUI.color = oldColor;

        Color oldTextCol = _badgeStyle.normal.textColor;
        _badgeStyle.normal.textColor = textColor;
        GUI.Label(rect, text, _badgeStyle);
        _badgeStyle.normal.textColor = oldTextCol;
    }

    private static Texture2D CreateSolidTexture(Color color)
    {
        Texture2D tex = new Texture2D(1, 1, TextureFormat.RGBA32, false) { hideFlags = HideFlags.DontSave };
        tex.SetPixel(0, 0, color);
        tex.Apply();
        return tex;
    }

    private void OnDestroy()
    {
        Destroy(_lineMaterial);
        Destroy(_panelBgTex);
        Destroy(_barBgTex);
        Destroy(_barFillTex);
        Destroy(_badgeTex);
    }
}