using UnityEngine;
using SkyTechSimulator;

[RequireComponent(typeof(SkyTechGameController))]
public class SkyTechVisualizer : MonoBehaviour
{
    [Header("=== 画面中央基準・頂点誘導ガイド表示設定 ===")]
    public bool ShowCenterCrosshair = true;
    public bool ShowPuppetApexGuide = true;
    public float GuideExtendUpRange = 2.75f;
    public float GuideFlexDownRange = 0.75f;

    [Header("=== 理想のマウスリード目印（ペースメーカー）設定 ===")]
    public bool ShowIdealMouseLead = true;
    [Tooltip("現在のピーク角に対してプラス何度の深さを目指すか")]
    public float LeadExtraDepthDeg = 6.5f;
    [Tooltip("位相をどれだけ先回りしてリードするか（ラジアン: 0.45 ≈ 約25度先回り）")]
    public float LeadPhaseAdvanceRad = 0.45f;
    [Tooltip("ターンがまだ無い時の初期リズム周期（秒/1往復）")]
    public float DefaultCyclePeriodSec = 2.4f;

    private float _measuredHalfTurnSec = 1.2f;
    private float _turnTimer = 0f;
    private float _prevThetaSign = 0f;
    private float _pacemakerPhase = 0f;
    private float _smoothedTargetDeg = 42.0f;

    private void EnforceVisualizerRanges()
    {
        if (GuideExtendUpRange < 2.0f) GuideExtendUpRange = 2.75f;
    }

    private void OnValidate() => EnforceVisualizerRanges();

    private SkyTechGameController _controller;
    private Material _lineMaterial;

    private LineRenderer _centerHorizLine, _leanInThresholdLine, _centerVertLine, _centerReticleCircle;
    private LineRenderer _heightMaxTickLine, _heightMinTickLine, _mouseGuideLine, _mouseCursorCircle;
    private LineRenderer _puppetStringLine, _apexTargetCircle;
    private LineRenderer _idealLeadOuterRing, _idealLeadInnerCore, _idealLeadOrbitLine, _idealLeadTetherLine;
    private LineRenderer _railLine, _absorberLeft, _absorberRight, _carriageLine, _skiBoardLine;
    private LineRenderer _legLine, _trunkLine, _hipJointCircle, _headCircle;
    private LineRenderer _comCircle, _comLine, _lowerComCircle, _upperComCircle, _twoComDistLine;
    private LineRenderer _forceLine, _ghostPeakLine;

    private Texture2D _panelBgTex, _barBgTex, _barFillTex, _badgeTex;
    private GUIStyle _panelStyle, _titleStyle, _labelStyle, _valueStyle, _badgeStyle, _guideStyle, _centerTagStyle;
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
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (cam == null)
        {
            cam = new GameObject("Main Camera") { tag = "MainCamera" }.AddComponent<Camera>();
        }

        cam.orthographic = true;
        float halfRail = (_controller != null && _controller.Config != null) ? _controller.Config.W_rail * 0.5f : 3.60f;
        float aspect = (cam.pixelWidth > 0 && cam.pixelHeight > 0) ? (float)cam.pixelWidth / cam.pixelHeight : (16f / 9f);
        cam.orthographicSize = Mathf.Max(1.6f, (halfRail + 0.35f) / Mathf.Max(0.5f, aspect));
        cam.transform.SetPositionAndRotation(new Vector3(0f, 0.85f, -10f), Quaternion.identity);
        cam.backgroundColor = new Color(0.063f, 0.082f, 0.125f);
        cam.clearFlags = CameraClearFlags.SolidColor;
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
        _turnTimer += dt;

        // 1. 現在のターンの切り替え周期（リズム）を自動計測して追従
        float curSign = Mathf.Abs(state.theta1) > (3.0f * Mathf.Deg2Rad) ? Mathf.Sign(state.theta1) : _prevThetaSign;
        if (_prevThetaSign != 0f && curSign != _prevThetaSign)
        {
            // 有効なターン切り替え時間（0.45秒〜3.0秒）なら移動平均でリズムを更新
            if (_turnTimer >= 0.45f && _turnTimer <= 3.0f)
            {
                _measuredHalfTurnSec = Mathf.Lerp(_measuredHalfTurnSec, _turnTimer, 0.45f);
            }
            _turnTimer = 0f;
        }
        if (curSign != 0f) _prevThetaSign = curSign;

        float cyclePeriod = Mathf.Clamp(_measuredHalfTurnSec * 2.0f, 1.1f, 4.5f);
        if (_controller.TurnCount == 0 && _turnTimer > 2.5f) cyclePeriod = DefaultCyclePeriodSec;
        float omegaRhythm = (Mathf.PI * 2.0f) / cyclePeriod;

        // 2. 現在のエッジ角と角速度から「現在のターン位相」を推定し、自身の周期と位相同期(PLL)させる
        float refPeakDeg = Mathf.Max(_controller.LastTurnPeakEdgeDeg, _controller.CurrentTurnPeakEdgeDeg);
        if (refPeakDeg < 20.0f) refPeakDeg = 35.0f;

        // 「今より少し深いターン」の目標角度（最低35°〜最大68°の範囲で +LeadExtraDepthDeg）
        float desiredPeakDeg = Mathf.Clamp(refPeakDeg + LeadExtraDepthDeg, 35.0f, 68.0f);
        _smoothedTargetDeg = Mathf.Lerp(_smoothedTargetDeg, desiredPeakDeg, 3.0f * dt);

        float normTheta = Mathf.Clamp((state.theta1 * Mathf.Rad2Deg) / Mathf.Max(15.0f, refPeakDeg), -1.2f, 1.2f);
        float normOmega = Mathf.Clamp((state.omega1 * Mathf.Rad2Deg) / (refPeakDeg * omegaRhythm), -1.2f, 1.2f);

        // 現在のスキーヤーの位相に対して、同じリズムで回しつつ緩やかに位相同期させる
        _pacemakerPhase += omegaRhythm * dt;
        if (Mathf.Abs(state.theta1) > 5.0f * Mathf.Deg2Rad || Mathf.Abs(state.omega1) > 0.15f)
        {
            float skierPhase = Mathf.Atan2(normTheta, normOmega);
            float phaseDiff = Mathf.DeltaAngle(_pacemakerPhase * Mathf.Rad2Deg, skierPhase * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            _pacemakerPhase += phaseDiff * Mathf.Clamp01(2.2f * dt);
        }

        // リード位相 = 現在の同期位相 + 先回りリード量 (LeadPhaseAdvanceRad)
        float leadPhase = _pacemakerPhase + LeadPhaseAdvanceRad;

        // 3. 目標エッジ角から理想のマウスX・Y座標を算出
        Vector3 centerWorld3 = cam.ScreenToWorldPoint(new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, -cam.transform.position.z));
        Vector2 centerWorld = new Vector2(centerWorld3.x, centerWorld3.y + _controller.MouseCenterOffsetY);

        float maxRollDeg = Mathf.Min(72.0f, cfg.theta_bootOutDeg - 4.0f);
        float targetSteerNorm = Mathf.Clamp01(_smoothedTargetDeg / Mathf.Max(20.0f, maxRollDeg));
        float fullRangeX = Mathf.Max(0.15f, cfg.mouseSteerFullRange);
        float idealAmpX = _controller.MouseLineDeadzone + targetSteerNorm * fullRangeX;

        // X座標: 左右のターン頂点へスムーズにリード
        float leadSin = Mathf.Sin(leadPhase);
        float idealX = centerWorld.x + idealAmpX * leadSin;

        // 【修正】Y座標: 下の中立基準線(centerWorld.y)より少し上にマージンをとり、
        // Lean-inライン(leanInSpan)との中間付近を中心として8の字を描く
        float leanInSpan = Mathf.Max(0.20f, _controller.MouseLeanInLineOffsetY);
        float bottomMargin = leanInSpan * 0.25f; // 下の基準線からの最低マージン（25%の高さ）
        float midBaseY     = leanInSpan * 0.48f; // 軌道の中心高さ（中立基準線とLean-inラインのほぼ中間）
        float verticalAmp  = leanInSpan * 0.18f; // 上下の振幅（最下点でも 0.48 - 0.18 = 30% の高さを維持し、最上点でも 66% に収める）

        // 切り替え時(X=0付近, cos(2θ)=+1)は中間より上(約66%高さ)で抜重・伸展を促し、
        // ターンピーク(|X|=最大, cos(2θ)=-1)では中間より少し下(約30%高さ＝基準線より少し上のマージン位置)で外傾を維持
        float idealOffsetY = Mathf.Max(bottomMargin, midBaseY + verticalAmp * Mathf.Cos(leadPhase * 2.0f));

        Vector3 leadPos = new Vector3(idealX, centerWorld.y + idealOffsetY, 0f);

        // 4. 理想の軌道（8の字リサージュガイド曲線）を薄く描画
        const int orbitSegs = 48;
        _idealLeadOrbitLine.positionCount = orbitSegs + 1;
        for (int i = 0; i <= orbitSegs; i++)
        {
            float th = (i / (float)orbitSegs) * Mathf.PI * 2.0f;
            float ox = centerWorld.x + idealAmpX * Mathf.Sin(th);
            float oy = centerWorld.y + Mathf.Max(bottomMargin, midBaseY + verticalAmp * Mathf.Cos(th * 2.0f));
            _idealLeadOrbitLine.SetPosition(i, new Vector3(ox, oy, 0f));
        }

        // 5. 動く目印（パルスする二重リング）とマウスからの誘導ラインを描画
        float pulse = 1.0f + 0.15f * Mathf.Sin(Time.time * 8.0f);
        SetCirclePositions(_idealLeadOuterRing, leadPos, 0.068f * pulse, 24);
        SetCirclePositions(_idealLeadInnerCore, leadPos, 0.022f, 16);

        if (input.has_mouse_world)
        {
            Vector3 mousePos = new Vector3(input.mouse_world.x, input.mouse_world.y, 0f);
            SetLine(_idealLeadTetherLine, mousePos, leadPos);

            // マウスが目印に近いほどエメラルドグリーンに輝く
            float dist = Vector2.Distance(input.mouse_world, new Vector2(leadPos.x, leadPos.y));
            float proximity = Mathf.Clamp01(1.0f - dist / 0.85f);
            Color ringCol = Color.Lerp(new Color(0.20f, 0.90f, 1.00f, 0.85f), new Color(0.20f, 1.00f, 0.55f, 0.95f), proximity);
            SetColor(_idealLeadOuterRing, ringCol);
        }
        else
        {
            _idealLeadTetherLine.positionCount = 0;
        }
    }

    private void SetupLineRenderers()
    {
        _centerHorizLine     = CreateLine("CenterHorizGuide",  0.014f, new Color(0.55f, 0.75f, 0.95f, 0.25f), -4);
        _leanInThresholdLine = CreateLine("LeanInGuideLine",   0.014f, new Color(1.00f, 0.45f, 0.75f, 0.32f), -4);
        _centerVertLine      = CreateLine("CenterVertGuide",   0.014f, new Color(0.55f, 0.75f, 0.95f, 0.28f), -4);
        _heightMaxTickLine   = CreateLine("HeightMaxTick",     0.016f, new Color(0.35f, 0.95f, 0.60f, 0.35f), -3);
        _heightMinTickLine   = CreateLine("HeightMinTick",     0.016f, new Color(1.00f, 0.60f, 0.25f, 0.35f), -3);
        _centerReticleCircle = CreateLine("CenterReticle",     0.015f, new Color(0.75f, 0.88f, 1.00f, 0.40f), -2);
        _mouseGuideLine      = CreateLine("MouseGuideLine",    0.012f, new Color(1.00f, 0.90f, 0.40f, 0.30f), -1);

        _railLine        = CreateLine("Rail",           0.035f, new Color(0.40f, 0.45f, 0.55f, 0.8f), 0);
        _absorberLeft    = CreateLine("AbsorberL",      0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _absorberRight   = CreateLine("AbsorberR",      0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _ghostPeakLine   = CreateLine("GhostPeak",      0.020f, new Color(0.3f, 0.8f, 1.0f, 0.25f),   2);
        _carriageLine    = CreateLine("Carriage",       0.030f, new Color(0.75f, 0.80f, 0.90f, 0.9f), 3);
        _skiBoardLine    = CreateLine("SkiBoard",       0.045f, new Color(0.90f, 0.95f, 1.00f, 1.0f), 4);
        _legLine         = CreateLine("LegSegment",     0.065f, Color.cyan,                           5);
        _trunkLine       = CreateLine("TrunkSegment",   0.075f, new Color(0.9f, 0.9f, 0.95f, 1.0f),   6);
        _hipJointCircle  = CreateLine("HipJoint",       0.025f, new Color(1.0f, 0.8f, 0.2f, 1.0f),    7);
        _headCircle      = CreateLine("HeadCircle",     0.030f, new Color(1.0f, 0.85f, 0.7f, 1.0f),   7);
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
    }

    private void UpdateCenterCrosshairGeometry(in SimInput input)
    {
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (!ShowCenterCrosshair || cam == null)
        {
            _centerHorizLine.positionCount = _leanInThresholdLine.positionCount = _centerVertLine.positionCount = 0;
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
        SetColor(_leanInThresholdLine, Color.Lerp(new Color(1.00f, 0.45f, 0.75f, 0.28f), new Color(1.00f, 0.30f, 0.65f, 0.85f), input.mouse_lean_in_norm));

        SetLine(_centerVertLine, new Vector3(centerWorld.x, centerWorld.y - orthoH * 0.95f, 0f), new Vector3(centerWorld.x, centerWorld.y + orthoH * 0.95f, 0f));
        SetColor(_centerVertLine, Color.Lerp(new Color(0.55f, 0.75f, 0.95f, 0.28f), new Color(0.40f, 0.90f, 1.00f, 0.50f), Mathf.Abs(input.phi_steer_cmd)));

        SetCirclePositions(_centerReticleCircle, new Vector3(centerWorld.x, centerWorld.y, 0f), 0.055f, 20);

        float tickHalfW = 0.28f;
        float upY = centerWorld.y + Mathf.Max(0.05f, GuideExtendUpRange);
        float downY = centerWorld.y - Mathf.Max(0.05f, GuideFlexDownRange);

        SetLine(_heightMaxTickLine, new Vector3(centerWorld.x - tickHalfW, upY, 0f), new Vector3(centerWorld.x + tickHalfW, upY, 0f));
        SetColor(_heightMaxTickLine, Color.Lerp(new Color(0.35f, 0.95f, 0.60f, 0.30f), new Color(0.35f, 1.00f, 0.55f, 0.85f), input.u_ext_norm));

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
    }

    private void DrawCenterGuideLabels()
    {
        if (!ShowCenterCrosshair) return;

        Camera cam = Camera.main;
        float cx = Screen.width * 0.5f, cy = Screen.height * 0.5f, leanInScreenY = cy - 45f;

        if (cam != null)
        {
            Vector3 wp = cam.ScreenToWorldPoint(new Vector3(cx, cy, -cam.transform.position.z));
            wp.y += _controller.MouseCenterOffsetY;
            cy = Screen.height - cam.WorldToScreenPoint(wp).y;

            wp.y += Mathf.Max(0.02f, _controller.MouseLeanInLineOffsetY);
            leanInScreenY = Screen.height - cam.WorldToScreenPoint(wp).y;
        }

        GUI.Label(new Rect(cx + 10f, leanInScreenY - 18f, 260f, 18f), "<color=#FF73BF>▲ LEAN-IN LINE (これより上で手動リーンイン)</color>", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, cy - 18f, 260f, 18f), "CENTER (中立基準線 / 基準脚長)", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, cy + 4f, 260f, 18f), "<color=#FFA040>▼ これより下でリーンアウト (脚屈曲なし)</color>", _centerTagStyle);
        GUI.Label(new Rect(cx - 140f, cy + 4f, 130f, 18f), "← 左へ頭頂誘導", _centerTagStyle);
    }

    private void DrawBiomechanicsPanel(in SimState state, SkyTechSimConfig cfg)
    {
        Rect panelRect = new Rect(15, 15, 355, 480);
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
        curY += 16;

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
            $"<color=#5DADE2>[マウス左右 / A・D]</color> 頭頂誘導   |   <color=#F4D03F>[Q / E]</color> 股関節固定({_controller.TargetLegRollDeg:+0;-0;0}°)   |   <color=#2ECC71>[W]</color> 直立リセット   |   <color=#E74C3C>[R]</color> リセット",
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