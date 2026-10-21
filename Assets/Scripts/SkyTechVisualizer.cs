using UnityEngine;
using SkyTechSimulator;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// SkyTechSportスキーシミュレーターのプロシージャル2DビジュアライザーおよびテレメトリHUD。
/// </summary>
[RequireComponent(typeof(SkyTechGameController))]
public class SkyTechVisualizer : MonoBehaviour
{
    [Header("=== 画面中央基準ガイド表示設定 ===")]
    [Tooltip("画面中央の十字基準線（左右重心移動・高さ基準）を表示するか")]
    public bool ShowCenterCrosshair = true;

    [Tooltip("画面中央から上方向の最大伸長(l_max)目安ラインまでの距離(m)")]
    public float GuideExtendUpRange = 0.55f;

    [Tooltip("画面中央から下方向の最小屈曲(l_min)目安ラインまでの距離(m)")]
    public float GuideFlexDownRange = 0.85f;

    private SkyTechGameController _controller;
    private Material _lineMaterial;

    // 画面中央基準ガイド用 LineRenderer
    private LineRenderer _centerHorizLine;
    private LineRenderer _centerVertLine;
    private LineRenderer _centerReticleCircle;
    private LineRenderer _heightMaxTickLine;
    private LineRenderer _heightMinTickLine;
    private LineRenderer _mouseGuideLine;
    private LineRenderer _mouseCursorCircle;

    private LineRenderer _railLine;
    private LineRenderer _absorberLeft;
    private LineRenderer _absorberRight;
    private LineRenderer _carriageLine;
    private LineRenderer _skiBoardLine;
    private LineRenderer _legLine;
    private LineRenderer _trunkLine;
    private LineRenderer _hipJointCircle;
    private LineRenderer _headCircle;
    private LineRenderer _comCircle;
    private LineRenderer _comLine;
    private LineRenderer _lowerComCircle;
    private LineRenderer _upperComCircle;
    private LineRenderer _twoComDistLine;
    private LineRenderer _forceLine;
    private LineRenderer _ghostPeakLine;

    private Texture2D _panelBgTex;
    private Texture2D _barBgTex;
    private Texture2D _barFillTex;
    private Texture2D _badgeTex;
    private GUIStyle _panelStyle;
    private GUIStyle _titleStyle;
    private GUIStyle _labelStyle;
    private GUIStyle _valueStyle;
    private GUIStyle _badgeStyle;
    private GUIStyle _guideStyle;
    private GUIStyle _centerTagStyle;
    private bool _guiInitialized = false;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void AutoInitializeIfMissing()
    {
        SkyTechGameController existing = Object.FindAnyObjectByType<SkyTechGameController>();
        if (existing == null)
        {
            GameObject simObj = new GameObject("SkyTechSimulator");
            simObj.AddComponent<SkyTechVisualizer>();
            Debug.Log("<color=#58D68D>[SkyTechVisualizer] シーン内にSkyTechSimulatorを自動生成しました。Playを開始します！</color>");
        }
    }

    private void Awake()
    {
        _controller = GetComponent<SkyTechGameController>();
        EnsureCameraSetup();

        Shader shader = Shader.Find("Universal Render Pipeline/2D/Sprite-Unlit-Default")
                     ?? Shader.Find("Universal Render Pipeline/Unlit")
                     ?? Shader.Find("Sprites/Default")
                     ?? Shader.Find("Unlit/Color")
                     ?? Shader.Find("Hidden/Internal-Colored");

        _lineMaterial = new Material(shader)
        {
            hideFlags = HideFlags.DontSave
        };

        SetupLineRenderers();
    }

    private void EnsureCameraSetup()
    {
        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (cam == null)
        {
            GameObject camObj = new GameObject("Main Camera");
            cam = camObj.AddComponent<Camera>();
            camObj.tag = "MainCamera";
        }

        cam.orthographic = true;
        float halfRail = (_controller != null && _controller.Config != null) ? _controller.Config.W_rail * 0.5f : 3.60f;
        float aspect = (cam.pixelWidth > 0 && cam.pixelHeight > 0) ? (float)cam.pixelWidth / cam.pixelHeight : (16f / 9f);
        float requiredSizeByWidth = (halfRail + 0.35f) / Mathf.Max(0.5f, aspect);
        cam.orthographicSize = Mathf.Max(1.6f, requiredSizeByWidth);
        cam.transform.position = new Vector3(0f, 0.85f, -10f);
        cam.transform.rotation = Quaternion.identity;
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
        if (_controller == null) return;

        SimState state = _controller.CurrentState;
        SimInput input = _controller.CurrentInput;
        SkyTechSimConfig cfg = _controller.Config;

        if (cfg == null) return;

        UpdateCenterCrosshairGeometry(state, input);
        UpdateDynamicGeometry(state, input, cfg);
    }

    private void SetupLineRenderers()
    {
        // 背景の画面中央基準ガイド（キャラクターやレールより背面に描画）
        _centerHorizLine     = CreateLineChild("CenterHorizGuide", 0.014f, new Color(0.55f, 0.75f, 0.95f, 0.22f), -4);
        _centerVertLine      = CreateLineChild("CenterVertGuide",  0.014f, new Color(0.55f, 0.75f, 0.95f, 0.28f), -4);
        _heightMaxTickLine   = CreateLineChild("HeightMaxTick",    0.016f, new Color(0.35f, 0.95f, 0.60f, 0.35f), -3);
        _heightMinTickLine   = CreateLineChild("HeightMinTick",    0.016f, new Color(0.35f, 0.75f, 1.00f, 0.35f), -3);
        _centerReticleCircle = CreateLineChild("CenterReticle",    0.015f, new Color(0.75f, 0.88f, 1.00f, 0.40f), -2);
        _mouseGuideLine      = CreateLineChild("MouseGuideLine",   0.012f, new Color(1.00f, 0.90f, 0.40f, 0.30f), -1);

        _railLine       = CreateLineChild("Rail",        0.035f, new Color(0.40f, 0.45f, 0.55f, 0.8f), 0);
        _absorberLeft   = CreateLineChild("AbsorberL",   0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _absorberRight  = CreateLineChild("AbsorberR",   0.030f, new Color(0.95f, 0.25f, 0.25f, 0.9f), 1);
        _ghostPeakLine  = CreateLineChild("GhostPeak",   0.020f, new Color(0.3f, 0.8f, 1.0f, 0.25f),   2);
        _carriageLine   = CreateLineChild("Carriage",    0.030f, new Color(0.75f, 0.80f, 0.90f, 0.9f), 3);
        _skiBoardLine   = CreateLineChild("SkiBoard",    0.045f, new Color(0.90f, 0.95f, 1.00f, 1.0f), 4);
        _legLine        = CreateLineChild("LegSegment",  0.065f, Color.cyan,                            5);
        _trunkLine      = CreateLineChild("TrunkSegment",0.075f, new Color(0.9f, 0.9f, 0.95f, 1.0f),    6);
        _hipJointCircle = CreateLineChild("HipJoint",    0.025f, new Color(1.0f, 0.8f, 0.2f, 1.0f),    7);
        _headCircle     = CreateLineChild("HeadCircle",  0.030f, new Color(1.0f, 0.85f, 0.7f, 1.0f),   7);
        _twoComDistLine = CreateLineChild("TwoCoMLine",  0.015f, new Color(1.0f, 1.0f, 1.0f, 0.45f),  8);
        _comLine        = CreateLineChild("CoMLine",     0.015f, new Color(1.0f, 0.95f, 0.2f, 0.45f),  8);
        _lowerComCircle = CreateLineChild("LowerCoMCircle", 0.025f, new Color(0.0f, 0.90f, 1.0f, 1.0f), 9);
        _upperComCircle = CreateLineChild("UpperCoMCircle", 0.042f, new Color(1.0f, 0.20f, 0.50f, 1.0f), 9);
        _comCircle      = CreateLineChild("CoMCircle",   0.025f, new Color(1.0f, 0.92f, 0.15f, 1.0f),  10);
        _forceLine      = CreateLineChild("ForceVector", 0.040f, Color.green,                           11);

        // マウスカーソル位置マーカー
        _mouseCursorCircle = CreateLineChild("MouseCursor", 0.018f, new Color(1.0f, 0.92f, 0.35f, 0.75f), 12);
    }

    private LineRenderer CreateLineChild(string objName, float width, Color color, int sortingOrder)
    {
        GameObject child = new GameObject($"_Vis_{objName}");
        child.transform.SetParent(transform, false);

        LineRenderer lr = child.AddComponent<LineRenderer>();
        lr.material = _lineMaterial;
        lr.useWorldSpace = true;
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = color;
        lr.endColor = color;
        lr.sortingLayerName = "Default";
        lr.sortingOrder = 100 + sortingOrder;
        lr.numCapVertices = 4;
        lr.numCornerVertices = 4;
        return lr;
    }

    private void UpdateRailGeometry()
    {
        SkyTechSimConfig cfg = _controller.Config;
        float halfRail = cfg != null ? cfg.W_rail * 0.5f : 3.60f;

        _railLine.positionCount = 2;
        _railLine.SetPosition(0, new Vector3(-halfRail, 0f, 0f));
        _railLine.SetPosition(1, new Vector3( halfRail, 0f, 0f));

        SetBoxPositions(_absorberLeft, new Vector3(-halfRail - 0.05f, 0f, 0f), new Vector2(0.10f, 0.14f));
        SetBoxPositions(_absorberRight, new Vector3(halfRail + 0.05f, 0f, 0f), new Vector2(0.10f, 0.14f));
    }

    private void UpdateCenterCrosshairGeometry(in SimState state, in SimInput input)
    {
        if (!ShowCenterCrosshair)
        {
            _centerHorizLine.positionCount = 0;
            _centerVertLine.positionCount = 0;
            _centerReticleCircle.positionCount = 0;
            _heightMaxTickLine.positionCount = 0;
            _heightMinTickLine.positionCount = 0;
            _mouseGuideLine.positionCount = 0;
            _mouseCursorCircle.positionCount = 0;
            return;
        }

        Camera cam = Camera.main ?? Object.FindAnyObjectByType<Camera>();
        if (cam == null) return;

        float depth = -cam.transform.position.z;
        Vector3 centerScreen = new Vector3(cam.pixelWidth * 0.5f, cam.pixelHeight * 0.5f, depth);
        Vector3 centerWorld3 = cam.ScreenToWorldPoint(centerScreen);
        Vector2 centerWorld = new Vector2(centerWorld3.x, centerWorld3.y);

        float orthoH = cam.orthographic ? cam.orthographicSize : 2.2f;
        float aspect = (cam.pixelWidth > 0 && cam.pixelHeight > 0) ? (float)cam.pixelWidth / cam.pixelHeight : (16f / 9f);
        float orthoW = orthoH * aspect;

        // 1. 画面中央の水平基準線（高さの基準 = 標準スタンス l_hold）
        _centerHorizLine.positionCount = 2;
        _centerHorizLine.SetPosition(0, new Vector3(centerWorld.x - orthoW * 0.95f, centerWorld.y, 0f));
        _centerHorizLine.SetPosition(1, new Vector3(centerWorld.x + orthoW * 0.95f, centerWorld.y, 0f));

        // 2. 画面中央の垂直基準線（左右重心移動の基準 = x_center）
        _centerVertLine.positionCount = 2;
        _centerVertLine.SetPosition(0, new Vector3(centerWorld.x, centerWorld.y - orthoH * 0.95f, 0f));
        _centerVertLine.SetPosition(1, new Vector3(centerWorld.x, centerWorld.y + orthoH * 0.95f, 0f));

        // 左右入力に応じて垂直基準線の色味をわずかにハイライト
        float steerMag = Mathf.Abs(input.phi_steer_cmd);
        Color vertCol = Color.Lerp(
            new Color(0.55f, 0.75f, 0.95f, 0.28f),
            new Color(0.40f, 0.90f, 1.00f, 0.50f),
            steerMag
        );
        _centerVertLine.startColor = vertCol;
        _centerVertLine.endColor = vertCol;

        // 3. 画面中央レティクル（原点リング）
        SetCirclePositions(_centerReticleCircle, new Vector3(centerWorld.x, centerWorld.y, 0f), 0.055f, 20);

        // 4. 上下の最大伸長(l_max)・最小屈曲(l_min)目盛りライン
        float tickHalfWidth = 0.28f;
        float upY = centerWorld.y + Mathf.Max(0.05f, GuideExtendUpRange);
        float downY = centerWorld.y - Mathf.Max(0.05f, GuideFlexDownRange);

        _heightMaxTickLine.positionCount = 2;
        _heightMaxTickLine.SetPosition(0, new Vector3(centerWorld.x - tickHalfWidth, upY, 0f));
        _heightMaxTickLine.SetPosition(1, new Vector3(centerWorld.x + tickHalfWidth, upY, 0f));
        Color maxTickCol = Color.Lerp(
            new Color(0.35f, 0.95f, 0.60f, 0.30f),
            new Color(0.35f, 1.00f, 0.55f, 0.85f),
            input.u_ext_norm
        );
        _heightMaxTickLine.startColor = maxTickCol;
        _heightMaxTickLine.endColor = maxTickCol;

        _heightMinTickLine.positionCount = 2;
        _heightMinTickLine.SetPosition(0, new Vector3(centerWorld.x - tickHalfWidth, downY, 0f));
        _heightMinTickLine.SetPosition(1, new Vector3(centerWorld.x + tickHalfWidth, downY, 0f));
        Color minTickCol = Color.Lerp(
            new Color(0.35f, 0.75f, 1.00f, 0.30f),
            new Color(0.30f, 0.85f, 1.00f, 0.85f),
            input.u_flex_norm
        );
        _heightMinTickLine.startColor = minTickCol;
        _heightMinTickLine.endColor = minTickCol;

        // 5. マウスカーソル位置と画面中央からのオフセットガイド線
        if (_controller.UseMouseControl && TryGetMouseWorldPosForVis(cam, out Vector2 mouseWorld))
        {
            Vector3 mPos3 = new Vector3(mouseWorld.x, mouseWorld.y, 0f);
            SetCirclePositions(_mouseCursorCircle, mPos3, 0.042f, 16);

            // 画面中央 → (mouseWorld.x, centerWorld.y) → (mouseWorld.x, mouseWorld.y) のL字オフセット表示
            _mouseGuideLine.positionCount = 3;
            _mouseGuideLine.SetPosition(0, new Vector3(centerWorld.x, centerWorld.y, 0f));
            _mouseGuideLine.SetPosition(1, new Vector3(mouseWorld.x, centerWorld.y, 0f));
            _mouseGuideLine.SetPosition(2, mPos3);

            bool isRight = (mouseWorld.x - centerWorld.x) > _controller.MouseLineDeadzone;
            bool isLeft = (mouseWorld.x - centerWorld.x) < -_controller.MouseLineDeadzone;
            Color cursorCol = isRight
                ? new Color(0.35f, 0.90f, 1.00f, 0.80f)
                : (isLeft ? new Color(1.00f, 0.65f, 0.25f, 0.80f) : new Color(0.85f, 0.90f, 0.95f, 0.65f));
            _mouseCursorCircle.startColor = cursorCol;
            _mouseCursorCircle.endColor = cursorCol;
        }
        else
        {
            _mouseGuideLine.positionCount = 0;
            _mouseCursorCircle.positionCount = 0;
        }
    }

    private static bool TryGetMouseWorldPosForVis(Camera cam, out Vector2 worldPos)
    {
        worldPos = Vector2.zero;
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

        screenPos.z = -cam.transform.position.z;
        Vector3 wp = cam.ScreenToWorldPoint(screenPos);
        worldPos = new Vector2(wp.x, wp.y);
        return true;
    }

    private void UpdateDynamicGeometry(in SimState state, in SimInput input, SkyTechSimConfig cfg)
    {
        Vector3 P = new Vector3(state.x_c, 0f, 0f);

        SetBoxPositions(_carriageLine, P + new Vector3(0f, -0.03f, 0f), new Vector2(0.24f, 0.06f));

        Vector3 skiTangent = new Vector3(Mathf.Cos(state.theta1), -Mathf.Sin(state.theta1), 0f);
        _skiBoardLine.positionCount = 2;
        _skiBoardLine.SetPosition(0, P - skiTangent * 0.16f);
        _skiBoardLine.SetPosition(1, P + skiTangent * 0.16f);

        float sinTheta1 = Mathf.Sin(state.theta1);
        float cosTheta1 = Mathf.Cos(state.theta1);
        Vector3 H = P + new Vector3(state.l1 * sinTheta1, state.l1 * cosTheta1, 0f);

        _legLine.positionCount = 2;
        _legLine.SetPosition(0, P);
        _legLine.SetPosition(1, H);

        Color legColor = Color.Lerp(new Color(0.2f, 0.85f, 1.0f, 1f), new Color(1.0f, 0.45f, 0.1f, 1f), input.u_ext_norm);
        _legLine.startColor = legColor;
        _legLine.endColor = legColor;

        SetCirclePositions(_hipJointCircle, H, 0.040f, 16);

        float theta2 = state.theta1 - state.phi;
        float sinTheta2 = Mathf.Sin(theta2);
        float cosTheta2 = Mathf.Cos(theta2);
        Vector3 trunkVec = new Vector3(sinTheta2, cosTheta2, 0f);
        Vector3 U = H + trunkVec * cfg.l2;
        Vector3 HeadPos = H + trunkVec * (cfg.l2 * 1.55f);

        _trunkLine.positionCount = 2;
        _trunkLine.SetPosition(0, H);
        _trunkLine.SetPosition(1, U);

        SetCirclePositions(_headCircle, HeadPos, 0.075f, 20);

        Vector3 CoM1 = P + new Vector3(cfg.c1 * state.l1 * sinTheta1, cfg.c1 * state.l1 * cosTheta1, 0f);
        SetCirclePositions(_lowerComCircle, CoM1, 0.040f, 16);

        Vector3 CoM2 = H + trunkVec * cfg.l2;
        SetCirclePositions(_upperComCircle, CoM2, 0.042f, 16);

        _twoComDistLine.positionCount = 2;
        _twoComDistLine.SetPosition(0, CoM1);
        _twoComDistLine.SetPosition(1, CoM2);

        Vector3 C = P + new Vector3(state.y_CoM, state.z_CoM, 0f);
        SetCirclePositions(_comCircle, C, 0.048f, 18);

        _comLine.positionCount = 2;
        _comLine.SetPosition(0, P);
        _comLine.SetPosition(1, C);

        Vector3 legAxis = new Vector3(sinTheta1, cosTheta1, 0f);
        float mg = cfg.M * cfg.g;
        float forceScale = 0.35f;
        float forceLen = (mg > 0.001f) ? (state.F_leg / mg) * forceScale : 0f;

        Vector3 forceEnd = P + legAxis * forceLen;
        _forceLine.positionCount = 2;
        _forceLine.SetPosition(0, P);
        _forceLine.SetPosition(1, forceEnd);

        Color forceColor;
        switch (state.gripState)
        {
            case SkiGripState.Carving:
                forceColor = new Color(0.2f, 0.95f, 0.3f, 1f);
                break;
            case SkiGripState.Unweighted:
                forceColor = new Color(1.0f, 0.85f, 0.2f, 1f);
                break;
            case SkiGripState.Skidding:
                forceColor = new Color(1.0f, 0.25f, 0.25f, 1f);
                break;
            case SkiGripState.RailLimitHit:
                forceColor = new Color(1.0f, 0.55f, 0.1f, 1f);
                break;
            default:
                forceColor = new Color(0.9f, 0.2f, 0.95f, 1f);
                break;
        }
        _forceLine.startColor = forceColor;
        _forceLine.endColor = forceColor;

        float peakRad = _controller.CurrentTurnPeakEdgeDeg * Mathf.Deg2Rad;
        if (peakRad > 0.05f)
        {
            float sign = Mathf.Sign(state.theta1);
            if (sign == 0f) sign = 1f;
            Vector3 ghostDir = new Vector3(Mathf.Sin(peakRad * sign), Mathf.Cos(peakRad * sign), 0f);
            _ghostPeakLine.positionCount = 2;
            _ghostPeakLine.SetPosition(0, P);
            _ghostPeakLine.SetPosition(1, P + ghostDir * 0.85f);
        }
        else
        {
            _ghostPeakLine.positionCount = 0;
        }
    }

    private void SetCirclePositions(LineRenderer lr, Vector3 center, float radius, int segments)
    {
        lr.positionCount = segments + 1;
        float deltaAngle = (Mathf.PI * 2f) / segments;
        for (int i = 0; i <= segments; i++)
        {
            float angle = i * deltaAngle;
            lr.SetPosition(i, center + new Vector3(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius, 0f));
        }
    }

    private void SetBoxPositions(LineRenderer lr, Vector3 center, Vector2 size)
    {
        lr.positionCount = 5;
        float hx = size.x * 0.5f;
        float hy = size.y * 0.5f;
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

        _panelStyle = new GUIStyle(GUI.skin.box)
        {
            normal = { background = _panelBgTex },
            padding = new RectOffset(12, 12, 10, 10)
        };

        _titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 13,
            fontStyle = FontStyle.Bold,
            normal = { textColor = new Color(0.4f, 0.85f, 1.0f) },
            alignment = TextAnchor.MiddleLeft
        };

        _labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            normal = { textColor = new Color(0.80f, 0.85f, 0.92f) },
            alignment = TextAnchor.MiddleLeft
        };

        _valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.white },
            alignment = TextAnchor.MiddleRight
        };

        _badgeStyle = new GUIStyle(GUI.skin.box)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { background = _badgeTex, textColor = Color.white }
        };

        _guideStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            normal = { textColor = new Color(0.75f, 0.82f, 0.90f) },
            alignment = TextAnchor.MiddleCenter
        };

        _centerTagStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 10,
            normal = { textColor = new Color(0.65f, 0.82f, 0.98f, 0.65f) },
            alignment = TextAnchor.MiddleLeft,
            richText = true
        };

        _guiInitialized = true;
    }

    private void OnGUI()
    {
        if (_controller == null) return;

        InitGUIStyles();

        SimState state = _controller.CurrentState;
        SkyTechSimConfig cfg = _controller.Config;
        if (cfg == null) return;

        DrawCenterGuideLabels();
        DrawBiomechanicsPanel(state, cfg);
        DrawScorePanel();
        DrawControlsGuidePanel();
    }

    private void DrawCenterGuideLabels()
    {
        if (!ShowCenterCrosshair) return;

        float cx = Screen.width * 0.5f;
        float cy = Screen.height * 0.5f;

        GUI.Label(new Rect(cx + 10f, cy - 18f, 190f, 18f), "CENTER (左右重心中立 / 基準高)", _centerTagStyle);
        GUI.Label(new Rect(cx - 125f, cy + 4f, 115f, 18f), "← 左重心シフト", _centerTagStyle);
        GUI.Label(new Rect(cx + 10f, cy + 4f, 115f, 18f), "右重心シフト →", _centerTagStyle);
    }

    private void DrawBiomechanicsPanel(in SimState state, SkyTechSimConfig cfg)
    {
        Rect panelRect = new Rect(15, 15, 355, 480);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        float curY = panelRect.y + 10;
        float leftX = panelRect.x + 12;
        float contentW = panelRect.width - 24;

        GUI.Label(new Rect(leftX, curY, contentW, 20), "BIOMECHANICS & TELEMETRY", _titleStyle);
        curY += 24;

        Color badgeBg;
        string badgeText;
        switch (state.gripState)
        {
            case SkiGripState.Carving:
                badgeBg = new Color(0.15f, 0.70f, 0.35f);
                badgeText = "GRIP: CARVING";
                break;
            case SkiGripState.Unweighted:
                badgeBg = new Color(0.85f, 0.65f, 0.10f);
                badgeText = "GRIP: UNWEIGHTED";
                break;
            case SkiGripState.Skidding:
                badgeBg = new Color(0.85f, 0.20f, 0.20f);
                badgeText = "SKIDDING (SNOW FAILURE!)";
                break;
            case SkiGripState.RailLimitHit:
                badgeBg = new Color(0.90f, 0.45f, 0.10f);
                badgeText = "STATUS: RAIL LIMIT HIT";
                break;
            case SkiGripState.BootOut:
                badgeBg = new Color(0.65f, 0.15f, 0.75f);
                badgeText = "CRASH: BOOT OUT!";
                break;
            case SkiGripState.HipCrash:
                badgeBg = new Color(0.75f, 0.10f, 0.20f);
                badgeText = "CRASH: HIP DOWN!";
                break;
            default:
                badgeBg = Color.gray;
                badgeText = state.gripState.ToString();
                break;
        }

        DrawBadge(new Rect(leftX, curY, contentW, 22), badgeText, badgeBg, Color.white);
        curY += 28;

        float lowHipDist = (1f - cfg.c1) * state.l1;
        float twoComGap = Mathf.Sqrt(lowHipDist * lowHipDist + cfg.l2 * cfg.l2 + 2f * lowHipDist * cfg.l2 * Mathf.Cos(state.phi));
        DrawMetricRow(leftX, ref curY, contentW, "CoM Gap <color=#FF3366>●上</color>-<color=#00E5FF>●下</color>", $"{twoComGap * 100f:F1} cm (基準: {cfg.CoMDistanceDefault * 100f:F0}cm)");
        curY += 2;

        float edgeDeg = Mathf.Abs(state.theta1) * Mathf.Rad2Deg;
        float comDeg = Mathf.Abs(state.Psi_CoM) * Mathf.Rad2Deg;
        float targetDeg = _controller.TargetLegRollDeg;
        DrawMetricRow(leftX, ref curY, contentW, "Edge θ1 vs <color=#FFEB3B>CoM</color> Ψ", $"{state.theta1 * Mathf.Rad2Deg:+0.0;-0.0;0.0}° [目標: {targetDeg:+0;-0;0}°] / {comDeg:F1}°");
        DrawProgressBar(new Rect(leftX, curY, contentW, 10), edgeDeg / 78.0f, new Color(0.2f, 0.8f, 1f));
        curY += 16;

        float phiDeg = state.phi * Mathf.Rad2Deg;
        string shiftInfo = (Mathf.Abs(state.dy_actual) > 0.005f) ? $" [Δy:{state.dy_actual * 100f:+0.0;-0.0;0.0}cm]" : "";
        DrawMetricRow(leftX, ref curY, contentW, "Hip φ / CoM Shift", $"{phiDeg:F1}° (±{cfg.phi_out_maxDeg:F0}°){shiftInfo}");
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
        float fLegG = (mg > 0.001f) ? (state.F_leg / mg) : 0f;
        float fVertG = (mg > 0.001f) ? (state.F_vert / mg) : 0f;
        DrawMetricRow(leftX, ref curY, contentW, "Load: F_leg / F_vert", $"{fLegG:F2} G / 鉛直:{fVertG:F2} G (ストレス:{state.snow_stress_ratio * 100f:F0}%)");
        curY += 2;

        float realLatG = (cfg.g > 0.001f) ? (Mathf.Abs(state.a_c) / cfg.g) : 0f;
        DrawMetricRow(leftX, ref curY, contentW, "Lateral G (Base / Real)", $"{Mathf.Abs(state.G_base):F2} G / {realLatG:F2} G (λ:{state.lambda_sustain * 100f:F0}%)");
        curY += 2;

        float vKmh = state.V * 3.6f;
        DrawMetricRow(leftX, ref curY, contentW, "Speed V", $"{vKmh:F1} km/h ({state.V:F1} m/s)");
        curY += 2;

        float carriageKmh = state.v_c * 3.6f;
        float targetCarriageKmh = state.v_c_target * 3.6f;
        DrawMetricRow(leftX, ref curY, contentW, "Carriage X / Speed", $"{state.x_c:+0.00;-0.00;0.00}m ({carriageKmh:+0.0;-0.0;0.0} / 目標:{targetCarriageKmh:+0.0;-0.0;0.0}km/h)");
        curY += 2;

        float currentR = (Mathf.Abs(state.kappa) > 0.005f) ? (1.0f / Mathf.Abs(state.kappa)) : 99.9f;
        DrawMetricRow(leftX, ref curY, contentW, "Flex Curvature κ", $"{state.kappa:+0.000;-0.000;0.000} /m (R={currentR:F1}m)");
        curY += 2;

        float staminaNorm = (cfg.maxStamina > 0.001f) ? Mathf.Clamp01(state.stamina / cfg.maxStamina) : 1f;
        DrawMetricRow(leftX, ref curY, contentW, "Anaerobic Stamina", $"{state.stamina:F0} J ({staminaNorm * 100f:F0}%)");
        Color staminaColor = staminaNorm > 0.35f ? new Color(0.3f, 0.9f, 0.4f) : new Color(0.95f, 0.3f, 0.2f);
        DrawProgressBar(new Rect(leftX, curY, contentW, 10), staminaNorm, staminaColor);
        curY += 16;

        string assistStr = _controller.AutoAngulationAssist ? "<color=#2ECC71>AUTO-ASSIST: ON</color>" : "<color=#BDC3C7>MANUAL (Tab)</color>";
        GUI.Label(new Rect(leftX, curY, contentW, 18), $"Control: {assistStr}", _labelStyle);
    }

    private void DrawScorePanel()
    {
        Rect panelRect = new Rect(Screen.width - 325, 15, 310, 205);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        float curY = panelRect.y + 10;
        float leftX = panelRect.x + 12;
        float contentW = panelRect.width - 24;

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

        Color evalBg = new Color(0.20f, 0.45f, 0.75f);
        if (_controller.LastTurnEvaluation.Contains("WORLD CUP"))
            evalBg = new Color(0.85f, 0.55f, 0.05f);
        else if (_controller.LastTurnEvaluation.Contains("DEEP"))
            evalBg = new Color(0.15f, 0.65f, 0.40f);
        else if (_controller.LastTurnEvaluation.Contains("CRASH") || _controller.LastTurnEvaluation.Contains("RAIL"))
            evalBg = new Color(0.80f, 0.20f, 0.20f);

        DrawBadge(new Rect(leftX, curY, contentW, 26), _controller.LastTurnEvaluation, evalBg, Color.white);
    }

    private void DrawControlsGuidePanel()
    {
        float panelW = Mathf.Min(760, Screen.width - 30);
        Rect panelRect = new Rect((Screen.width - panelW) * 0.5f, Screen.height - 62, panelW, 50);
        GUI.Box(panelRect, GUIContent.none, _panelStyle);

        float curY = panelRect.y + 6;
        GUI.Label(new Rect(panelRect.x, curY, panelW, 18),
            $"<color=#F4D03F>[Q / E]</color> ロール倒れ込み(±2° / 長押し連続: {_controller.TargetLegRollDeg:+0;-0;0}°)   |   <color=#2ECC71>[W]</color> 中立(0°)リセット   |   <color=#58D68D>[Space / ↑ / マウス上]</color> 脚伸展   |   <color=#85C1E9>[S / マウス下]</color> 屈曲",
            _guideStyle);

        curY += 18;
        GUI.Label(new Rect(panelRect.x, curY, panelW, 18),
            "<color=#5DADE2>[A / D or ← / → or 画面中央より左右マウス]</color> 体幹側屈(重心移動)   |   <color=#F39C12>[↓]</color> 側屈中立戻し   |   <color=#E74C3C>[R]</color> リセット",
            _guideStyle);
    }

    private void DrawMetricRow(float x, ref float y, float width, string label, string value)
    {
        float labelW = width * 0.46f;
        float valueW = width * 0.54f;
        GUI.Label(new Rect(x, y, labelW, 18), label, _labelStyle);
        GUI.Label(new Rect(x + labelW, y, valueW, 18), value, _valueStyle);
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
            Rect fillRect = new Rect(rect.x + 1, rect.y + 1, (rect.width - 2) * fill01, rect.height - 2);
            GUI.color = fillColor;
            GUI.DrawTexture(fillRect, _barFillTex);
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

    private Texture2D CreateSolidTexture(Color color)
    {
        Texture2D tex = new Texture2D(1, 1, TextureFormat.RGBA32, false)
        {
            hideFlags = HideFlags.DontSave
        };
        tex.SetPixel(0, 0, color);
        tex.Apply();
        return tex;
    }

    private void OnDestroy()
    {
        if (_lineMaterial != null) Destroy(_lineMaterial);
        if (_panelBgTex != null) Destroy(_panelBgTex);
        if (_barBgTex != null) Destroy(_barBgTex);
        if (_barFillTex != null) Destroy(_barFillTex);
        if (_badgeTex != null) Destroy(_badgeTex);
    }
}