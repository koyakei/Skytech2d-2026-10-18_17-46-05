using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace SkyTechSimulator
{
    /// <summary>
    /// AirPods (CMHeadphoneMotionManager) のヘッドトラッキング（頭の傾き・ロール・ピッチ）を管理するクラス
    /// iOS実機では HeadphoneMotionBridge.mm を呼び出し、
    /// Unityエディタ上では Q/E キー（頭の左右傾き）やマウス上下でシミュレーションテスト可能です。
    /// </summary>
    public static class AirPodsMotionManager
    {
#if !UNITY_EDITOR && UNITY_IOS
        [DllImport("__Internal")]
        private static extern void HeadphoneMotion_Start();

        [DllImport("__Internal")]
        private static extern void HeadphoneMotion_Stop();

        [DllImport("__Internal")]
        private static extern bool HeadphoneMotion_IsActive();

        [DllImport("__Internal")]
        private static extern void HeadphoneMotion_GetAttitude(out float roll, out float pitch, out float yaw);
#endif

        private static bool _isStarted = false;
        private static bool _editorSimulatedActive = false;

        // 生データ（ラジアン）
        private static float _rawRollRad = 0f;
        private static float _rawPitchRad = 0f;
        private static float _rawYawRad = 0f;

        // キャリブレーション用ニュートラルオフセット（ラジアン）
        private static float _neutralRollRad = 0f;
        private static float _neutralPitchRad = 0f;
        private static float _neutralYawRad = 0f;

        // 平滑化済み角度（度）
        private static float _smoothedRollDeg = 0f;
        private static float _smoothedPitchDeg = 0f;

        // エディタシミュレーション用
        private static float _editorSimRollDeg = 0f;
        private static float _editorSimPitchDeg = 0f;

        /// <summary>
        /// AirPods 等のヘッドフォンモーションが現在アクティブに動作しているか
        /// </summary>
        public static bool IsActive
        {
            get
            {
#if !UNITY_EDITOR && UNITY_IOS
                try { return HeadphoneMotion_IsActive(); } catch { return false; }
#else
                return _editorSimulatedActive;
#endif
            }
        }

        /// <summary>
        /// モーション更新の開始
        /// </summary>
        public static void Start()
        {
            if (_isStarted) return;
#if !UNITY_EDITOR && UNITY_IOS
            try
            {
                HeadphoneMotion_Start();
                _isStarted = true;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[AirPodsMotionManager] HeadphoneMotion_Start failed: {ex.Message}");
            }
#else
            _isStarted = true;
            _editorSimulatedActive = true;
#endif
        }

        /// <summary>
        /// モーション更新の停止
        /// </summary>
        public static void Stop()
        {
            if (!_isStarted) return;
#if !UNITY_EDITOR && UNITY_IOS
            try
            {
                HeadphoneMotion_Stop();
                _isStarted = false;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[AirPodsMotionManager] HeadphoneMotion_Stop failed: {ex.Message}");
            }
#else
            _isStarted = false;
            _editorSimulatedActive = false;
#endif
        }

        /// <summary>
        /// 現在の頭の姿勢を中立（0度）としてキャリブレーション
        /// </summary>
        public static void Calibrate()
        {
            UpdateRawAttitude();
            _neutralRollRad = _rawRollRad;
            _neutralPitchRad = _rawPitchRad;
            _neutralYawRad = _rawYawRad;

            _smoothedRollDeg = 0f;
            _smoothedPitchDeg = 0f;

#if UNITY_EDITOR
            _editorSimRollDeg = 0f;
            _editorSimPitchDeg = 0f;
#endif
            Debug.Log("<color=#58D68D>[AirPodsMotionManager] Calibrated neutral head attitude (0°).</color>");
        }

        /// <summary>
        /// ニュートラル補正済みのロール角 [度] (右傾き: +, 左傾き: -)
        /// </summary>
        public static float GetRollDeg()
        {
            UpdateRawAttitude();
#if !UNITY_EDITOR && UNITY_IOS
            float diffRad = _rawRollRad - _neutralRollRad;
            return diffRad * Mathf.Rad2Deg;
#else
            return _editorSimRollDeg;
#endif
        }

        /// <summary>
        /// ニュートラル補正済みのピッチ角 [度] (前傾うなずき: +, 後傾: -)
        /// </summary>
        public static float GetPitchDeg()
        {
            UpdateRawAttitude();
#if !UNITY_EDITOR && UNITY_IOS
            float diffRad = _rawPitchRad - _neutralPitchRad;
            return diffRad * Mathf.Rad2Deg;
#else
            return _editorSimPitchDeg;
#endif
        }

        /// <summary>
        /// ニュートラル補正済みのヨー角 [度] (首の左右振り向き)
        /// </summary>
        public static float GetYawDeg()
        {
            UpdateRawAttitude();
#if !UNITY_EDITOR && UNITY_IOS
            float diffRad = _rawYawRad - _neutralYawRad;
            return diffRad * Mathf.Rad2Deg;
#else
            return 0f;
#endif
        }

        /// <summary>
        /// 毎フレーム呼び出して平滑化されたロール角 [度] を取得（微小な手ブレ・頭ブレをカット）
        /// </summary>
        public static float UpdateAndGetSmoothedRollDeg(float dt, float smoothingTau = 0.08f)
        {
            float target = GetRollDeg();
            float alpha = dt / (Mathf.Max(0.005f, smoothingTau) + dt);
            _smoothedRollDeg = Mathf.Lerp(_smoothedRollDeg, target, alpha);
            return _smoothedRollDeg;
        }

        /// <summary>
        /// 毎フレーム呼び出して平滑化されたピッチ角 [度] を取得
        /// </summary>
        public static float UpdateAndGetSmoothedPitchDeg(float dt, float smoothingTau = 0.08f)
        {
            float target = GetPitchDeg();
            float alpha = dt / (Mathf.Max(0.005f, smoothingTau) + dt);
            _smoothedPitchDeg = Mathf.Lerp(_smoothedPitchDeg, target, alpha);
            return _smoothedPitchDeg;
        }

        /// <summary>
        /// 生の姿勢角度を更新
        /// </summary>
        private static void UpdateRawAttitude()
        {
#if !UNITY_EDITOR && UNITY_IOS
            if (!_isStarted) Start();
            try
            {
                HeadphoneMotion_GetAttitude(out _rawRollRad, out _rawPitchRad, out _rawYawRad);
            }
            catch
            {
                _rawRollRad = _rawPitchRad = _rawYawRad = 0f;
            }
#endif
        }

#if UNITY_EDITOR
        /// <summary>
        /// エディタ用シミュレーション入力の更新（Q/Eでロール傾き、キー離すと緩やかに中立復帰）
        /// </summary>
        public static void UpdateEditorSimulation(float dt, bool keyQ, bool keyE, float mouseDeltaY = 0f)
        {
            _editorSimulatedActive = true;
            float targetRoll = 0f;
            if (keyQ) targetRoll -= 35f;
            if (keyE) targetRoll += 35f;

            if (keyQ || keyE)
            {
                _editorSimRollDeg = Mathf.MoveTowards(_editorSimRollDeg, targetRoll, 90f * dt);
            }
            else
            {
                _editorSimRollDeg = Mathf.MoveTowards(_editorSimRollDeg, 0f, 60f * dt);
            }

            if (Mathf.Abs(mouseDeltaY) > 0.001f)
            {
                _editorSimPitchDeg = Mathf.Clamp(_editorSimPitchDeg + mouseDeltaY * 20f, -40f, 40f);
            }
            else
            {
                _editorSimPitchDeg = Mathf.MoveTowards(_editorSimPitchDeg, 0f, 25f * dt);
            }
        }

        /// <summary>エディタ用シミュレーション傾き角の直接設定</summary>
        public static void SetEditorSimulatedRoll(float rollDeg)
        {
            _editorSimRollDeg = rollDeg;
            _editorSimulatedActive = true;
        }

        /// <summary>エディタ用シミュレーション有効/無効設定</summary>
        public static void SetEditorSimulatedActive(bool active)
        {
            _editorSimulatedActive = active;
        }
#endif
    }
}
