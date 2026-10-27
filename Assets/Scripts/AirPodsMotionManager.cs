using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace SkyTechSimulator
{
    /// <summary>
    /// AirPods (CMHeadphoneMotionManager) のヘッドトラッキング（頭の傾き・ロール・ピッチ）を管理するクラス
    /// iOS実機ではCoreMotionのヘッドフォンモーションAPIを直接呼び出し、
    /// Unityエディタではキーボード（Z/Cキー）やインスペクターでシミュレーション可能
    /// </summary>
    public static class AirPodsMotionManager
    {
#if !UNITY_EDITOR && UNITY_IOS
        [DllImport("__Internal")]
        private static extern bool AirPods_IsAvailable();

        [DllImport("__Internal")]
        private static extern bool AirPods_IsConnected();

        [DllImport("__Internal")]
        private static extern void AirPods_Start();

        [DllImport("__Internal")]
        private static extern void AirPods_Stop();

        [DllImport("__Internal")]
        private static extern void AirPods_Calibrate();

        [DllImport("__Internal")]
        private static extern float AirPods_GetRollDeg();

        [DllImport("__Internal")]
        private static extern float AirPods_GetPitchDeg();

        [DllImport("__Internal")]
        private static extern float AirPods_GetYawDeg();
#else
        private static bool _editorSimulatedConnected = false;
        private static float _editorSimulatedRollDeg = 0f;
        private static float _editorSimulatedPitchDeg = 0f;
        private static float _editorNeutralRollOffset = 0f;
#endif

        private static bool _isStarted = false;
        private static float _smoothedRollDeg = 0f;
        private static float _smoothedPitchDeg = 0f;

        /// <summary>デバイス/OSがAirPodsモーションに対応しているか</summary>
        public static bool IsAvailable
        {
            get
            {
#if !UNITY_EDITOR && UNITY_IOS
                try { return AirPods_IsAvailable(); } catch { return false; }
#else
                return true;
#endif
            }
        }

        /// <summary>AirPods等の対応ヘッドフォンが現在接続されているか</summary>
        public static bool IsConnected
        {
            get
            {
#if !UNITY_EDITOR && UNITY_IOS
                try { return AirPods_IsConnected(); } catch { return false; }
#else
                return _editorSimulatedConnected;
#endif
            }
        }

        /// <summary>モーション更新の開始</summary>
        public static void Start()
        {
            if (_isStarted) return;
#if !UNITY_EDITOR && UNITY_IOS
            try
            {
                AirPods_Start();
                _isStarted = true;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[AirPodsMotionManager] Start failed: {ex.Message}");
            }
#else
            _isStarted = true;
#endif
        }

        /// <summary>モーション更新の停止</summary>
        public static void Stop()
        {
            if (!_isStarted) return;
#if !UNITY_EDITOR && UNITY_IOS
            try
            {
                AirPods_Stop();
                _isStarted = false;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[AirPodsMotionManager] Stop failed: {ex.Message}");
            }
#else
            _isStarted = false;
#endif
        }

        /// <summary>現在の頭の傾きを中立（0度）としてキャリブレーション</summary>
        public static void Calibrate()
        {
#if !UNITY_EDITOR && UNITY_IOS
            try { AirPods_Calibrate(); } catch { }
#else
            _editorNeutralRollOffset = _editorSimulatedRollDeg;
#endif
            _smoothedRollDeg = 0f;
            _smoothedPitchDeg = 0f;
        }

        /// <summary>
        /// 頭の左右かしげロール角 [度] (右傾き: +, 左傾き: -)
        /// </summary>
        public static float GetRollDeg()
        {
#if !UNITY_EDITOR && UNITY_IOS
            try { return AirPods_GetRollDeg(); } catch { return 0f; }
#else
            return Mathf.DeltaAngle(_editorNeutralRollOffset, _editorSimulatedRollDeg);
#endif
        }

        /// <summary>
        /// 頭の前後うなずきピッチ角 [度] (前傾: +, 後傾: -)
        /// </summary>
        public static float GetPitchDeg()
        {
#if !UNITY_EDITOR && UNITY_IOS
            try { return AirPods_GetPitchDeg(); } catch { return 0f; }
#else
            return _editorSimulatedPitchDeg;
#endif
        }

        /// <summary>
        /// 毎フレーム呼び出して平滑化されたロール角を取得（ノイズ低減）
        /// </summary>
        public static float UpdateAndGetSmoothedRollDeg(float dt, float smoothingTau = 0.08f)
        {
            float rawRoll = GetRollDeg();
            float alpha = dt / (Mathf.Max(0.005f, smoothingTau) + dt);
            _smoothedRollDeg = Mathf.Lerp(_smoothedRollDeg, rawRoll, alpha);
            return _smoothedRollDeg;
        }

        /// <summary>
        /// 毎フレーム呼び出して平滑化されたピッチ角を取得（ノイズ低減）
        /// </summary>
        public static float UpdateAndGetSmoothedPitchDeg(float dt, float smoothingTau = 0.08f)
        {
            float rawPitch = GetPitchDeg();
            float alpha = dt / (Mathf.Max(0.005f, smoothingTau) + dt);
            _smoothedPitchDeg = Mathf.Lerp(_smoothedPitchDeg, rawPitch, alpha);
            return _smoothedPitchDeg;
        }

#if UNITY_EDITOR
        /// <summary>エディタ用シミュレーション接続状態の設定</summary>
        public static void SetEditorSimulatedConnected(bool connected)
        {
            _editorSimulatedConnected = connected;
        }

        /// <summary>エディタ用シミュレーション傾き角の設定</summary>
        public static void SetEditorSimulatedRoll(float rollDeg)
        {
            _editorSimulatedRollDeg = rollDeg;
            _editorSimulatedConnected = true;
        }
#endif
    }
}
