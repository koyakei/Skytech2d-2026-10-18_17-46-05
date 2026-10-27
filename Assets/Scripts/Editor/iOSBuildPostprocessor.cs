#if UNITY_EDITOR
using System.IO;
using UnityEditor;
using UnityEditor.Callbacks;
using UnityEditor.iOS.Xcode;
using UnityEngine;

namespace SkyTechSimulator.Editor
{
    /// <summary>
    /// iOSビルド時に Info.plist へ NSMotionUsageDescription を追加し、
    /// CoreMotion.framework を Xcode プロジェクトに確実にリンクするポストプロセス
    /// </summary>
    public static class iOSBuildPostprocessor
    {
        [PostProcessBuild(999)]
        public static void OnPostProcessBuild(BuildTarget target, string pathToBuiltProject)
        {
            if (target != BuildTarget.iOS) return;

            // 1. Info.plist に NSMotionUsageDescription を追加
            string plistPath = Path.Combine(pathToBuiltProject, "Info.plist");
            if (File.Exists(plistPath))
            {
                var plist = new PlistDocument();
                plist.ReadFromFile(plistPath);

                PlistElementDict rootDict = plist.root;
                const string motionKey = "NSMotionUsageDescription";
                const string motionDesc = "AirPodsの頭部モーション（傾き・うなずき）を取得してスキーヤーの姿勢制御を行うために使用します。";

                rootDict.SetString(motionKey, motionDesc);
                plist.WriteToFile(plistPath);
                Debug.Log($"<color=#58D68D>[iOSBuildPostprocessor] Info.plist に {motionKey} を設定しました。</color>");
            }

            // 2. Xcode プロジェクトに CoreMotion.framework をリンク
            string projPath = PBXProject.GetPBXProjectPath(pathToBuiltProject);
            if (File.Exists(projPath))
            {
                var pbx = new PBXProject();
                pbx.ReadFromFile(projPath);

                // Unity 2019.3+ では UnityFramework ターゲットにフレームワークを追加
                string frameworkTargetGuid = pbx.GetUnityFrameworkTargetGuid();
                if (!string.IsNullOrEmpty(frameworkTargetGuid))
                {
                    pbx.AddFrameworkToProject(frameworkTargetGuid, "CoreMotion.framework", false);
                }

                string mainTargetGuid = pbx.GetUnityMainTargetGuid();
                if (!string.IsNullOrEmpty(mainTargetGuid))
                {
                    pbx.AddFrameworkToProject(mainTargetGuid, "CoreMotion.framework", false);
                }

                pbx.WriteToFile(projPath);
                Debug.Log("<color=#58D68D>[iOSBuildPostprocessor] CoreMotion.framework を Xcode プロジェクトにリンクしました。</color>");
            }
        }
    }
}
#endif

