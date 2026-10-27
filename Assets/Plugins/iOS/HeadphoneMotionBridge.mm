#import <CoreMotion/CoreMotion.h>

static CMHeadphoneMotionManager *_headphoneManager = nil;

extern "C" {
    void HeadphoneMotion_Start() {
        if (_headphoneManager == nil) {
            _headphoneManager = [[CMHeadphoneMotionManager alloc] init];
        }
        if ([_headphoneManager isDeviceMotionAvailable] && ![_headphoneManager isDeviceMotionActive]) {
            [_headphoneManager startDeviceMotionUpdates];
        }
    }

    void HeadphoneMotion_Stop() {
        if (_headphoneManager != nil && [_headphoneManager isDeviceMotionActive]) {
            [_headphoneManager stopDeviceMotionUpdates];
        }
    }

    bool HeadphoneMotion_IsActive() {
        return (_headphoneManager != nil && [_headphoneManager isDeviceMotionActive] && _headphoneManager.deviceMotion != nil);
    }

    // Roll(頭を左右にかしげる角度: ラジアン), Pitch(前後頷き), Yaw を取得
    void HeadphoneMotion_GetAttitude(float *roll, float *pitch, float *yaw) {
        if (_headphoneManager != nil && _headphoneManager.deviceMotion != nil) {
            CMAttitude *att = _headphoneManager.deviceMotion.attitude;
            *roll  = (float)att.roll;
            *pitch = (float)att.pitch;
            *yaw   = (float)att.yaw;
        } else {
            *roll = *pitch = *yaw = 0.0f;
        }
    }
}

