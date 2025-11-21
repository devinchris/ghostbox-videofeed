#ifndef SmoothData4D_h
#define SmoothData4D_h
#include <math.h>
#include <ReefwingAHRS.h>

class SmoothQuaternionData {
    public:
        SmoothQuaternionData(); // Konstruktor
        void smoothQuaternion(Quaternion& _Quat, unsigned long currentTime);
        void initSmoothing(float targetFrequency, float alpha=0.47f);

    private:
        unsigned long lastSmoothedTime;
        float targetFrequency;
        float alpha;
        float FAST_ALPHA;
        float DEADZONE;
        float MOTION_THRESHOLD;
        float _smoothedQuaternions[4];
        float _lastQuaternions[4];
        float qDiff[4];
        float qBetrag;
        bool _isInitialized;
        void normalizeQuaternion(float q[4]);
        /*
         *int T_RELAX;
         *int delta;
         *unsigned long deltaTime;
         */
};

#endif