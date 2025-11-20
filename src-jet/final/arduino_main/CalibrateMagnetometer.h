/* #ifndef CalibrateMagnetometer_h
#define CalibrateMagnetometer_h

#include "Arduino_BMI270_BMM150.h"
#include <NanoBLEFlashPrefs.h>


class CalibrateMagnetometer {
public:
    CalibrateMagnetometer();

    void calibrate();

    void getHardIronOffset(float& mh_x, float& mh_y, float& mh_z);
    float MAG_HARD_IRON_OFFSET[3] = {0, 0, 0};


    float MAG_SOFT_IRON_OFFSET[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    }

private:
    bool isCalibrated;
    NanoBLEFlashPrefs flashPrefs;

    static const int CALIBRATION_SAMPLES = 2000;
    float mx_raw, my_raw, mz_raw;
    float[2000] _mx, _my, _mz;

    float mx_min, my_min, mz_min;
    float mx_max, my_max, mz_max;

    bool isCalibrated = false;
}
#endif */