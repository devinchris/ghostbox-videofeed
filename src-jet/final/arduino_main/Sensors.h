#ifndef SENSORS_H
#define SENSORS_H

#include <BMM150.h>
#include <BMI270.h>

class Sensors {
    public:
        Sensors();
        void init(const float SAMPLE_FREQ = 25.0f);

        void getSensorData(float& ax, float& ay, float& az,
                           float& gx, float& gy, float& gz,
                           float& mx, float& my, float& mz,
                           float& tempC
                           )
    private:
        BMI270 imu;
        BMM150 mag;
}

#endif