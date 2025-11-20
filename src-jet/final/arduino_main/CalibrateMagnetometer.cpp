/* #include "CalibrateMagnetometer.h"
#include "Arduino_BMI270_BMM150.h"
#include <algorithm>                // Nötig für std::max_element


CalibrateMagnetometer::CalibrateMagnetometer(){

}

void CalibrateMagnetometer::calibrate(){
    int counter = 0;

    while(counter < CALIBRATION_SAMPLES){
        if(!IMU.magneticFieldAvailable()){
            break;
        }
        IMU.readMagneticField(mx_raw, my_raw, mz_raw);

        _mx[counter] = mx_raw;
        _my[counter] = my_raw;
        _mz[counter] = mz_raw;

        if(mx_raw > mx_max){
            mx_max = mx_raw;
        } else if( mx_raw < mx_min){
            mx_min = mx_raw;
        }

        if(my_raw > my_max){
            my_max = my_raw;
        } else if( my_raw < my_min){
            my_min = my_raw;
        }

        if(mz_raw > mz_max){
            mz_max = mz_raw;
        } else if( mz_raw < mz_min){
            mz_min = mz_raw;
        }

        counter++;
    }

    isCalibrated = true;
}

void CalibrateMagnetometer::getHardIronOffset(float& mh_x, float& mh_y, float& mh_z){
    if(!isCalibrated){
        calibrate();
    }
    mh_x = mx_min + (mx_max - mx_min) / 2.0f;
    mh_y = my_min + (my_max - my_min) / 2.0f;
    mh_z = mz_min + (mz_max - mz_min) / 2.0f;
}

 */