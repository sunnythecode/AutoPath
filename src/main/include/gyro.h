#include "AHRS.h"

class gyro {
    public:
    AHRS* ahrs;
    gyro() {
        ahrs = new AHRS(frc::SerialPort::kMXP);
    }



};