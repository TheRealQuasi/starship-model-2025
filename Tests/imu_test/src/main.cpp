
#include "starship_imu.h"

STARSHIP_IMU starship_imu;

void setup(void) {
  starship_imu.start_imu();
}

void loop(void) {
    // bmi088.getAcceleration(&ax, &ay, &az);
    // bmi088.getGyroscope(&gx, &gy, &gz);
    // temp = bmi088.getTemperature();

    // Serial.print(ax);
    // Serial.print(",");
    // Serial.print(ay);
    // Serial.print(",");
    // Serial.print(az);
    // Serial.print(",");

    // Serial.print(gx);
    // Serial.print(",");
    // Serial.print(gy);
    // Serial.print(",");
    // Serial.print(gz);
    // Serial.print(",");

    // Serial.print(temp);

    // Serial.println();

    // delay(50);
}
