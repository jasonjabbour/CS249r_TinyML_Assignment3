/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @brief   Sample & upload data to Edge Impulse Studio.
 * @details Select 1 or multiple sensors by un-commenting the defines and select
 * a desired sample frequency. When this sketch runs, you can see raw sample
 * values outputted over the serial line. Now connect to the studio using the
 * `edge-impulse-data-forwarder` and start capturing data
 */
#define SAMPLE_ACCELEROMETER
#define SAMPLE_GYROSCOPE
//#define SAMPLE_PROXIMITY

/**
 * Configure the sample frequency. This is the frequency used to send the data
 * to the studio regardless of the frequency used to sample the data from the
 * sensor. This differs per sensors, and can be modified in the API of the sensor
 */
#define FREQUENCY_HZ        10

/* Include ----------------------------------------------------------------- */
#include <Arduino_LSM6DSOX.h>
#include "VL53L1X.h"
#include <stdarg.h>

/* Constants --------------------------------------------------------------- */
#if (FREQUENCY_HZ <= 0)
#error "FREQUENCY_HZ should have a value greater than 0"
#endif
#define INTERVAL_MS         (1000 / FREQUENCY_HZ)
#define CONVERT_G_TO_MS2    9.80665f

/* Forward declerations ---------------------------------------------------- */
void ei_printf(const char *format, ...);

/* Private variables ------------------------------------------------------- */
static unsigned long last_interval_ms = 0;

#ifdef SAMPLE_ACCELEROMETER || defined SAMPLE_GYROSCOPE
float Ax, Ay, Az;
float Gx, Gy, Gz;
#endif

#ifdef SAMPLE_PROXIMITY
VL53L1X proximity;
#endif

void setup() {
    /* Init serial */
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("Edge Impulse sensor data ingestion\r\n");

    /* Init & start sensors */
#ifdef SAMPLE_ACCELEROMETER || defined SAMPLE_GYROSCOPE
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
#endif

#ifdef SAMPLE_PROXIMITY
    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C
    proximity.setBus(&Wire1);
    if (!proximity.init()){
      Serial.println("Failed to detect and initialize ToF sensor!");
      while (1);
    }
    proximity.setDistanceMode(VL53L1X::Long);
    proximity.setMeasurementTimingBudget(10000);
    proximity.startContinuous(10);
#endif
}

void loop() {

    delay(INTERVAL_MS);

#ifdef SAMPLE_ACCELEROMETER
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
  }
    ei_printf(" Ax: %f, Ay: %f, Az: %f,"
        ,Ax * CONVERT_G_TO_MS2
        ,Ay * CONVERT_G_TO_MS2
        ,Az * CONVERT_G_TO_MS2
    );
#endif

#ifdef SAMPLE_GYROSCOPE
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);
  }
    ei_printf(" Gx: %f, Gy: %f, Gz: %f,"
        ,Gx
        ,Gy
        ,Gz
    );
#endif

#ifdef SAMPLE_PROXIMITY
    ei_printf("%d,",
        proximity.read()
    );
#endif

    ei_printf("\r\n");

}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...)
{
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}
