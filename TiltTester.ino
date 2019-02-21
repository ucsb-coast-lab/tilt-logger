// Include SparkFunLSM9DS1 library and its dependencies
#include <ArduinoJSON.h>
#include <SparkFunLSM9DS1.h>
#include <SPI.h>
#include <Wire.h>

LSM9DS1 imu;  // Create an LSM9DS1 object

// Mag address must be 0x1E, would be 0x1C if SDO_M is LOW
#define LSM9DS1_M   0x1E
// Accel/gyro address must be 0x6B, would be 0x6A if SDO_AG is LOW
#define LSM9DS1_AG  0x6B

// Convert from microseconds for deep sleep
#define uS_TO_S_FACTOR 1000000
// Goes to sleep for this many seconds (14.5 Minutes)
#define TIME_TO_SLEEP 870


// Global variables to keep track of update rates
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;

// Global variables to print to serial monitor at a steady rate (ms)
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 1000;

void setupDevice()
{
  // Use IMU_MODE_I2C
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
}

void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;

  // [scale] can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 2000;

  /**
   * [sampleRate] sets the output data rate (ODR) of the gyro
   * 1 = 14.9 Hz
   * 2 = 59.5 Hz
   * 3 = 119 Hz
   * 4 = 238 Hz
   * 5 = 476 Hz
   * 6 = 952 Hz
   */
  imu.settings.gyro.sampleRate = 1;

  // [bandwidth] can set the cutoff frequency of the gyro.
  imu.settings.gyro.bandwidth = 0;

  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false;

  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true;

  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  imu.settings.gyro.HPFCutoff = 1;

  // [flipX], [flipY], and [flipZ] are booleans that switch orientation
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true;

  // [enableX], [enableY], and [enableZ] can turn on or off
  imu.settings.accel.enableX = true;
  imu.settings.accel.enableY = true;
  imu.settings.accel.enableZ = true;

  // [scale] sets the full-scale range of the accelerometer.
  imu.settings.accel.scale = 8;

  // [sampleRate] sets the output data rate (ODR)
  // Only applies if gyro disabled
  imu.settings.accel.sampleRate = 1;

  /**
   * [bandwidth] sets the anti-aliasing filter bandwidth.
   * -1 = bandwidth determined by sample rates
   * 0 = 408 Hz
   * 1 = 211 Hz
   * 2 = 105 Hz
   * 3 = 50 Hz
   */
  imu.settings.accel.bandwidth = 3;

  // [highResEnable] enables or disables high resolution
  imu.settings.accel.highResEnable = false;

  // [highResBandwidth] sets the LP cutoff frequency
  imu.settings.accel.highResBandwidth = 0;
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true;

  // [scale] sets the full-scale range of the magnetometer
  imu.settings.mag.scale = 12;

  /**
   * [sampleRate] sets the output data rate (ODR) of the
   * 0 = 0.625 Hz  4 = 10 Hz
   * 1 = 1.25 Hz   5 = 20 Hz
   * 2 = 2.5 Hz    6 = 40 Hz
   * 3 = 5 Hz      7 = 80 Hz
   */
  imu.settings.mag.sampleRate = 1;

  // [tempCompensationEnable] enables or disables
  imu.settings.mag.tempCompensationEnable = false;

  /**
   * [XYPerformance] sets the x and y-axis performance
   * 0 = Lowest Power & Performance : 3 is Maximum
   */
  imu.settings.mag.XYPerformance = 3;

  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3;

  // [lowPowerEnable] enables or disables low power mode
  imu.settings.mag.lowPowerEnable = false;

  /**
   * [operatingMode] sets the operating mode
   * 0 = continuous conversion
   * 1 = single-conversion
   * 2 = power down
   */
  imu.settings.mag.operatingMode = 0;
}

uint16_t initLSM9DS1()
{
  setupDevice();
  setupGyro();
  setupAccel();
  setupMag();

  return imu.begin();
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();
  Serial.println();

  startTime = millis();
}

void loop()
{
  // Period of 30 seconds
  uint32_t period = 1 * 30000L;
  // Overall JSON for the whole burst
  DynamicJSONBuffer bufferburst;
  JSONArray& fullburst = bufferburst.createArray();
  // Loop through readings for 1 minute
  for(uint32_t tStart = millis();  (millis()-tStart) < period;  ){
    if (imu.accelAvailable())
    {
        imu.readAccel();
        accelReadCounter++;
    }
    if (imu.gyroAvailable())
    {
        imu.readGyro();
        gyroReadCounter++;
    }
    if (imu.magAvailable())
    {
        imu.readMag();
        magReadCounter++;
    }
    fullburst.add(writeSingleJSON());
    // printSensorReadings();
  }

  // Sleep for TIME_TO_SLEEP Seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();

}

// Creates JSON Data for a single reading
void writeSingleJSON() {
  DynamicJSONBuffer bufferjson;
  JSONObject& burstdata = bufferjson.createObject();
  // Write accel data to jsonobject
  burstdata["ax"].set(imu.calcAccel(imu.ax));
  burstdata["ay"].set(imu.calcAccel(imu.ay));
  burstdata["az"].set(imu.calcAccel(imu.az));
  // Write gyro data to jsonobject
  burstdata["gx"].set(imu.calcAccel(imu.gx));
  burstdata["gy"].set(imu.calcAccel(imu.gy));
  burstdata["gz"].set(imu.calcAccel(imu.gz));
  // Write mag data to jsonobject
  burstdata["mx"].set(imu.calcAccel(imu.mx));
  burstdata["my"].set(imu.calcAccel(imu.my));
  burstdata["mz"].set(imu.calcAccel(imu.mz));
  // Create Timestamp
  unsigned long runtime = (millis() - startTime);
  burstdata["rt"].set(runtime);
  time_t current = time.now();
  burstdata["rn"].set(current);
}

void printSensorReadings()
{
  float runTime = (float)(millis() - startTime) / 1000.0;
  float accelRate = (float)accelReadCounter/runTime;
  float gyroRate = (float)gyroReadCounter/runTime;
  float magRate = (float)magReadCounter/runTime;
  Serial.print("A: ");
  Serial.print("(");
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az));
  Serial.print(")");
  Serial.print("\t| ");
  Serial.print(accelRate);
  Serial.println(" Hz");
  Serial.print("G: ");
  Serial.print("(");
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz));
  Serial.print(")");
  Serial.print("\t| ");
  Serial.print(gyroRate);
  Serial.println(" Hz");
  Serial.print("M: ");
  Serial.print("(");
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz));
  Serial.print(")");
  Serial.print("\t| ");
  Serial.print(magRate);
  Serial.println(" Hz");
  Serial.println();
}
