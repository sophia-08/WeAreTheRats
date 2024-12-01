// #include "LSM6DS3.h"
#include "Adafruit_BNO055.h"
// #include <Adafruit_Sensor.h>
#include <Wire.h>

// #define TOM
const float accelerationThreshold = 2.5; // threshold of significant in G's

const int numSamples = 500; // 119;
double samples[numSamples][6];

int samplesRead = 0;
#define MOUSE_LEFT D9
#define MOUSE_RIGHT D8
#define MOUSE_ACTIVATE D6
#define IMU_RESET D0
#define SWITCH_DEVICE_MODE D10
#define DEBUG_2 D2
#define DEBUG_3 D3

#define out_samples 100

#define SMOOTHING_RATIO 0.8
#define SENSITIVITY_X 30
#define SENSITIVITY_Y 35

#define DEVICE_MOUSE_MODE 0
#define DEVICE_KEYBOARD_MODE 1
int deviceMode;


float accelX, accelY, accelZ, // units m/s/s i.e. accelZ if often 9.8 (gravity)
    gyroX, gyroY, gyroZ,      // units dps (degrees per second)
    gyroDriftX, gyroDriftY, gyroDriftZ, // units dps
    gyroRoll, gyroPitch, gyroYaw,       // units degrees (expect major drift)
    gyroCorrectedRoll, gyroCorrectedPitch,
    gyroCorrectedYaw, // units degrees (expect minor drift)
    accRoll, accPitch,
    accYaw, // units degrees (roll and pitch noisy, yaw not possible)
    complementaryRoll, complementaryPitch,
    complementaryYaw; // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
uint8_t readData;

float roll0, pitch0, yaw0;
float roll, pitch, yaw;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


int ledgreen = 0;
int ledred = 0;
#define LED_CHARGER 23
#define LIGHT_ON LOW
#define LIGHT_OFF HIGH

// const uint8_t BLEUART_UUID_SERVICE[] =
// {
//     0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
//     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
// };

void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_CHARGER, OUTPUT);
  pinMode(IMU_RESET, OUTPUT);
  pinMode(DEBUG_2, OUTPUT);
  pinMode(DEBUG_3, OUTPUT);

  // Mystery of why !Serial not ready:
  // The "Serial" is always valid for an Arduino Uno, therefor that piece of
  // code does not wait. In the Leonardo, the "Serial" could be zero, if the
  // serial monitor has not been opened yet.

  // while (!Serial) {
  //   digitalWrite(LED_RED, LIGHT_ON);
  //   delay(10);
  //   digitalWrite(LED_RED, LIGHT_OFF);
  //   delay(100);
  // }

  Serial.println("We are the rats\n");
  Serial.println("-------------------------------------\n");

  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  delay(0.1);
  digitalWrite(IMU_RESET, LOW);
  delay(0.1);
  digitalWrite(IMU_RESET, HIGH);

  pinMode(DEVICE_ACTIVATE, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(D10, INPUT_PULLUP);

  digitalWrite(DEVICE_ACTIVATE, HIGH);
  digitalWrite(D7, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);
  digitalWrite(MOUSE_LEFT, HIGH);
  digitalWrite(D10, HIGH);

  digitalWrite(LED_RED, LIGHT_OFF);
  digitalWrite(LED_BLUE, LIGHT_OFF);
  digitalWrite(LED_GREEN, LIGHT_OFF);
  digitalWrite(LED_CHARGER, LIGHT_OFF);


  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    systemHaltWithledPattern(LED_RED, 3);
  }

  // calibrateIMU(250, 250);
  lastTime = micros();
  deviceMode = DEVICE_MOUSE_MODE;
  Serial.print("bno mode ");
  Serial.println(bno.getMode());
}
float minAccl = 10;
float minGyro = 10;
float maxAccl = -10;
float maxGyro = -10;
float rangeOfAccl, rangeOfGyro;
int tensorIndex = 0;

int count = 0;
int tmp = 0;

#define report_freq 2

int lastx, lasty;
int left, right;
int last_left, last_right;

bool inference_started = false;

#define PRECISION 4
float lastAx, lastAy, lastAz;
float lastHeading, lastRoll;
bool startedChar = false;
int t1 = 0;
int ledCount;
bool needSendKeyRelease = false;
float xAngle, yAngle, lastXAngle, lastYAngle;

bool d2;
sensors_event_t orientationData, linearAccelData, angVelData, magneticData;

bool readIMU() {

  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  return true;
}

bool readIMUOrientation1() {
  int loop = 0;
  d2 = !d2;
  digitalWrite(DEBUG_2, d2);
  // Wait upto 25*0.5 ms. The IMU was configured to 100Hz, so shall has new data
  // every 10ms
  while (loop <= 25) {
    loop++;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    if (orientationData.orientation.heading == lastHeading &&
        orientationData.orientation.roll == lastRoll) {
      delay(0.5);
    } else {
      break;
    }
  }

  lastHeading = orientationData.orientation.heading;
  lastRoll = orientationData.orientation.roll;
  return true;
}

bool readIMUOrientation() {
  int loop = 0;
  d2 = !d2;
  digitalWrite(DEBUG_2, d2);
  // Wait upto 25*0.5 ms. The IMU was configured to 100Hz, so shall has new data
  // every 10ms

  // while (loop <= 25) {
  //    loop++;
  //    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //    if (linearAccelData.acceleration.x == lastAx &&
  //    linearAccelData.acceleration.y == lastAy &&
  //    linearAccelData.acceleration.z == lastAz) {
  //      delay(0.5);
  //    } else {
  //      lastAx = linearAccelData.acceleration.x;
  //      lastAy = linearAccelData.acceleration.y;
  //      lastAz = linearAccelData.acceleration.z;
  //      break;
  //    }
  //  }
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&magneticData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  return true;
}

/**
 * @brief Use the given led indicate system halt. the led shall blink at
 * frequence of give duration
 *
 * @param led
 * @param seconds
 */
void systemHaltWithledPattern(int led, int seconds) {

  for (;;) {

    digitalWrite(led, LIGHT_ON);
    delay(1000 * seconds);

    digitalWrite(led, LIGHT_OFF);
    delay(1000 * seconds);
  }
}

int lastSent;
int currentSent;

void loop() {
  // ledred = !ledred;
  //  digitalWrite(LED_RED, ledred);
  // digitalWrite(LED_BLUE, digitalRead(D7));
  // digitalWrite(LED_RED, digitalRead(MOUSE_RIGHT));
  // digitalWrite(LED_GREEN, digitalRead(D9));
  // digitalWrite(LED_CHARGER, digitalRead(D10));
  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 5000 < 50) {
    digitalWrite(LED_GREEN, LIGHT_ON);
  } else {
    digitalWrite(LED_GREEN, LIGHT_OFF);
  }

    // In mouse mode, we only need orientation.
    readIMUOrientation();
    Serial.print(orientationData.orientation.pitch);
    Serial.print(",");
    Serial.print(orientationData.orientation.roll);
    Serial.print(",");
    Serial.print(orientationData.orientation.heading);
    Serial.print(",");
    Serial.print(magneticData.magnetic.x);
    Serial.print(",");
    Serial.print(magneticData.magnetic.y);
    Serial.print(",");
    Serial.println(magneticData.magnetic.z);
}