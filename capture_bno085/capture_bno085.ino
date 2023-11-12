
#define BNO085
// #define IMU_USE_RESET
#define IMU_USE_INT
#define sep ","
#define PRECISION 5

#ifdef BNO085
#include "Adafruit_BNO08x.h"
#endif

#include "local_constants.h"
#include "system.h"
#include <Wire.h>

const float accelerationThreshold = 2.5; // threshold of significant in G's

const int numSamples = 500; // 119;
double samples[numSamples][6];

int samplesRead = -1;
#define out_samples 100

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

#ifdef BNO085
// New data available.  currently for keyboard, new data available every 10ms;
// for mouse, every 20ms
bool newData = false;

// Rotation Vector. i, j, k, real
float rtVector[4];

// Linear acceleration, x, y, z
float accl[3];

// gyro, x, y, z
float gyro[3];

// calibration status
int calStatus;

#define BNO08X_RESET -1
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
// sh2_SensorId_t reportType = SH2_ROTATION_VECTOR; // SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 20000;
void setReports() {
  int dataRate;
  if (deviceMode == DEVICE_MOUSE_MODE) {
    dataRate = 20 * 1000;
  } else {
    dataRate = 10 * 1000;
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, dataRate)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, dataRate)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, dataRate)) {
    Serial.println("Could not enable gyroscope");
  }
}

void quaternionToEuler(float qi, float qj, float qk, float qr, euler_t *ypr,
                       bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}
#endif

int ledgreen = 0;
int ledred = 0;
void setup() {
  configGpio();
  Serial.begin(115200);
  int i = 0;
  while (!Serial) {
    digitalWrite(LED_RED, LIGHT_ON);
    delay(10);
    // if (++i > 1000) {
    //   break;
    // }
  }
  digitalWrite(LED_RED, LIGHT_OFF);
  deviceMode = DEVICE_KEYBOARD_MODE;
#ifdef BNO085
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    systemHaltWithledPattern(LED_RED, 3);
  }
  // Serial.println("BNO08x Found!");

  setReports();
  delay(100);
#endif

  // calibrateIMU(250, 250);
  lastTime = micros();
}

float minAccl = 10;
float minGyro = 10;
float maxAccl = -10;
float maxGyro = -10;
float rangeOfAccl, rangeOfGyro;
int tensorIndex = 0;

int count = 0;
int tmp = 0;

#define report_freq 1

int lastx, lasty;
int left, right;
int last_left, last_right;

bool inference_started = false;


float lastAx, lastAy, lastAz;
float lastHeading, lastRoll;
bool startedChar = false;
int t1 = 0;
int ledCount;
bool needSendKeyRelease = false;
float xAngle, yAngle, lastXAngle, lastYAngle;

bool d2;

int lastSent;
int currentSent;
int sleepCount;

void loop() {

  ledCount++;
  // pluse the green led to indicate system alive.
  if (ledCount % 1000 < 30) {
    if (deviceMode == DEVICE_MOUSE_MODE) {
      digitalWrite(LED_GREEN, LIGHT_ON);
      digitalWrite(LED_BLUE, LIGHT_OFF);
    } else {
      digitalWrite(LED_BLUE, LIGHT_ON);
      digitalWrite(LED_GREEN, LIGHT_OFF);
    }

  } else {
    if (digitalRead(MOUSE_ACTIVATE) == HIGH) {
      digitalWrite(LED_GREEN, LIGHT_ON);
    } else {
      digitalWrite(LED_GREEN, LIGHT_OFF);
    }

      digitalWrite(LED_BLUE, LIGHT_OFF);
    
  }

  // Capture has not started, ignore until user activate keypad
  if (!startedChar) {
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      samplesRead = -1;
      return;
    } else {
      // User activate keypad, check whether 2s passed since last capture
      // int currentTime = millis();
      // if (currentTime < t1 + 2000) {
      //   return;
      // }
      // t1 = currentTime;
      samplesRead = -1;
      startedChar = true;
    }
  }

  // User finger is on keyboard_activation pad
  // To begin, wait 200ms
  // delay(200);

  // Loop to read 20 samples, at 100Hz, takes 200ms
  // This is better than delay, clear up data in IMU.
  for (int i=0; i<20;) {
    while (digitalRead(IMU_INT) == HIGH) {
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
    }
    if (newData) {
      i++;
      newData = false;
    }
  }
    // if (digitalRead(KEYPAD_ACTIVATE) == HIGH) {
  
    // } 
  while (true) {
  wait:
    // User deactivated keypad
    if (digitalRead(KEYPAD_ACTIVATE) == LOW) {
      startedChar = false;
      break;
    }

// BNO085 pull IMU_INT LOW when data is ready
// so do nothing in case of IMU_INT high
#ifdef IMU_USE_INT
    while (digitalRead(IMU_INT) == HIGH) {
    }
#endif
    static uint32_t last = 0;
    long now = micros();
    if (bno08x.getSensorEvent(&sensorValue)) {
    }

    if (newData) {
      uint32_t now = micros();
      newData = false;

      // Wait for hand to rest 
      if (samplesRead == -1) {
        if (abs(accl[0]) + abs(accl[1]) + abs(accl[2]) > 1) {
          // Serial.println("wait idle");
          continue;
        }
        digitalWrite(LED_BLUE, LIGHT_ON);
        samplesRead = 0;
        continue;
      }
      
      // wait for hand to move
      if (samplesRead == 0) {
        if (abs(accl[0]) + abs(accl[1]) + abs(accl[2]) < 4) {
          // Serial.println("wait move");
          continue;
        }
      }      

      // Capture samples until keyboard_activation is release.

      Serial.print(samplesRead);
      Serial.print(sep);
      Serial.print(now - last);
      Serial.print(sep);
      last = now;
      Serial.print(calStatus);
      // Serial.print(sep);
      // This is accuracy in the range of 0 to 3
      int i;
      for (i = 0; i < 4; i++) {
        Serial.print(sep);
        Serial.print(rtVector[i],PRECISION);
      }
      for (i = 0; i < 3; i++) {
        Serial.print(sep);
        Serial.print(accl[i],PRECISION);
      }
      for (i = 0; i < 3; i++) {
        Serial.print(sep);
        Serial.print(gyro[i],PRECISION);
      }

      quaternionToEuler(rtVector[0], rtVector[1], rtVector[2], rtVector[3],
                        &ypr, false);

      Serial.print(sep);
      Serial.print(ypr.yaw,PRECISION);
      Serial.print(sep);
      Serial.print(ypr.pitch,PRECISION);
      Serial.print(sep);
      Serial.print(ypr.roll,PRECISION);
      Serial.println("");

      samplesRead++;
    }
  }
}

void configGpio() {
  // enable battery measuring.
  pinMode(VBAT_ENABLE, OUTPUT);
  // Due to hardware limitation, do not set to high on Seeed nrf52
  digitalWrite(VBAT_ENABLE, LOW);

  // Read charge state. Low is charging.
  pinMode(BAT_CHARGE_STATE, INPUT);

  // Set charge mode. Set to high charging current (100mA)
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

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

#ifdef IMU_USE_RESET
  pinMode(IMU_RESET, OUTPUT);
  // Reset IMU
  digitalWrite(IMU_RESET, HIGH);
  delay(0.1);
  digitalWrite(IMU_RESET, LOW);
  delay(0.1);
  digitalWrite(IMU_RESET, HIGH);
#endif

#ifdef IMU_USE_INT
  pinMode(IMU_INT, INPUT_PULLUP);
#endif

  pinMode(MOUSE_ACTIVATE, INPUT_PULLUP);
  pinMode(MOUSE_RIGHT, INPUT_PULLUP);
  pinMode(MOUSE_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_LEFT, INPUT_PULLUP);
  pinMode(KEYPAD_RIGHT, INPUT_PULLUP);
  pinMode(KEYPAD_CENTER, INPUT_PULLUP);
  pinMode(KEYPAD_UP, INPUT_PULLUP);
  pinMode(KEYPAD_DOWN, INPUT_PULLUP);

  digitalWrite(MOUSE_ACTIVATE, HIGH);
  digitalWrite(MOUSE_RIGHT, HIGH);
  digitalWrite(MOUSE_LEFT, HIGH);
  digitalWrite(KEYPAD_LEFT, HIGH);
  digitalWrite(KEYPAD_RIGHT, HIGH);
  digitalWrite(KEYPAD_CENTER, HIGH);
  digitalWrite(KEYPAD_UP, HIGH);
  digitalWrite(KEYPAD_DOWN, HIGH);

  digitalWrite(LED_RED, LIGHT_OFF);
  digitalWrite(LED_BLUE, LIGHT_OFF);
  digitalWrite(LED_GREEN, LIGHT_OFF);
}

// {'a': 212, 'b': 324, 'c': 180, 'd': 204, 'e': 238, 'f': 266, 'g': 263, 'h': 293, 'i': 287, 'j': 293, 'k': 247, 'l': 232, 'm': 255, 'n': 332, 'o': 350, 'p': 267, 'q': 268, 'r': 257, 's': 220, 't': 310, 'u': 287, 'v': 247, 'w': 236, 'x': 255, 'y': 269, 'z': 249}
