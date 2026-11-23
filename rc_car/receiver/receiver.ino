#include <RH_ASK.h>
#include <SPI.h> // Required by RadioHead to compile

// Speed: 2000 bps, RX pin: 11, TX pin: 12 (DATA on 433MHz RX connects to pin 11)
RH_ASK driver(2000, 11, 12);

// Hardware wiring:
// Right motor -> IN1, IN2 (and ENA)
// Left motor  -> IN3, IN4 (and ENB)
#define ENA 5
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENB 6

struct DataPacket {
  int16_t xAxis;
  int16_t yAxis;
};
DataPacket data;

int maxSpeed = 150; // Safe for 3-12V motors on 3S LiPo
unsigned long lastSignalTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastFailsafeLogTime = 0;
unsigned long lastStatsLogTime = 0;
unsigned long rxValidCount = 0;
unsigned long rxBadLenCount = 0;
unsigned long rxOutOfRangeCount = 0;
int prevLeft = 9999;
int prevRight = 9999;
bool radioReady = false;

const unsigned long LOG_INTERVAL_MS = 250;
const unsigned long FAILSAFE_TIMEOUT_MS = 3000;
const unsigned long STATS_INTERVAL_MS = 2000;
const unsigned long MOTOR_TEST_DURATION_MS = 5000;

void driveMotors(int left, int right);
void runStartupMotorTest();
const char* dirLabel(int spd);
bool isValidAxis(int16_t val);

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  driveMotors(0, 0);

  Serial.println(F("=== RC Car Receiver Data-Integrity Debug Start ==="));
  Serial.print(F("Expected packet size: "));
  Serial.print(sizeof(DataPacket));
  Serial.println(F(" bytes (2x int16_t) | RX=D11 | 2000 bps"));
  Serial.println(F("Mapping: Right=IN1/IN2/ENA(D5), Left=IN3/IN4/ENB(D6)"));

  radioReady = driver.init();
  if (radioReady) {
    Serial.println(F("[OK] RadioHead ASK receiver initialized."));
  } else {
    Serial.println(F("[ERROR] RadioHead ASK driver.init() FAILED! Check RX wiring."));
  }

  // runStartupMotorTest();
  lastSignalTime = millis();
}

void runStartupMotorTest() {
  Serial.println(F("[TEST] Starting 5s full-speed (255) forward motor test..."));
  driveMotors(255, 255);
  delay(MOTOR_TEST_DURATION_MS);

  driveMotors(0, 0);
  Serial.println(F("[TEST] Startup motor test complete -> Motors STOPPED. Entering RF/joystick loop."));
}

void loop() {
  uint8_t buf[sizeof(data) + 4]; // Slight extra room so oversized bad frames are caught
  uint8_t buflen = sizeof(buf);
  unsigned long now = millis();

  if (radioReady && driver.recv(buf, &buflen)) {
    if (buflen != sizeof(data)) {
      rxBadLenCount++;
      Serial.print(F("[BAD_LEN] Got "));
      Serial.print(buflen);
      Serial.print(F(" bytes (expected "));
      Serial.print(sizeof(data));
      Serial.print(F(") | Raw hex:"));
      for (uint8_t i = 0; i < buflen && i < sizeof(buf); i++) {
        Serial.print(F(" 0x"));
        if (buf[i] < 0x10) Serial.print(F("0"));
        Serial.print(buf[i], HEX);
      }
      Serial.println();
    } else {
      DataPacket candidate;
      memcpy(&candidate, buf, sizeof(candidate));

      bool xOk = isValidAxis(candidate.xAxis);
      bool yOk = isValidAxis(candidate.yAxis);

      if (!xOk || !yOk) {
        rxOutOfRangeCount++;
        Serial.print(F("[CORRUPT_VAL] Len=4 but out-of-range X="));
        Serial.print(candidate.xAxis);
        Serial.print(F(" ("));
        Serial.print(xOk ? F("OK") : F("BAD"));
        Serial.print(F(") Y="));
        Serial.print(candidate.yAxis);
        Serial.print(F(" ("));
        Serial.print(yOk ? F("OK") : F("BAD"));
        Serial.print(F(") | Raw hex:"));
        for (uint8_t i = 0; i < buflen; i++) {
          Serial.print(F(" 0x"));
          if (buf[i] < 0x10) Serial.print(F("0"));
          Serial.print(buf[i], HEX);
        }
        Serial.println(F(" -> Ignored packet"));
      } else {
        data = candidate;
        lastSignalTime = now;
        rxValidCount++;

        int throttle = map(data.yAxis, 0, 1023, -maxSpeed, maxSpeed);
        int steering = map(data.xAxis, 0, 1023, -maxSpeed, maxSpeed);

        // Center deadzone
        if (abs(throttle) < 25) throttle = 0;
        if (abs(steering) < 25) steering = 0;

        int leftSpeed = constrain(throttle + steering, -maxSpeed, maxSpeed);
        int rightSpeed = constrain(throttle - steering, -maxSpeed, maxSpeed);

        driveMotors(leftSpeed, rightSpeed);

        if (now - lastLogTime >= LOG_INTERVAL_MS) {
          lastLogTime = now;
          Serial.print(F("[VALID_RX#"));
          Serial.print(rxValidCount);
          Serial.print(F("] Len="));
          Serial.print(buflen);
          Serial.print(F(" | X:"));
          Serial.print(data.xAxis);
          Serial.print(F(" Y:"));
          Serial.print(data.yAxis);
          Serial.print(F(" | L:"));
          Serial.print(leftSpeed);
          Serial.print(F(" ("));
          Serial.print(dirLabel(leftSpeed));
          Serial.print(F(") PWM:"));
          Serial.print(abs(leftSpeed));
          Serial.print(F(" | R:"));
          Serial.print(rightSpeed);
          Serial.print(F(" ("));
          Serial.print(dirLabel(rightSpeed));
          Serial.print(F(") PWM:"));
          Serial.println(abs(rightSpeed));
        }
      }
    }
  }

  // Failsafe: Stop motors if no valid signal received for FAILSAFE_TIMEOUT_MS
  if (now - lastSignalTime > FAILSAFE_TIMEOUT_MS) {
    driveMotors(0, 0);

    if (now - lastFailsafeLogTime >= 1000) {
      lastFailsafeLogTime = now;
      Serial.print(F("[FAILSAFE] No valid packet for "));
      Serial.print(now - lastSignalTime);
      Serial.println(F("ms -> Motors STOPPED"));
    }
  }

  // Periodic health summary so you see whether RF packets arrive at all
  if (now - lastStatsLogTime >= STATS_INTERVAL_MS) {
    lastStatsLogTime = now;
    Serial.print(F("[STATS] Valid="));
    Serial.print(rxValidCount);
    Serial.print(F(" | BadLen="));
    Serial.print(rxBadLenCount);
    Serial.print(F(" | OutOfRange="));
    Serial.print(rxOutOfRangeCount);
    Serial.print(F(" | LastValidAge="));
    Serial.print(now - lastSignalTime);
    Serial.println(F("ms"));
  }
}

bool isValidAxis(int16_t val) {
  return (val >= 0 && val <= 1023);
}

void driveMotors(int left, int right) {
  bool changed = (left != prevLeft) || (right != prevRight);

  // Right Motor -> IN1, IN2 (ENA controls right channel speed)
  if (right > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (right < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(right));

  // Left Motor -> IN3, IN4 (ENB controls left channel speed)
  if (left > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (left < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(left));

  if (changed) {
    prevLeft = left;
    prevRight = right;
    Serial.print(F("[MOTOR_CMD] L="));
    Serial.print(left);
    Serial.print(F(" ("));
    Serial.print(dirLabel(left));
    Serial.print(F(") | R="));
    Serial.print(right);
    Serial.print(F(" ("));
    Serial.println(dirLabel(right));
  }
}

const char* dirLabel(int spd) {
  if (spd > 0) return "FWD";
  if (spd < 0) return "REV";
  return "STOP";
}