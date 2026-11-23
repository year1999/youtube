#include <RH_ASK.h>
#include <SPI.h> // Required by RadioHead to compile

// Speed: 2000 bps, RX pin: 11, TX pin: 12 (DATA on 433MHz TX connects to pin 12)
RH_ASK driver(2000, 11, 12);

struct DataPacket {
  int16_t xAxis;
  int16_t yAxis;
};
DataPacket data;

const unsigned long LOG_INTERVAL_MS = 250;
unsigned long lastLogTime = 0;
unsigned long packetCounter = 0;
bool radioReady = false;

void setup() {
  Serial.begin(9600);
  delay(200); // Give USB-serial time to settle after reset/connect

  Serial.println(F("=== RC Car Transmitter Debug Start ==="));
  Serial.println(F("RF config: 2000 bps | RX=D11 | TX=D12 | Packet=4 bytes"));

  radioReady = driver.init();
  if (radioReady) {
    Serial.println(F("[OK] RadioHead ASK driver initialized."));
  } else {
    Serial.println(F("[ERROR] RadioHead ASK driver.init() FAILED! Check wiring/pins."));
  }
}

void loop() {
  data.xAxis = analogRead(A0);
  data.yAxis = analogRead(A1);

  bool queuedOk = false;
  bool sentOk = false;

  if (radioReady) {
    queuedOk = driver.send((uint8_t *)&data, sizeof(data));
    if (queuedOk) {
      driver.waitPacketSent();
      sentOk = true;
    }
  }

  unsigned long now = millis();
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    packetCounter++;

    Serial.print(F("#"));
    Serial.print(packetCounter);
    Serial.print(F(" | X(A0): "));
    Serial.print(data.xAxis);
    Serial.print(F(" | Y(A1): "));
    Serial.print(data.yAxis);
    Serial.print(F(" | TX(D12): "));

    if (!radioReady) {
      Serial.println(F("NOT_INIT"));
    } else if (!queuedOk) {
      Serial.println(F("QUEUE_FAIL"));
    } else if (sentOk) {
      Serial.println(F("SENT_OK"));
    } else {
      Serial.println(F("SEND_FAIL"));
    }
  }

  // Keep loop responsive for RadioHead while throttling serial spam
  delay(20);
}