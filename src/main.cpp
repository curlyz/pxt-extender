#include <Arduino.h>

#include "RSFEC.h"

uint8_t buffer[20];
uint8_t bufferPointer = 0;
uint8_t bufferReturnLength = 0;      // this set how many bytes to sent back as a response

uint8_t sendBuffer[32];
#define ECC_LENGTH 10

RS::ReedSolomon<10, ECC_LENGTH> rs;

const uint8_t pina[] = {
    PIN_PC5,
    PIN_PC4,
    PIN_PC3,
    PIN_PC2,
    PIN_PC1,
    PIN_PC0,
};

const uint8_t pind[] = {
    PIN_PD4,
    PIN_PD3,
    PIN_PD2,
    PIN_PB0,
    PIN_PD7,
    PIN_PD6,
};

enum CommandType {
  // admin command
  Cmd_FirmwareQuery = 0xfe,
  Cmd_ResetPort = 0xfd,
  // basic command
  Cmd_ReadPort = 0x01,
  Cmd_WritePort = 0x02,
  // Cmd_PinModePort = 0x03,
  // functional commands
  Cmd_ServoControl = 0x04,
  Cmd_AttachInterrupt = 0x05,
  Cmd_AttachSchmittInterrupt = 0x06,
  Cmd_PulseCounter = 0x07,
  Cmd_ReadDHT = 0x08,
  Cmd_ReadDS18B20 = 0x09,
  Cmd_ReadUltrasonic = 0x0a,

};

void Serial_HeadUp() {
  // Since we aim to have interrupt based UART
  // We need to have a header to indicate the start of the data so uart can initialize in time
  // this will happen after a short pulse
  //   Serial.write(bufferReturnLength + 1);
}

void serial_ResponseBack() {
  // return the buffer
  //   uint8_t crc = 0;
  //   for (uint8_t i = 0; i < 15; i++) {
  //     Serial.write(buffer[i]);
  //     crc += buffer[i];
  //   }
  //   Serial.write(crc);
  //   Serial.flush();

  // generate checksum hash
  rs.Encode(buffer, sendBuffer);
  // send the sendBuffer
  for (uint8_t i = 0; i < 20; i++) {
    Serial.write(sendBuffer[i]);
  }
  Serial.flush();
}

void handle_FirmwareQuery() {
}
void handle_ReadPort() {
  /*
      Read the digitalRead of pina and pind of the channel
      And analogRead, pack in 2 bytes, MSB first
  */
  uint8_t channel = buffer[2];
  pinMode(pina[channel], INPUT);
  pinMode(pind[channel], INPUT);
  //   uint8_t pinaData = digitalRead(pina[channel]);
  uint8_t pindData = digitalRead(pind[channel]);
  uint16_t analogData = analogRead(pina[channel]);
  uint8_t pinaData = analogData > 256;
  bufferReturnLength = 5;
  //   buffer[1] = (pinaData << 1) | pindData;
  buffer[1] = pinaData;
  buffer[2] = pindData;
  buffer[3] = (uint8_t) (analogData >> 8);
  buffer[4] = (uint8_t) (analogData & 0xff);
}

inline void __writePort__(uint8_t port, uint8_t channel, uint8_t mode, uint8_t data) {
  /*
    write port is a single byte data
    first 2 bit is pinMode
    4 bit per channel

    First 4 bit
    0b0000 = IGNORE
    0b0001 = INPUT
    0b0010 = OUTPUT
    0b0011 = INPUT_PULLUP

    Last 4 bit
    0b0000 = IGNORE
    0b0001 = HIGH
    0b0010 = LOW
    0b0011 = TOGGLE

    Note: there is no PWM support yet

    There is channel parameter
    0 -> use pina
    1 -> use pind
  */
  uint8_t _pinMode = mode;
  uint8_t pinData = data;
  switch (_pinMode) {
    case 0x00 :
      // ignore
      break;
    case 0x01 :
      pinMode(channel ? pina[port] : pind[port], INPUT);
      break;
    case 0x02 :
      pinMode(channel ? pina[port] : pind[port], INPUT_PULLUP);
      break;
    case 0x03 :
      pinMode(channel ? pina[port] : pind[port], OUTPUT);
      break;
    default :
      break;
  }

  switch (pinData) {
    case 0x03 :
      // ignore
      break;
    case 0x01 :
      digitalWrite(channel ? pina[port] : pind[port], HIGH);
      break;
    case 0x00 :
      digitalWrite(channel ? pina[port] : pind[port], LOW);
      break;
    case 0x02 :
      digitalWrite(channel ? pina[port] : pind[port], !digitalRead(channel ? pina[port] : pind[port]));
      break;
    default :
      break;
  }
}
void handle_WritePort() {
  uint8_t port = buffer[2];
  uint8_t channel = buffer[3];
  uint8_t mode = buffer[4];
  uint8_t data = buffer[5];
  __writePort__(port, channel, mode, data);
}

uint8_t servo_enableMask = 0x00;
uint16_t servo_pulseDuration[6];
uint16_t servo_currentDuration[6];
uint16_t servo_updateSpeed[6];
unsigned long servo_lastUpdate;      // this will store the pulse duration for each servo
void handle_ServoControl() {
  /*
      the duration is sent in 2 bytes, MSB first
      the update interval is 50ms by default
      set the bit of enableMask and set the pulseDuration
      there are also parameter of updateSpeed, which is an uint8_t
      updateSpeed clarify how many step to reach, 0 mean instanly
  */
  uint8_t port = buffer[2];
  servo_pulseDuration[port] = (buffer[3] << 8) | buffer[4];
  servo_updateSpeed[port] = (buffer[5] << 8) | buffer[6];
  servo_lastUpdate = 0;
  bitSet(servo_enableMask, port);
}
void routine_ServoControl() {
  /*
      this routine will be called every 50ms
      it will check if the servo is enabled
      if it is enabled, it will check if it is time to update
      if it is time to update, it will update the servo

  */
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();

  for (uint8_t channelId = 0; channelId < 6; channelId++) {
    if (!bitRead(servo_enableMask, channelId)) {
      return;
    }

    // calculate the next duration
    if (servo_updateSpeed[channelId] == 0) {
      servo_currentDuration[channelId] = servo_pulseDuration[channelId];
    } else {
      if (servo_pulseDuration[channelId] > servo_currentDuration[channelId]) {
        servo_currentDuration[channelId] += servo_updateSpeed[channelId];
        if (servo_currentDuration[channelId] > servo_pulseDuration[channelId]) {
          servo_currentDuration[channelId] = servo_pulseDuration[channelId];
        }
      } else {
        servo_currentDuration[channelId] -= servo_updateSpeed[channelId];
        if (servo_currentDuration[channelId] < servo_pulseDuration[channelId]) {
          servo_currentDuration[channelId] = servo_pulseDuration[channelId];
        }
      }
    }
    pinMode(pind[channelId], OUTPUT);
    digitalWrite(pind[channelId], HIGH);
    delayMicroseconds(servo_currentDuration[channelId]);
    digitalWrite(pind[channelId], LOW);

    // Serial.printf("Servo %d: %d\n", channelId, servo_currentDuration[channelId]);
    servo_lastUpdate = millis();
  }
}

uint8_t interrupt_enableMask = 0x00;
uint8_t interrupt_subscribeId[6];
uint8_t interrupt_lastState[6];

void handle_AttachInterrupt() {
  /*
        the interrupt_enableMask is a bit mask that will be used to enable the interrupt
        the interrupt_subscribeMsgId is the message id that will be sent back when the interrupt is triggered

  */
  // just enable the flag and send back the latest state (same as ReadPort Command)
  uint8_t port = buffer[2];
  uint8_t subscribeId = buffer[3];
  bitSet(interrupt_enableMask, port);
  interrupt_subscribeId[port] = subscribeId;
  uint8_t pinaData = digitalRead(pina[port]);
  uint8_t pindData = digitalRead(pind[port]);
  bufferReturnLength = 4;
  buffer[1] = (pinaData << 1) | pindData;
  buffer[2] = analogRead(port) >> 8;
  buffer[3] = analogRead(port) & 0xff;
}
void routine_InterruptControl() {
  /*
      if a channel is enabled, each time it change the state, a package will be sent back
      to save memory space, the interrupt_lastState is used to store the last state of the 6 channel

      if either pina or pind channel is changed, this event will trigger
      the data sent back is 1 byte, with 2 bit of data

      the respon
  */
  for (uint8_t channelId = 0; channelId < 6; channelId++) {
    if (!bitRead(interrupt_enableMask, channelId)) {
      return;
    }

    uint8_t pinaData = digitalRead(pina[channelId]);
    uint8_t pindData = digitalRead(pind[channelId]);
    uint8_t state = (pinaData << 1) | pindData;
    if (state != interrupt_lastState[channelId]) {
      // send back the data
      interrupt_lastState[channelId] = state;
      bufferReturnLength = 4;
      buffer[0] = interrupt_subscribeId[channelId];
      buffer[1] = state;
      uint16_t value = analogRead(channelId);
      buffer[2] = value >> 8;
      buffer[3] = value & 0xff;

      Serial_HeadUp();
      for (uint8_t i = 0; i < bufferReturnLength; i++) {
        Serial.write(buffer[i]);
      }
    }
  }
}

uint8_t pulseCounter_enableMask = 0x00;
uint8_t pulseCounter_subscribeId[6];
uint8_t pulseCounter_lastState[2];      // this will be shared by 6 channel, use bitRead and bitWrite to modify
uint16_t pulseCounter_debounceTime[6];
uint16_t pulseCounter_frameDuration[6];
uint8_t pulseCounter_countedValue[6][2];
uint8_t pulseCounter_finalCount[6][2];
unsigned long pulseCounter_lastUpdate[6];
void handle_PulseCounter() {
  /*
      pulse counter require 2 param per channel
      pulse counter only use pind
      pulse counter need debounce time and frame duration

      debounce time are time between state change of the pind
      frame duration are time between the last state change and the pulse counter value is sent back

  */
  uint8_t port = buffer[2];
  pulseCounter_debounceTime[port] = (buffer[3] << 8) | buffer[4];
  pulseCounter_frameDuration[port] = (buffer[5] << 8) | buffer[6];
  bitSet(pulseCounter_enableMask, port);

  /// at the same time, read the pulse count for both channel
  buffer[1] = pulseCounter_finalCount[port][0];
  buffer[2] = pulseCounter_finalCount[port][1];
  pulseCounter_finalCount[port][0] = 0;
  pulseCounter_finalCount[port][1] = 0;
  bufferReturnLength = 3;
}
void routine_PulseCounterCountrol() {
  /*
    Pulse counter routine will read for both pin a and pind
    If changed, the countedValue will increment
    Withint the debounce time, ignore the change
    After the last recorded timestamp change for either channel, bring the counterValue to finalCount
    And reset counterValue

  */
  for (uint8_t channelId = 0; channelId < 6; channelId++) {
    if (millis() - pulseCounter_lastUpdate[channelId] > pulseCounter_frameDuration[channelId]) {
      // within frame duration
      pulseCounter_finalCount[channelId][0] = pulseCounter_countedValue[channelId][0];
      pulseCounter_finalCount[channelId][1] = pulseCounter_countedValue[channelId][1];
      pulseCounter_countedValue[channelId][0] = 0;
      pulseCounter_countedValue[channelId][1] = 0;
      pulseCounter_lastUpdate[channelId] = millis();
    }

    // update final value

    if (!bitRead(pulseCounter_enableMask, channelId)) {
      return;
    }
    pinMode(pind[channelId], INPUT);
    pinMode(pina[channelId], INPUT);

    uint8_t pindData = digitalRead(pind[channelId]);
    uint16_t analog = analogRead(pina[channelId]);

    uint8_t pinaData = analog > 10;

    if (bitRead(pulseCounter_lastState[1], channelId) == pindData && bitRead(pulseCounter_lastState[0], channelId) == pinaData) {
      // no change
      continue;
    }

    // // there is change
    // // check if it is within debounce time
    // if (millis() - pulseCounter_lastUpdate[channelId] < pulseCounter_debounceTime[channelId]) {
    //   // within debounce time
    //   continue;
    // }

    // update lastState

    // update lastUpdate
    pulseCounter_lastUpdate[channelId] = millis();
    // which channel changed? incremen the countedValue accordingly
    if (bitRead(pulseCounter_lastState[1], channelId) != pindData) {
      // pind changed
      pulseCounter_countedValue[channelId][1]++;
    }
    if (bitRead(pulseCounter_lastState[0], channelId) != pinaData) {
      // pina changed
      pulseCounter_countedValue[channelId][0]++;
    }

    bitWrite(pulseCounter_lastState[1], channelId, pindData);
    bitWrite(pulseCounter_lastState[0], channelId, pinaData);

    // check if it is within frame duration, after frame duration, stage the value to final Value
  }
}

void handle_AttachSchmittInterrupt() {
  buffer[1] = 0xff;      // not supported
  bufferReturnLength = 2;
}

void handle_ReadDHT() {
  buffer[1] = 0xff;      // not supported
  bufferReturnLength = 2;
}
void handle_ReadDS18B20() {
  buffer[1] = 0xff;      // not supported
  bufferReturnLength = 2;
}
void handle_ReadUltrasonic() {
  buffer[1] = 0xff;      // not supported
  bufferReturnLength = 2;
}
void handle_ResetPort() {
  /*
      Reset the port to default state
      default state is INPUT

      also clear the bit mask of interrupt, pulse counter, and servo
  */
  uint8_t port = buffer[2];
  pinMode(pina[port], INPUT);
  pinMode(pind[port], INPUT);
  bitClear(interrupt_enableMask, port);
  bitClear(pulseCounter_enableMask, port);
  bitClear(servo_enableMask, port);
}

void routine_ReceiveData() {
  /*
      Read the data from serial and store in buffer
  */

  bool isAvailable = false;
  while (true) {
    while (Serial.available()) {
      uint8_t data = Serial.read();
      buffer[bufferPointer++] = data;
      //   Serial.write(data);
      isAvailable = true;
      if (bufferPointer == 20) {
        bufferPointer = 0;
        break;
      }
    }
    delay(1);
    if (!Serial.available()) {
      break;
    }
  }
  bufferPointer = 0;
  bufferReturnLength = 2;
  if (!isAvailable) {
    return;
  }
  /*

    Process data
    first byte is messageId
    second byte is command
    starting from third bytes, it is data
  */
  uint8_t msgId = buffer[0];
  uint8_t command = buffer[1];
  switch (command) {
    case Cmd_FirmwareQuery :
      handle_FirmwareQuery();
      break;
    case Cmd_ResetPort :
      handle_ResetPort();
      break;
    case Cmd_ReadPort :
      handle_ReadPort();
      break;
    case Cmd_WritePort :
      handle_WritePort();
      break;
    case Cmd_ServoControl :
      handle_ServoControl();
      break;
    case Cmd_AttachInterrupt :
      handle_AttachInterrupt();
      break;
    case Cmd_AttachSchmittInterrupt :
      handle_AttachSchmittInterrupt();
      break;
    case Cmd_PulseCounter :
      handle_PulseCounter();
      break;
    case Cmd_ReadDHT :
      handle_ReadDHT();
      break;
    case Cmd_ReadDS18B20 :
      handle_ReadDS18B20();
      break;
    case Cmd_ReadUltrasonic :
      handle_ReadUltrasonic();
      break;
    default :
      break;
  }
  // mind that buffer[0] is always kept as messageId
  Serial_HeadUp();
  buffer[0] = msgId;
  serial_ResponseBack();
  //   for (uint8_t i = 0; i < bufferReturnLength; i++) {
  //     Serial.write(buffer[i]);
  //   }
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(pina[i], INPUT);
    pinMode(pind[i], INPUT);
  }
}
void loop() {
  routine_ReceiveData();
  // update on changes, sent back data
  routine_ServoControl();
  routine_InterruptControl();
  routine_PulseCounterCountrol();
  //   Serial.write(0xae);
  //   bitSet(interrupt_enableMask, 0);
}