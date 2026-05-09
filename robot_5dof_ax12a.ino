#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

#define ID_BASE      1
#define ID_SHOULDER 2
#define ID_ELBOW    3
#define ID_WRIST    4
#define ID_GRIPPER  5

int pos[5] = {512, 520, 420, 720, 650};

void setup() {
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  uint8_t ids[5] = {ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER};

  for (int i = 0; i < 5; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_POSITION);
    dxl.writeControlTableItem(MOVING_SPEED, ids[i], 80);
    dxl.torqueOn(ids[i]);
    dxl.setGoalPosition(ids[i], pos[i]);
  }
}

void moveMotor() {
  dxl.setGoalPosition(ID_BASE, pos[0]);
  dxl.setGoalPosition(ID_SHOULDER, pos[1]);
  dxl.setGoalPosition(ID_ELBOW, pos[2]);
  dxl.setGoalPosition(ID_WRIST, pos[3]);
  dxl.setGoalPosition(ID_GRIPPER, pos[4]);
}

void setTorque(bool on) {
  uint8_t ids[5] = {ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER};
  for (int i = 0; i < 5; i++) {
    if (on) dxl.torqueOn(ids[i]);
    else dxl.torqueOff(ids[i]);
  }
}

void loop() {
  if (!DEBUG_SERIAL.available()) return;

  String packet = DEBUG_SERIAL.readStringUntil('*');

  int c, n1, n2, n3, n4, n5;
  int parsed = sscanf(packet.c_str(), "%d,%d,%d,%d,%d,%d", &c, &n1, &n2, &n3, &n4, &n5);

  if (parsed < 1) return;

  if (c == 0 && parsed == 6) {
    pos[0] = constrain(n1, 0, 1023);
    pos[1] = constrain(n2, 0, 1023);
    pos[2] = constrain(n3, 0, 1023);
    pos[3] = constrain(n4, 0, 1023);
    pos[4] = constrain(n5, 0, 1023);
    moveMotor();
  }
  else if (c == 1 && parsed >= 2) {
    setTorque(n1 == 1);
  }
  else if (c == 3) {
    DEBUG_SERIAL.print("Positions:");
    DEBUG_SERIAL.print(dxl.getPresentPosition(ID_BASE)); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(dxl.getPresentPosition(ID_SHOULDER)); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(dxl.getPresentPosition(ID_ELBOW)); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(dxl.getPresentPosition(ID_WRIST)); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println(dxl.getPresentPosition(ID_GRIPPER));
  }
}
