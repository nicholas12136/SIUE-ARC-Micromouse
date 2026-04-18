#include <Romi32U4.h>
#include <PololuRPiSlave.h>

Romi32U4Encoders encoders;
Romi32U4Motors   motors;
Romi32U4ButtonA  buttonA;

// ── Shared data struct — Pi reads/writes this over I²C ───────────────
struct Data {
  uint8_t wallFront;
  uint8_t wallLeft;
  uint8_t wallRight;
  uint8_t moveDone;
  uint8_t command;
  uint8_t rawRightHigh;
  uint8_t rawRightLow;
  uint8_t rawLeftHigh;
  uint8_t rawLeftLow;
  uint8_t encLeftHigh;
  uint8_t encLeftLow;
  uint8_t encRightHigh;
  uint8_t encRightLow;
  uint8_t buttonA;      
};

PololuRPiSlave<struct Data, 14> slave;  // update count

// ── Tuning ────────────────────────────────────────────────────────────
const int   BASE_SPEED      = 200;
const float KP_ENC          = 0.65;
const int   CELL_TARGET     = 1055;  //900;
const int TURN_TARGET_LEFT  = 685;
const int TURN_TARGET_RIGHT = 655;   // Was 660
const int TURN180_TARGET    = 1410;  // Was 1350
const int   TURN_SPEED      = 100;
const int   RAMP_STEP       = 6;
const int   RAMP_DELAY      = 10;
const int   DECEL_START     = 100;  

// ── Sensor pins ───────────────────────────────────────────────────────
const int IR_FRONT_PIN = 11;
const int IR_LEFT_PIN  = A2;
const int IR_RIGHT_PIN = A3;

// ── Wall Detetion ─────────────────────────────────────────────────────
// volatile bool frontWallDetected = false;
const int WALL_THRESHOLD_LEFT  = 200;
const int WALL_THRESHOLD_RIGHT = 200;

// ── Wall Centering ─────────────────────────────────────────────────────
const float KP_WALL         = 0.05;
const float KD_WALL         = 0.25;

// ── Sensor reads ──────────────────────────────────────────────────────
bool wallFront() { return digitalRead(IR_FRONT_PIN) == LOW; }
bool wallLeft()  { return analogRead(IR_LEFT_PIN)   > WALL_THRESHOLD_LEFT; }
bool wallRight() { return analogRead(IR_RIGHT_PIN)  > WALL_THRESHOLD_RIGHT; }

// ── Motion primitives ─────────────────────────────────────────────────
void moveOneCell()
{
  // frontWallDetected = false; 
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  // Ramp up
  int lastWallError = 0;
  for (int spd = 0; spd <= BASE_SPEED; spd += RAMP_STEP)
  {
    if (wallFront())
    {
      motors.setSpeeds(0, 0);
      delay(200);
      return;
    }

    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();

    int encoderError   = L - R;
    int wallError      = 0;
    int wallErrorDelta = 0;
    if (wallLeft() && wallRight())
    {
      wallError      = analogRead(IR_RIGHT_PIN) - analogRead(IR_LEFT_PIN);
      wallErrorDelta = wallError - lastWallError;
      lastWallError  = wallError;
    }

    int correction = (int)(KP_ENC  * encoderError)
                   + (int)(KP_WALL * wallError)
                   + (int)(KD_WALL * wallErrorDelta);
    motors.setSpeeds(spd - correction, spd + correction);
    delay(RAMP_DELAY);
  }

  // Cruise
  lastWallError = 0;
  while (true)
  {
    if (wallFront())
    {
      motors.setSpeeds(0, 0);
      delay(200);
      return;
    }

    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();
    if ((L >= CELL_TARGET - DECEL_START) && (R >= CELL_TARGET - DECEL_START)) break;

    int encoderError   = L - R;
    int wallError      = 0;
    int wallErrorDelta = 0;
    if (wallLeft() && wallRight())
    {
      wallError      = analogRead(IR_RIGHT_PIN) - analogRead(IR_LEFT_PIN);
      wallErrorDelta = wallError - lastWallError;
      lastWallError  = wallError;
    }

    int correction = (int)(KP_ENC  * encoderError)
                   + (int)(KP_WALL * wallError)
                   + (int)(KD_WALL * wallErrorDelta);
    motors.setSpeeds(BASE_SPEED - correction, BASE_SPEED + correction);
  }

  // Ramp down
  lastWallError = 0;
  for (int spd = BASE_SPEED; spd >= 0; spd -= RAMP_STEP)
  {
    if (wallFront())
    {
      motors.setSpeeds(0, 0);
      delay(200);
      return;
    }

    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();

    int encoderError   = L - R;
    int wallError      = 0;
    int wallErrorDelta = 0;
    if (wallLeft() && wallRight())
    {
      wallError      = analogRead(IR_RIGHT_PIN) - analogRead(IR_LEFT_PIN);
      wallErrorDelta = wallError - lastWallError;
      lastWallError  = wallError;
    }

    int correction = (int)(KP_ENC  * encoderError)
                   + (int)(KP_WALL * wallError)
                   + (int)(KD_WALL * wallErrorDelta);
    motors.setSpeeds(spd - correction, spd + correction);
    delay(RAMP_DELAY);
  }

  motors.setSpeeds(0, 0);
  delay(400);
}

void turnRight90()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  while (true) {
    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();
    int avgCount = (abs(L) + abs(R)) / 2;
    if (avgCount >= TURN_TARGET_RIGHT) break;  // Changed
  }
  motors.setSpeeds(0, 0);
  delay(400);
}

void turnLeft90()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  while (true) {
    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();
    int avgCount = (abs(L) + abs(R)) / 2;
    if (avgCount >= TURN_TARGET_LEFT) break;  // Changed
  }
  motors.setSpeeds(0, 0);
  delay(400);
}

void turnRight180()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);  // CHANGED: back to original
  while (true) {
    int16_t L = encoders.getCountsLeft();
    int16_t R = encoders.getCountsRight();
    int avgCount = (abs(L) + abs(R)) / 2;
    if (avgCount >= TURN180_TARGET) break;
  }
  motors.setSpeeds(0, 0);
  delay(400);
}

// Interrupt service routine — fires instantly when front IR goes LOW
// void onFrontWall()
// {
//   frontWallDetected = true;
// }

// // ── Setup ─────────────────────────────────────────────────
void setup()
{
  Serial.begin(9600);
  pinMode(IR_FRONT_PIN, INPUT);
  // NO attachInterrupt here
  slave.init(20);
  buttonA.waitForButton();
  delay(1000);
  slave.buffer.moveDone = true;
  slave.buffer.command  = 0;
  slave.finalizeWrites();
  Serial.println("Romi ready.");
}

// ── loop ─────────────────────────────────────────────────
void loop()
{
  slave.updateBuffer();

  slave.buffer.wallFront = wallFront();
  slave.buffer.wallLeft  = wallLeft();
  slave.buffer.wallRight = wallRight();
  slave.buffer.buttonA   = buttonA.isPressed() ? 1 : 0;  // add this

  int rawRight = analogRead(IR_RIGHT_PIN);
  slave.buffer.rawRightHigh = (rawRight >> 8) & 0xFF;
  slave.buffer.rawRightLow  = rawRight & 0xFF;

  int rawLeft = analogRead(IR_LEFT_PIN);
  slave.buffer.rawLeftHigh = (rawLeft >> 8) & 0xFF;
  slave.buffer.rawLeftLow  = rawLeft & 0xFF;

  int16_t encLeft  = encoders.getCountsLeft();
  int16_t encRight = encoders.getCountsRight();
  slave.buffer.encLeftHigh  = (encLeft  >> 8) & 0xFF;
  slave.buffer.encLeftLow   = encLeft  & 0xFF;
  slave.buffer.encRightHigh = (encRight >> 8) & 0xFF;
  slave.buffer.encRightLow  = encRight & 0xFF;

  uint8_t cmd = slave.buffer.command;
  if (cmd != 0)
  {
    slave.buffer.moveDone = false;
    slave.finalizeWrites();
    switch (cmd)
    {
      case 1: Serial.println("CMD: move forward"); moveOneCell();   break;
      case 2: Serial.println("CMD: turn right");   turnRight90();   break;
      case 3: Serial.println("CMD: turn left");    turnLeft90();    break;
      case 4: Serial.println("CMD: turn 180");     turnRight180();  break;
    }
    slave.buffer.moveDone = true;
    slave.buffer.command  = 0;
    slave.finalizeWrites();
  }

  slave.finalizeWrites();
}

// Check Battery Voltage loop()
// =========================================

// void loop()
// {
//   // Battery voltage check
//   float voltage = analogRead(A1) * (3 * 5.0 / 1023.0);
//   Serial.print("Battery: ");
//   Serial.print(voltage);
//   Serial.println("V");
//   delay(1000);
// }

// Check Sensor readings setup() & loop()
// ===========================================

// void setup() { Serial.begin(9600); }

// void loop()
// {
//   Serial.print("Left: ");  Serial.print(analogRead(A2));
//   Serial.print("  Right: "); Serial.println(analogRead(A3));
//   delay(300);
// }