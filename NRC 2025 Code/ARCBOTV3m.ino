
// ====== PINS & HARDWARE SETUP ====== //
// Motors (L298N)
#define ENA 5     // PWM for left motor speed
#define IN1 6     // Left motor direction
#define IN2 7
#define ENB 9     // PWM for right motor speed
#define IN3 8     // Right motor direction
#define IN4 10

// Encoders (interrupt pins for Mega: 2,3,18,19,20,21)
#define LEFT_ENCODER_A 2    // Interrupt 0
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 18  // Interrupt 5
#define RIGHT_ENCODER_B 19

// Ultrasonic Sensors
#define FRONT_TRIG 22
#define FRONT_ECHO 23
#define LEFT_TRIG 24
#define LEFT_ECHO 25
#define RIGHT_TRIG 26
#define RIGHT_ECHO 27

// ====== GLOBALS ====== //
volatile long left_ticks = 0;   // Encoder counters
volatile long right_ticks = 0;
float TICKS_PER_INCH = 100.0;   // CALIBRATE THIS (see instructions below)

enum RobotState { EXPLORE, RETURN, SPEED_RUN };
RobotState currentState = EXPLORE;

// Flood Fill Maze Grid (10x10)
#define MAZE_SIZE 10
int maze[MAZE_SIZE][MAZE_SIZE]; // Stores distance to center
bool walls[MAZE_SIZE][MAZE_SIZE][4]; // Walls: [x][y][N,E,S,W]

int current_x = 0;     // Start at corner (0,0)
int current_y = 0;
int heading = 0;       // 0=North,1=East,2=South,3=West

// ====== MOTOR CONTROL ====== //
void setMotors(int left_speed, int right_speed) {
  // Left Motor
  analogWrite(ENA, abs(left_speed));
  digitalWrite(IN1, (left_speed > 0) ? HIGH : LOW);
  digitalWrite(IN2, (left_speed > 0) ? LOW : HIGH);

  // Right Motor
  analogWrite(ENB, abs(right_speed));
  digitalWrite(IN3, (right_speed > 0) ? HIGH : LOW);
  digitalWrite(IN4, (right_speed > 0) ? LOW : HIGH);
}

// Encoder ISRs (update ticks)
void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == digitalRead(LEFT_ENCODER_A)) {
    left_ticks++; 
  } else { 
    left_ticks--; 
  }
}
void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == digitalRead(RIGHT_ENCODER_A)) {
    right_ticks++; 
  } else { 
    right_ticks--; 
  }
}

// ====== ULTRASONIC SENSORS ====== //
#include <NewPing.h>

#define MAX_DISTANCE 200 // Maximum distance to ping in cm
NewPing frontSonar(FRONT_TRIG, FRONT_ECHO, MAX_DISTANCE);
NewPing leftSonar(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE);
NewPing rightSonar(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE);

float getFrontDistance() { return frontSonar.ping_cm() * 0.3937; } // Convert cm to inches
float getLeftDistance() { return leftSonar.ping_cm() * 0.3937; }
float getRightDistance() { return rightSonar.ping_cm() * 0.3937; }

// ====== FLOOD FILL ALGORITHM ====== //
void floodFill(int target_x1, int target_y1, int target_x2, int target_y2) {
  // Reset distances
  for(int x=0; x<MAZE_SIZE; x++) 
    for(int y=0; y<MAZE_SIZE; y++) 
      maze[x][y] = 9999;

  // Set target cells (e.g., center or start)
  maze[target_x1][target_y1] = 0;
  maze[target_x2][target_y2] = 0;

  bool updated = true;
  while(updated) {
    updated = false;
    for(int x=0; x<MAZE_SIZE; x++) {
      for(int y=0; y<MAZE_SIZE; y++) {
        for(int dir=0; dir<4; dir++) {
          if(!walls[x][y][dir]) { // No wall
            int nx = x, ny = y;
            switch(dir) {
              case 0: ny++; break; // North
              case 1: nx++; break; // East
              case 2: ny--; break; // South
              case 3: nx--; break; // West
            }
            if(nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE) {
              if(maze[x][y] > maze[nx][ny] + 1) {
                maze[x][y] = maze[nx][ny] + 1;
                updated = true;
              }
            }
          }
        }
      }
    }
  }
}

// ====== MOVEMENT ====== //
void driveForward(float inches, int speed=150) {
  long target_ticks = inches * TICKS_PER_INCH;
  long start_left = left_ticks;
  long start_right = right_ticks;

  // PID only during explore/return
  if(currentState != SPEED_RUN) { 
    float KP=0.8, KI=0.01, KD=0.1;
    float integral=0, last_error=0;
    while(abs(left_ticks - start_left) < target_ticks) {
      long error = (left_ticks - start_left) - (right_ticks - start_right);
      integral += error;
      float correction = KP*error + KI*integral + KD*(error - last_error);
      setMotors(speed - correction, speed + correction);
      last_error = error;
      delay(10);
    }
  } else { // Speed run: full throttle
    setMotors(255, 255);
    while(abs(left_ticks - start_left) < target_ticks) {}
  }
  setMotors(0, 0);
}

void turn90(bool clockwise, int speed=100) {
  float turn_circumference = PI * 6.8; 
  long target_ticks = (turn_circumference / 4) * TICKS_PER_INCH;

  left_ticks = 0; 
  right_ticks = 0;

  // High-speed turn for speed run
  int motor_speed = (currentState == SPEED_RUN) ? 255 : speed;
  
  while(abs(left_ticks) < target_ticks) {
    setMotors(clockwise ? motor_speed : -motor_speed, 
             clockwise ? -motor_speed : motor_speed);
  }
  setMotors(0, 0);
}

// ====== MAZE LOGIC ====== //
void updateWalls() {
  if(currentState != EXPLORE) return; // Only map during exploration

  if(getFrontDistance() < 5.0) 
    walls[current_x][current_y][heading] = true;
  if(getLeftDistance() < 3.0) 
    walls[current_x][current_y][(heading + 3) % 4] = true;
  if(getRightDistance() < 3.0) 
    walls[current_x][current_y][(heading + 1) % 4] = true;
}
int getNextDirection() {
  int next_dir = -1;
  int min_dist = 9999;
  
  for(int dir=0; dir<4; dir++) {
    if(!walls[current_x][current_y][dir]) {
      int nx = current_x, ny = current_y;
      switch(dir) {
        case 0: ny++; break; case 1: nx++; break;
        case 2: ny--; break; case 3: nx--; break;
      }
      if(maze[nx][ny] < min_dist) {
        min_dist = maze[nx][ny];
        next_dir = dir;
      }
    }
  }
  return next_dir;
}

// ====== MAIN LOOP ====== //
void setup() {
  // Initialize pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); // ... Initialize all motor/sensor pins ...
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);

  // Start at corner (0,0), facing North
  current_x = 0;
  current_y = 0;
  heading = 0;

currentState = EXPLORE;
floodFill(4, 4, 5, 5); // Initial path calculation
}

void loop() {
  bool at_goal = false;
  if(currentState == EXPLORE || currentState == SPEED_RUN) {
    at_goal = (current_x >=4 && current_x <=5 && current_y >=4 && current_y <=5);
  } else if(currentState == RETURN) {
    at_goal = (current_x == 0 && current_y == 0);
  }

  if(at_goal) {
    switch(currentState) {
      case EXPLORE: 
        currentState = RETURN;
        floodFill(0, 0, 0, 0); // Target start
        break;
      case RETURN: 
        currentState = SPEED_RUN;
        floodFill(4, 4, 5, 5); // Target center again
        break;
      case SPEED_RUN: 
        while(1) setMotors(0,0); // Stop forever
    }
    return;
  }

  // Get next move
  int next_dir = getNextDirection();
  if(next_dir == -1) return; // Error
  
  // Turn
  int turn_angle = (next_dir - heading + 4) % 4;
  if(turn_angle == 1) turn90(true, (currentState == SPEED_RUN) ? 255 : 100);
  if(turn_angle == 3) turn90(false, (currentState == SPEED_RUN) ? 255 : 100);
  if(turn_angle == 2) { turn90(true); turn90(true); }
  heading = next_dir;

  // Move
  driveForward(10.0, (currentState == SPEED_RUN) ? 255 : 150);

  // Update position
  switch(next_dir) {
    case 0: current_y++; break; case 1: current_x++; break;
    case 2: current_y--; break; case 3: current_x--; break;
  }

  // Update walls only during exploration
  updateWalls();
  floodFill( // Re-flood based on state
    (currentState == RETURN) ? 0 : 4,
    (currentState == RETURN) ? 0 : 4,
    (currentState == RETURN) ? 0 : 5,
    (currentState == RETURN) ? 0 : 5
  );
}