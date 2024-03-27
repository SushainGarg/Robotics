#include <Wire.h>
#include <Zumo32U4.h>
#define MAZE_WIDTH 14
#define MAZE_HEIGHT 10

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proximitySensors;
Zumo32U4Buzzer buzzer;

const uint16_t straightSpeed = 100;
float unitDistance = 1000;

// The delay to use between first detecting an intersection and
// starting to turn.  During this time, the robot drives
// straight.  Ideally this delay would be just long enough to get
// the robot from the point where it detected the intersection to
// the center of the intersection.
const uint16_t intersectionDelay = 130;

// Motor speed when turning.  400 is the max speed.
const uint16_t turnSpeed = 150;

// Motor speed when turning during line sensor calibration.
const uint16_t calibrationSpeed = 200;

// This line sensor threshold is used to detect intersections.
const uint16_t LeftsensorThreshold = 300;
const uint16_t CentersensorThreshold = 200;
const uint16_t RightsensorThreshold = 300;
bool useEmitters = true;

// This line sensor threshold is used to detect the end of the
// maze.

// The number of line sensors we are using.
const uint8_t 
numSensors = 5;

// For angles measured by the gyro, our convention is that a
// value of (1 << 29) represents 45 degrees.  This means that a
// uint32_t can represent any angle between 0 and 360.
const int32_t gyroAngle45 = 0x20000000;

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;
int mazeGrid[MAZE_HEIGHT][MAZE_WIDTH] = {0};

uint16_t lineSensorValues[numSensors];
const float wheelDiameter = 3.9;
const float wheelCircumference = wheelDiameter * 3.14159; // Distance is calculated in centimeters
const int encoderResolution = 900;
bool objectDetected = false;
int objectLocationStep = 0;
int currentStep = 0;
struct Coordinate {
    int x;
    int y;
};

char path[100];
uint8_t pathLength = 0;

void turnSensorSetup(){
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();
}

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate(){
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

static void lineSensorSetup()
{
  display.clear();
  display.print(F("Line cal"));

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  // We use the gyro to turn so that we don't turn more than
  // necessary, and so that if there are issues with the gyro
  // then you will know before actually starting the robot.

  turnSensorReset();

  // Turn to the left 90 degrees.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while((int32_t)turnAngle < turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  while((int32_t)turnAngle > -turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while((int32_t)turnAngle < 0)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);

  // Show the line sensor readings on the display until button A is
  // pressed.
  display.clear();
  while(!buttonA.getSingleDebouncedPress())
  {
    readSensors();
  }

  display.clear();
}

uint16_t readSensors()
{
  return lineSensors.readLine(lineSensorValues ,QTR_EMITTERS_ON, true);
}

bool aboveLine(uint8_t sensorIndex){
  if(sensorIndex == 0){
    return lineSensorValues[sensorIndex] > LeftsensorThreshold;
  }

  if(sensorIndex == 2){
    return lineSensorValues[sensorIndex] > CentersensorThreshold;
  }

  if(sensorIndex == 4){
    return lineSensorValues[sensorIndex] > RightsensorThreshold;
  }
  
}


// turning Robot by 45 Degrees
void turnLeft90(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(-turnSpeed, turnSpeed);
  while((int32_t)turnAngle < turnAngle45)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

// turning Robot by 90 Degrees
void turnLeftMaze(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(-turnSpeed, turnSpeed);
  while((int32_t)turnAngle < turnAngle90)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

// turning Robot by 5 Degrees
void slightLeft(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(-turnSpeed, turnSpeed);
  while((int32_t)turnAngle < turnAngle1 * 5)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

int currentDegrees = 0;

// making robot return to origin position
void zeroDegreeTurn(){
  currentDegrees = currentDegrees % 360;
  if(currentDegrees > 0){
    motors.setSpeeds(0 , 0);
    turnSensorReset();
    motors.setSpeeds(turnSpeed, -turnSpeed);
    while((int32_t)turnAngle > -turnAngle1 * currentDegrees)
    {
      turnSensorUpdate();
    }
    motors.setSpeeds(0 , 0);
    currentDegrees = 0;
  } else if(currentDegrees < 0){
    motors.setSpeeds(0 , 0);
    turnSensorReset();
    motors.setSpeeds(-turnSpeed, turnSpeed);
    while((int32_t)turnAngle < turnAngle1 * currentDegrees)
    {
      turnSensorUpdate();
    }
    motors.setSpeeds(0 , 0);
    currentDegrees = 0;
  }else{
    return;
  }
}

// turning Robot by 45 Degrees
void turnRight45(){
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  while((int32_t)turnAngle > -turnAngle45)
  {
    turnSensorUpdate();
  }
}

// coverting encoder counts to measurable distance
float countsToDistance(long counts){

  return (counts/(float)encoderResolution) * wheelCircumference;
}

float calculateDistance(){
  long leftCounts = encoders.getCountsLeft();
  long rightCounts = encoders.getCountsRight();

  float leftDistance = countsToDistance(leftCounts);
  float rightDistance = countsToDistance(rightCounts);

  float totalDistance = (leftDistance + rightDistance)/2.0;

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  return totalDistance;

}

void encodersReset(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}


float establishUnitDistance(){
  encodersReset();
  motors.setSpeeds(100, 100);
  delay(500);
  motors.setSpeeds(0, 0);
  return calculateDistance();
}


float calculateDistanceWithoutReset(){
  long leftCounts = encoders.getCountsLeft();
  long rightCounts = encoders.getCountsRight();

  float leftDistance = countsToDistance(leftCounts);
  float rightDistance = countsToDistance(rightCounts);

  
  return (leftDistance + rightDistance)/2.0;
}

// Turning robot 45 degrees 
void turnRight90(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  while((int32_t)turnAngle > -turnAngle45)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

// Turning robot 90 degrees 
void turnRightMaze(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  while((int32_t)turnAngle > -turnAngle90)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

// Turning robot 5 degrees 
void slightRight(){
  motors.setSpeeds(0 , 0);
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  while((int32_t)turnAngle > -turnAngle1 * 5)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0 , 0);
}

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
  proximitySensors.initThreeSensors(); // Initialize the three proximity sensors
  // Calibrate the gyro and show readings from it until the user
  // presses button A.
  turnSensorSetup();

  lineSensorSetup();
  encoders.init();
  unitDistance = establishUnitDistance();

  
}




int posX = 0; // X position
int posY = 0; // Y position
int direction = 0; // 0: North, 1: East, 2: South, 3: West

Coordinate objectCoordinates[3]; // Array to store coordinates of up to 3 objects
int objectCount = 0; // Counter for the number of objects detected

void markPath(int steps) {
    for (int i = 0; i < steps; i++) {
        mazeGrid[posY][posX] = 1;
        
        switch (direction) {
            case 0: posY--; break; // North
            case 1: posX++; break; // East
            case 2: posY++; break; // South
            case 3: posX--; break; // West
        }
        
        posX = max(0, min(MAZE_WIDTH - 1, posX));
        posY = max(0, min(MAZE_HEIGHT - 1, posY));
    }
}

int manhattanDistance(Coordinate a, Coordinate b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

void printMazeGrid() {
    for (int i = 0; i < MAZE_HEIGHT; i++) {
        for (int j = 0; j < MAZE_WIDTH; j++) {
            Serial.print(mazeGrid[i][j]);
            Serial.print(" "); // Print a space for readability
        }
        Serial.println(); // Print a newline at the end of each row
    }
}


// Find the nearest object
int findNearestObject() {
    int nearestObjectIndex = -1;
    int shortestDistance = INT8_MAX;
    Coordinate currentPosition = {posX, posY};

    for (int i = 0; i < objectCount; i++) {
        int distance = manhattanDistance(currentPosition, objectCoordinates[i]);
        if (distance < shortestDistance) {
            shortestDistance = distance;
            nearestObjectIndex = i;
        }
    }

    return nearestObjectIndex;
}

void turnToDirection(int desiredDirection) {
    // Calculate the difference between the current direction and the desired direction
    int rightTurns = (desiredDirection - direction + 4) % 4; // Number of right turns to face the desired direction
    int leftTurns = (direction - desiredDirection + 4) % 4; // Number of left turns to face the desired direction

    // Turn the shortest way to face the desired direction
    if (rightTurns <= leftTurns) {
        // If right turns are equal or fewer, turn right
        for (int i = 0; i < rightTurns; i++) {
             direction = (direction + 1) % 4;
            turnRightMaze();
        }
    } else {
        // If left turns are fewer, turn left
        for (int i = 0; i < leftTurns; i++) {
            direction = (direction + 3) % 4;
            turnLeftMaze();
        }
    }
}


void navigateToCoordinate(Coordinate target) {
    while (posX != target.x || posY != target.y) {
        // Determine the desired direction based on the target position
        int desiredDirection;
        if (posX < target.x) {
            desiredDirection = 1; // East
        } else if (posX > target.x) {
            desiredDirection = 3; // West
        } else if (posY < target.y) {
            desiredDirection = 2; // South
        } else if (posY > target.y) {
            desiredDirection = 0; // North
        } else {
            // Already at the target position
            return;
        }

        // Turn to face the desired direction
        turnToDirection(desiredDirection);

        // Move forward if the next cell in the desired direction is part of the path
        if (mazeGrid[posY + (desiredDirection == 2) - (desiredDirection == 0)][posX + (desiredDirection == 1) - (desiredDirection == 3)] == 1) {
            moveForwardOneStep(straightSpeed);
        }
    }
}

void navigateToObjects(){
   while (objectCount > 0) {
        int nearestObjectIndex = findNearestObject();
        if (nearestObjectIndex != -1) {
            navigateToCoordinate(objectCoordinates[nearestObjectIndex]);
            // After reaching the object, remove it from the list
            for (int i = nearestObjectIndex; i < objectCount - 1; i++) {
                objectCoordinates[i] = objectCoordinates[i + 1];
            }
            objectCount--; // Decrease the object count
        }
    }

    // After navigating to all objects, return to the starting position (0,0)
    Coordinate start = {0, 0};
    navigateToCoordinate(start);
}


int unitCounter;
char orientation = 'u';
bool justTurned = false; // Flag to indicate that the robot has just finished turning
int turnCount = 0;

// prompting robot to move back in case on being aboe the borders
void slightBack(){
  motors.setSpeeds(-straightSpeed, -straightSpeed);
  delay(50);
}


// Defined a struct to provide commands for robot to move.
struct Command {
    enum Type { FORWARD, TURN_RIGHT, TURN_LEFT , TURN_RIGHT_45 , TURN_LEFT_45 , TURN_LEFT_180 ,TURN_RIGHT_180, SPECIAL , END} type;
    int steps; // Only relevant for FORWARD commands
};

// commands array during robot movement.
Command mazeCommands[] = {
    {Command::FORWARD, 4}, {Command::TURN_RIGHT, 0}, 
    {Command::FORWARD, 8}, {Command::TURN_LEFT, 0},
    {Command::FORWARD, 3}, {Command::TURN_LEFT_180, 0},
    {Command::FORWARD, 2}, {Command::TURN_LEFT, 0},
    {Command::FORWARD, 3}, {Command::TURN_LEFT, 0},
    {Command::FORWARD, 3}, {Command::SPECIAL, 0},
    {Command::FORWARD, 3}, {Command::TURN_LEFT, 0},
    {Command::FORWARD, 2},{Command::TURN_LEFT, 0},
    {Command::FORWARD, 3},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 1},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 3},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 4},{Command::TURN_LEFT, 0},
    {Command::FORWARD, 7},{Command::TURN_LEFT, 0},
    {Command::FORWARD, 3},{Command::TURN_LEFT, 0},
    {Command::FORWARD, 2},{Command::TURN_LEFT_180, 0},
    {Command::FORWARD, 2},{Command::TURN_RIGHT_45, 0},
    {Command::FORWARD, 6},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 2},{Command::TURN_RIGHT_180, 0},
    {Command::FORWARD, 2},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 5},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 2},{Command::TURN_RIGHT_180, 0},
    {Command::FORWARD, 2},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 3},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 9},{Command::TURN_RIGHT, 0},
    {Command::FORWARD, 4},{Command::TURN_RIGHT_180, 0},
    {Command::FORWARD, 5},{Command::TURN_LEFT, 0},
    {Command::FORWARD, 9},{Command::END, 0},
};


int objectLocationIndex[3] = {-1,-1,-1}; // Index in mazeCommands where object was detected
int objectOrder = -1;
// One full turn of maze.
int cycle = 0;

// code to map and navigate the maze
void navigateMaze() {
    for (int i = 0; i < sizeof(mazeCommands)/sizeof(mazeCommands[0]); ++i) {
      Command cmd = mazeCommands[i];
      if(cycle > 0 && i == 0){
          cmd.steps += 1;
      }
        switch (cmd.type) {
            case Command::FORWARD:
                markPath(cmd.steps);
                moveForwardSteps(cmd.steps , i);
                // if (checkForObject() && objectDetected) {
                //     objectDetected = false;
                //     objectOrder++;
                //     objectLocationIndex[objectOrder] = i;
                // }
                break;
            case Command::TURN_RIGHT:
                direction = (direction + 1) % 4; // Turn 90 degrees right
                turnRightMaze();
                break;
            case Command::TURN_LEFT:
                direction = (direction + 3) % 4; // Turn 90 degrees left (or 270 degrees right)
                turnLeftMaze();
                break;
            case Command::TURN_LEFT_45:
                direction = (direction + 3) % 4;
                turnLeft90();
                break;
            case Command::TURN_RIGHT_45:  
                 direction = (direction + 1) % 4;
                turnRight90();
                break;
            case Command::TURN_LEFT_180:
                direction = (direction + 2) % 4; // Turn 180 degrees
                turnLeftMaze();
                turnLeftMaze();
                break;
            case Command::TURN_RIGHT_180:
                direction = (direction + 2) % 4;
                turnRightMaze();
                turnRightMaze();
                break;
            case Command::SPECIAL:
                direction = (direction + 2) % 4;
                turnRightMaze();
                while(aboveLine(0) || aboveLine(4)){
                  lineSensors.read(lineSensorValues); // Read line sensor values
                  slightRight();
                }
                break;
            case Command::END:
              direction = (direction + 2) % 4;
              turnRightMaze();
              turnRight90();
              break;
        }
    }
   
   cycle++;
}

// backtrach in case on detecting object
void backtrack(int mazeIndex){
    int difference = mazeCommands[mazeIndex].steps - currentStep;
    mazeCommands[mazeIndex].steps = currentStep;

    if(mazeIndex == 4 || mazeIndex == 28 || mazeIndex == 34 || mazeIndex == 40){
      mazeCommands[mazeIndex + 2].steps = mazeCommands[mazeIndex+2].steps - difference ;
    }
    if(mazeIndex == 16 ){
       mazeCommands[mazeIndex + 4].steps = mazeCommands[mazeIndex+4].steps - difference ;
    }
}

// Mviing forward by a specified amount of steps and detecting objects
void moveForwardSteps(int steps , int mazeIndex) {
  for (int i = 0; i < steps; i++) {
    moveForwardOneStep(straightSpeed);
    currentStep++;
    if(mazeIndex == 4 || mazeIndex == 16 || mazeIndex == 28 || mazeIndex == 34 || mazeIndex == 40){
        if (checkForObject()) {
          objectDetected = true;
          objectLocationStep = currentStep;
          objectOrder++;
          objectLocationIndex[objectOrder] = i;
          objectCoordinates[objectCount++] = {posX, posY};
          backtrack(mazeIndex);
          currentStep = 0;
          return;
      }
    }
  }

  currentStep = 0;
}

// Moving forward by one Step
void moveForwardOneStep(int straightSpeed) {
  float distance = 0;
  encoders.getCountsAndResetLeft(); // Reset encoder counts
  encoders.getCountsAndResetRight();
  while (distance < unitDistance) {
    distance = calculateDistanceWithoutReset();
    lineSensors.read(lineSensorValues); // Read line sensor values
    motors.setSpeeds(straightSpeed, straightSpeed);
    // Line detection and reaction logic
    if(aboveLine(0) || aboveLine(2) || aboveLine(4)){
      slightBack();
      if(aboveLine(0)){
        if(!aboveLine(2) && !aboveLine(4) || aboveLine(2) && !aboveLine(4) || (aboveLine(2) && aboveLine(4))){
          slightRight();
          turnCount += 1;
          currentDegrees = currentDegrees - 10;
        }
        // Additional conditions can be added here
      }
      if(aboveLine(4)){
        if(!aboveLine(2) && !aboveLine(0) || (aboveLine(2) && aboveLine(0))){
          slightLeft();
          turnCount += 1;
          currentDegrees = currentDegrees + 10;
        }
        // Additional conditions can be added here
      }
      if(aboveLine(2)){
        if(!aboveLine(4) && !aboveLine(0)){
          turnRight45();
        }
      }
       if(aboveLine(2) && aboveLine(4) && !aboveLine(2)){
        turnLeft90();
      }
    }
  }
  motors.setSpeeds(0, 0); // Stop after moving one unit distance
  unitCounter++;
  delay(100); // Delay between movements
}


//checking for an object using proximity sensor
bool checkForObject() {
    // Implement object detection logic
    // Return true if an object is detected
  proximitySensors.read(); // Read the proximity sensor values
if (proximitySensors.countsFrontWithLeftLeds() >=6 || proximitySensors.countsFrontWithRightLeds() >= 6) {
    // Detected an object in front, turn around
    motors.setSpeeds(0, 0);
    // Play a beep sound at 1000 Hz for 1000 milliseconds (1 second) at full volume.
    buzzer.playFrequency(1000, 1000, 15);
    // Wait for the beep to finish. Adjust the delay if you change the beep duration.
    delay(10000);
    return true; // Two right turns to make a 180-degree turn
  }else{
    return false;
  }
    
}


void loop() {
    navigateMaze(); // To navigate thorugh the maze
    buzzer.playFrequency(1000, 5000, 15);
    // Wait for the beep to finish. Adjust the delay if you change the beep duration.
    delay(10000);
    navigateToObjects(); // to find objects

}

