#define LED_Y_PIN A0
#define LED_G_PIN A1
#define LED_R_PIN A2
#define LED_B_PIN A3

#define LCD_PIN A4

#define PROJ_PIN A5 // pin for projector

#define LF_PIN 2        // LEFT FORWARD - IN1
#define LB_PIN 4        // LEFT BACKWARD - IN2
#define RF_PIN 7        // RIGHT FORWARD - IN3
#define RB_PIN 8        // RIGHT BACKWARD - IN4

#define LEFT_PWM_PIN 3  // LEFT SPEED - ENA
#define RIGHT_PWM_PIN 5 // RIGHT SPEED - ENB

// Head Motor pins
#define HL_PIN 12      // HEAD LEFT - IN3
#define HR_PIN 13      // HEAD_RIGHT - IN4
#define HEAD_PWM_PIN 9  // HEAD SPEED - ENB

// Possible additional motor control pins - TODO: Rename pin definitions 
#define ARM_OPEN_PIN 10
#define ARM_CLOSE_PIN 11
#define ARM_PWM_PIN 6

// Potential pin to connect to button for universal stop all motors
#define STOP_PIN A6

#define DEFAULT_SPEED 150
#define MAX_SPEED 255 // Should never exceed 255
#define INC 50

#define ON HIGH
#define OFF LOW

int WHEEL_SPEED = DEFAULT_SPEED;
int TURN_SPEED = 75;   // May need to be adjusted
int DETECT_DIST = 50;
unsigned long LIGHT_TOGGLE_TIME = 750; // Sets how often to toggle LEDs before first command
unsigned long lastToggleTime = millis();
bool instructionReceived = false;
bool turnedRed = false;
bool roamingModeOn = false;

// LED and LCD states
int LED_R;
int LED_B;
int LED_Y;
int LED_G;
int LCD;

// Motor states
int LEFT_FORWARD;
int LEFT_BACK;
int RIGHT_FORWARD;
int RIGHT_BACK;

// Head state
int TURN_HEAD_LEFT;
int TURN_HEAD_RIGHT;

// Projector state
int PROJ_BUTTON_STATE;
int PROJ_STATE = 0;
unsigned long PROJ_INTERVAL = 2000;
unsigned long projTimerStart;

void setup() {
  Serial.begin(9600); //Sets the data rate in bits per second (baud) for serial data transmission
  
  // Set up motors as output
  pinMode(LF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);
  pinMode(HL_PIN, OUTPUT);
  pinMode(HR_PIN, OUTPUT);

  // Set up PWM pins
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(HEAD_PWM_PIN, OUTPUT);

  // Set up LEDs and LCD as output
  pinMode(LED_Y_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(LCD_PIN, OUTPUT);

  // Set up Projector on button
  pinMode(PROJ_PIN, OUTPUT);

  // Power off all outputs and set their states to be off.
  digitalWrite(LCD_PIN, LOW);
  LCD = OFF;

  digitalWrite(PROJ_PIN, LOW);

  powerOffAllLEDs();
  setAllLEDsOFF();
  powerOffAllMotors();
  setAllWheelsOff();
  setTurnHeadOff();

}

void loop() {
  if (Serial.available() > 0) {
    // Read in instruction string.
    String instruction = Serial.readStringUntil('\n');
//    char instruction = Serial.read();

    // Based on first char of instruction set the appropriate actions
    switch (instruction[0]) {
      case 's':
        // Full stop motors
        roamingModeOn = false;
        setAllWheelsOff();
        setTurnHeadOff();
      case 'm':
        // move wheels
        setMotors(instruction[1]);
        break;
      case 'w':
        // speed of wheels
        setWheelSpeed(instruction[1]);
      case 'l':
        // leds
        turnOnLED(instruction[1], instruction[2]);
        break;
      case 'd':
        // lcd
        LCD = ON;
        break;
      case 'h':
        // turn head
        turnHead(instruction[1]);
        break;
      case 'p':
        if (instruction[1] == '1') {
          // Check if projector is off then turn on
          if (!PROJ_STATE){
            // simulate holding in projector button for 2 sec
            if (PROJ_BUTTON_STATE == OFF) {
              PROJ_STATE = 1 - PROJ_STATE;
              PROJ_BUTTON_STATE = ON;
              digitalWrite(PROJ_PIN, HIGH);
              projTimerStart = millis();
              Serial.println("Time Started");
            } 
          }
        }
        else if (instruction[1] == '0') {
          // Check if projector is on, then turn off
          if (PROJ_STATE){
            // simulate holding in projector button for 2 sec
            if (PROJ_BUTTON_STATE == OFF) {
              PROJ_STATE = 1 - PROJ_STATE;
              PROJ_BUTTON_STATE = ON;
              digitalWrite(PROJ_PIN, HIGH);
              projTimerStart = millis();
              Serial.println("Time Started");
            } 
          }
        }
        
        
        break;
      case 'r':
        // Start roaming mode
        roam(instruction[1]);
        break;
      case 'q':
        // quit - turn off everything
        resetAll();
        break;
        
    }

    Serial.print("Received instruction: ");
    Serial.println(instruction);
    instructionReceived = true;
  }


  // After updating the states, write the values to the pins.
  digitalWrite(LB_PIN, LEFT_BACK);
  digitalWrite(LF_PIN, LEFT_FORWARD);
  digitalWrite(RB_PIN, RIGHT_BACK);
  digitalWrite(RF_PIN, RIGHT_FORWARD);
  analogWrite(LEFT_PWM_PIN, WHEEL_SPEED);
  analogWrite(RIGHT_PWM_PIN, WHEEL_SPEED);

  digitalWrite(HL_PIN, TURN_HEAD_LEFT);
  digitalWrite(HR_PIN, TURN_HEAD_RIGHT);
  analogWrite(HEAD_PWM_PIN, TURN_SPEED);  // May be able to do this in setup if we don't ever change the speed...

  digitalWrite(LED_R_PIN, LED_R);
  digitalWrite(LED_B_PIN, LED_B);
  digitalWrite(LED_G_PIN, LED_G);
  digitalWrite(LED_Y_PIN, LED_Y);
  digitalWrite(LCD_PIN, LCD);

  // Turn off Projector pin if it has been on for 2 seconds
  if (millis() - projTimerStart >= PROJ_INTERVAL && PROJ_BUTTON_STATE == ON) {
  PROJ_BUTTON_STATE = OFF;
  digitalWrite(PROJ_PIN, LOW);
  Serial.print("Time ENDED: ");
  Serial.println(millis() - projTimerStart);
  }

  // Toggle lights if no commands were received yet
  if (!instructionReceived)
  {
    if (millis() - lastToggleTime >= LIGHT_TOGGLE_TIME)
    {
    turnedRed = !turnedRed;
    lastToggleTime = millis();
    LED_R = turnedRed ? ON : OFF;
    LED_B = turnedRed ? OFF : ON;
    }
  }
}

void roam(char instruction)
{
  if (roamingModeOn)
  {
    // Roam
    switch (instruction) {
      case 'f':
        // move wheels
        setMotors('f');
        break;
      case 'b':
        // move wheels
        setMotors('b');
      case '0':
        // move wheels
        setMotors('r');
        break;
      case '1':
        // move wheels
        setMotors('l');
        break;
      case 'u':
        // Speed Up
        setWheelSpeed('u');
        break;
      case 'n':
        // Slow Down
        setWheelSpeed('n');
        break;
    }
  }
  else if (instruction == 's')
  {
    // Start Roaming
    roamingModeOn = true;
    setMotors("f");
  }
}

/* Set all LED pins with LOW signal. */
void powerOffAllLEDs()
{
  digitalWrite(LED_Y_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
}

/* Set the state of all LEDs to be OFF. */
void setAllLEDsOFF()
{
  LED_R = OFF;
  LED_B = OFF;
  LED_Y = OFF;
  LED_G = OFF;
}

/* Set all motor controller pins with LOW signal. */
void powerOffAllMotors()
{
  digitalWrite(LF_PIN, LOW);
  digitalWrite(LB_PIN, LOW);
  digitalWrite(RF_PIN, LOW);
  digitalWrite(RB_PIN, LOW);
  digitalWrite(HL_PIN, LOW);
  digitalWrite(HR_PIN, LOW);
}

/* Set the state of all wheel motors to be false. */
void setAllWheelsOff()
{
  LEFT_FORWARD = LOW;
  LEFT_BACK = LOW;
  RIGHT_FORWARD = LOW;
  RIGHT_BACK = LOW;
}

/* Set state of all head motors to be off. */
void setTurnHeadOff()
{
  TURN_HEAD_LEFT = LOW;
  TURN_HEAD_RIGHT = LOW;
}

/* Set everything off */
void resetAll()
{
  setAllLEDsOFF();
  setAllWheelsOff();
  setTurnHeadOff();
  LCD = OFF;

  // If the projector was on, turn it off
  if (PROJ_STATE) {
    digitalWrite(PROJ_PIN, HIGH);
    delay(2);
    digitalWrite(PROJ_PIN, LOW);
  }
}

/* 
 *  Sets the state of the wheel motors. 
 *  dir: incoming instruction code
 */
void setMotors(char dir)
{
  setAllWheelsOff();
  switch (dir) {
    case 's':
      // stop
      setAllWheelsOff();
      break;
    case 'f':
      // go forward
      LEFT_FORWARD = HIGH;
      RIGHT_FORWARD = HIGH;
      break;
    case 'b':
      // go backward
      LEFT_BACK = HIGH;
      RIGHT_BACK = HIGH;
      break;
    case 'l':
      // turn left
      RIGHT_FORWARD = HIGH;
      LEFT_BACK = HIGH;
      break;
    case 'r':
      // turn right
      LEFT_FORWARD = HIGH;      
      RIGHT_BACK = HIGH;
      break;
    default:
      break;
  }
}

/* 
 *  Sets the wheel speed for the motors.
 *  dir: incoming instruction code
 */
void setWheelSpeed(char dir = 'n')
{
  switch(dir) {
    case 'u':
      // speed up
      WHEEL_SPEED += INC;
      if (WHEEL_SPEED > MAX_SPEED) {
        WHEEL_SPEED = MAX_SPEED;
      }
      break;
    case 'd':
      // speed down
      WHEEL_SPEED -= INC;
      if (WHEEL_SPEED < 0) {
        WHEEL_SPEED = 0;
      }
      break;
    case 'n':
      // normal speed
      WHEEL_SPEED = DEFAULT_SPEED;
      break;
  }

}

/* 
 *  Sets the state of the LEDs. 
 *  LED: instruction corresponding to the color
 */
void turnOnLED(char LED, char state)
{
//  setAllLEDsOFF();
  boolean turnOn = state == 'h' ? ON : OFF;
  switch (LED) {
  case 'y':
    LED_Y = turnOn;
    break;
  case 'g':
    LED_G = turnOn;
    break;
  case 'r':
    LED_R = turnOn;
    break;
  case 'b':
    LED_B = turnOn;
    break;
  case 'o':
    setAllLEDsOFF();
    break;
  default:
    break;
  }
}

/*
 * Sets the direction the head should turn
 * dir: direction of the head
 */
void turnHead(char dir)
{
  if (dir == 'l') {
    // rotate head left
    TURN_HEAD_LEFT = HIGH;
    TURN_HEAD_RIGHT = LOW;
  } else if (dir == 'r') {
    // rotate head right
    TURN_HEAD_LEFT = LOW;
    TURN_HEAD_RIGHT = HIGH;
  } else if (dir == 's') {
    // stop head
    setTurnHeadOff();
  } else {
    // default - do not rotate
    setTurnHeadOff();
  }
}
