#define LED_Y_PIN A0
#define LED_G_PIN A1
#define LED_R_PIN A2
#define LED_B_PIN A3

#define LCD_PIN A4
// #define SERVO_PIN 10

#define LF_PIN 2        // LEFT FORWARD - IN1
#define LB_PIN 4        // LEFT BACKWARD - IN2
#define RF_PIN 7        // RIGHT FORWARD - IN3
#define RB_PIN 8        // RIGHT BACKWARD - IN4

#define LEFT_PWM_PIN 3  // LEFT SPEED - ENA
#define RIGHT_PWM_PIN 5 // RIGHT SPEED - ENB

// Head Motor pins
#define HL_PIN 10      // HEAD LEFT - IN1
#define HR_PIN 11      // HEAD_RIGHT - IN2
#define HEAD_PWM_PIN 6  // HEAD SPEED - ENA

// Possible additional motor control pins
#define IN3_PIN 12
#define IN4_PIN 13
#define PWM_PIN 9

#define DEFAULT_SPEED 150
#define MAX_SPEED 255 // Should never exceed 255
#define INC 50

#define ON HIGH
#define OFF LOW

int WHEEL_SPEED = DEFAULT_SPEED;
int TURN_SPEED = 100;   // May need to be adjusted

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

  // Power off all outputs and set their states to be off.
  digitalWrite(LCD_PIN, LOW);
  LCD = OFF;
  TURN_HEAD_LEFT = LOW;
  TURN_HEAD_RIGHT = LOW;
  powerOffAllLEDs();
  setAllLEDsOFF();
  powerOffAllMotors();
  setAllWheelsOff();

}

void loop() {
  if (Serial.available() > 0) {
    // Read in instruction string.
    String instruction = Serial.readStringUntil('\n');
//    char instruction = Serial.read();

    // Based on first char of instruction set the appropriate actions
    switch (instruction[0]) {
      case 'm':
        // move wheels
        setMotors(instruction[1]);
        break;
      case 's':
        // speed of wheels
        setWheelSpeed(instruction[1]);
      case 'l':
        // leds
        turnOnLED(instruction[1]);
        break;
      case 'd':
        // lcd
        LCD = ON;
        break;
      case 'h':
        // turn head
        turnHead(instruction[1]);
      case 'q':
        // quit - turn off everything
        setAllLEDsOFF();
        setAllWheelsOff();
        LCD = OFF;
    }

    Serial.print("Received instruction: ");
    Serial.println(instruction);
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
  digitalWrite(LCD_PIN, LCD);
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
      break;
    case 'r':
      // turn right
      LEFT_FORWARD = HIGH;
      break;
    default:
      break;
  }
}

/* 
 *  Sets the wheel speed for the motors.
 *  dir: incoming instruction code
 */
void setWheelSpeed(char dir)
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
void turnOnLED(char LED)
{
  setAllLEDsOFF();
  switch (LED) {
  case 'y':
    LED_Y = ON;
    break;
  case 'g':
    LED_G = ON;
    break;
  case 'r':
    LED_R = ON;
    break;
  case 'b':
    LED_B = ON;
    break;
  case 'o':
    setAllLEDsOFF();
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
    TURN_HEAD_LEFT = LOW;
    TURN_HEAD_RIGHT = LOW;
  } else {
    // default - do not rotate
    TURN_HEAD_LEFT = LOW;
    TURN_HEAD_RIGHT = LOW;
  }
}
