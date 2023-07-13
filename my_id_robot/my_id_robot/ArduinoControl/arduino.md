# ArduinoControl Documentation
This is the documentation for the ArduinoControl program. It will include information on the serial communication as well as how the incoming messages are interpreted by the code.

[Return to Main](../../../README.md)
## Setup and Pinout
Baud Rate: 9600
| Arduino Pin | Pin Alias       | Description                                                                                                                                                               |
|-------------|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| D2          | "LF_PIN"        | Connected to IN1 on the L298N Motor driver for the wheels. When driven high it spins the left wheel in the forward direction.                                             |
| D3          | "LEFT_PWM_PIN"  | Connected to ENA on the L298N Motor driver for the wheels. Program uses PWM to write the speed value to the driver which will impact either direction of the left motor.  |
| D4          | "LB_PIN"        | Connected to IN2 on the L298N Motor driver for the wheels. When driven high it spins the left wheel in the backward direction.                                            |
| D5          | "RIGHT_PWM_PIN" | Connected to ENB on the L298N Motor driver for the wheels. Program uses PWM to write the speed value to the driver which will impact either direction of the right motor. |
| D6          | "HEAD_PWM_PIN"  | Connected to ENA on the second L298N Motor drive. Program uses PWM to write the speed value to the driver which will impact either direction of the head motor.           |
| D7          | "RF_PIN"        | Connected to IN3 on the L298N Motor driver for the wheels. When driven high it spins the right wheel in the forward direction.                                            |
| D8          | "RB_PIN"        | Connected to IN4 on the L298N Motor driver for the wheels. When driven high it spins the right wheel in the backward direction.                                           |
| D9          |                 |                                                                                                                                                                           |
| D10         | "HL_PIN"        | Connected to IN1 on the second L298N Motor driver. When driven high it spins the head left.                                                                               |
| D11         | "HR_PIN"        | Connected to IN2 on the second L298N Motor driver. When driven high it spins the head right.                                                                              |
| D12         |                 |                                                                                                                                                                           |
| D13         |                 |                                                                                                                                                                           |
| A0          | "LED_Y_PIN"     | Connected to the YELLOW LED.                                                                                                                                              |
| A1          | "LED_G_PIN"     | Connected to the GREEN LED.                                                                                                                                               |
| A2          | "LED_R_PIN"     | Connected to the RED LED.                                                                                                                                                 |
| A3          | "LED_B_PIN"     | Connected to the BLUE LED.                                                                                                                                                |
| A4          | "LCD_PIN"       | Connected to both LCDs on the R2D2.                                                                                                                                       |
| A5          | "PROJ_PIN"       | Connected to pin for projector button.                                                                                                                                       |
## Schematic
![image](https://github.com/Myapi314/R2D2Project/assets/97209406/1872a832-ad23-4fe4-9d23-5e6626a209b1)

## Instructions
The program accepts 1-2 character strings ending in "\n". The first character indicates the category of instruction: move the wheels, change the speed, turn on an led, turn the head, turn on the lcds. The second character (if there is one) indicates a sub-instruction in that category.

Default Speed Value: 150

Speed Increment Value: 50
| Instruction Code | Description                                                            |
|------------------|------------------------------------------------------------------------|
| "ms"             | Stop all wheels.                                                       |
| "mf"             | Move wheels forward.                                                   |
| "mb"             | Move wheels backward.                                                  |
| "ml"             | Move/turn left.                                                        |
| "mr"             | Move/turn right.                                                       |
| "m"              | DEFAULT - changes nothing                                              |
| "su"             | Speed up. Increases speed up to max value of 255 by defined INC value. |
| "sd"             | Slow down. Decreases speed to min value of 0, by defined INC value.    |
| "sn"             | Set speed to default speed.                                            |
| "s"              | DEFAULT - changes nothing                                              |
| "lyh"             | Turn on yellow LED.                                                    |
| "lgh"             | Turn on green LED.                                                     |
| "lrh"             | Turn on red LED.                                                       |
| "lbh"             | Turn on blue LED.                                                      |
| "lyo"             | Turn off yellow LED.                                                    |
| "lgo"             | Turn off green LED.                                                     |
| "lro"             | Turn off red LED.                                                       |
| "lbo"             | Turn off blue LED.                                                      |
| "lo"             | Turn off all LEDs.                                                     |
| "l"              | DEFAULT - changes nothing                                              |
| "hl"             | Turn head left.                                                        |
| "hr"             | Turn head right.                                                       |
| "hs"             | Stop head.                                                        |
| "h"              | DEFAULT - changes nothing                                              |
| "d"              | Turn on LCD                                                            |
| "p"              | "Press" projector button                                                            |
| "q"              | Quit - turn off everything.                                            |
