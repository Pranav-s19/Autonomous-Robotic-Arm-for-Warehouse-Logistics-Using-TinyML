#include <Servo.h>
#include <Stepper.h>

Servo servo1;
Servo servo2;
Servo servo3;

const int buttonPin1 = 3; // Pin for the button
int buttonState1 = HIGH; // Initial button state (pull-up, so HIGH means not pressed)

const int buttonPin2 = 2; // Pin for the button
int buttonState2 = HIGH; // Initial button state (pull-up, so HIGH means not pressed)

unsigned long previousMillis1 = 0; // Stores the last time servo1 was moved
unsigned long previousMillis2 = 0; // Stores the last time servo2 was moved
unsigned long previousMillis3 = 0; // Stores the last time servo3 was moved
unsigned long previousMillisStepper = 0; // Stores the last time the stepper was moved
const unsigned long interval = 1000; // 1 second interval

int stepState = 0; // Tracks which operation is currently running

bool buttonPressed1 = false;
bool buttonPressed2 = false;

int servo1Angle = 180; // Initial angle for servo1
int servo2Angle = 180; // Initial angle for servo2
int servo3Angle = 90; // Initial angle for servo3
const int stepDelayServo1 = 15; // Delay between steps for servo1 in milliseconds
const int stepDelayServo2 = 20; // Delay between steps for servo2 in milliseconds
const int stepDelayServo3 = 15; // Delay between steps for servo3 in milliseconds

const int stepsPerRevolution = 2048; // Steps for one revolution of the stepper motor
Stepper stepper(stepsPerRevolution, 4, 6, 5, 7); // Initialize stepper motor pins

void setup() {
  servo1.attach(10); // Attach the first servo mg90s to pin 9
  servo2.attach(9); // Attach the second servo mg945 to pin 10
  servo3.attach(11); // Attach the third servo to pin 11

  pinMode(buttonPin1, INPUT_PULLUP); // Set the button pin as input with internal pull-up
  pinMode(buttonPin2, INPUT_PULLUP); // Set the button pin as input with internal pull-up


  servo1.write(180); // Initialize the first servo to 180 degrees
  servo2.write(180); // Initialize the second servo to 180 degrees
  servo3.write(90);  // Initialize the third servo to 90 degrees

  stepper.setSpeed(15); // Set the speed of the stepper motor (RPM)
}

void loop() {
  buttonState1 = digitalRead(buttonPin1); // Read the button state

  if (buttonState1 == LOW && !buttonPressed1) { // Button is pressed (active low)
    buttonPressed1 = true; // Mark button as pressed
    stepState = 0; // Start from the first step
  }

  if (buttonPressed1) {
    unsigned long currentMillis = millis();

    switch (stepState) {
      case 0: // Servo 1 and 2 move from 180 to 140 and 155
        if (currentMillis - previousMillis1 >= stepDelayServo1) {
          previousMillis1 = currentMillis;
          if (servo1Angle > 140) {
            servo1Angle--;
            servo1.write(servo1Angle);
          }
        }
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle > 155) {
            servo2Angle--;
            servo2.write(servo2Angle);
          }
        }
        if (servo1Angle == 140 && servo2Angle == 155) stepState++;
        break;

      case 1: // Servo 3 moves from 90 to 20
        if (currentMillis - previousMillis3 >= stepDelayServo3) {
          previousMillis3 = currentMillis;
          if (servo3Angle > 20) {
            servo3Angle--;
            servo3.write(servo3Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 2: // Servo 2 moves from 140 to 165
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle < 175) {
            servo2Angle++;
            servo2.write(servo2Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 3: // Stepper moves from 0 to 90
        if (currentMillis - previousMillisStepper >= interval) {
          previousMillisStepper = currentMillis;
          stepper.step(stepsPerRevolution / 4); // Move stepper motor 90 degrees
          stepState++;
        }
        break;

      case 4: // Servo 2 moves back from 160 to 155
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle > 155) {
            servo2Angle--;
            servo2.write(servo2Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 5: // Servo 3 moves back from 20 to 90
        if (currentMillis - previousMillis3 >= stepDelayServo3) {
          previousMillis3 = currentMillis;
          if (servo3Angle < 90) {
            servo3Angle++;
            servo3.write(servo3Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 6: // Servo 1 and 2 move back to 180 simultaneously
        if (currentMillis - previousMillis1 >= stepDelayServo1) {
          previousMillis1 = currentMillis;
          if (servo1Angle < 180) {
            servo1Angle++;
            servo1.write(servo1Angle);
          }
        }
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle < 180) {
            servo2Angle++;
            servo2.write(servo2Angle);
          }
        }
        if (servo1Angle == 180 && servo2Angle == 180) {
          stepState++;
        }

        break;

      case 7: // Stepper moves back from 90 to 0
        if (currentMillis - previousMillisStepper >= interval) {
          previousMillisStepper = currentMillis;
          stepper.step(-stepsPerRevolution / 4); // Rotate stepper motor back to 0
        }
        if (servo1Angle == 180 && servo2Angle == 180) {
          buttonPressed1 = false; // Reset button press state
        }
        
        break;
    }
  }

  buttonState2 = digitalRead(buttonPin2); // Read the button state

  if (buttonState2 == LOW && !buttonPressed2) { // Button is pressed (active low)
    buttonPressed2 = true; // Mark button as pressed
    stepState = 0; // Start from the first step
  }

  if (buttonPressed2) {
    unsigned long currentMillis = millis();

   switch (stepState) {
      case 0: // Servo 1 and 2 move from 180 to 140 and 155
        if (currentMillis - previousMillis1 >= stepDelayServo1) {
          previousMillis1 = currentMillis;
          if (servo1Angle > 140) {
            servo1Angle--;
            servo1.write(servo1Angle);
          }
        }
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle > 155) {
            servo2Angle--;
            servo2.write(servo2Angle);
          }
        }
        if (servo1Angle == 140 && servo2Angle == 155) stepState++;
        break;

      case 1: // Servo 3 moves from 90 to 20
        if (currentMillis - previousMillis3 >= stepDelayServo3) {
          previousMillis3 = currentMillis;
          if (servo3Angle > 20) {
            servo3Angle--;
            servo3.write(servo3Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 2: // Servo 2 moves from 140 to 165
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle < 180) {
            servo2Angle++;
            servo2.write(servo2Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 3: // Stepper moves from 90 to 0
        if (currentMillis - previousMillisStepper >= interval) {
          previousMillisStepper = currentMillis;
          stepper.step(-stepsPerRevolution / 4); // Move stepper motor 90 degrees
          stepState++;
        }
        break;

      case 4: // Servo 2 moves back from 160 to 155
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle > 155) {
            servo2Angle--;
            servo2.write(servo2Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 5: // Servo 3 moves back from 20 to 90
        if (currentMillis - previousMillis3 >= stepDelayServo3) {
          previousMillis3 = currentMillis;
          if (servo3Angle < 90) {
            servo3Angle++;
            servo3.write(servo3Angle);
          } else {
            stepState++;
          }
        }
        break;

      case 6: // Servo 1 and 2 move back to 180 simultaneously
        if (currentMillis - previousMillis1 >= stepDelayServo1) {
          previousMillis1 = currentMillis;
          if (servo1Angle < 180) {
            servo1Angle++;
            servo1.write(servo1Angle);
          }
        }
        if (currentMillis - previousMillis2 >= stepDelayServo2) {
          previousMillis2 = currentMillis;
          if (servo2Angle < 180) {
            servo2Angle++;
            servo2.write(servo2Angle);
          }
        }
        if (servo1Angle == 180 && servo2Angle == 180) {
          stepState++;
        }

        break;

      case 7: // Stepper moves back from 0 to 90
        if (currentMillis - previousMillisStepper >= interval) {
          previousMillisStepper = currentMillis;
          stepper.step(stepsPerRevolution / 4); // Rotate stepper motor back to 0
        }
        if (servo1Angle == 180 && servo2Angle == 180) {
          buttonPressed2 = false; // Reset button press state
        }
        
        break;
    }
  }
}
