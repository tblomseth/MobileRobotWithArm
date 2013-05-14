#include <Servo.h>
#include "MakeblockStepper.h"
#include <Me_BaseShield.h>
#include <Me_InfraredReceiver.h>
#include <Me_ServoDriver.h>
#include <Me_MotorDriver.h>
#include <Me_BaseShieldMotorDriver.h>

int posStartS1;
int posStartS2;
int posCurrentS1;
int posCurrentS2;
int inc = 5;
uint8_t motorSpeed = 100;
int leftSpeed = 60;
int rightSpeed = 60;
const int stepsPerRevolution = 3200; // change this to fit the number of steps per revolution for your motor
int stepperSpeed = 18;
int armMicroSwitchPort = PORT_8;

Me_BaseShield baseShield;
Me_ServoDriver servoDriver( PORT_2 ); //can ONLY be PORT_1,PORT_2
Me_InfraredReceiver infraredReceiver; 
Me_MotorDriver motorDriver1( PORT_1 );
Me_BaseShieldMotorDriver baseShieldMotorDriver;// use M1 and M2 ports on BaseShield
// initialize the stepper library on pins 8 through 11:
MakeblockStepper armStepper( stepsPerRevolution, baseShield, PORT_3 );

void setup() {
  baseShield.begin();
  // initialize serial communication with computer:
  Serial.begin( 9600 );
  // initialize servo driver:
  servoDriver.Servos_begin();
  // initialize Infrared Receiver:
  infraredReceiver.begin();
  // Init arm motor driver
  motorDriver1.begin();
  // initialize BaseShield Motor Driver:
  baseShieldMotorDriver.begin();
  // Set stepper speed
  armStepper.setSpeed( stepperSpeed );

  /*
  posStartS1 = posCurrentS1 = servoDriver.readServo1();
   Serial.println( "Servo 1 start pos: " + posStartS1 );
   posStartS2 = posCurrentS2 = servoDriver.readServo2();
   Serial.println( "Servo 2 start pos: " + posStartS2 );
   */
}

void loop()
{
  int key = infraredReceiver.read();
  if(key>=0)
  {
    //Serial.println(key);
    switch (key)
    {
    case IR_TEST_BUTTON: 
      //readPositions();
      moveArmAndTiltGripper();
      break;
    case IR_RETURN_BUTTON: 
      resetServos();
      break;
    case IR_BUTTON_0: 
      motorDriver1.stop();
      break;
    case IR_BUTTON_1: 
      motorDriver1.run( motorSpeed );
      break;
    case IR_BUTTON_2: 
      runS2CounterClockwise();
      break;
    case IR_BUTTON_3:
      runS1Clockwise();
      break;
    case IR_BUTTON_4: 
      motorDriver1.run( -motorSpeed );
      break;
    case IR_BUTTON_5: 
      runS2Clockwise();
      break;
    case IR_BUTTON_6:
      runS1CounterClockwise();
      break;
    case IR_BUTTON_7:
      //stepArmOneRevolutionClockwise();
      break;
    case IR_BUTTON_8:
      //stepArmOneRevolutionCounterclockwise();
      break;
    case IR_BUTTON_9:
      initializeArm();
      break;
    case IR_PLUS_BUTTON:
      runForward();
      break;
    case IR_PREVIOUS_BUTTON:
      runLeft();
      break;
    case IR_PLAY_BUTTON:
      runStop();
      break;
    case IR_NEXT_BUTTON:
      runRight();
      break;
    case IR_MINUS_BUTTON:
      runBack();
      break;
    default:
      break;
    }
  }
}

/*
 * Mobility functions
 */

void runForward() {
  Serial.println("run forward");
  baseShieldMotorDriver.runMotors(leftSpeed,-rightSpeed);
}

void runLeft() {
  Serial.println("run left");
  baseShieldMotorDriver.runMotors(0,-rightSpeed + 10); //baseShieldMotorDriver.runMotors(-leftSpeed,-rightSpeed);
}

void runRight() {
  Serial.println("run right");
  baseShieldMotorDriver.runMotors(leftSpeed - 10,0);
}

void runStop() {
  Serial.println("run stop");
  baseShieldMotorDriver.stopMotors();
}

void runBack() {
  Serial.println("run back");
  baseShieldMotorDriver.runMotors( -leftSpeed, rightSpeed );
}

/*
 * Servo functions
 */

void readPositions() {
  Serial.println( "Servo 1 position:" );
  Serial.println( servoDriver.readServo1() );
  Serial.println( "Servo 2 position:" );
  Serial.println( servoDriver.readServo2() );

}

void runS1Clockwise() {
  servoDriver.writeServo1( posCurrentS1 + inc );
  posCurrentS1 = servoDriver.readServo1();
}

void runS2Clockwise() {
  if (posCurrentS2 <= 110 - inc) {
    servoDriver.writeServo2( posCurrentS2 + inc );
    posCurrentS2 = servoDriver.readServo2();
  }
}

void runS1CounterClockwise() {
  servoDriver.writeServo1( posCurrentS1 - inc );
  posCurrentS1 = servoDriver.readServo1();
}

void runS2CounterClockwise() {
  if ( posCurrentS2 >= 75 + inc ) {
    servoDriver.writeServo2( posCurrentS2 - inc );
    posCurrentS2 = servoDriver.readServo2();
  }
}

void resetServos() {
  servoDriver.writeServo1( posStartS1 );
  posCurrentS1 = servoDriver.readServo1();
  servoDriver.writeServo2( posStartS2 );
  posCurrentS2 = servoDriver.readServo2();
}

/*
 * Arm stepper functions
 */
/*
void stepArmOneRevolutionClockwise() {
 Serial.println( "Stepping one revolution clockwise" );
 armStepper.setSpeed( stepperSpeed );
 armStepper.step( stepsPerRevolution );
 //armStepper.setSpeed( 20 );
 armStepper.step( stepsPerRevolution );
 //armStepper.setSpeed( 30 );
 armStepper.step( stepsPerRevolution );
 //armStepper.setSpeed( 75 );
 armStepper.step( stepsPerRevolution );  
 }
 
 void stepArmOneRevolutionCounterclockwise() {
 Serial.println( "Stepping one revolution counterclockwise" );
 armStepper.step( -stepsPerRevolution );
 }
 */

void initializeArm() {
  int microSwitchState = baseShield.readMePortOutsidePin( armMicroSwitchPort ); 
  while ( microSwitchState == HIGH ) {
    armStepper.step( 1 ); 
    delay( 1 );
    microSwitchState = baseShield.readMePortOutsidePin( armMicroSwitchPort );
  }
}

function moveArmAndTiltGripper( gripperStartAngle, tiltDifference, armSteps ) {
  if ( armSteps == 0 || tiltDifference == 0 ) return;  
  int armStepsLeft = abs( armSteps );
  int stepDirection = armSteps / abs( armSteps );
  int gripperAngle = gripperStartAngle;
  int tiltLeft = abs ( tiltDifference );
  int tiltDirection = tiltDifference / abs( tiltDifference );
  int stepsPerShortWalk = floor( armStepsLeft / tiltLeft );
  Serial.println( "stepsPerShortWalk: " );
  Serial.println( stepsPerShortWalk );

  int numberOfLongWalks = armStepsLeft - ( tiltLeft * stepsPerShortWalk );
  Serial.println( "numberOfLongWalks:" );
  Serial.println( numberOfLongWalks );

  int numberOfShortWalks = tiltLeft - numberOfLongWalks;
  Serial.println( "numberOfShortWalks:" );
  Serial.println( numberOfShortWalks );
  int iterations = 0;
  for ( ; tiltLeft > 0; tiltLeft--) {
      console.log(++iterations + ')----------');
      
    // Move arm
      if ( tiltLeft > numberOfShortWalks ) {
          armStepper.step( stepDirection * ( stepsPerShortWalk + 1 ) );
          armStepsLeft -= stepsPerShortWalk + 1 ;
      } else {
          armStepper.step( stepDirection * stepsPerShortWalk );
          armStepsLeft -= stepsPerShortWalk;
      }

    Serial.println( "Steps left: " );
    Serial.println( armStepsLeft );
    // Tilt gripper
    gripperAngle += tiltDirection;
    servoDriver.writeServo1( gripperAngle );
    Serial.println( "Gripper angle: " );
    Serial.println( gripperAngle );
    
    // Wait some
    delay( 100 );
  }
}


