#include "AiEsp32RotaryEncoder.h"

#include "Arduino.h"
#include <ESP_FlexyStepper.h>

// 400 c'est le min je crois pour pas que le motor perdre les pédales

//speed rotary encoder
#define ROTARY_ENCODER_A_PIN 27 //CLK
#define ROTARY_ENCODER_B_PIN 26 //DT
#define ROTARY_ENCODER_BUTTON_PIN 32 //SW
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_ACCELERATION 30000
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

// stroke rotary encoder
#define ROTARY_ENCODER2_A_PIN 13 //CLK
#define ROTARY_ENCODER2_B_PIN 35 //DT
#define ROTARY_ENCODER2_BUTTON_PIN 14 //SW
#define ROTARY_ENCODER2_STEPS 4
#define ROTARY_ENCODER2_ACCELERATION 7000
AiEsp32RotaryEncoder rotaryEncoder2 = AiEsp32RotaryEncoder(ROTARY_ENCODER2_A_PIN, ROTARY_ENCODER2_B_PIN, ROTARY_ENCODER2_BUTTON_PIN, -1, ROTARY_ENCODER2_STEPS);

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
  rotaryEncoder2.readEncoder_ISR();
}

// IO pin assignments
const int MOTOR_STEP_PIN = 33;
const int MOTOR_DIRECTION_PIN = 25;

// Speed and stroke settings
const int MIN_SPEED = 480;//894; // under that value the speed doesn't reduce I don't know why
const int MAX_SPEED = 20000; // après 35 pas plus de vitesse vibro_speed = 5200
const int MIN_STROKE = 400; // 400 vibro_stroke = 148
const int MAX_STROKE = 3050; //15000 3000 3100  3050
int distanceToTravel = 0; // it's the distance to travel
const int ACCELERATION_IN_STEPS_PER_SECOND = 115000; //115000 vibro_accel = 8000000
const int DECELERATION_IN_STEPS_PER_SECOND = 115000; //115000

bool flag = false; // to stop motor
// create the stepper motor object
ESP_FlexyStepper stepper;

int previousDirection = 1;

void setup()
{
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);

  pinMode(ROTARY_ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER2_B_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  //speed
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(MIN_SPEED, MAX_SPEED, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(ROTARY_ENCODER_ACCELERATION);
  
  //stroke
  rotaryEncoder2.begin();
  rotaryEncoder2.setup(readEncoderISR);
  rotaryEncoder2.setBoundaries(MIN_STROKE, MAX_STROKE, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder2.setAcceleration(ROTARY_ENCODER2_ACCELERATION);

  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(MIN_SPEED);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);

  // Not start the stepper instance as a service in the "background" as a separate task
  // and the OS of the ESP will take care of invoking the processMovement() task regularily so you can do whatever you want in the loop function
  stepper.startAsService(0);
}

void loop()
{
  stepper.setSpeedInStepsPerSecond(rotaryEncoder.readEncoder());
  
  // just move the stepper back and forth in an endless loop
  if (stepper.getDistanceToTargetSigned() == 0)
  {
    //delay(5000);
    previousDirection *= -1;
    if ((previousDirection > 0) && (!flag))
    {
    distanceToTravel = rotaryEncoder2.readEncoder();
    }
    //Serial.print(distanceToTravel);
    long relativeTargetPosition = -distanceToTravel * previousDirection; // mettre rotaryEncoder.readEncoder() à la place de DISTAN...
    //Serial.printf("Moving stepper by %ld steps\n", relativeTargetPosition);
    stepper.setTargetPositionRelativeInSteps(relativeTargetPosition);
  }

  if (rotaryEncoder.encoderChanged())
  {
    Serial.println(rotaryEncoder.readEncoder());
    flag=false; //machine run again if machine has stopped
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    Serial.println("button pressed");
    flag=true; //machine stops
    distanceToTravel=0;
  }

  if (rotaryEncoder2.encoderChanged())
  {
    Serial.println(rotaryEncoder2.readEncoder());
  }
  //if (rotaryEncoder2.isEncoderButtonClicked())
  //{
  //  Serial.println("button 2 pressed");
  //}
}
