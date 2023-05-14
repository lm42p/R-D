#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "FastAccelStepper.h"

//speed rotary encoder
#define ROTARY_ENCODER_A_PIN 27 //CLK
#define ROTARY_ENCODER_B_PIN 26 //DT
#define ROTARY_ENCODER_BUTTON_PIN 32 //SW
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_ACCELERATION 2000 //30000 3000
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
const int MIN_SPEED = 2000; //set min speed in us/step
const int MAX_SPEED = 50; //set max speed in us/step 50 20 5
const int MIN_STROKE = 10; // 10 vibro_stroke = 10
const int MAX_STROKE = 2950; //15000 3000 3100  3050, 2800,2000, 3050, 3000

// Motor acceleration
int motorAcceleration = 1600000; //115000 vibro_accel = 8000000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

long target = 0; // it's the target
int previousDirection = 1;
bool stopped = true; //machine stopped
unsigned long lastButtonPress = 0; // avoid rebounce when button is pressed
bool green = false; // green side of the machine is used
bool red = false; // red side of the machine is used
const int GAP = 50;

void setup() {
  Serial.begin(115200);
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);

  pinMode(ROTARY_ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER2_B_PIN, INPUT_PULLUP);

  //speed
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(MAX_SPEED, MIN_SPEED, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(ROTARY_ENCODER_ACCELERATION);

  //stroke
  rotaryEncoder2.begin();
  rotaryEncoder2.setup(readEncoderISR);
  rotaryEncoder2.setBoundaries(MIN_STROKE, MAX_STROKE, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder2.setAcceleration(ROTARY_ENCODER2_ACCELERATION);

  engine.init();
  stepper = engine.stepperConnectToPin(MOTOR_STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(MOTOR_DIRECTION_PIN);
    //stepper->setEnablePin(enablePinStepper); //not used yet
    stepper->setAutoEnable(true);

    stepper->setSpeedInUs(2000);  //1000// the parameter is us/step !!!
    stepper->setAcceleration(740000); //100 200 1600000, 16000, 160000, 160000, 320000
      }

// determination which side is used
  while (((millis() - lastButtonPress < 2000)) || not((green) || (red))){
// determine which side is used
    if (rotaryEncoder2.isEncoderButtonClicked()) { // if speed button clicked then stop the machine
      if (millis() - lastButtonPress > 50) {
        green = true;
      }
      lastButtonPress = millis();
      Serial.println("green side is used");
    }
    if (rotaryEncoder.isEncoderButtonClicked()) { // if speed button clicked then stop the machine
      if (millis() - lastButtonPress > 50) {
        red = true;
      }
      lastButtonPress = millis();
      Serial.println("red side is used");
    }
  }

  if (green && not(red)){
    stepper->setCurrentPosition(GAP); // put a reference at 50 to provide a physical gap
  }
  else if (red && not(green)){
    stepper->setCurrentPosition(-GAP); // put a reference at -50 to provide a physical gap
  }
  else stepper->setCurrentPosition(((MAX_STROKE/2)+GAP)); // put a reference at -50 to provide a physical gap}

    // This provide the physical gap
    stepper->moveTo(0, true);

// This is for safety. We have to put the speed and stroke at their minimum before we can
// start the moves
  while (not(rotaryEncoder.readEncoder() == MIN_SPEED)  || not(rotaryEncoder2.readEncoder() == MAX_STROKE)) {}

}

void loop() {
  // just move the stepper back and forth in an endless loop
  if (not(stepper->isRunning()) && not(stopped)) {
    previousDirection *= -1;
    target = abs(rotaryEncoder2.readEncoder() - MAX_STROKE + MIN_STROKE); // to inverse
    long relativeTargetPosition = -target * previousDirection;
    if (green&&not(red)){
      if (relativeTargetPosition > 0) relativeTargetPosition = 0; // > is on green side
    }
    else if (red&&not(green)){
      if (relativeTargetPosition < 0) relativeTargetPosition = 0; // < is on red side
    }
    else{
    }
    stepper->moveTo(relativeTargetPosition, true);
  }

  // speed rotary encoder
  if (rotaryEncoder.encoderChanged()) { // don't change the speed when it's stopped
    Serial.println(rotaryEncoder.readEncoder());
    stepper->setSpeedInUs(rotaryEncoder.readEncoder());
    stopped = false; //otherwise we can change speed and stroke during stop
  }
  if (rotaryEncoder.isEncoderButtonClicked()) { // if speed button clicked then stop the machine
    if (millis() - lastButtonPress > 500) { //avant 50
      if (not(stopped)) {
        stopped = true;
      }
    }
    lastButtonPress = millis();
    Serial.println(stopped);
  }

  // stroke rotary encoder
  if (rotaryEncoder2.encoderChanged()) {
    Serial.println(rotaryEncoder2.readEncoder());
    stopped = false; //otherwise we can change speed and stroke during stop
  }
}
