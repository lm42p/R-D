#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "FastAccelStepper.h"

// As in StepperDemo for Motor 1 on ESP32
//#define enablePinStepper 26 // pas utilisé encore

//speed rotary encoder
#define ROTARY_ENCODER_A_PIN 27 //CLK
#define ROTARY_ENCODER_B_PIN 26 //DT
#define ROTARY_ENCODER_BUTTON_PIN 32 //SW
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_ACCELERATION 3000 //30000
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
const int MAX_SPEED = 50; //set max speed in us/step
const int MIN_STROKE = 10; // 10 vibro_stroke = 10
const int MAX_STROKE = 3050; //15000 3000 3100  3050

// Motor acceleration
int motorAcceleration = 1600000; //115000 vibro_accel = 8000000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
long target=0; // it's the target
int previousDirection = 1;
bool stopped = false;
unsigned long lastButtonPress = 0;  

void setup() {
  Serial.begin(115200);
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);

  pinMode(ROTARY_ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER2_B_PIN, INPUT_PULLUP);

  //speed
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(MAX_SPEED,MIN_SPEED, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
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

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(50);  //1000// the parameter is us/step !!!
    stepper->setAcceleration(1600000); //100 200 160000

    stepper->setCurrentPosition(0); // put a reference at 0
  }
}

void loop() {
  // just move the stepper back and forth in an endless loop
  if (not(stepper->isRunning()) && not(stopped)){
    previousDirection *= -1;
    target = rotaryEncoder2.readEncoder();
    long relativeTargetPosition = -target * previousDirection;
    stepper->moveTo(relativeTargetPosition,true);
  }

// speed rotary encoder
  if (rotaryEncoder.encoderChanged())
  {
    Serial.println(rotaryEncoder.readEncoder());
    stepper->setSpeedInUs(rotaryEncoder.readEncoder());
  }
  //if (rotaryEncoder.isEncoderButtonClicked())
  //{
  //  Serial.println("button pressed");
  //  flag=true; //machine stops
  //  distanceToTravel=0;
  //}

// stroke rotary encoder
  if (rotaryEncoder2.encoderChanged())
  {
    Serial.println(rotaryEncoder2.readEncoder());
  }
  if (rotaryEncoder2.isEncoderButtonClicked()){
    if (millis() - lastButtonPress > 50){
    stopped = not(stopped);
    }
    lastButtonPress = millis();
    Serial.println(stopped);
  }
}
