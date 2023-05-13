
#include <AccelStepper.h>

//User-defined values
long receivedSteps = 0; //Number of steps
long receivedSpeed = 0; //Steps / second
long receivedMaxSpeed = 0; //Steps / second
long receivedAcceleration = 0; //Steps / second^2
long CurrentPosition = 0;
char receivedCommand; //a letter sent from the terminal
long StartTime = 0;
long PreviousTime = 0;
//-------------------------------------------------------------------------------
bool newData, runallowed = false; // booleans for new data from serial, and runallowed flag
bool lastStepPosition = false; //follows the steps to see if the last step was preformed
bool pingpong_CW = true;
bool pingpong_CCW = true;
bool pingpongAllowed = false;
//-------------------------------------------------------------------------------
AccelStepper stepper(1, 33, 25);// direction Digital 9 (CCW), pulses Digital 8 (CLK)

void setup()
{
  Serial.begin(115200); //define a baud rate

  //setting up some default values for maximum speed and maximum acceleration
  Serial.println("Default speed: 400 steps/s, default acceleration: 800 steps/s^2.");
  stepper.setMaxSpeed(400); //SPEED = Steps / second
  stepper.setAcceleration(800); //ACCELERATION = Steps /(second)^2
  stepper.disableOutputs(); //disable outputs
}

void loop() 
{
  //Constantly looping through these 4 functions. 
  //We only use non-blocking commands, so something else (should also be non-blocking) can be done during the movement of the motor

  RunTheMotor(); //function to handle the motor   
  PingPong();
}


void RunTheMotor() //function for the motor
{
  if (stepper.distanceToGo() != 0)
  {
    stepper.enableOutputs(); //enable pins
    stepper.run(); //step the motor (this will step the motor by 1 step at each loop) 
    lastStepPosition = true;
  }
  else //program enters this part if the runallowed is FALSE, we do not do anything
  {
     stepper.disableOutputs(); //disable outputs
     if(lastStepPosition == true)
     {
      lastStepPosition = false;
     }
    return;
  }
}

void PingPong()
{       
    if(pingpongAllowed == true) //If the pingpong function is allowed we enter
    {
      if(pingpong_CW == false) //CW rotation is not yet done
      {  
        stepper.moveTo(5000); //set a target position, it should be an absolute. relative (move()) leads to "infinite loop"
        
        if(stepper.distanceToGo() == 0) //When the above number of steps are completed, we manipulate the variables
        {
            pingpong_CW = true; //CW rotation is now done
            pingpong_CCW = false; //CCW rotation is not yet done - this allows the code to enter the next ifs
        }      
      }
        
      if(pingpong_CW == true && pingpong_CCW == false) //CW is completed and CCW is not yet done
      {
        stepper.moveTo(0); //Absolute position 
        
        if(stepper.distanceToGo() == 0) //When the number of steps are completed
          {
            pingpong_CCW = true; //CCW is now done
            pingpong_CW = false; //CW is not yet done. This allows the code to enter the first if again!
          }     
      }
    }
}

void RotateRelative()
{
  //We move X steps from the current position of the stepper motor in a given direction (+/-).  
  runallowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed); //set speed 
  stepper.move(receivedSteps); //set relative distance and direction
}

void RotateAbsolute()
{
  //We move to an absolute position. 
  //The AccelStepper library keeps track of the position.
 
  runallowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed); //set speed
  stepper.moveTo(receivedSteps); //set relative distance  
}

void PrintCommands()
{ 
  //Printing the commands
  Serial.println(" 'C' : Prints all the commands and their functions.");
  Serial.println(" 'P' : Rotates the motor - relative using move().");
  Serial.println(" 'R' : Rotates the motor - absolute using moveTo().");
  Serial.println(" 'S' : Stops the motor immediately.");
  Serial.println(" 'A' : Sets an acceleration value.");
  Serial.println(" 'V' : Sets a speed value using setSpeed().");
  Serial.println(" 'v' : Sets a speed value using setMaxSpeed().");
  Serial.println(" 'L' : Prints the current position/location of the motor using currentPosition().");
  Serial.println(" 'U' : Updates the current position and makes it as the new 0 position using setCurrentPosition().");
  Serial.println(" 'K' : Demonstrates an oscillating motion.");
