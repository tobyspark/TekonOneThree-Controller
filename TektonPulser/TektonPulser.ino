/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

# define kStepPin 10
# define kDirPin  16
# define kProxPin 3
# define kLEDPin  17

// At 800 steps this gives ~2m travel, and this works
// At 1600 steps this should give 2m travel, but it gives silky smooth ~1m travel

# define kMaxSteps 19000

# define kPulseMicros 5 // Triggers on rising and falling edge

//# define SERIALDEBUG

unsigned long stepPulseTime = 0;
bool          stepPulseState = false;

int positionNext = 0;
int positionLast = 0;

int desiredSteps = 0;
int currentSteps = 0;

int lastProxSteps = -1;

bool stepDirToClose = true;

unsigned long commandLastTime = 0;
unsigned long commandFreqMicros = 1000000/60;

unsigned long startTime = 0;
uint16_t zeroStepOffset = 100;
bool riseToZero = true;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(kStepPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kProxPin, INPUT);
  digitalWrite(kDirPin, true);
}

// FOLLOWING 60FPS SIN CALC
#if true
void loop() {
  if (riseToZero) delay(5000);
  while (riseToZero)
  {
    stepPulseState = !stepPulseState;
    digitalWrite(kStepPin, stepPulseState);
    currentSteps += 1;
    delayMicroseconds(kPulseMicros * 500);
    if (currentSteps > zeroStepOffset)
    {
      currentSteps = 0;
      startTime = micros();
      riseToZero = false;
    }
  }
 
  unsigned long currentTime = micros() - startTime;
  unsigned long stepPulseMicros = currentTime - stepPulseTime;
  unsigned long commandMicros = currentTime - commandLastTime;
  
  // TASK: Correct for position drift
  if (digitalRead(kProxPin))
  {
    lastProxSteps = currentSteps;
    //Serial.print("PROX: "); //// NO NO NO! Causes horrible mechanical vibration!
    //Serial.println(currentSteps);
  }
  if (currentSteps == 0 && lastProxSteps != -1)
  {    
    // Get offset and correct
    currentSteps = (kMaxSteps / 2) - lastProxSteps;
    stepDirToClose = (currentSteps < 0);
    digitalWrite(kDirPin, stepDirToClose);
    while (currentSteps != 0)
    {
      stepPulseState = !stepPulseState;
      digitalWrite(kStepPin, stepPulseState);
      currentSteps += stepDirToClose ? 1 : -1;
      delayMicroseconds(kPulseMicros * 500);
    }
    
    // Don't do this again until a new value has been read
    lastProxSteps = -1;
    
    // Reset time so sin wave picks up from bottom
    startTime = micros();
  }
  
  // TASK: Command position at 60fps
  if (commandMicros > commandFreqMicros)
  {
    positionLast = positionNext;
    float normalisedPosition = (sin(((currentTime*1.0) / 1000000.0) - HALF_PI) + 1) / 2.0; // What we will receive over serial, ultimately.
    positionNext = (normalisedPosition * kMaxSteps);
    commandLastTime = currentTime;
  }
  
  // TASK: Interpolate instantaneous desired position for this loop from 60fps commanded position
  float lerpPos = (float) commandMicros / (float) commandFreqMicros;
  desiredSteps = positionLast + lerpPos*(positionNext - positionLast);
  
  int deltaSteps = desiredSteps - currentSteps;
  if (deltaSteps != 0)
  {
    if (deltaSteps > 0 && !stepDirToClose)
    {
      digitalWrite(kDirPin, HIGH);
      stepDirToClose = true;
      return; // Direction change has to be 4micros ahead of step
    }
    if (deltaSteps < 0 && stepDirToClose)
    {
      digitalWrite(kDirPin, LOW);
      stepDirToClose = false;
      return; // Direction change has to be 4micros ahead of step
    }
    
    if (stepPulseMicros > kPulseMicros)
    {
      stepPulseState = !stepPulseState;
      digitalWrite(kStepPin, stepPulseState);
      currentSteps += stepDirToClose ? 1 : -1;
      stepPulseTime = micros(); // Lets not use currentTime, we might have moved on?
    }
  }
  
  if (deltaSteps != 0)  digitalWrite(kLEDPin, HIGH);
  else                  digitalWrite(kLEDPin, LOW);
  
  #ifdef SERIALDEBUG
//  Serial.print(desiredSteps);
//  Serial.print(", ");
//  Serial.print(stepDirToClose);
//  Serial.print(", ");
//  Serial.println(deltaSteps);
  #endif
}
#endif
