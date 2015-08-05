/*
TEKTON ONE THREE
SINUSOIDAL MOTOR DRIVE
Toby Harris 2015
*/

// HARD WON TRUTH: DON'T DO ANY LOGGING, SERIAL ETC. WHEN DRIVE IN MOTION.

const bool frontUnit = false;

// ** 200 STEPS **
// DIPSWITCHES ON ON ON ON
# define kMaxSteps 10000

# define kStepPin 10
# define kDirPin  16
# define kProxPin 3
# define kKillPin 5
# define kLEDPin  17

# define kPulseMicros 5 // Triggers on rising and falling edge

unsigned long stepPulseTime = 0;
bool          stepPulseState = false;

int positionNext = 0;
int positionLast = 0;

int desiredSteps = 0;
int currentSteps = 0;

int lastProxSteps = -1;

// Start this dir value with whatever gives UP
bool stepDirToClose = true;

unsigned long commandLastTime = 0;
unsigned long commandFreqMicros = 1000000/60;

unsigned long startTime = 0;
uint16_t zeroStepOffset = 100;
bool firstRun = true;
bool killTriggered = false;

void setup() {
  pinMode(kStepPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kProxPin, INPUT);
  pinMode(kKillPin, INPUT);
  digitalWrite(kDirPin, true);
}

void loop() {
  // On first run, wait a few seconds to ensure everything fully powered and to allow some warning to anybody near machine
  if (firstRun) delay(frontUnit ? 5000 : 6000);
  // On first run, rise up safe margin from bottom end-stop
  while (firstRun)
  {
    stepPulseState = !stepPulseState;
    digitalWrite(kStepPin, stepPulseState);
    currentSteps += 1;
    delayMicroseconds(kPulseMicros * 500);
    if (currentSteps > zeroStepOffset)
    {
      currentSteps = 0;
      startTime = micros();
      firstRun = false;
    }
  }
 
  unsigned long currentTime = micros() - startTime;
  unsigned long stepPulseMicros = currentTime - stepPulseTime;
  unsigned long commandMicros = currentTime - commandLastTime;
  
  if (!killTriggered && digitalRead(kKillPin)) killTriggered = true;
  if (killTriggered && currentSteps == 0)
  {
    while (true)
    {
      delay(1000);
    }
  }
  
  // TASK: Correct for position drift
  if (digitalRead(kProxPin))
  {
    lastProxSteps = currentSteps;
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
    // 1.9x is max current program will go without hitting pulsing limit
    // ...but 1.9x gets error after a number of cycles when bar is fixed on 
    positionLast = positionNext;
    float normalisedPosition = (sin(((currentTime*(frontUnit ? 1.7 : 1.5)) / 1000000.0) - HALF_PI) + 1) / 2.0; // What we will receive over serial, ultimately.
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
}
