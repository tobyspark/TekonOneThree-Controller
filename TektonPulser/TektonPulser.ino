
# define kStepPin 10
# define kDirPin  16
# define kErrorPin 15
# define kProxPin 00
# define kLEDPin  17

//# define usbEcho

// r = 0.0239
// travel = 2
// rev_distance = 2 * pi * r => 0.1502
// max_travel_turns = travel / rev_distance => 13.3184

// One physical step = 1.8deg => 200 steps => 0.8mm travel
// 4x microsteps recommended as minimum => 800 steps

// steps = 800
// max_travel_steps = steps * max_travel_turns => 10,654.7242
// step_distance = 2 * pi * r / steps => 0.0002

// steps = 1600
// max_travel_steps = steps * max_travel_turns => 21,309.4484
// step_distance = 2 * pi * r / steps => 9.3855e-5

// steps = 6400
// max_travel_steps = steps * max_travel_turns => 85,237.7938
// step_distance = 2 * pi * r / steps => 2.3464e-5

// At 800 steps 20000 gives ~2m travel, and this works if with a little vibration
// At 1600 steps 20000 should give 2m travel, but it gives silky smooth ~1m travel
# define kMaxSteps 20000

// Triggers on rising and falling edge.
// Minimum value for driver unit: 2.5 micros
// Minimum resolution of arduino's micros() function = 4 micros
# define kPulseMicros 5
# define kLinearTravelSlowdownFactor 2

bool validSerialDetected = false;

unsigned long stepPulseTime = 0;
bool          stepPulseState = false;

int positionNext = 0;
int positionLast = 0;

int desiredSteps = 0;
int currentSteps = 0;

bool stepDir = false; // Needs to be whichever causes bar to rise up.

unsigned long commandLastTime = 0;
unsigned long commandLastPeriod = 1;

void setup() {
  pinMode(kStepPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kErrorPin, INPUT);
  pinMode(kProxPin, INPUT);
  pinMode(kLEDPin, OUTPUT);
  Serial1.begin(115200);
  
  #ifdef usbEcho
  Serial.begin(9600);
  #endif
  
  // rise to position 0, where proximity sensor will detect
  digitalWrite(kDirPin, stepDir);
//  while (!digitalRead(kProxPin))
//  {
//    digitalWrite(kStepPin, HIGH);   
//    delayMicroseconds(kPulseMicros*kLinearTravelSlowdownFactor);
//    digitalWrite(kStepPin, LOW);
//    delayMicroseconds(kPulseMicros*kLinearTravelSlowdownFactor);
//  }

  while (!validSerialDetected)
  {
    if (Serial1.read() == 'T') validSerialDetected = true;
  } 
}

void loop() {
  unsigned long currentTime = micros();
  unsigned long stepPulseMicros = currentTime - stepPulseTime;
  unsigned long commandMicros = currentTime - commandLastTime;
  
  # define kCommandPositionMax 255
  if (Serial1.available())
  {
    positionLast = positionNext;
    //int commandedPosition = Serial1.read();
    //positionNext = (commandedPosition * kMaxSteps / kCommandPositionMax) - kMaxSteps/2;
    float commandedPosition = (float)Serial1.read()/(float)kCommandPositionMax;
    positionNext = commandedPosition * kMaxSteps - kMaxSteps/2;
    commandLastPeriod = currentTime - commandLastTime;
    commandLastTime = currentTime;
    commandMicros = 0;
    
    #ifdef usbEcho
    Serial.print("Command--------------------");
    Serial.println(commandedPosition);
    #endif
  }
  
  float lerpPos = (float) commandMicros / (float) commandLastPeriod;
  desiredSteps = positionLast + lerpPos*(positionNext - positionLast);
  
    #ifdef usbEcho
    Serial.print(positionNext);
    Serial.print(", ");
    Serial.print(positionLast);
    Serial.print(", ");
    Serial.print(lerpPos);
    Serial.print(", ");
    Serial.println(desiredSteps);
    #endif
  
  int deltaSteps = desiredSteps - currentSteps;
  if (deltaSteps != 0)
  {
    if (deltaSteps > 0 && !stepDir)
    {
      digitalWrite(kDirPin, HIGH);
      stepDir = true;
      return; // Direction change has to be 4micros ahead of step
    }
    if (deltaSteps < 0 && stepDir)
    {
      digitalWrite(kDirPin, LOW);
      stepDir = false;
      return; // Direction change has to be 4micros ahead of step
    }
    
    if (stepPulseMicros > kPulseMicros)
    {
      stepPulseState = !stepPulseState;
      digitalWrite(kStepPin, stepPulseState);
      currentSteps += stepDir ? 1 : -1;
      stepPulseTime = micros(); // Lets not use currentTime, we might have moved on?
    }
  }
  
  if (deltaSteps != 0)  digitalWrite(kLEDPin, HIGH);
  else                  digitalWrite(kLEDPin, LOW);
}
