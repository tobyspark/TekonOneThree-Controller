
# define kStepPin 10
# define kDirPin  16
# define kErrorPin 15
# define kProxPin 00
# define kLEDPin  17

enum DriveState 
{ riseToKnownPosition,
  waitingInKnownPosition,
  sequenceInit,
  sequenceRun
};
  
// # define usbEcho

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
# define kMaxSteps 40000

// Triggers on rising and falling edge.
// Minimum value for driver unit: 2.5 micros
// Minimum resolution of arduino's micros() function = 4 micros
# define kPulseMicros 5
# define kLinearTravelSlowdownFactor 2

DriveState driveState = riseToKnownPosition;

uint16_t serialBytes[3];
char serialIndex = 0;

unsigned long stepPulseTime = 0;
bool          stepPulseState = false;

uint16_t positionNext = 0;
uint16_t positionLast = 0;

uint16_t desiredSteps = 0;
uint16_t currentSteps = 0;

bool stepDir = false; // Needs to be whichever causes bar to rise up.

unsigned long commandLastTime = 0;
unsigned long commandLastPeriod = 0;

void setup() {
  pinMode(kStepPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kErrorPin, INPUT);
  pinMode(kProxPin, INPUT);
  pinMode(kLEDPin, OUTPUT);
  Serial1.begin(115200);
  
  digitalWrite(kDirPin, stepDir);
  
  #ifdef usbEcho
  Serial.begin(115200);
  #endif
}

void loop() {
  unsigned long currentTime = micros();
  unsigned long stepPulseMicros = currentTime - stepPulseTime;
  unsigned long commandMicros = currentTime - commandLastTime;
    
  if (Serial1.available())
  {    
    int readChar = Serial1.read();
    if ((readChar & 0xC0) == 0x00) // vpos 2nd, 3rd byte code 00
    {
      serialBytes[++serialIndex] = readChar & 0x3F;
    }
    else if ((readChar & 0xC0) == 0x40) // sequence run vpos start code 01
    {
      if (driveState != riseToKnownPosition) driveState = sequenceRun;
      serialIndex = 0;
      serialBytes[serialIndex] = readChar & 0x3F;
    }
    else if ((readChar & 0xC0) == 0xC0) // sequence init vpos start code 11
    {
      if (driveState != riseToKnownPosition) driveState = sequenceInit;
      serialIndex = 0;
      serialBytes[serialIndex] = readChar & 0x3F;
      
      #ifdef usbEcho
      Serial.println("Bits 11: sequence init vpos start");
      #endif
    }
    
    if (serialIndex == 2)
    {
      serialIndex = 0;
      
      positionLast = positionNext;
      positionNext = (serialBytes[0] << 12) 
                     + 
                     (serialBytes[1] << 6)
                     + 
                     (serialBytes[2]);
      commandLastPeriod = currentTime - commandLastTime;
      commandLastTime = currentTime;
      commandMicros = 0;
      
      #ifdef xxxusbEcho
      Serial.print("Command--------------------");
      Serial.println(positionNext);
      #endif
    }
  }
  
  switch (driveState)
  {
    case riseToKnownPosition:
      desiredSteps = kMaxSteps; // Rise up (most of) range of slider
      if (true || digitalRead(kProxPin)) // ...until proximity sensor detects baton, half way up.
      {
        currentSteps = kMaxSteps / 2;
        desiredSteps = currentSteps;
        driveState = waitingInKnownPosition;
      }
      break;
    case waitingInKnownPosition:
      Serial1.write("K"); // Known position
      Serial1.flush();
      delay(0.01);
      break;
    case sequenceInit:
      desiredSteps = positionNext;
      if (currentSteps == desiredSteps)
      {
        Serial1.write("S"); // Start position
        Serial1.flush();
        delay(0.01);  
      }
      break;
    case sequenceRun:
      if (commandMicros > commandLastPeriod)
      {
        desiredSteps = positionNext;
      }
      else if (commandLastPeriod <= 0)
      {
        desiredSteps = currentSteps;
        #ifdef usbEcho
        Serial.print("WARNING: commandLastPeriod <= 0");
        #endif
      }
      else
      {
        float lerpPos = (float) commandMicros / (float) commandLastPeriod;
        desiredSteps = positionLast + lerpPos*int(positionNext - positionLast);   
      }
      break;
  }
    
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
  
  #ifdef xxxxusbEcho
  Serial.print(positionNext);
  Serial.print(", ");
  Serial.print(positionLast);
  Serial.print(", ");
  //Serial.print(lerpPos);
  //Serial.print(", ");
  Serial.println(desiredSteps);
  #endif
  
  #ifdef usbEcho
  Serial.print(currentSteps);
  Serial.print(", ");
  Serial.println(deltaSteps);
  #endif
  
  if (deltaSteps != 0)  digitalWrite(kLEDPin, HIGH);
  else                  digitalWrite(kLEDPin, LOW);
}
