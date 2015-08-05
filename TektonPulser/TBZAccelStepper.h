#import "AccelStepper.h"

class TBZAccelStepper : public AccelStepper
{
  public:
    TBZAccelStepper(uint8_t stepPin, uint8_t dirPin) : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin, 0, 0, true) {};
  
  protected:   
    bool   stepState = false;
    bool   dirState = false;
    
    void   step(long step)
    {
      if (dirState != _direction)
      {
        dirState = _direction;
        digitalWrite(_pin[1], _direction);
        delayMicroseconds(5);
      }
      
      stepState = !stepState;
      digitalWrite(_pin[0], stepState);
    }
};
