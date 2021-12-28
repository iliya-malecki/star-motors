#include <Stepper.h>
#include <EncButton.h>
#include <YetAnotherPcInt.h>

//#define steppers_speed 32
#define steps_per_revolution 200
#define DO_STEPS 3

struct RealStepper {
  Stepper stepper;
  int backlash;

  RealStepper(const Stepper&& stpr, int bl) 
  : stepper(stpr), backlash(bl)
  {
  } 
};

// const int stepper_count = 2;
// const int enc_type = 1;
// const int pin1 = 2;
// const int pin2 = 3;
// const int pin3 = 4;
template <int stepper_count, int enc_type, int pin1, int pin2, int pin3, int low, int high>
struct Axis {
  int target_steps = 0;
  EncButton <enc_type, pin1, pin2, pin3> encoder;
  int pcint_pins[2] = {pin1, pin2};
  int enc_high = high;
  int enc_low = low;
  bool is_moving_forward = true;
  RealStepper real_steppers[stepper_count]; 
  Axis(RealStepper (&&s)[stepper_count]) : real_steppers(s)
  {};

  void step_steppers() {
    // a tad ugly, but skipping 2n if-statements or so compared to checking the same thing in the loop
    // we have n=5 (steppers total), so im happy with it

    bool gotta_compensate = false;
    if ((target_steps > 0) != is_moving_forward)
    {
      is_moving_forward = !is_moving_forward;
      gotta_compensate = true;
    }

    if (target_steps > 0)
    {
      for (RealStepper& real_stepper : real_steppers)
      {
        real_stepper.stepper.step(
          -(DO_STEPS + (gotta_compensate ? real_stepper.backlash : 0)));
      }
      target_steps--;
    }

    else if (target_steps < 0)
    {
      for (RealStepper& real_stepper : real_steppers)
      {
        real_stepper.stepper.step(
            DO_STEPS + (gotta_compensate ? real_stepper.backlash : 0));
      }
      target_steps++;
    }

  };

  void tick_encoder() {
    encoder.tick();
    int increment = encoder.hold()? 5 : 1; //TODO: check if this function works as expected
    if (encoder.isLeft())
    {
      target_steps += increment;
      Serial.println("isLeft");
    }
    else if (encoder.isRight())
    {
      target_steps += -increment;
      Serial.println("isRight");
    }
  };

  void set_speed(int speed) {
    for (RealStepper& real_stepper : real_steppers){
      real_stepper.stepper.setSpeed(speed);
    }
  };
};

Axis<2, EB_TICK, 21, 20, A6, A5, A7> azimuth ({

  RealStepper(
    Stepper(steps_per_revolution, 9, 10, 8, 11),
    0),

  RealStepper(
    Stepper(steps_per_revolution, 50, 52, 51, 53),
    0)

  });



Axis<3, EB_TICK, A14, A15, A2, A3, A4> peleng ({

  RealStepper(
    Stepper(steps_per_revolution, 38, 40, 39, 41),
    0),

  RealStepper(
    Stepper(steps_per_revolution, 42, 44, 43, 45),
    0),

  RealStepper(
    Stepper(steps_per_revolution, 46, 48, 47, 49),
    0)

  });


void setup() 
{
  Serial.begin(9600);
  azimuth.set_speed(21);
  peleng.set_speed(32);

  azimuth.encoder.counter = 0;
  peleng.encoder.counter = 0;


  pinMode(azimuth.enc_high, OUTPUT);
  pinMode(azimuth.enc_low, OUTPUT);
  digitalWrite(azimuth.enc_high, HIGH);
  digitalWrite(azimuth.enc_low, LOW);

  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[0]), isr_azimuth, CHANGE);
  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[1]), isr_azimuth, CHANGE);
  
  pinMode(peleng.enc_high, OUTPUT);
  pinMode(peleng.enc_low, OUTPUT);
  digitalWrite(peleng.enc_high, HIGH);
  digitalWrite(peleng.enc_low, LOW);

  pinMode(peleng.pcint_pins[0], INPUT_PULLUP);
  pinMode(peleng.pcint_pins[1], INPUT_PULLUP);
  PcInt::attachInterrupt(peleng.pcint_pins[0], isr_peleng, &peleng.pcint_pins[0], CHANGE);
  PcInt::attachInterrupt(peleng.pcint_pins[1], isr_peleng, &peleng.pcint_pins[1], CHANGE);
}

void loop()
{
  delay(1); //TODO: check if it does anything useful

  peleng.encoder.tick();
  azimuth.encoder.tick();

  azimuth.step_steppers();
  peleng.step_steppers();
}

//FIXME: пофиксить варнинги: убрать аргументы в самом конце, если не используется "софтверный" pcint интерапт
void isr_azimuth(int* pin, bool pinstate)
{
  azimuth.tick_encoder();
};

void isr_peleng(int* pin, bool pinstate)
{
  peleng.tick_encoder();
};
