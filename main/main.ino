#include <Stepper.h>
#include <EncButton.h>
#include <YetAnotherPcInt.h>

#define DT_1    20
#define CLK_1   21
#define SW_1    A6
#define LOW_1   A5
#define HIGH_1  A7

#define DT_2    A15
#define CLK_2   A14
#define SW_2    A2
#define LOW_2   A3
#define HIGH_2  A4

#define azimuth_steppers_speed 5
#define peleng_steppers_speed  5
#define steps_per_revolution   2000
#define DO_STEPS 3
#define steppers_count_per_axis 3

// struct RealStepper
// {
//   RealStepper(const Stepper&& stpr, int bl) 
//   : stepper(stpr), backlash(bl)
//   {
//   }
  
//   Stepper stepper;
//   int backlash;
// };

template <int steppers_count, int enc_type, int pin1, int pin2, int pin3>
struct Axis 
{
  int target_steps = 0;
  EncButton <enc_type, pin1, pin2, pin3> encoder;
  int pcint_pins[2];
  Stepper steppers[3];
  int stepper_count;
  Axis(const Stepper (&s)[steppers_count]){
    steppers = s;
    stepper_count = steppers_count;
  }
};


Axis<2, EB_TICK, CLK_1, DT_1, SW_1> azimuth ({
  Stepper(steps_per_revolution, 11,  10, 8,  9),
  Stepper(steps_per_revolution, 22, 26, 24, 28)
  });
Axis<3, EB_TICK, CLK_2, DT_2, SW_2> peleng ({
  Stepper(steps_per_revolution, 38, 39, 40, 41),
  Stepper(steps_per_revolution, 42, 43, 44, 45),
  Stepper(steps_per_revolution, 46, 47, 48, 49)
  });

template <int steppers_count, int enc_type, int pin1, int pin2, int pin3>
void step_steppers(Axis <steppers_count, enc_type, pin1, pin2, pin3> & axis, int axis_index)
{
  for (int i = 0; i < steppers_count_per_axis; i++)
  {
    if ((axis.target_steps > 0) | true)
      steppers[axis_index][i].step(-DO_STEPS);
    else if (axis.target_steps < 0)
      steppers[axis_index][i].step(DO_STEPS);
  }

  if (axis.target_steps > 0)
    axis.target_steps--;
  else if (axis.target_steps < 0)
    axis.target_steps++;

  // Serial.println(axis.target_steps);
}

template <int steppers_count, int enc_type, int pin1, int pin2, int pin3>
void tick_encoder(Axis <steppers_count, enc_type, pin1, pin2, pin3> & axis)
{
  axis.encoder.tick();
  int increment = axis.encoder.isPress()? 600 : 1;
  if (axis.encoder.isLeft())
  {
    axis.target_steps += increment;
    Serial.println("isLeft");
  }
  else if (axis.encoder.isRight())
  {
    axis.target_steps += -increment;
    Serial.println("isRight");
  }
}



void setup() 
{
  Serial.begin(9600);
  Serial.println(azimuth.steppers_count)
  for (int i = 0; i < steppers_count_per_axis; i++)
  {
    steppers[0][i].setSpeed(azimuth_steppers_speed);
    steppers[1][i].setSpeed(peleng_steppers_speed);
  }

  azimuth.encoder.counter = 0;
  peleng.encoder.counter = 0;

  azimuth.pcint_pins[0] = DT_1;
  azimuth.pcint_pins[1] = CLK_1;
  peleng.pcint_pins[0] = DT_2;
  peleng.pcint_pins[1] = CLK_2;

  pinMode(HIGH_1, OUTPUT);
  pinMode(LOW_1, OUTPUT);
  digitalWrite(HIGH_1, HIGH);
  digitalWrite(LOW_1, LOW);

  // pinMode(azimuth.pcint_pins[0], INPUT_PULLUP);
  // pinMode(azimuth.pcint_pins[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[0]), isr_azimuth, CHANGE);
  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[1]), isr_azimuth, CHANGE);
  

  pinMode(LOW_2, OUTPUT);
  pinMode(HIGH_2, OUTPUT);
  digitalWrite(HIGH_2, HIGH);
  digitalWrite(LOW_2, LOW);

  pinMode(peleng.pcint_pins[0], INPUT_PULLUP);
  pinMode(peleng.pcint_pins[1], INPUT_PULLUP);
  PcInt::attachInterrupt(peleng.pcint_pins[0], isr_peleng, &peleng.pcint_pins[0], RISING);
  PcInt::attachInterrupt(peleng.pcint_pins[1], isr_peleng, &peleng.pcint_pins[1], RISING);
}

void loop()
{
  delay(1);

  // TODO: этот тик не разрешает нам проверят нажата кнопка или нет 
  peleng.encoder.tick();
  azimuth.encoder.tick();

  step_steppers<2, EB_TICK, CLK_1, DT_1, SW_1>(azimuth, 0);
  step_steppers<3, EB_TICK, CLK_2, DT_2, SW_2>(peleng, 1);
}

// FIXME: пофиксить варнинги: убрать аргументы в самом конце, если не используется "софтверный" pcint интерапт
void isr_azimuth(int* pin, bool pinstate)
{
  tick_encoder<EB_TICK, CLK_1, DT_1, SW_1>(azimuth);
};

void isr_peleng(int* pin, bool pinstate)
{
  tick_encoder<EB_TICK, CLK_2, DT_2, SW_2>(peleng);
};
