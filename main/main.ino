#include <Stepper.h>
#include <EncButton.h>
#include <YetAnotherPcInt.h>

#define DT_1 20
#define CLK_1 21
#define SW_1 A6
#define LOW_1 A5
#define HIGH_1 A7

// CLK_2 либо 19, либо 18. Та же история с DT_2 поскольку пины могут быть перепутаны. Источник из магазина amperka
#define DT_2 A15
#define CLK_2 A14
#define SW_2 A2
#define LOW_2 A3
#define HIGH_2 A4

#define steps_per_revolution 2000
#define DO_STEPS 3

// на каждую ось расчитано только 2 степпера
#define steppers_count_per_axis 2

struct RealStepper
{
  RealStepper(const Stepper&& stpr, int bl) 
  : stepper(stpr), backlash(bl)
  {
  }
  
  Stepper stepper;
  int backlash;
};

template <int enc_type, int pin1, int pin2, int pin3>
struct Axis 
{
  int target_steps = 0;
  EncButton <enc_type, pin1, pin2, pin3> encoder;
  int pcint_pins[2];
};

// средние пины степперов должны быть поменяны иначе он будет только в одну сторону вне зависимости позитивное или негативное значение передается в .step()
// создаем степперы тут потому что они должны быть инициализированы глобально, чтобы к ним был доступ внутри лупа. Запихнуть их в Axis не получится, поскольку никак не можем
// инициализировать их пустыми конструкторами (проблема точно не изучена, потому что мы забыли в чем конкретно была проблема с Axis)
Stepper steppers[1][steppers_count_per_axis] {
  {
    Stepper(steps_per_revolution, 11,  10, 8,  9),
    Stepper(steps_per_revolution, 22, 26, 24, 28),
  }
};

Axis<EB_TICK, CLK_1, DT_1, SW_1> azimuth;
Axis<EB_TICK, CLK_2, DT_2, SW_2> peleng;

template <int enc_type, int pin1, int pin2, int pin3>
void step_steppers(Axis <enc_type, pin1, pin2, pin3> & axis, int stepper_index)
{
//  int i = 0;

  // TODO: надо исправить это все говно с индексами когда все степперы будут подключены
    
//  for (i = 0; i < steppers_count_per_axis; i++)
//  {
    if (axis.target_steps > 0)
    {
      steppers[stepper_index][0].step(-DO_STEPS);
      axis.target_steps--;
      Serial.println(axis.target_steps);
    }
    else if (axis.target_steps < 0)
    {
      steppers[stepper_index][0].step(DO_STEPS);
      axis.target_steps++;
      Serial.println(axis.target_steps);
    }
//  }
}

template <int enc_type, int pin1, int pin2, int pin3>
void tick_encoder(Axis <enc_type, pin1, pin2, pin3> & axis)
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
  
  // Для разных Axis разная скорость поворотов степперов
  // TODO: эту хуйню тоже надо исправить когда все степперы подключены
  steppers[0][0].setSpeed(5);
  //steppers[0][1].setSpeed(30)

  // TODO: надо проверить зачем .counter
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
  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[0]), isr_azimuth, RISING);
  attachInterrupt(digitalPinToInterrupt(azimuth.pcint_pins[1]), isr_azimuth, RISING);
  

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
  peleng.encoder.tick(); // TODO: этот тик не разрешает нам проверят нажата кнопка или нет 

  step_steppers<EB_TICK, CLK_1, DT_1, SW_1>(azimuth, 0);
  step_steppers<EB_TICK, CLK_2, DT_2, SW_2>(peleng, 0);
}

void isr_azimuth(int* pin, bool pinstate)
{
  tick_encoder<EB_TICK, CLK_1, DT_1, SW_1>(azimuth);
};

void isr_peleng(int* pin, bool pinstate)
{
  tick_encoder<EB_TICK, CLK_2, DT_2, SW_2>(peleng);
};
