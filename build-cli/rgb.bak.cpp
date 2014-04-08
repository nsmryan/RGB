#include <Arduino.h>
#include "TimerOne.h"
#include <rgb.h>

#define periodHZ 100
#define dutyCycleResolution 100
#define usecInSec (1000 * 1000)
#define periodInUSec (usecInSec / (periodHZ * dutyCycleResolution))
#define STACK_DEPTH 8


void RGBIsr(void);
void interpret();

volatile int16 newDutyCycle = 0;
volatile RGBControl rgb = {false, 0, 0, true};
volatile RGBLed led = {false, 12, 0};

uint16 instrPtr = 0;
uint16 retPtr = 0;
uint16 paramPtr = 0;
uint16 indicatorLEDOn = true;

extern prog_uint8_t prog[1000] PROGMEM;

uint16 retStack[STACK_DEPTH];
uint16 paramStack[STACK_DEPTH];

//todo- compile time control stack to eliminate loop ptr on ret stack
//tos in var to reduce ++ and --
//generalize to multiple leds.

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(led.pin, OUTPUT);
   
  Timer1.initialize(periodHZ);
  Timer1.attachInterrupt(RGBIsr);
}

void loop()
{
  if (rgb.sem)
  {
    rgb.sem = false;
    rgb.countPeriods++;

    if (rgb.countPeriods % 100 == 0)
    {
      digitalWrite(13, indicatorLEDOn);
      indicatorLEDOn ^= 1;
    }

    if (rgb.interpreting)
    {
      interpret();
    }
  }
}

void interpret()
{
  uint16 arg;
  byte opcode = pgm_read_byte(&prog[instrPtr]);

  switch (opcode)
  {
    case DONE_OPCODE:
      rgb.interpreting = false;
      return;
      break;

    case RET_OPCODE:
      instrPtr = retStack[--retPtr];
      break;

    case LOOP_OPCODE:
      arg = retStack[--retPtr];
      if (arg == 0)
      {
        instrPtr++;
        retPtr--;
      }
      else
      {
        retStack[retPtr++] -= 1;
        instrPtr = retStack[retPtr - 2];
      }
      break;

    case FOR_OPCODE:
      arg = paramStack[--paramPtr];
      instrPtr++;
      retStack[retPtr++] = instrPtr;
      retStack[retPtr++] = arg;
      break;

    case CALL_OPCODE:
      arg = paramStack[--paramPtr];
      retStack[retPtr++] = instrPtr + 1;
      instrPtr = arg;
      break;

    case INCR_OPCODE:
      arg = paramStack[--paramPtr];
      newDutyCycle += arg;
      newDutyCycle = min(newDutyCycle, dutyCycleResolution);
      instrPtr++;
      return;
      break;

    case DEC_OPCODE:
      arg = paramStack[--paramPtr];
      newDutyCycle -= arg;
      newDutyCycle = max(newDutyCycle, 0);
      instrPtr++;
      return;
      break;

    case SET_OPCODE:
      newDutyCycle = paramStack[--paramPtr];
      instrPtr++;
      break;

    case WAIT_OPCODE:
      arg = paramStack[--paramPtr];
      if (arg == 0)
      {
        instrPtr++;
      }
      else
      {
        paramStack[paramPtr++] -= 1;
        return;
      }
      break;

    default:
      arg = pgm_read_word((prog_uint16_t*)&prog[instrPtr]);
      paramStack[paramPtr++] = arg;
      instrPtr += 2;
  }
  interpret();
}

void RGBIsr(void)
{
  rgb.ticks++;
  if (rgb.ticks >= dutyCycleResolution)
  {
    rgb.ticks = 0;
    led.ledOn = true;
    rgb.sem = true;
    led.dutyCycle = newDutyCycle;
    if (led.dutyCycle > 0)
    {
      digitalWrite(led.pin, HIGH);
    }
  }

  if (led.ledOn && rgb.ticks >= led.dutyCycle)
  {
    led.ledOn = false;
    digitalWrite(led.pin, LOW);
  }
}

