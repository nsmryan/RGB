#include "TimerOne.h"
#include <rgb.h>

#define periodHZ 100
#define dutyCycleResolution 100
#define usecInSec (1000 * 1000)
#define periodInUSec (usecInSec / (periodHZ * dutyCycleResolution))
#define NUM_LEDS 15
#define USER_INPUT 0
#define SHIFT_REGISTER 0

#define OE 9
#define SRCLK 10
#define RCLK 11
#define SERPIN 12 


void RGBIsr(void);
void interpret();
void setPins();

volatile RGBControl rgb = {false, 0, 0, true, 0, 0};
volatile RGBLed leds[NUM_LEDS];
ProgState state;

uint16 indicatorLEDOn = true;

extern prog_uint8_t prog[1000] PROGMEM;

//todo- compile time control stack to eliminate loop ptr on ret stack
//tos in var to reduce ++ and --
//stack manipulation doesn't work yet.
//write parser or monad instance (look at operational monad)
//multiple modes

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(OE, OUTPUT);
  pinMode(SERPIN, OUTPUT);
  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);

  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].ledOn = false;
    leds[i].dutyCycle = 0;
    leds[i].newDutyCycle = 0;
  }
   
#ifdef SHIFT_REGISTER
  Timer1.initialize(periodHZ);
  Timer1.attachInterrupt(RGBIsr);
#endif

  rgb.minCycle = dutyCycleResolution;
  rgb.maxCycle = 0;

  Serial.begin(9600);
}

#if USER_INPUT
void serialEvent()
{
  int byte = Serial.read();

  switch (byte)
  {
    case '1':
      leds[0].newDutyCycle += 10;
      break;

    case '2':
      leds[1].newDutyCycle += 10;
      break;

    case '3':
      leds[2].newDutyCycle += 10;
      break;

    case 'q':
      leds[0].newDutyCycle += 1;
      break;

    case 'w':
      leds[1].newDutyCycle += 1;
      break;

    case 'e':
      leds[2].newDutyCycle += 1;
      break;

    case 'a':
      leds[0].newDutyCycle -= 1;
      break;

    case 's':
      leds[1].newDutyCycle -= 1;
      break;

    case 'd':
      leds[2].newDutyCycle -= 1;
      break;

    case 'z':
      leds[0].newDutyCycle -= 10;
      break;

    case 'x':
      leds[1].newDutyCycle -= 10;
      break;

    case 'c':
      leds[2].newDutyCycle -= 10;
      break;

    case '8':
      leds[0].newDutyCycle = 100;
      break;

    case '9':
      leds[1].newDutyCycle = 100;
      break;

    case '0':
      leds[2].newDutyCycle = 100;
      break;

    case 'i':
      leds[0].newDutyCycle = 0;
      break;

    case 'o':
      leds[1].newDutyCycle = 0;
      break;

    case 'p':
      leds[2].newDutyCycle = 0;
      break;

    default:
      break;
  }
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].newDutyCycle = max(leds[i].newDutyCycle, 0);
    leds[i].newDutyCycle = min(leds[i].newDutyCycle, 100);
    Serial.print(leds[i].newDutyCycle, DEC);
    Serial.print(" ");
  }
  Serial.println();
}
#endif

void loop()
{
#if USER_INPUT
  return;
#elif SHIFT_REGISTER
  leds[0].ledOn = 1;
  leds[1].ledOn = 0;
  leds[2].ledOn = 0;
  setPins();
#else
  if (rgb.sem)
  {
    rgb.sem = false;
    rgb.countPeriods++;
    rgb.minCycle = dutyCycleResolution;
    rgb.maxCycle = 0;

    if (rgb.countPeriods % 100 == 0)
    {
      digitalWrite(13, indicatorLEDOn);
      indicatorLEDOn ^= 1;
    }

    if (rgb.interpreting)
    {
      interpret();
      //for (int i = 0; i < NUM_LEDS; i++)
      //{
      //  rgb.newMaxCycle = max(leds[i].newDutyCycle, rgb.newMaxCycle);
      //  rgb.newMinCycle = min(leds[i].newDutyCycle, rgb.newMinCycle);
      //}
    }
  }
#endif
}

void interpret()
{
  uint16 arg, arg2;
  byte opcode = pgm_read_byte(&prog[state.instrPtr]);

  switch (opcode)
  {
    case DONE_OPCODE:
      rgb.interpreting = false;
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i].newDutyCycle = 0;
      }
      return;
      break;

    case RET_OPCODE:
      state.retPtr--;
      state.instrPtr = state.retStack[state.retPtr];
      break;

    case LOOP_OPCODE:
      state.retPtr--;
      arg = state.retStack[state.retPtr];
      if (arg == 0)
      {
        state.instrPtr++;
        state.retPtr--;
      }
      else
      {
        state.retStack[state.retPtr]--;
        state.retPtr++;
        state.instrPtr = state.retStack[state.retPtr - 2];
      }
      break;

    case FOR_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.instrPtr++;
      state.retStack[state.retPtr] = state.instrPtr;
      state.retPtr++;
      state.retStack[state.retPtr] = arg;
      state.retPtr++;
      break;

    case CALL_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.retStack[state.retPtr] = state.instrPtr + 1;
      state.retPtr++;
      state.instrPtr = arg;
      break;

    case INCR_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramPtr--;
      arg2 = state.paramStack[state.paramPtr];
      leds[arg2].newDutyCycle += arg;
      leds[arg2].newDutyCycle = min(leds[arg2].newDutyCycle, dutyCycleResolution);
      state.instrPtr++;
      break;

    case DEC_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramPtr--;
      arg2 = state.paramStack[state.paramPtr];
      leds[arg2].newDutyCycle -= arg;
      leds[arg2].newDutyCycle = max(leds[arg2].newDutyCycle, 0);
      state.instrPtr++;
      break;

    case SET_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramPtr--;
      arg2 = state.paramStack[state.paramPtr];
      leds[arg2].newDutyCycle = arg;
      state.instrPtr++;
      break;

    case WAIT_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      if (arg == 0)
      {
        state.instrPtr++;
      }
      else
      {
        state.paramStack[state.paramPtr] -= 1;
        state.paramPtr++;
        return;
      }
      break;

    case DUP_OPCODE:
      arg = state.paramStack[state.paramPtr-1];
      state.paramStack[state.paramPtr] = arg;
      state.paramPtr++;
      state.instrPtr++;
      break;

    case DROP_OPCODE:
      state.paramPtr--;
      state.instrPtr++;
      break;

    case SWAP_OPCODE:
      arg = state.paramStack[state.paramPtr-1];
      state.paramStack[state.paramPtr-1] = state.paramStack[state.paramPtr-2];
      state.paramStack[state.paramPtr-2] = arg;
      state.instrPtr++;
      break;

    case YIELD_OPCODE:
      return;
      break;

    case TOA_OPCODE:
      arg = state.paramStack[state.paramPtr-1];
      state.regA = arg;
      state.instrPtr++;
      break;

    case FROMA_OPCODE:
      arg = state.regA;
      state.paramStack[state.paramPtr] = arg;
      state.paramPtr++;
      state.instrPtr++;
      break;

    case ADD_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr-1];
      state.paramStack[state.paramPtr-1] = arg + arg2;
      state.instrPtr++;
      break;

    case SUB_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr-1];
      state.paramStack[state.paramPtr-1] = arg2 - arg;
      state.instrPtr++;
      break;

    case MULT_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr - 1];
      state.paramStack[state.paramPtr - 1] = arg * arg2;
      state.instrPtr++;
      break;

    case MOD_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr - 1];
      state.paramStack[state.paramPtr - 1] = arg2 % arg;
      state.instrPtr++;
      break;

    case NIP_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramStack[state.paramPtr - 1] = arg;
      state.instrPtr++;
      break;

    case IF_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramPtr--;
      arg2 = state.paramStack[state.paramPtr];
      if (arg2)
      {
        state.retStack[state.retPtr] = state.instrPtr + 1;
        state.retPtr++;
        state.instrPtr = arg;
      }
      else
      {
        state.instrPtr++;
      }
      break;

    case GT_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr - 1];
      state.paramStack[state.paramPtr - 1] = arg2 > arg;
      state.instrPtr++;
      break;

    case LT_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr - 1];
      state.paramStack[state.paramPtr - 1] = arg2 < arg;
      state.instrPtr++;
      break;

    case EQ_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      arg2 = state.paramStack[state.paramPtr - 1];
      state.paramStack[state.paramPtr - 1] = arg2 == arg;
      state.instrPtr++;
      break;

    case IFELSE_OPCODE:
      state.paramPtr--;
      arg = state.paramStack[state.paramPtr];
      state.paramPtr--;
      arg2 = state.paramStack[state.paramPtr];
      state.paramPtr--;
      if (state.paramStack[state.paramPtr])
      {
        state.retStack[state.retPtr] = state.instrPtr + 1;
        state.retPtr++;
        state.instrPtr = arg;
      }
      else
      {
        state.retStack[state.retPtr] = state.instrPtr + 1;
        state.retPtr++;
        state.instrPtr = arg;
      }
      break;

    default:
      arg = pgm_read_word((prog_uint16_t*)&prog[state.instrPtr]);
      state.paramStack[state.paramPtr] = arg;
      state.paramPtr++;
      state.instrPtr += 2;
  }
  interpret();
}

void setPins()
{
  int i;

  digitalWrite(RCLK, LOW);
  digitalWrite(OE, HIGH);

  for (i = 0; i < NUM_LEDS; i++)
  {
    digitalWrite(SRCLK, LOW);

    digitalWrite(SERPIN, leds[i].ledOn);

    digitalWrite(SRCLK, HIGH);
  }
  digitalWrite(OE, LOW);
  digitalWrite(RCLK, HIGH);
}

void RGBIsr(void)
{
  int i;
  int changed = false;

  if (rgb.ticks >= dutyCycleResolution)
  {
    rgb.ticks = 0;
    rgb.sem = true;
    rgb.maxCycle = rgb.newMaxCycle;
    rgb.minCycle = rgb.newMinCycle;

    for (i = 0; i < NUM_LEDS; i++)
    {
      if (leds[i].newDutyCycle > 0)
      {
        leds[i].ledOn = true;
      }
      leds[i].dutyCycle = leds[i].newDutyCycle;
    }
    setPins();
  }
/*
  if (rgb.ticks < rgb.minCycle || rgb.ticks > rgb.maxCycle)
  {
    rgb.ticks++;
    return;
  }
*/
  for (i = 0; i < NUM_LEDS; i++)
  {
    if (leds[i].ledOn && rgb.ticks >= leds[i].dutyCycle)
    {
      leds[i].ledOn = false;
      changed = true;
    }
  }
  if (changed)
  {
    setPins();
  }
  rgb.ticks++;
}

