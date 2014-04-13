#include <Arduino.h>
#include "TimerOne.h"
#include <rgb.h>


#define PRINT_RUN_TIME 0
#define PRINT_STARTUP 1


void Isr(void);
bool inner();
void setPins();
void printFiber(int fiberNum);
void printFibers();
int availableMemory();

Fiber fibers[NUM_FIBERS];
FiberRegisters currentFiber;
Scheduler scheduler;
Output outs[NUM_FIBERS];
uint8 masks[9] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};
uint8 currentOutputs[NUM_SHIFT_REGS];

uint16 indicatorLEDOn = true;

extern prog_uint16_t prog_segments[NUM_FIBERS][PROG_SIZE] PROGMEM;
extern prog_uint16_t fiberOffset[NUM_FIBERS] PROGMEM;
extern prog_uint8_t fiberBitsUsed[NUM_FIBERS] PROGMEM;

//TODO
//macros for opcode defs
//user input from hardware
//eeprom configuration
//compilation on host

void setup()
{
  int fiberIndex;

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(MR_PIN_NUM, OUTPUT);
  digitalWrite(MR_PIN_NUM, LOW);
  digitalWrite(MR_PIN_NUM, HIGH);

  pinMode(OE_PIN_NUM, OUTPUT);
  digitalWrite(OE_PIN_NUM, HIGH);

  pinMode(SEROUT_PIN_NUM, OUTPUT);
  digitalWrite(SEROUT_PIN_NUM, LOW);

  pinMode(SRCLK_PIN_NUM, OUTPUT);
  digitalWrite(SRCLK_PIN_NUM, LOW);

  pinMode(STORCLK_PIN_NUM, OUTPUT);
  digitalWrite(STORCLK_PIN_NUM, LOW);

  Serial.begin(9600);

  //TODO change to load from EEEPROM

  for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
  {
    fibers[fiberIndex].registers.instrPtr = (prog_uint16_t*)pgm_read_word(&fiberOffset[fiberIndex]);
    fibers[fiberIndex].registers.retPtr = &fibers[fiberIndex].state.retStack[0];
    fibers[fiberIndex].registers.paramPtr = &fibers[fiberIndex].state.paramStack[0];
    fibers[fiberIndex].registers.regA = 0;
    fibers[fiberIndex].registers.regB = 0;
    fibers[fiberIndex].registers.output = 0;
    fibers[fiberIndex].registers.pwm = 0;

    outs[fiberIndex].pwm = 0;
    outs[fiberIndex].output = 0;
  }

  Timer1.initialize(PERIOD_US);
  Timer1.attachInterrupt(Isr);

  scheduler.done = true;
}

void serialEvent()
{
  int byte = Serial.read();

  if (byte == -1)
    return;

  switch (byte)
  {
    case 'p':
      printFibers();
      break;
    
    case 'm':
      Serial.print("availableMemory = ");
      Serial.println(availableMemory());
      break;

    case 't':
      Serial.print("uptime = ");
      Serial.print(" seconds = ");
      Serial.print(scheduler.frames);
      Serial.print(", ms = ");
      Serial.print(scheduler.subframes * DUTY_CYCLE_RESOLUTION);
      Serial.print(", us = ");
      Serial.print(scheduler.ticks);
      Serial.println();
      break;

    default:
      Serial.print("unexpected input = ");
      Serial.println((char*)&byte);
      break;
  }
}

void loop()
{
  int fiberNum;
  int i;

  if (scheduler.sem)
  {
    scheduler.done = false;
    scheduler.sem = false;

    if (scheduler.missedTiming)
    {
      digitalWrite(13, HIGH);
    }
    else
    {
      digitalWrite(13, (millis() / 1000) % 2 == 0);
    }

    Serial.println("outputs");
    for (fiberNum = 0; fiberNum < NUM_FIBERS; fiberNum++)
    {
      scheduler.fiberIndex = fiberNum;

      memcpy(&currentFiber,
             &fibers[fiberNum].registers,
             sizeof(FiberRegisters));

      while (inner());

      memcpy(&fibers[fiberNum].registers,
             &currentFiber,
             sizeof(FiberRegisters));
      //printFiber(fiberNum);
      //Serial.println();
      //Serial.print(outs[fiberNum].output);
    }
    //Serial.println();

    //Serial.println("Current Output");
    //for (i = 0; i < 2; i++)
    //{
    //  Serial.print((unsigned int)currentOutputs[2 - i - 1], HEX);
    //}
    //Serial.println("");

    scheduler.done = true;


#if PRINT_RUN_TIME
    Serial.println(Timer1.read());
#endif
  }
  //TODO consider entering low power state until
  //interrupt wakes us up.
}

bool inner()
{
  uint16 arg, arg2;
  uint16 opcode = pgm_read_word(currentFiber.instrPtr);

  switch (opcode)
  {
    case DONE_OPCODE:
      return false;
      break;

    case RET_OPCODE:
      currentFiber.retPtr--;
      currentFiber.instrPtr = *currentFiber.retPtr;
      break;

    case LOOP_OPCODE:
      arg = (uint16)*(currentFiber.retPtr - 1);
      if (arg == 0)
      {
        currentFiber.instrPtr++;
        currentFiber.retPtr -= 2;
      }
      else
      {
        *(currentFiber.retPtr - 1) = (prog_uint16_t*)(arg-1);
        currentFiber.instrPtr = (prog_uint16_t*)*(currentFiber.retPtr - 2);
      }
      break;

    case FOR_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      currentFiber.instrPtr++;
      *currentFiber.retPtr = currentFiber.instrPtr;
      currentFiber.retPtr++;
      *currentFiber.retPtr = (prog_uint16_t*)arg;
      currentFiber.retPtr++;
      break;

    case CALL_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      *currentFiber.retPtr = currentFiber.instrPtr + 1;
      currentFiber.retPtr++;
      currentFiber.instrPtr = (prog_uint16_t*)arg;
      break;

    case INCR_OPCODE:
      arg = *(currentFiber.paramPtr-1);
      *(currentFiber.paramPtr-1) = arg+1;
      currentFiber.instrPtr++;
      break;

    case DEC_OPCODE:
      arg = *(currentFiber.paramPtr-1);
      *(currentFiber.paramPtr-1) = arg-1;
      currentFiber.instrPtr++;
      break;

    case OUTPUT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      currentFiber.output = arg;
      currentFiber.instrPtr++;
      break;

    case WAIT_OPCODE:
      arg = *(currentFiber.paramPtr-1);
      if (arg == 0)
      {
        currentFiber.paramPtr--;
        currentFiber.instrPtr++;
      }
      else
      {
        *(currentFiber.paramPtr-1) = arg-1;
      }
      return false;
      break;

    case DUP_OPCODE:
      arg = *(currentFiber.paramPtr - 1);
      *currentFiber.paramPtr = arg;
      currentFiber.paramPtr++;
      currentFiber.instrPtr++;
      break;

    case DROP_OPCODE:
      currentFiber.paramPtr--;
      currentFiber.instrPtr++;
      break;

    case SWAP_OPCODE:
      arg = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = *(currentFiber.paramPtr - 2);
      *(currentFiber.paramPtr - 2) = arg;
      currentFiber.instrPtr++;
      break;

    case YIELD_OPCODE:
      currentFiber.instrPtr++;
      return false;
      break;

    case TOA_OPCODE:
      arg = *(--currentFiber.paramPtr);
      currentFiber.regA = arg;
      currentFiber.instrPtr++;
      break;

    case FROMA_OPCODE:
      arg = currentFiber.regA;
      *currentFiber.paramPtr = arg;
      currentFiber.paramPtr++;
      currentFiber.instrPtr++;
      break;

    case ADD_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr--);
      *currentFiber.paramPtr = arg + arg2;
      currentFiber.instrPtr++;
      break;

    case SUB_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 - arg;
      currentFiber.instrPtr++;
      break;

    case MULT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg * arg2;
      currentFiber.instrPtr++;
      break;

    case MOD_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 % arg;
      currentFiber.instrPtr++;
      break;

    case NIP_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      *(currentFiber.paramPtr - 1) = arg;
      currentFiber.instrPtr++;
      break;

    case IF_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      currentFiber.paramPtr--;
      arg2 = *currentFiber.paramPtr;
      if (arg2)
      {
        currentFiber.instrPtr++;
      }
      else
      {
        currentFiber.instrPtr += arg;
      }
      break;

    case GT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 > arg;
      currentFiber.instrPtr++;
      break;

    case LT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 < arg;
      currentFiber.instrPtr++;
      break;

    case EQ_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 == arg;
      currentFiber.instrPtr++;
      break;

    case EVER_OPCODE:
      currentFiber.instrPtr = (prog_uint16_t*)*(currentFiber.retPtr - 2);
      break;

    case TOB_OPCODE:
      arg = *(--currentFiber.paramPtr);
      currentFiber.regB = arg;
      currentFiber.instrPtr++;
      break;

    case FROMB_OPCODE:
      *(currentFiber.paramPtr++) = currentFiber.regB;
      currentFiber.instrPtr++;
      break;

    case THEN_OPCODE:
      currentFiber.instrPtr++;
      break;

    case SET_PWM_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      currentFiber.pwm = arg;
      currentFiber.instrPtr++;
      break;

    case LSHIFT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 << arg;
      currentFiber.instrPtr++;
      break;

    case RSHIFT_OPCODE:
      currentFiber.paramPtr--;
      arg = *currentFiber.paramPtr;
      arg2 = *(currentFiber.paramPtr - 1);
      *(currentFiber.paramPtr - 1) = arg2 >> arg;
      currentFiber.instrPtr++;
      break;

    default:
      *currentFiber.paramPtr = opcode;
      currentFiber.paramPtr++;
      currentFiber.instrPtr++;
  }

  return true;
}

void printFiber(int fiberNum)
{
  int stackIndex;

  Serial.println();
  Serial.print(fiberNum);
  Serial.println(":");
  Serial.print("ps = ");
  Serial.print((uint16)fibers[fiberNum].registers.paramPtr);
  Serial.print(" rs = ");
  Serial.print((uint16)fibers[fiberNum].registers.retPtr);
  Serial.print(" ip = ");
  Serial.print((uint16)fibers[fiberNum].registers.instrPtr);
  Serial.print(" output = ");
  Serial.println((uint16)fibers[fiberNum].registers.output);

  for (stackIndex = 0; stackIndex < PS_STACK_DEPTH; stackIndex++)
  {
    Serial.print(" ");
    Serial.print(fibers[fiberNum].state.paramStack[stackIndex]);
  }

  Serial.println();

  for (stackIndex = 0; stackIndex < RS_STACK_DEPTH; stackIndex++)
  {
    Serial.print(" ");
    Serial.print((uint16)fibers[fiberNum].state.retStack[stackIndex]);
  }
}

void printFibers()
{
  int fiberIndex;

  for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
  {
    printFiber(fiberIndex);
    Serial.println();
  }
}

int availableMemory()
{
  // Use 1024 with ATmega168
  int size = 2048;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
      free(buf);
  return size;
}

void setPins()
{
  int shiftReg;
  int bitIndex;
  int mux;
  char outBit;

  //storage register low, clock low.
  CLEAR_PINS_0_7(STORCLK | SRCLK);

  for (shiftReg = 0; shiftReg < NUM_SHIFT_REGS; shiftReg++)
  {
    for (bitIndex = 0; bitIndex < 8; bitIndex++)
    {
      outBit = 1 &
               (currentOutputs[NUM_SHIFT_REGS - shiftReg - 1] >>
                8 - bitIndex - 1);

      //Set serial input line.
      if (outBit)
      {
        SET_PINS_0_7(SEROUT);
      }
      else
      {
        CLEAR_PINS_0_7(SEROUT);
      }

      SET_PINS_0_7(SRCLK); //rising edge commits bit
      CLEAR_PINS_0_7(SRCLK); //go low on clock

      CLEAR_PINS_0_7(SEROUT); //from tutorial, prevents "bleed through"
    }
  }

  SET_PINS_0_7(OE);      //output off
  SET_PINS_0_7(STORCLK); //storage register commit bits
  CLEAR_PINS_0_7(OE);    //output enable
}

void Isr(void)
{
  int fiberIndex;
  char shiftIndex;
  char outputIndex;
  char bitsUsed;
  char bitsLeft;
  char output;

  if ((scheduler.ticks % DUTY_CYCLE_RESOLUTION) == 0)
  {
    if (!scheduler.done)
    {
      scheduler.missedTiming = true;
    }

    scheduler.sem = true;
    scheduler.ticks = 0;
    scheduler.subframes++;

    if ((scheduler.subframes * DUTY_CYCLE_RESOLUTION) % PERIOD_US == 0)
    {
      scheduler.frames++;
      scheduler.subframes = 0;
    }

    for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
    {
      outs[fiberIndex].pwm    = fibers[fiberIndex].registers.pwm;
      outs[fiberIndex].output = fibers[fiberIndex].registers.output;
    }
  }

  outputIndex = NUM_SHIFT_REGS - 1;
  bitsLeft = 8;
  for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
  {
    bitsUsed = (uint16)pgm_read_word(&fiberBitsUsed[fiberIndex]);

    if (bitsUsed == 0)
    {
      //Fiber doesn't use any output bits.
      continue;
    }

    output = 0;

    if ((bitsUsed | SKIP_BIT) == SKIP_BIT)
    {
      //These bits are unused and should be skipped by
      //filling them with 0s.
      bitsUsed &= MASK_OUT_SKIP_BIT;
    }
    else if (scheduler.ticks < outs[fiberIndex].pwm)
    {
      //Fiber is currently "on" in PWM cycle.
      output = outs[fiberIndex].output;
    }
    //else use the default output of 0.

    //fit bitsUsed into outputs.
    while (bitsUsed > bitsLeft)
    {
      currentOutputs[outputIndex] <<= bitsLeft;
      currentOutputs[outputIndex] |=
        ((output >> bitsUsed-bitsLeft) &
         masks[bitsLeft]);
      bitsUsed -= bitsLeft;
      //output >>= bitsLeft;
      bitsLeft = 8;
      outputIndex--;
    }

    //have we used up our bits?
    if (bitsUsed != 0)
    {
      //partial output byte left to fill.
      currentOutputs[outputIndex] <<= bitsUsed;
      currentOutputs[outputIndex] |= (output & masks[bitsUsed]);
      bitsLeft -= bitsUsed;
    }

    if (bitsLeft == 0)
    {
      bitsLeft = 8;
      outputIndex--;
    }
  }

  setPins();

  scheduler.ticks++;
}

