#include <Arduino.h>
#include "TimerOne.h"
#include <rgb.h>


#define PRINT_RUN_TIME 0
#define PRINT_STARTUP 1


void Isr(void);
bool inner();
void setPins(char output[NUM_MULTIPLEXERS]);
void printFiber(int fiberNum);
void printFibers();
int availableMemory();

Fiber fibers[NUM_FIBERS];
FiberRegisters currentFiber;
Scheduler scheduler;
uint8 pins[NUM_FIBERS];

uint16 indicatorLEDOn = true;

extern prog_uint16_t prog_segments[NUM_FIBERS][PROG_SIZE] PROGMEM;
extern FiberConfig fiberConfigs[NUM_FIBERS];

//TODO
//macros for opcode defs
//user input from hardware
//eeprom configuration
//compilation on host
//hold multiplexers off (using one pin for all) until ready to use.

void setup()
{
  int fiberIndex;

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(MR, OUTPUT);
  digitalWrite(MR, LOW);
  digitalWrite(MR, HIGH);

  pinMode(OE, OUTPUT);
  digitalWrite(OE, HIGH);

  pinMode(SEROUT, OUTPUT);
  digitalWrite(SEROUT, LOW);

  pinMode(SRCLK, OUTPUT);
  digitalWrite(SRCLK, LOW);

  pinMode(STORCLK, OUTPUT);
  digitalWrite(STORCLK, LOW);

  //TODO change to load from EEEPROM

  for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
  {
    pins[fiberIndex] = DUTY_CYCLE_RESOLUTION;
    fibers[fiberIndex].registers.instrPtr = fiberConfigs[fiberIndex].entryPoint;
    fibers[fiberIndex].registers.retPtr = &fibers[fiberIndex].state.retStack[0];
    fibers[fiberIndex].registers.paramPtr = &fibers[fiberIndex].state.paramStack[0];
    fibers[fiberIndex].registers.regA = 0;
    fibers[fiberIndex].registers.regB = 0;
    fibers[fiberIndex].registers.output = 0;
  }

  Timer1.initialize(PERIOD_US);
  Timer1.attachInterrupt(Isr);

  scheduler.done = true;

  Serial.begin(9600);
}

void serialEvent()
{
  int byte = Serial.read();

  switch (byte)
  {
    case 'p':
      printFibers();
      break;
    
    case 'm':
      Serial.print("availableMemory = ");
      Serial.println(availableMemory());
      break;

    default:
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
    }
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

void setPins(char outputs[NUM_MULTIPLEXERS])
{
  int i;
  int mux;
  char outBit;

  digitalWrite(STORCLK, LOW); //storage register clock

  for (i = 0; i < 8 * NUM_MULTIPLEXERS; i++)
  {
    outBit = 1 & (outputs[i / NUM_PINS_PER_MUX] >> (i % NUM_PINS_PER_MUX));

    digitalWrite(SRCLK, LOW); //go low on clock

    digitalWrite(SEROUT, outBit); //send bit

    digitalWrite(SRCLK, HIGH); //rising edge commits bit

    digitalWrite(SEROUT, 0); //from tutorial, prevents "bleed through"
  }

  digitalWrite(OE, HIGH);     //output off
  digitalWrite(STORCLK, HIGH);//storage register commit bits
  digitalWrite(OE, LOW);      //output enable
}

void Isr(void)
{
  int fiberIndex;
  char mux;
  char outputs[NUM_MULTIPLEXERS] = {};

  if ((scheduler.ticks % DUTY_CYCLE_RESOLUTION) == 0)
  {
    if (!scheduler.done)
    {
      scheduler.missedTiming = true;
    }

    scheduler.sem = true;
    scheduler.ticks = 0;
    scheduler.epochs++;

    for (fiberIndex = 0; fiberIndex < NUM_FIBERS; fiberIndex++)
    {
      pins[fiberIndex] = fibers[fiberIndex].registers.output;
    }
  }

  fiberIndex = 0;
  for (mux = 0; mux < NUM_MULTIPLEXERS; mux++)
  {
    for (; fiberIndex < ((mux+1) * NUM_PINS_PER_MUX); fiberIndex++)
    {
      outputs[mux] <<= 1;
      if (scheduler.ticks < pins[fiberIndex])
      {
        outputs[mux] |= 1;
      }
    }
  }
  setPins(outputs);

  scheduler.ticks++;
}

