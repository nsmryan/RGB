#include <Arduino.h>
#include "TimerOne.h"
#include <rgb.h>


#define PRINT_RUN_TIME 0
#define PRINT_STARTUP 1
#define FIBERS 1

#define OP_2ARG(op) \
      currentFiber.paramPtr--; \
      arg = *currentFiber.paramPtr; \
      arg2 = *(currentFiber.paramPtr - 1); \
      *(currentFiber.paramPtr - 1) = arg2 op arg; \
      currentFiber.instrPtr++;

void Isr(void);
void inner();
void setPins();
void printFiber(int fiberNum);
void printFibers();
int availableMemory();

Fiber *fibers[MAX_NUM_FIBERS];
uint16 numFibers = 1;
FiberRegisters currentFiber;

volatile Scheduler scheduler;
volatile Output outs[NUM_OUTPUTS];
volatile Output currentOuts[NUM_OUTPUTS];
volatile uint8 currentOutputs[NUM_OUTPUTS];
uint8 shiftRegisterOutputs[NUM_OUTPUTS];

const uint8 masks[9] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

uint16 indicatorLEDOn = true;

extern prog_uint16_t prog[PROG_SIZE] PROGMEM;
extern prog_uint8_t outputBits[NUM_OUTPUTS] PROGMEM;

prog_uint8_t outputBits[NUM_OUTPUTS] PROGMEM = 
  {
    1, 3, 3, 3, 3, 3
    //1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
  };

//TODO
//macros for opcode defs
//user input from hardware
//eeprom configuration
//compilation on host

void setup()
{
  int fiberIndex;
  int outputIndex;

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
  fibers[0] = (Fiber*)malloc(sizeof(Fiber));
  fibers[0]->registers.instrPtr = (prog_uint16_t*)&prog[0];
  fibers[0]->registers.retPtr = (prog_uint16_t**)malloc(DEFAULT_PS_SIZE);
  fibers[0]->registers.paramPtr = (uint16*)malloc(DEFAULT_RS_SIZE);
  fibers[0]->registers.regA = 0;
  fibers[0]->registers.regB = 0;

  for (outputIndex = 0; outputIndex < NUM_OUTPUTS; outputIndex++)
  {
    currentOutputs[outputIndex] = 0;
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

#if FIBERS
    for (fiberNum = 0; fiberNum < numFibers; fiberNum++)
    {
      scheduler.fiberIndex = fiberNum;

      memcpy(&currentFiber,
             &fibers[fiberNum]->registers,
             sizeof(FiberRegisters));

      inner();

      memcpy(&fibers[fiberNum]->registers,
             &currentFiber,
             sizeof(FiberRegisters));
      //printFiber(fiberNum);
      //Serial.println();
      //Serial.print(outs[fiberNum].output);
    }
#endif
    //Serial.println();

    //if ((millis() % 1000) == 0)
    //{
    //  uint8 arr[6] = {1, 2, 3, 4, 5, 6};
    //  for (i = 0; i < 6; i++)
    //  {
    //    Serial.print(*(uint16*)&arr[i], HEX);
    //    Serial.print(" ");
    //  }
    //  Serial.println("");
    //}

    scheduler.done = true;


#if PRINT_RUN_TIME
    Serial.println(Timer1.read());
#endif
  }
  //TODO consider entering low power state until
  //interrupt wakes us up.
}

void inner()
{
  uint16 arg, arg2;
  uint16 opcode;

  while (true)
  {
    opcode = pgm_read_word(currentFiber.instrPtr);

    switch (opcode)
    {
      case DONE_OPCODE:
        return;
        break;

      case RET_OPCODE:
        currentFiber.instrPtr = RS_POP();
        break;

      case LOOP_OPCODE:
        arg = (uint16)RS_PEEK();
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
        arg = PS_POP();
        currentFiber.instrPtr++;
        RS_PUSH(currentFiber.instrPtr);
        RS_PUSH((prog_uint16_t*)arg);
        break;

      case CALL_OPCODE:
        arg = PS_POP();
        RS_PUSH(currentFiber.instrPtr + 1);
        currentFiber.instrPtr = (prog_uint16_t*)arg;
        break;

      case INCR_OPCODE:
        PS_SET(PS_PEEK()+1);
        currentFiber.instrPtr++;
        break;

      case DEC_OPCODE:
        PS_SET(PS_PEEK()-1);
        currentFiber.instrPtr++;
        break;

      case OUTPUT_OPCODE:
        arg = PS_POP();
        arg2 = PS_POP();
        outs[arg].output = arg2;
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
        return;
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
        return;
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
        OP_2ARG(+)
        break;

      case SUB_OPCODE:
        OP_2ARG(-)
        break;

      case MULT_OPCODE:
        OP_2ARG(*)
        break;

      case MOD_OPCODE:
        OP_2ARG(%)
        break;

      case NIP_OPCODE:
        arg = PS_POP();
        *(currentFiber.paramPtr - 1) = arg;
        currentFiber.instrPtr++;
        break;

      case IF_OPCODE:
        arg = PS_POP();
        arg2 = PS_POP();
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
        OP_2ARG(>)
        break;

      case LT_OPCODE:
        OP_2ARG(<)
        break;

      case EQ_OPCODE:
        OP_2ARG(==)
        break;

      case EVER_OPCODE:
        currentFiber.instrPtr = (prog_uint16_t*)*(currentFiber.retPtr - 2);
        break;

      case TOB_OPCODE:
        arg = PS_POP();
        currentFiber.regB = arg;
        currentFiber.instrPtr++;
        break;

      case FROMB_OPCODE:
        PS_PUSH(currentFiber.regB);
        currentFiber.instrPtr++;
        break;

      case SET_PWM_OPCODE:
        arg = PS_POP();
        arg2 = PS_POP();
        outs[arg].pwm = arg2;
        currentFiber.instrPtr++;
        break;

      case LSHIFT_OPCODE:
        OP_2ARG(<<);
        break;

      case RSHIFT_OPCODE:
        OP_2ARG(>>);
        break;

      case REPEAT_OPCODE:
        currentFiber.instrPtr = RS_PEEK();
        break;

      case BEGIN_OPCODE:
        RS_PUSH(currentFiber.instrPtr+1);
        currentFiber.instrPtr++;
        break;

      case FIBER_INDEX_OPCODE:
        PS_PUSH(scheduler.fiberIndex);
        currentFiber.instrPtr++;
        break;

      case JMP_OPCODE:
        currentFiber.instrPtr = (prog_uint16_t*)PS_POP();
        break;

      case NEW_FIBER_OPCODE:
        arg = PS_POP();  //stack size
        arg2 = PS_POP(); //entry point
        if (numFibers > MAX_NUM_FIBERS)
        {
          PS_PUSH(0);
        }
        else
        {
          fibers[numFibers] = (Fiber*)malloc(sizeof(Fiber));
          fibers[numFibers]->registers.retPtr = (prog_uint16_t**)malloc(arg*sizeof(uint16));
          fibers[numFibers]->registers.paramPtr = (uint16*)malloc(arg*sizeof(uint16));
          fibers[numFibers]->registers.instrPtr = (prog_uint16_t*)arg2;
          fibers[numFibers]->registers.regA = 0;
          fibers[numFibers]->registers.regB = 0;
          PS_PUSH((uint16)fibers[numFibers]);
          numFibers++;
        }
        currentFiber.instrPtr++;
        break;
      break;

      default:
        *currentFiber.paramPtr = opcode;
        currentFiber.paramPtr++;
        currentFiber.instrPtr++;
        break;
    }
  }
}

void printFiber(int fiberNum)
{
  int stackIndex;

  Serial.println();
  Serial.print(fiberNum);
  Serial.println(":");
  Serial.print("ps = ");
  Serial.print((uint16)fibers[fiberNum]->registers.paramPtr);
  Serial.print(" rs = ");
  Serial.print((uint16)fibers[fiberNum]->registers.retPtr);
  Serial.print(" ip = ");
  Serial.print((uint16)fibers[fiberNum]->registers.instrPtr);

  //for (stackIndex = 0; stackIndex < PS_STACK_DEPTH; stackIndex++)
  //{
  //  Serial.print(" ");
  //  Serial.print(fibers[fiberNum]->state.paramStack[stackIndex]);
  //}

  Serial.println();

  //for (stackIndex = 0; stackIndex < RS_STACK_DEPTH; stackIndex++)
  //{
  //  Serial.print(" ");
  //  Serial.print((uint16)fibers[fiberNum]->state.retStack[stackIndex]);
  //}
}

void printFibers()
{
  int fiberIndex;

  for (fiberIndex = 0; fiberIndex < numFibers; fiberIndex++)
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
  int delayTime = 6;
  char outputValue;

  //storage register low, clock low.
  CLEAR_PINS_0_7(STORCLK | SRCLK);

  for (shiftReg = 0; shiftReg < NUM_SHIFT_REGS; shiftReg++)
  {
    outputValue = shiftRegisterOutputs[NUM_SHIFT_REGS - shiftReg - 1];

    for (bitIndex = 0; bitIndex < 8; bitIndex++)
    {
      outBit = 1 &
               (outputValue  >>
                (8 - bitIndex - 1));

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
  char i;
  char shiftIndex;
  char fiberIndex;
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

    for (outputIndex = 0; outputIndex < NUM_OUTPUTS; outputIndex++)
    {
      currentOuts[outputIndex].output = outs[outputIndex].output;
      currentOuts[outputIndex].pwm = outs[outputIndex].pwm;
    }
  }

  outputIndex = NUM_SHIFT_REGS - 1;
  bitsLeft = 8;
  for (i = 0; i < NUM_OUTPUTS; i++)
  {
    bitsUsed = (uint16)pgm_read_word(&outputBits[i]);

    if (bitsUsed == 0)
    {
      //Output doesn't use any output bits.
      continue;
    }

    output = 0;

    if ((bitsUsed | SKIP_BIT) == SKIP_BIT)
    {
      //These bits are unused and should be skipped by
      //filling them with 0s.
      bitsUsed &= MASK_OUT_SKIP_BIT;
    }
    else if (scheduler.ticks < currentOuts[i].pwm)
    {
      //Output is currently "on" in PWM cycle.
      output = currentOuts[i].output;
    }
    //else use the default output of 0.

    //fit bitsUsed into outputs.
    while (bitsUsed > bitsLeft)
    {
      shiftRegisterOutputs[outputIndex] <<= bitsLeft;
      shiftRegisterOutputs[outputIndex] |=
        ((output >> bitsUsed-bitsLeft) &
         masks[bitsLeft]);
      bitsUsed -= bitsLeft;
      bitsLeft = 8;
      outputIndex--;
    }

    //have we used up our bits?
    if (bitsUsed != 0)
    {
      //partial output byte left to fill.
      shiftRegisterOutputs[outputIndex] <<= bitsUsed;
      shiftRegisterOutputs[outputIndex] |= (output & masks[bitsUsed]);
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

