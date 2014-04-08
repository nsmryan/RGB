#define MSB_BYTE 0x80
#define OPCODE_BIT MSB_BYTE

#define DONE_OPCODE   (OPCODE_BIT | 0)
#define RET_OPCODE    (OPCODE_BIT | 1)
#define LOOP_OPCODE   (OPCODE_BIT | 2)
#define FOR_OPCODE    (OPCODE_BIT | 3)
#define CALL_OPCODE   (OPCODE_BIT | 4)
#define INCR_OPCODE   (OPCODE_BIT | 5)
#define DEC_OPCODE    (OPCODE_BIT | 6)
#define SET_OPCODE    (OPCODE_BIT | 7)
#define WAIT_OPCODE   (OPCODE_BIT | 8)
#define DUP_OPCODE    (OPCODE_BIT | 9)
#define DROP_OPCODE   (OPCODE_BIT | 10)
#define SWAP_OPCODE   (OPCODE_BIT | 11)
#define YIELD_OPCODE  (OPCODE_BIT | 12)
#define TOA_OPCODE    (OPCODE_BIT | 13)
#define FROMA_OPCODE  (OPCODE_BIT | 14)
#define TOR_OPCODE    (OPCODE_BIT | 15)
#define FROMR_OPCODE  (OPCODE_BIT | 16)
#define ADD_OPCODE    (OPCODE_BIT | 17)
#define SUB_OPCODE    (OPCODE_BIT | 18)
#define MULT_OPCODE   (OPCODE_BIT | 19)
#define MOD_OPCODE    (OPCODE_BIT | 20)
#define NIP_OPCODE    (OPCODE_BIT | 21)
#define IF_OPCODE     (OPCODE_BIT | 22)
#define GT_OPCODE     (OPCODE_BIT | 23)
#define LT_OPCODE     (OPCODE_BIT | 24)
#define EQ_OPCODE     (OPCODE_BIT | 25)
#define IFELSE_OPCODE (OPCODE_BIT | 26)

#define STACK_DEPTH 64

typedef char uint8;
typedef unsigned int uint16;
typedef int int16;

typedef struct
{
  uint8 sem;
  uint8 ticks;
  uint16 countPeriods;
  uint8 interpreting;
  uint16 newMinCycle;
  uint16 minCycle;
  uint16 newMaxCycle;
  uint16 maxCycle;
} RGBControl;

typedef struct
{
  uint8 ledOn;
  int16 dutyCycle;
  uint16 newDutyCycle;
} RGBLed;

typedef struct
{
  uint16 retStack[STACK_DEPTH];
  uint16 paramStack[STACK_DEPTH];
  uint16 instrPtr;
  uint16 retPtr;
  uint16 paramPtr;
  uint16 regA;
} ProgState;

