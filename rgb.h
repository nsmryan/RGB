#define PERIOD_HZ 1000
#define PERIOD_US 1000
#define DUTY_CYCLE_RESOLUTION 16
#define NUM_FIBERS 16
#define NUM_PINS_PER_MUX 8

#define NUM_MULTIPLEXERS 2

//Output enable (active low)
#define OE 6

//Shift register clock
#define SRCLK 3

//Storage Register Clock
#define STORCLK 4

//Master Reset
#define MR 5

//Serial out
#define SEROUT 2

#define MSB_BYTE 0x80
#define OPCODE_BIT MSB_BYTE

#define PROG_SIZE 100

#define DONE_OPCODE    (OPCODE_BIT | 0)
#define RET_OPCODE     (OPCODE_BIT | 1)
#define LOOP_OPCODE    (OPCODE_BIT | 2)
#define FOR_OPCODE     (OPCODE_BIT | 3)
#define CALL_OPCODE    (OPCODE_BIT | 4)
#define INCR_OPCODE    (OPCODE_BIT | 5)
#define DEC_OPCODE     (OPCODE_BIT | 6)
#define OUTPUT_OPCODE  (OPCODE_BIT | 7)
#define WAIT_OPCODE    (OPCODE_BIT | 8)
#define DUP_OPCODE     (OPCODE_BIT | 9)
#define DROP_OPCODE    (OPCODE_BIT | 10)
#define SWAP_OPCODE    (OPCODE_BIT | 11)
#define YIELD_OPCODE   (OPCODE_BIT | 12)
#define TOA_OPCODE     (OPCODE_BIT | 13)
#define FROMA_OPCODE   (OPCODE_BIT | 14)
#define TOR_OPCODE     (OPCODE_BIT | 15)
#define FROMR_OPCODE   (OPCODE_BIT | 16)
#define ADD_OPCODE     (OPCODE_BIT | 17)
#define SUB_OPCODE     (OPCODE_BIT | 18)
#define MULT_OPCODE    (OPCODE_BIT | 19)
#define MOD_OPCODE     (OPCODE_BIT | 20)
#define NIP_OPCODE     (OPCODE_BIT | 21)
#define IF_OPCODE      (OPCODE_BIT | 22)
#define GT_OPCODE      (OPCODE_BIT | 23)
#define LT_OPCODE      (OPCODE_BIT | 24)
#define EQ_OPCODE      (OPCODE_BIT | 25)
#define EVER_OPCODE    (OPCODE_BIT | 26)
#define TOB_OPCODE     (OPCODE_BIT | 27)
#define FROMB_OPCODE   (OPCODE_BIT | 28)
#define THEN_OPCODE    (OPCODE_BIT | 29)

#define PS_STACK_DEPTH 10
#define RS_STACK_DEPTH 10


typedef char uint8;
typedef unsigned int uint16;
typedef int int16;


typedef struct
{
  uint16 ticks;
  uint16 epochs;
  uint8 sem;
  uint8 done;
  uint8 missedTiming;
  uint8 fiberIndex;
} Scheduler;

typedef struct
{
  prog_uint16_t *retStack[RS_STACK_DEPTH];
  uint16 paramStack[PS_STACK_DEPTH];
} FiberState;

typedef struct
{
  char bitsUsed;
  prog_uint16_t *entryPoint;
} FiberConfig;

typedef struct
{
  prog_uint16_t *instrPtr;
  prog_uint16_t **retPtr;
  uint16 *paramPtr;
  uint16 regA;
  uint16 regB;
  uint8  output;
  uint8  pwm;
} FiberRegisters;

typedef struct
{
  FiberRegisters registers;
  FiberState state;
} Fiber;

