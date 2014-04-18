#define PERIOD_HZ 1000
#define PERIOD_US 1000
#define DUTY_CYCLE_RESOLUTION 20
#define FRAMES_PER_SECOND (PERIOD_HZ / DUTY_CYCLE_RESOLUTION)
#define MAX_NUM_FIBERS  20
#define NUM_OUTPUTS 6
#define NUM_PINS_PER_MUX 8
#define LED_OFF 0
#define LED_ON DUTY_CYCLE_RESOLUTION

#define NUM_SHIFT_REGS 2

//0 and 1 ommited as they are reserved for serial communication.
#define PIN_2 (B00000100)
#define PIN_3 (B00001000)
#define PIN_4 (B00010000)
#define PIN_5 (B00100000)
#define PIN_6 (B01000000)
#define PIN_7 (B10000000)
#define SET_PINS_0_7(bits) (PORTD = PORTD | (bits & B11111100))
#define CLEAR_PINS_0_7(bits) (PORTD = PORTD & ~(bits & B11111100))
#define GET_PINS_0_7 PIND

//6 and 7 ommited as they are reserved the crystal oscillator.
#define PIN_0 (B00000001)
#define PIN_1 (B00000010)
#define PIN_2 (B00000100)
#define PIN_3 (B00001000)
#define PIN_4 (B00010000)
#define PIN_5 (B00100000)
#define SET_PINS_8_13(bits) (PORTB = PORTB | (bits & B00111111))
#define CLEAR_PINS_8_13(bits) (PORTB = PORTB & ~(bits & B00111111))
#define GET_PINS_8_13 PINB

#define LED_7_SEG_0 B0001000
#define LED_7_SEG_1 B1011011
#define LED_7_SEG_2 B0100010
#define LED_7_SEG_3 B0010010
#define LED_7_SEG_4 B1010001
#define LED_7_SEG_5 B0010100
#define LED_7_SEG_6 B0000100
#define LED_7_SEG_7 B1011010
#define LED_7_SEG_8 B0000000
#define LED_7_SEG_9 B0010000

//Output enable (active low)
#define OE_PIN_NUM 6
#define OE PIN_6

//Shift register clock
#define SRCLK_PIN_NUM 3
#define SRCLK PIN_3

//Storage Register Clock
#define STORCLK_PIN_NUM 4
#define STORCLK PIN_4

//Master Reset
#define MR_PIN_NUM 5
#define MR PIN_5

//Serial out
#define SEROUT_PIN_NUM 2
#define SEROUT PIN_2

//High bit in word indicates it is an opcode,
//not a literal number.
#define MSB_WORD 0x8000
#define OPCODE_BIT MSB_WORD

#define SKIP_BIT 0x80
#define SKIP_BITS(bits) (bits | SKIP_BIT)
#define MASK_OUT_SKIP_BIT 0x7F

#define PROG_SIZE 100


#define DEFAULT_PS_SIZE 50
#define DEFAULT_RS_SIZE 50

#define PS_PUSH(x)     (*(currentFiber.paramPtr++) = x)
#define PS_POP()       (*(--currentFiber.paramPtr))
#define PS_PEEK()      (*(currentFiber.paramPtr-1))
#define PS_PEEK_NTH(n) (*(currentFiber.paramPtr-n-1))
#define PS_SET(n)      (*(currentFiber.paramPtr-1) = n)

#define RS_PUSH(x)     (*(currentFiber.retPtr++) = x)
#define RS_POP()       (*(--currentFiber.retPtr))
#define RS_PEEK()      (*(currentFiber.retPtr-1))
#define RS_PEEK_NTH(n) (*(currentFiber.retPtr-n-1))
#define RS_SET(n)      (*(currentFiber.retPtr-1) = n)


typedef char uint8;
typedef unsigned int uint16;
typedef int int16;


typedef enum
{
  DONE_OPCODE = OPCODE_BIT,
  RET_OPCODE,
  LOOP_OPCODE,
  FOR_OPCODE,
  CALL_OPCODE,
  INCR_OPCODE,
  DEC_OPCODE,
  OUTPUT_OPCODE,
  WAIT_OPCODE,
  DUP_OPCODE,
  DROP_OPCODE,
  SWAP_OPCODE,
  YIELD_OPCODE,
  TOA_OPCODE,
  FROMA_OPCODE,
  TOR_OPCODE,
  FROMR_OPCODE,
  ADD_OPCODE,
  SUB_OPCODE,
  MULT_OPCODE,
  MOD_OPCODE,
  NIP_OPCODE,
  IF_OPCODE,
  GT_OPCODE,
  LT_OPCODE,
  EQ_OPCODE,
  EVER_OPCODE,
  TOB_OPCODE,
  FROMB_OPCODE,
  SET_PWM_OPCODE,
  LSHIFT_OPCODE,
  RSHIFT_OPCODE,
  REPEAT_OPCODE,
  BEGIN_OPCODE,
  FIBER_INDEX_OPCODE,
  JMP_OPCODE,
  NEW_FIBER_OPCODE,
  NUM_OPCODES
} OPCODE_ENUM;

typedef struct
{
  uint16 ticks;
  uint16 subframes;
  uint16 frames;
  uint8 sem;
  uint8 done;
  uint8 missedTiming;
  uint8 fiberIndex;
} Scheduler;

typedef struct
{
  prog_uint16_t **retStack;
  uint16 *paramStack;
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
} FiberRegisters;

typedef struct
{
  FiberRegisters registers;
  FiberState state;
} Fiber;

typedef struct
{
  uint16 pwm;
  uint16 output;
} Output;
