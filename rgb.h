#define PERIOD_HZ 1000
#define PERIOD_US 1000
#define DUTY_CYCLE_RESOLUTION 20
#define FRAMES_PER_SECOND (PERIOD_HZ / DUTY_CYCLE_RESOLUTION)
#define NUM_FIBERS 6
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

#define SET_PWM_OPCODE (OPCODE_BIT | 30)
#define LSHIFT_OPCODE  (OPCODE_BIT | 31)
#define RSHIFT_OPCODE  (OPCODE_BIT | 32)
#define REPEAT_OPCODE  (OPCODE_BIT | 33)
#define BEGIN_OPCODE   (OPCODE_BIT | 34)

#define PS_STACK_DEPTH 50
#define RS_STACK_DEPTH 50


typedef char uint8;
typedef unsigned int uint16;
typedef int int16;


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
  uint16  output;
  uint8  pwm;
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
