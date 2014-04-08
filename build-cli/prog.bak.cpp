#include <Arduino.h>
#include <rgb.h>

prog_uint8_t prog[1000] PROGMEM =
 {
   2, 0, FOR_OPCODE,
     0, 0, SET_OPCODE,
      100, 0, FOR_OPCODE,
        1, 0, INCR_OPCODE,
        5, 0, WAIT_OPCODE,
      LOOP_OPCODE,
      100, 0, FOR_OPCODE,
        1, 0, DEC_OPCODE,
        5, 0, WAIT_OPCODE,
      LOOP_OPCODE, 
    LOOP_OPCODE, 
    DONE_OPCODE
  };

