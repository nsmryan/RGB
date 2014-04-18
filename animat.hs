{-# LANGUAGE GADTs, Rank2Types, FlexibleInstances #-}

module Main where

import Prelude hiding (drop)

import Control.Monad
import Control.Monad.Operational
import Control.Monad.Trans
import Control.Monad.Identity
import Control.Monad.Reader
import Control.Monad.Writer.Lazy

import Debug.Trace

import System.Process

import Data.Maybe
import qualified Data.List as L
import Data.Bits
import System.Environment
import Control.Monad.State.Strict
import Control.Applicative
import qualified Data.Map as M
import qualified Text.ParserCombinators.Parsec as P

type Sym = String

type NameMap = M.Map String Int

data CompState = CompState 
               {
                 compiledProgram :: [CInstr]
               , nameMap :: NameMap
               } deriving (Show)

data Instr = For
           | Incr
           | Dec
           | Call
           | Loop
           | Ret
           | Set
           | Wait
           | Push Int
           | Drop
           | Swap
           | Dup
           | Yield
           | FromA
           | ToA
           | FromR
           | ToR
           | Add
           | Sub
           | Mult
           | Mod
           | Nip
           | If
           | GreaterThen
           | LessThen
           | EqualTo
           | Begin
           | Output
           | Ever
           | ToB
           | FromB
           | SetPWM
           | LShift
           | RShift
           | Repeat
           | Jmp
           | FiberIndex
           | NewFiber
           | Done deriving (Eq)

instance Show Instr where
  show For   = "FOR_OPCODE"
  show Dec   =  "DEC_OPCODE"
  show Incr  =  "INCR_OPCODE"
  show Call  =  "CALL_OPCODE"
  show Loop  =  "LOOP_OPCODE"
  show Ret   =  "RET_OPCODE"
  show Done  =  "DONE_OPCODE"
  show Set   =  "SET_OPCODE"
  show Wait  =  "WAIT_OPCODE"
  show Drop  =  "DROP_OPCODE"
  show Swap  =  "SWAP_OPCODE"
  show Dup   =  "DUP_OPCODE"
  show ToA   =  "TOA_OPCODE"
  show FromA =  "FROMA_OPCODE"
  show ToR   =  "TOR_OPCODE"
  show FromR =  "FROMR_OPCODE"
  show Add   =  "ADD_OPCODE"
  show Mult  =  "MULT_OPCODE"
  show Mod   =  "MOD_OPCODE"
  show Sub   =  "SUB_OPCODE"
  show Nip   =  "NIP_OPCODE"
  show If    =  "IF_OPCODE"
  show GreaterThen =  "GT_OPCODE"
  show LessThen =  "LT_OPCODE"
  show EqualTo = "EQ_OPCODE"
  show (Push n) = show n
  show Begin = "BEGIN_OPCODE"
  show Repeat = "REPEAT_OPCODE"
  show Output = "OUTPUT_OPCODE"
  show Yield = "YIELD_OPCODE"
  show Ever = "EVER_OPCODE"
  show ToB = "TOB_OPCODE"
  show FromB = "FROMB_OPCODE"
  show SetPWM = "SET_PWM_OPCODE"
  show LShift = "LSHIFT_OPCODE"
  show RShift = "RSHIFT_OPCODE"
  show Jmp = "JMP_OPCODE"
  show NewFiber = "NEW_FIBER_OPCODE"
  show FiberIndex = "FIBER_INDEX_OPCODE"

data CInstr = CInstr Instr
            | Tick Sym

instance Show CInstr where
  show (CInstr instr) = show instr
  show (Tick str) = "Label:" ++ str

instance Monoid CompState where
  mempty = CompState [] M.empty
  (CompState p m) `mappend` (CompState p' m') =
    CompState (p ++ p')  (m `mappend` m')

type Prog = StateT Int (Writer CompState) ()

resolveName mapping (Tick name) =
    case M.lookup name mapping of
      Nothing -> error "Label " ++ name ++ " not found"
      Just loc -> "(uint16)&prog[" ++ show loc ++ "]"
resolveName mapping instr = show instr

compileProg :: Prog -> String
compileProg instrs = let
  CompState comped mapping = execWriter (evalStateT instrs 0)
  secondPass = map (resolveName mapping) comped
  prog = L.intercalate ", " secondPass
  in header ++ start ++ prog ++ ", " ++ show Done ++ footer
header = "#include <rgb.h>\n\n"
start = "prog_uint16_t prog[2000] PROGMEM = {"
footer = "};"

msb n = (shiftR n 8) .&. 0xFF 
lsb n = n .&. 0xFF

compileInstr :: Instr -> Prog
compileInstr instr = do
  modify succ
  tell $ CompState [CInstr instr] M.empty
  
(wait:forLoop:incr:dec:swap:dup:drop:loop:done:ret:set:callInstr:yield:toA:fromA:toR:[]) =
  map compileInstr
    [ Wait, For, Incr, Dec, Swap, Dup
    , Drop, Loop, Done, Ret, Set, Call
    , Yield, ToA, FromA, ToR]
(fromR:add:sub:mult:modInstr:begin:repeatLoop:jmpInstr:fiberIndex:[]) =
  map compileInstr
    [ FromR, Add, Sub, Mult, Mod, Begin, Repeat, Jmp, FiberIndex]

(ever:toB:fromB:setPWM:lshift:rshift:output:newFiber:[]) = 
  map compileInstr
    [Ever,ToB,FromB,SetPWM,LShift,RShift, Output, NewFiber]

push n = compileInstr (Push n)

tick name = do
  modify succ
  tell $ CompState [Tick name] M.empty

func name = do
  loc <- get
  tell $ CompState [] (M.singleton name loc)

call name = do
  tick name
  callInstr

jmp name = do
  tick name
  jmpInstr


main = do
  run prog

run program = do
  writeFile "prog.ino" (compileProg program)
  runCommand "make upload"

prog = do
  tick "fading"; push 20; newFiber
  tick "fading"; push 20; newFiber
  tick "fading"; push 20; newFiber
  tick "fading"; push 20; newFiber
  tick "fading"; push 20; newFiber
  tick "fading"; push 20; newFiber
  done

  func "fading"
  fiberIndex; push 20; mult; wait
  begin;
    push 0; push 20; forLoop;
      dup; fiberIndex; setPWM; incr; push 1; wait;
    loop;
    push 20; forLoop;
      dup; fiberIndex; setPWM; dec; push 1; wait;
    loop;
    drop; incr; push 8; modInstr; dup; fiberIndex; output;
  repeatLoop
  done

allColors = do
  tick "allColors"; push 40; newFiber
  tick "allColors"; push 40; newFiber
  tick "allColors"; push 40; newFiber
  tick "allColors"; push 40; newFiber
  tick "allColors"; push 40; newFiber

  func "allColors"
  begin;
    push 1; fiberIndex; push 3; mult; add; setPWM;
    push 1; fiberIndex; push 3; mult; add; output;
    done;
    push 1; push 1; call "output";
    push 0; push 20; forLoop;
      incr; dup; push 1; fiberIndex; push 3; mult; add; incr; setPWM;
      push 1; wait;
    loop; drop;
  repeatLoop

  func "output";
    fiberIndex; push 3; mult; add; incr; output; 

  func "doNothing"
  done;

