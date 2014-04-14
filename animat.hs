{-# LANGUAGE GADTs, Rank2Types, FlexibleInstances #-}

module Main where

import Control.Monad
import Control.Monad.Operational
import Control.Monad.Trans
import Control.Monad.Identity
import Control.Monad.Reader
import Control.Monad.Writer.Lazy

import Debug.Trace

import Data.Maybe
import Data.List
import Data.Bits
import System.Environment
import Control.Monad.State.Strict
import Control.Applicative
--import qualified Data.Map as M
import qualified Text.ParserCombinators.Parsec as P

type Sym = String

--type NameMap = M.Map String Int
type NameMap = [(String, Int)]

data CompState = CompState 
               {
                 compiledProgram :: String
               , offset :: Int
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
  show SetPWM = "SETPWM_OPCODE"
  show LShift = "LSHIFT_OPCODE"
  show RShift = "RSHIFT_OPCODE"

instance Monoid CompState where
  mempty = CompState "" 0 []
  (CompState p o m) `mappend` (CompState p' o' m') =
    CompState (p ++ p') (o + o') (m ++ m')

type Prog = ReaderT (NameMap, Int)(Writer CompState) ()

compileProg :: Prog -> String
compileProg instrs = let
  (CompState prog loc mapping) =
    execWriter (runReaderT instrs (mapping, loc))
  in header ++ start ++ prog ++ ", " ++ show Done ++ footer
header = "#include <rgb.h>\n\n"
start = "prog_uint16_t prog[1000] PROGMEM = {"
footer = "};"

addSymToProg = undefined --sym (CompState prog n mapping) = ProgState n $ M.insert sym n mapping 

--nextLocation (Push _) = incrLoc 2
--nextLocation (Tick _) = incrLoc 2
--nextLocation instr = incrLoc 1

--incrLoc i = do
--  (ProgState n mapping) <- get
--  put $ ProgState (n + i) mapping

--appendInstr = return . Just

msb n = shiftR (n .&. 0xFF00) 8
lsb n = n .&. 0xFF

--makeFuncMap (Func sym) = modify $ addSymToProg sym
--makeFuncMap instr = nextLocation instr

--secsToTicks secs = round $ 100 * secs
--waitSeconds seconds = wait $ secsToTicks seconds

compileInstr :: Instr -> Prog
compileInstr instr = do
  (mapping, loc) <- ask
  tell $ CompState (show instr ++ ", ") 0 mapping
  
(wait:forLoop:incr:dec:swap:dup:drop:loop:done:ret:set:callInstr:yield:toA:fromA:toR:[]) =
  map compileInstr
    [ Wait, For, Incr, Dec, Swap, Dup
    , Drop, Loop, Done, Ret, Set, Call
    , Yield, ToA, FromA, ToR]
(fromR:add:sub:mult:mod:begin:repeatLoop:[]) =
  map compileInstr
  [ FromR, Add, Sub, Repeat, Begin, Mod, Mult]
push n = compileInstr (Push n)

func name = do
  (mapping, loc) <- ask
  tell $ CompState "" 0 (insert (name,loc) mapping)

call name = do
  env <- ask
  let ~(Just loc) = lookup name (fst env)
  push loc
  callInstr


main = do
    print . compileProg $ prog

prog = do
  func "test"
  push 0
  env <- ask
  traceShow env $ begin
  fromA
  incr
  call "test"
  toA
  repeatLoop

