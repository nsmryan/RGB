import Data.List
import Data.Maybe
import Data.Bits
import System.Environment
import Control.Monad.State.Strict
import Control.Applicative
import qualified Data.Map as M
import qualified Text.ParserCombinators.Parsec as P

type Sym = String

data Instr = For
           | Incr
           | Dec
           | Func Sym
           | Call
           | Loop
           | Return
           | Set
           | Wait
           | Push Int
           | Drop
           | Swap
           | Dup
           | Yield
           | Tick Sym
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
           | IfElse
           | Done deriving (Show, Eq)

addSymToProg sym (ProgState n mapping) = ProgState n $ M.insert sym n mapping 

nextLocation (Push _) = incrLoc 2
nextLocation (Tick _) = incrLoc 2
nextLocation instr = incrLoc 1

incrLoc i = do
  (ProgState n mapping) <- get
  put $ ProgState (n + i) mapping

appendInstr = return . Just
progToC :: Instr -> State ProgState (Maybe String)
progToC For = appendInstr "FOR_OPCODE"
progToC Dec = appendInstr "DEC_OPCODE"
progToC Incr = appendInstr "INCR_OPCODE"
progToC (Func sym) = return Nothing
progToC Call = appendInstr "CALL_OPCODE"
progToC (Tick sym) = do
  (ProgState n mapping) <- get
  case sym `M.lookup` mapping of
    (Just location) -> do
      Just pushLoc <- progToC (Push location)
      return $ Just $ pushLoc
    otherwise -> error $ "could not find: " ++ sym
progToC Loop = appendInstr "LOOP_OPCODE"
progToC Return = appendInstr "RET_OPCODE"
progToC Done = appendInstr "DONE_OPCODE"
progToC Set = appendInstr "SET_OPCODE"
progToC Wait = appendInstr "WAIT_OPCODE"
progToC Drop = appendInstr "DROP_OPCODE"
progToC Swap = appendInstr "SWAP_OPCODE"
progToC Dup = appendInstr "DUP_OPCODE"
progToC Yield = appendInstr "YIELD_OPCODE"
progToC ToA = appendInstr "TOA_OPCODE"
progToC FromA = appendInstr "FROMA_OPCODE"
progToC ToR = appendInstr "TOR_OPCODE"
progToC FromR = appendInstr "FROMR_OPCODE"
progToC Add = appendInstr "ADD_OPCODE"
progToC Mult = appendInstr "MULT_OPCODE"
progToC Mod = appendInstr "MOD_OPCODE"
progToC Sub = appendInstr "SUB_OPCODE"
progToC Nip = appendInstr "NIP_OPCODE"
progToC If = appendInstr "IF_OPCODE"
progToC GreaterThen = appendInstr "GT_OPCODE"
progToC LessThen = appendInstr "LT_OPCODE"
progToC EqualTo = appendInstr "EQ_OPCODE"
progToC IfElse = appendInstr "IFELSE_OPCODE"
progToC (Push n) = return $ Just $ show (lsb n) ++ ", " ++ show (msb n) where
msb n = shiftR (n .&. 0xFF00) 8
lsb n = n .&. 0xFF

makeFuncMap (Func sym) = modify $ addSymToProg sym
makeFuncMap instr = nextLocation instr

data ProgState = ProgState Int (M.Map String Int)
emptyProgState = ProgState 0 M.empty

secsToTicks secs = round $ 100 * secs
waitSeconds seconds = wait $ secsToTicks seconds

wait 0 = []
wait ticks = [Push ticks, Wait]
waitInstr = [Wait]
forInstr = [For]
forN n = [Push n, For]
incr n = [Push n, Incr]
incrInstr = [Incr]
dec n = [Push n, Dec]
decInstr = [Dec]
push n = [Push n]
swapInstr = [Swap]
dupInstr = [Dup]
dropInstr = [Drop]
loop = [Loop]
done = [Done]
func sym = [Func sym]
ret = [Return]
set n = [Push n, Set]
setInstr = [Set]
call sym = [Tick sym, Call]
callInstr = [Call]
yield = [Yield]
tick sym = [Tick sym]
toA = [ToA]
fromA = [FromA]
toR = [ToR]
fromR = [FromR]
add = [Add]
sub = [Sub]
mult = [Mult]
mod = [Mod]

instrsToC :: [Instr] -> String
instrsToC instrs = header ++ start ++ mid ++ end where
  header = "#include <rgb.h>\n\n"
  start = "prog_uint8_t prog[1000] PROGMEM = {"
  prog = catMaybes $ evalState (prog' >> mapM progToC instrs) emptyProgState
  prog' = mapM_ makeFuncMap instrs
  mid = intercalate ", " prog 
  end = "};"

main = do
  args <- getArgs
  case args of
    (fileName:[]) -> do
      contents <- readFile fileName
      case P.parse ledLang "" contents of
        Left err -> print err
        Right prog -> putStrLn $ instrsToC $ prog
    otherwise -> putStrLn $ instrsToC $ prog

symbol = P.oneOf "~`!@#$^&*()_-+={[}]|/\\\"':;?/>.<,"
wordName = do
  first <- P.letter
  rest <- P.many (P.letter <|> P.digit <|> symbol)
  P.spaces
  return (first : rest)

simpleWord nam instr = P.string nam >> P.many1 P.space >> return [instr]
callParser = simpleWord "call" Call
forParser = simpleWord "for" For
incrParser = simpleWord "incr" Incr
decParser = simpleWord "dec" Dec
funcParser = do
  P.string ":" 
  P.spaces
  name <- wordName
  P.spaces
  return [Func name]
loopParser =  simpleWord "loop" Loop
returnParser = simpleWord ";" Return
setParser = simpleWord "set" Set
waitParser = simpleWord "wait" Wait
pushParser = do
  num <- P.many1 P.digit
  P.spaces
  return [Push (read num)]
dropParser = simpleWord "drop" Drop
swapParser =  simpleWord "swap" Swap
dupParser =  simpleWord "dup" Dup
yieldParser = simpleWord "yield" Yield
tickParser = do
  P.string "' " 
  P.spaces
  nam <- P.many1 (P.letter <|> P.digit <|> symbol)
  P.spaces
  return [Tick nam]
fromAParser = simpleWord "a>" FromA
toAParser = simpleWord ">a" ToA
fromRParser = simpleWord "r>" FromR
toRParser = simpleWord ">r" ToR
addParser = simpleWord "+" Add
subParser = simpleWord "-" Sub
multParser = simpleWord "*" Mult
modParser = simpleWord "%" Mod
doneParser = simpleWord "done" Done
nipParser = simpleWord "nip" Nip
ifParser = simpleWord "if" If
gtParser = simpleWord ">" GreaterThen
ltParser = simpleWord "<" LessThen
eqParser = simpleWord "=" EqualTo
ifElseParser = simpleWord "ifelse" IfElse
wordParser = do
  nam <- P.many1 (P.letter <|> P.digit <|> symbol)
  P.spaces
  return [Tick nam, Call]
ledLang :: P.Parser [Instr]
ledLang = concat <$> P.many parseExpr
parseExpr = foldl1 (<|>) $ map P.try
  [
    forParser, incrParser, decParser, callParser,
    loopParser, returnParser, setParser, waitParser,
    doneParser, dropParser, swapParser, dupParser,
    yieldParser, tickParser, fromAParser, toAParser,
    fromRParser, toRParser, addParser, subParser,
    multParser, pushParser, modParser, nipParser,
    ifParser, gtParser, ltParser, eqParser,
    ifElseParser, funcParser, wordParser
   ]

prog = concat
  [
    tick "fadeAll", push 1, call "times",
    done,

    func "fadeAll",
      tick "pushGreens", call "fadeEach",
      tick "pushReds", call "fadeEach",
      tick "pushBlues", call "fadeEach",
    ret,

    func "pushReds",
      push 0,
      push 3,
      push 6,
      push 9,
      push 12,
    ret,

    func "pushGreens",
      push 1,
      push 4,
      push 7,
      push 10,
      push 13,
    ret,

    func "pushBlues",
      push 2,
      push 5,
      push 8,
      push 11,
      push 14,
    ret,

    --( word -- )
    func "fadeEach",
      forN 100,
        dupInstr,
        callInstr,
        forN 4,
          incr 1,
        loop,
        wait 1,
      loop,
      forN 100,
        dupInstr,
        callInstr,
        forN 4,
          dec 1,
        loop,
        wait 1,
      loop,
      dropInstr,
    ret,

    func "times",
      forInstr,
        dupInstr,
        callInstr,
      loop,
      dropInstr,
    ret,

    done
  ]
