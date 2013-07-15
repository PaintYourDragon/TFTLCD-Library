#ifndef _pin_magic_
#define _pin_magic_

// This header file serves two purposes:
//
// 1) Isolate non-portable MCU port- and pin-specific identifiers and
//    operations so the library code itself remains somewhat agnostic
//    (PORTs and pin numbers are always referenced through macros).
//
// 2) GCC doesn't always respect the "inline" keyword, so this is a
//    ham-fisted manner of forcing the issue to minimize function calls.
//    This sometimes makes the library a bit bigger than before, but fast++.
//    However, because they're macros, we need to be SUPER CAREFUL about
//    parameters -- for example, write8(x) may expand to multiple PORT
//    writes that all refer to x, so it needs to be a constant or fixed
//    variable and not something like *ptr++ (which, after macro
//    expansion, may increment the pointer repeatedly and run off into
//    la-la land).  Macros also give us fune-grained control over which
//    operations are inlined on which boards (balancing speed against
//    available program space).

// When using the TFT shield, control and data pins exist in set physical
// locations, but the ports and bitmasks corresponding to each vary among
// boards.  A separate set of pin definitions is given for each supported
// board type.
// When using the TFT breakout board, control pins are configurable but
// the data pins are still fixed -- making every data pin configurable
// would be much too slow.  The data pin layouts are not the same between
// the shield and breakout configurations -- for the latter, pins were
// chosen to keep the tutorial wiring manageable more than making optimal
// use of ports and bitmasks.  So there's a second set of pin definitions
// given for each supported board.

// Shield pin usage:
// LCD Data Bit :   7   6   5   4   3   2   1   0
// Digital pin #:   7   6  13   4  11  10   9   8
// Uno port/pin : PD7 PD6 PB5 PD4 PB3 PB2 PB1 PB0
// Mega port/pin: PH4 PH3 PB7 PG5 PB5 PB4 PH6 PH5
// Leo port/pin : PE6 PD7 PC7 PD4 PB7 PB6 PB5 PB4
// Breakout pin usage:
// LCD Data Bit :   7   6   5   4   3   2   1   0
// Uno dig. pin :   7   6   5   4   3   2   9   8
// Uno port/pin : PD7 PD6 PD5 PD4 PD3 PD2 PB1 PB0
// Mega dig. pin:  29  28  27  26  25  24  23  22
// Mega port/pin: PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0 (one contiguous PORT)
// Leo dig. pin :   7   6   5   4   3   2   9   8
// Leo port/pin : PE6 PD7 PC6 PD4 PD0 PD1 PB5 PB4

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)

 // Arduino Uno, Duemilanove, etc.

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  // LCD control lines:
  // RD (read), WR (write), CD (command/data), CS (chip select)
  #define RD_PORT PORTC
  #define WR_PORT PORTC
  #define CD_PORT PORTC
  #define CS_PORT PORTC
  #define RD_MASK B00000001
  #define WR_MASK B00000010
  #define CD_MASK B00000100
  #define CS_MASK B00001000

  // These are macros for I/O operations...

  // Write 8-bit value to LCD data lines
// Ah HA.  Sometimes d is a const.  This makes the inline code
// simpler in those cases.
// The inline can't even match it because it resolves to a number
  #define write8inline(d) { \
   PORTD = (PORTD & B00101111) | ((d) & B11010000); \
   PORTB = (PORTB & B11010000) | ((d) & B00101111); \
   WR_STROBE; } // STROBEs are defined later

  #define write0x00() write8inline(0x00)

  #define Xwrite0x00() {                                          \
    uint8_t _temp;                                               \
    asm volatile(                                                \
      "in   %[tmp]  , %[portd]" "\n\t" /* _temp  = PORTD      */ \
      "andi %[tmp]  , 0x2F"     "\n\t" /* _temp &= B00101111  */ \
      "out  %[portd], %[tmp]"   "\n\t" /* PORTD  = _temp      */ \
      "in   %[tmp]  , %[portb]" "\n\t" /* _temp  = PORTB      */ \
      "andi %[tmp]  , 0xD0"     "\n\t" /* _temp &= B11010000  */ \
      "out  %[portb], %[tmp]"   "\n\t" /* PORTB  = _temp      */ \
      "cbi  %[portc], 1"        "\n\t" /* WR low  (active)    */ \
      "sbi  %[portc], 1"        "\n\t" /* WR high (inactive)  */ \
      : [tmp]   "=d" (_temp)                                     \
      : [portd] "I"  (_SFR_IO_ADDR(PORTD)),                      \
        [portb] "I"  (_SFR_IO_ADDR(PORTB)),                      \
        [portc] "I"  (_SFR_IO_ADDR(PORTC))                       \
    );                                                           \
  }

  #define write0x22() write8inline(0x22)

  #define Xwrite0x22() {                                             \
    uint8_t _temp;                                                  \
    asm volatile(                                                   \
      "in   %[tmp]  , %[portd]" "\n\t" /* _temp  = PORTD         */ \
      "andi %[tmp]  , 0x2F"     "\n\t" /* _temp &= B00101111     */ \
      "ori  %[tmp]  , %[maskd]" "\n\t" /* _temp |= (0x22 & 0xD0) */ \
      "out  %[portd], %[tmp]"   "\n\t" /* PORTD  = _temp         */ \
      "in   %[tmp]  , %[portb]" "\n\t" /* _temp  = PORTB         */ \
      "andi %[tmp]  , 0xD0"     "\n\t" /* _temp &= B11010000     */ \
      "ori  %[tmp]  , %[maskb]" "\n\t" /* _temp |= (0x22 & 0x2F) */ \
      "out  %[portb], %[tmp]"   "\n\t" /* PORTB  = _temp         */ \
      "cbi  %[portc], 1"        "\n\t" /* WR low  (active)       */ \
      "sbi  %[portc], 1"        "\n\t" /* WR high (inactive)     */ \
      : [tmp]   "=d" (_temp)                                        \
      : [portd] "I"  (_SFR_IO_ADDR(PORTD)),                         \
        [maskd] "M"  (0x22 & 0xD0),                                 \
        [portb] "I"  (_SFR_IO_ADDR(PORTB)),                         \
        [maskb] "M"  (0x22 & 0x2F),                                 \
        [portc] "I"  (_SFR_IO_ADDR(PORTC))                          \
    );                                                              \
  }

  #define Xwrite8inline(d) {                                      \
    uint8_t _temp1, _temp2;                                      \
    asm volatile(                                                \
      "in   %[tmp1] , %[portd]" "\n\t" /* _temp1  = PORTD     */ \
      "andi %[tmp1] , 0x2F"     "\n\t" /* _temp1 &= B00101111 */ \
      "mov  %[tmp2] , %[val]"   "\n\t" /* _temp2  = d         */ \
      "andi %[tmp2] , 0xD0"     "\n\t" /* _temp2 &= B11010000 */ \
      "or   %[tmp1] , %[tmp2]"  "\n\t" /* _temp1 |= _temp2    */ \
      "out  %[portd], %[tmp1]"  "\n\t" /* PORTD   = _temp1    */ \
      "in   %[tmp1] , %[portb]" "\n\t" /* _temp1  = PORTB     */ \
      "andi %[tmp1] , 0xD0"     "\n\t" /* _temp1 &= B11010000 */ \
      "mov  %[tmp2] , %[val]"   "\n\t" /* _temp2  = d         */ \
      "andi %[tmp2] , 0x2F"     "\n\t" /* _temp2 &= B00101111 */ \
      "or   %[tmp1] , %[tmp2]"  "\n\t" /* _temp1 |= _temp2    */ \
      "out  %[portb], %[tmp1]"  "\n\t" /* PORTB   = _temp1    */ \
      "cbi  %[portc], 1"        "\n\t" /* WR low  (active)    */ \
      "sbi  %[portc], 1"        "\n\t" /* WR high (inactive)  */ \
      : [tmp1] "=d" (_temp1),                                    \
        [tmp2] "=d" (_temp2)                                     \
      : [portd] "I" (_SFR_IO_ADDR(PORTD)),                       \
        [portb] "I" (_SFR_IO_ADDR(PORTB)),                       \
        [val]   "r" (d),                                         \
        [portc] "I" (_SFR_IO_ADDR(PORTC))                        \
    );                                                           \
  }

//  #define read8inline() (RD_STROBE, (PIND & B11010000) | (PINB & B00101111))

  // Read 8-bit value from LCD data lines
  #define read8(result) { /* result MUST be uint8_t in caller */ \
    uint8_t _temp;                                               \
    asm volatile(                                                \
      "cbi  %[portc] , 0"       "\n\t" /* RD low (active)     */ \
      "rjmp .+0"                "\n\t" /* 125 nS nop          */ \
      "rjmp .+0"                "\n\t" /* (pixel read         */ \
      "rjmp .+0"                "\n\t" /*      requires       */ \
      "rjmp .+0"                "\n\t" /*     400 nS minimum) */ \
      "in   %[res]   , %[pind]" "\n\t" /* result = PIND       */ \
      "in   %[tmp]   , %[pinb]" "\n\t" /* _temp  = PINB       */ \
      "sbi  %[portc] , 0"       "\n\t" /* RD high (inactive)  */ \
      "andi %[res]   , 0xD0"    "\n\t" /* result &= B11010000 */ \
      "andi %[tmp]   , 0x2F"    "\n\t" /* _temp  &= B00101111 */ \
      "or   %[res]   , %[tmp]"  "\n\t" /* result |= _temp     */ \
      : [res]   "=d" (result),                                   \
        [tmp]   "=d" (_temp)                                     \
      : [portc] "I"  (_SFR_IO_ADDR(PORTC)),                      \
        [pind]  "I"  (_SFR_IO_ADDR(PIND)),                       \
        [pinb]  "I"  (_SFR_IO_ADDR(PINB))                        \
      );                                                         \
    }

  // These set the PORT directions as required before the write and read
  // operations.  Because write operations are much more common than reads,
  // the data-reading functions in the library code set the PORT(s) to
  // input before a read, and restore them back to the write state before
  // returning.  This avoids having to set it for output inside every
  // drawing method.  The default state has them initialized for writes.
  #define setWriteDirInline() { DDRD |=  B11010000; DDRB |=  B00101111; }
  #define setReadDirInline()  { DDRD &= ~B11010000; DDRB &= ~B00101111; }

 #else // Uno w/Breakout board

  #define write8inline(d) { \
   PORTD = (PORTD & B00000011) | ((d) & B11111100); \
   PORTB = (PORTB & B11111100) | ((d) & B00000011); \
   WR_STROBE; }
//  #define read8inline() (RD_STROBE, (PIND&  B11111100)|(PINB&  B00000011))
  #define setWriteDirInline()      { DDRD|= B11111100;  DDRB|= B00000011; }
  #define setReadDirInline()       { DDRD&=~B11111100;  DDRB&=~B00000011; }

  #define read8(result) { /* result MUST be uint8_t in caller */ \
    uint8_t _temp;                                               \
    asm volatile(                                                \
      "cbi  %[portc] , 0"       "\n\t" /* RD low (active)     */ \
      "rjmp .+0"                "\n\t" /* 125 nS nop          */ \
      "rjmp .+0"                "\n\t" /* (pixel read         */ \
      "rjmp .+0"                "\n\t" /*      requires       */ \
      "rjmp .+0"                "\n\t" /*     400 nS minimum) */ \
      "in   %[res]   , %[pind]" "\n\t" /* result = PIND       */ \
      "in   %[tmp]   , %[pinb]" "\n\t" /* _temp  = PINB       */ \
      "sbi  %[portc] , 0"       "\n\t" /* RD high (inactive)  */ \
      "andi %[res]   , 0xFC"    "\n\t" /* result &= B11111100 */ \
      "andi %[tmp]   , 0x03"    "\n\t" /* _temp  &= B00000011 */ \
      "or   %[res]   , %[tmp]"  "\n\t" /* result |= _temp     */ \
      : [res]   "=d" (result),                                   \
        [tmp]   "=d" (_temp)                                     \
      : [portc] "I"  (_SFR_IO_ADDR(PORTC)),                      \
        [pind]  "I"  (_SFR_IO_ADDR(PIND)),                       \
        [pinb]  "I"  (_SFR_IO_ADDR(PINB))                        \
      );                                                         \
    }

 #endif

  // As part of the inline control, macros reference other macros...if any
  // of these are left undefined, an equivalent function version (non-inline)
  // is declared later.  The Uno has a moderate amount of program space, so
  // only write8() is inlined -- that one provides the most performance
  // benefit, but also generates the most bloat.
  #define write8 write8inline

#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 

 // Arduino Mega, ADK, etc.

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  #define RD_PORT PORTF
  #define WR_PORT PORTF
  #define CD_PORT PORTF
  #define CS_PORT PORTF
  #define RD_MASK B00000001
  #define WR_MASK B00000010
  #define CD_MASK B00000100
  #define CS_MASK B00001000

  #define write8inline(d) { \
   PORTH = (PORTH & B10000111)|(((d) & B11000000)>>3)|(((d) & B00000011)<<5); \
   PORTB = (PORTB & B01001111)|(((d) & B00101100)<<2); \
   PORTG = (PORTG & B11011111)|(((d) & B00010000)<<1); \
   WR_STROBE; }
  #define read8inline() (RD_STROBE, \
   ((PINH & B00011000) << 3) | ((PINB & B10110000) >> 2) | \
   ((PING & B00100000) >> 1) | ((PINH & B01100000) >> 5))
  #define setWriteDirInline() { \
   DDRH |=  B01111000; DDRB |=  B10110000; DDRG |=  B00100000; }
  #define setReadDirInline() { \
   DDRH &= ~B01111000; DDRB &= ~B10110000; DDRG &= ~B00100000; }

  // Strobe is wonky on Mega w/shield.  Haven't worked out the underlying
  // reason, but an interim kludge is just to use inverted levels. ???
  #define RD_STROBE RD_IDLE, RD_ACTIVE

 #else // Mega w/Breakout board

  #define write8inline(d)               { PORTA = (d); WR_STROBE; }
  #define read8inline()       (RD_STROBE, PINA)
  #define setWriteDirInline()             DDRA  = 0xff
  #define setReadDirInline()              DDRA  = 0

 #endif

  // All of the functions are inlined on the Arduino Mega.  When using the
  // breakout board, the macro versions aren't appreciably larger than the
  // function equivalents, and they're super simple and fast.  When using
  // the shield, the macros become pretty complicated...but this board has
  // so much code space, the macros are used anyway.  If you need to free
  // up program space, some macros can be removed, at a minor cost in speed.
  #define write8            write8inline
  #define read8             read8inline
  #define setWriteDir       setWriteDirInline
  #define setReadDir        setReadDirInline
  #define writeRegister8    writeRegister8inline
  #define writeRegister16   writeRegister16inline
  #define writeRegisterPair writeRegisterPairInline

#elif defined(__AVR_ATmega32U4__)

 // Arduino Leonardo

 #ifdef USE_ADAFRUIT_SHIELD_PINOUT

  #define RD_PORT PORTF
  #define WR_PORT PORTF
  #define CD_PORT PORTF
  #define CS_PORT PORTF
  #define RD_MASK B10000000
  #define WR_MASK B01000000
  #define CD_MASK B00100000
  #define CS_MASK B00010000

  #define write8inline(d) { \
   PORTE = (PORTE & B10111111) | (((d) & B10000000)>>1); \
   PORTD = (PORTD & B01101111) | (((d) & B01000000)<<1) | ((d) & B00010000); \
   PORTC = (PORTC & B01111111) | (((d) & B00100000)<<2); \
   PORTB = (PORTB & B00001111) | (((d) & B00001111)<<4); \
   WR_STROBE; }
  #define read8inline() (RD_STROBE, \
   (((PINE & B01000000) << 1) | ((PIND & B10000000) >> 1) | \
    ((PINC & B10000000) >> 2) | ((PINB & B11110000) >> 4) | \
     (PIND & B00010000)))
  #define setWriteDirInline() { \
   DDRE |=  B01000000; DDRD |=  B10010000; \
   DDRC |=  B10000000; DDRB |=  B11110000; }
  #define setReadDirInline() { \
   DDRE &= ~B01000000; DDRD &= ~B10010000; \
   DDRC &= ~B10000000; DDRB &= ~B11110000; }

 #else // Leonardo w/Breakout board

  #define write8inline(d) { \
   uint8_t dr1 = (d) >> 1, dl1 = (d) << 1; \
   PORTE = (PORTE & B10111111) | (dr1 & B01000000); \
   PORTD = (PORTD & B01101100) | (dl1 & B10000000) | (((d) & B00001000)>>3) | \
                                 (dr1 & B00000010) |  ((d) & B00010000); \
   PORTC = (PORTC & B10111111) | (dl1 & B01000000); \
   PORTB = (PORTB & B11001111) |(((d) & B00000011)<<4); \
   WR_STROBE; }

  #define read8inline() (RD_STROBE, \
   (((PINE & B01000000) | (PIND & B00000010)) << 1) | \
   (((PINC & B01000000) | (PIND & B10000000)) >> 1) | \
    ((PIND & B00000001)<<3) | ((PINB & B00110000)>>4) | (PIND & B00010000))
  #define setWriteDirInline() { \
   DDRE |=  B01000000; DDRD |=  B10010011; \
   DDRC |=  B01000000; DDRB |=  B00110000; }
  #define setReadDirInline() { \
   DDRE &= ~B01000000; DDRD &= ~B10010011; \
   DDRC &= ~B01000000; DDRB &= ~B00110000; }

 #endif

  // On the Leonardo, only the write8() macro is used -- though even that
  // might be excessive given the code size and available program space
  // on this board.  You may need to disable this to get any sizable
  // program to compile.
  #define write8 write8inline

#else

 #error "Board type unsupported / not recognized"

#endif

// Stuff common to all Arduino board types:

#ifdef USE_ADAFRUIT_SHIELD_PINOUT

 // Control signals are ACTIVE LOW (idle is HIGH)
 // Command/Data: LOW = command, HIGH = data
 // These are single-instruction operations and always inline
 #define RD_ACTIVE  RD_PORT &= ~RD_MASK
 #define RD_IDLE    RD_PORT |=  RD_MASK
 #define WR_ACTIVE  WR_PORT &= ~WR_MASK
 #define WR_IDLE    WR_PORT |=  WR_MASK
 #define CD_COMMAND CD_PORT &= ~CD_MASK
 #define CD_DATA    CD_PORT |=  CD_MASK
 #define CS_ACTIVE  CS_PORT &= ~CS_MASK
 #define CS_IDLE    CS_PORT |=  CS_MASK

#else // Breakout board

 // When using the TFT breakout board, control pins are configurable.
 #define RD_ACTIVE  *rdPort &=  rdPinUnset
 #define RD_IDLE    *rdPort |=  rdPinSet
 #define WR_ACTIVE  *wrPort &=  wrPinUnset
 #define WR_IDLE    *wrPort |=  wrPinSet
 #define CD_COMMAND *cdPort &=  cdPinUnset
 #define CD_DATA    *cdPort |=  cdPinSet
 #define CS_ACTIVE  *csPort &=  csPinUnset
 #define CS_IDLE    *csPort |=  csPinSet

#endif

// Data read and write strobes, ~2 instructions and always inline
#ifndef RD_STROBE
 #define RD_STROBE  RD_ACTIVE, RD_IDLE
#endif
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

// These higher-level operations are usually functionalized,
// except on Mega where's there's gobs and gobs of program space.

// Set value of TFT register: 8-bit address, 8-bit value
#define writeRegister8inline(a, d) { \
  CD_COMMAND; write8(a); CD_DATA; write8(d); }

// Set value of TFT register: 16-bit address, 16-bit value
// See notes at top about macro expansion, hence hi & lo temp vars
#define writeRegister16inline(a, d) { \
  uint8_t hi, lo; \
  hi = (a) >> 8; lo = (a); CD_COMMAND; write8(hi); write8(lo); \
  hi = (d) >> 8; lo = (d); CD_DATA   ; write8(hi); write8(lo); }

// Set value of 2 TFT registers: Two 8-bit addresses (hi & lo), 16-bit value
#define writeRegisterPairInline(aH, aL, d) { \
  uint8_t hi = (d) >> 8, lo = (d); \
  CD_COMMAND; write8(aH); CD_DATA; write8(hi); \
  CD_COMMAND; write8(aL); CD_DATA; write8(lo); }

#endif // _pin_magic_
