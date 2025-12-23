/*
test code for LAZULUM--test different functions with serial, breadboard, etc.
*/

#include <math.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#endif //not necessrary for synth

#define BLINK(pin, ms, blinks) do {cli(); for (int i=0; i<blinks; i++) { digitalWrite(pin, HIGH); delay(ms); digitalWrite(pin, LOW); delay(ms);} sei();} while (0)

/* POLYPHONIC LOGIC STRUCTURE:
ISR (20 kHz):    update oscillators, write audio
                 send flags to update envelopes, LFOs, voice allocation

main loop:       update envelopes & voice arr, handle serial i/o 
*/
/*GLOBAL CONSTANTS
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
const uint32_t CPU_RATE = 16000000; //timer clock runs at 16MHz
const int NUMVOICES = 4; const int DEADNAME = -1;
const int bits_phase_accumulator = 32;
const uint16_t SAMPLE_RATE = 16000; //20k Hz sample rate

const int WAVETABLE_SIZE = 256;
int prescaler_timer1 = 1; //hardcoded for now, determined by TCCR1B
const uint16_t ISR_MARGIN = 50; //measured in ticks
const uint16_t FRAME_TICKS = (F_CPU / (prescaler_timer1 * SAMPLE_RATE)); //how many ticks between interrupts
const uint16_t WCET_TICKS = 650; //estimated worst case scenario for ISR code to take to execute
const uint16_t SAFE_LIMIT = FRAME_TICKS - WCET_TICKS - ISR_MARGIN; // if this is exceeded, break the ISR for that instance
//ADSR params
uint8_t potA, potD, potR; //potentiometer values
volatile unsigned long samplecnt = 0;
volatile bool voicemixflag, envupdateflag, midicompleteflag, printflag, LEDflag, safeguardflag = false;
uint16_t output = 0; //0-255
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//MIDI globals
volatile uint8_t MIDIstatus, MIDInote, MIDIvelo, MIDIbyte_index;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
typedef enum {IDLE, ATTACK, DECAY, SUSTAIN, RELEASE} EnvelopeStage;
typedef enum {SINE, SQUARE, SAW, TRIANGLE, MOOG, SHARK, FORMANT, MS20, NUM_OSC_TYPES} OscillatorType;

typedef struct {
  volatile bool pressed;
  volatile uint8_t env_amp;
  volatile uint8_t raw;
  volatile EnvelopeStage stage;
  volatile uint16_t envIndex;
  volatile uint16_t amp;
  volatile uint32_t phase;
  volatile uint32_t phase_inc;
  int name;
  unsigned long startcnt;
  float f;
  uint8_t a_inc, d_inc, r_inc;
  uint8_t amp_before_r;
  const uint8_t * wavetable;
} Voice;
typedef struct {
  volatile bool active; volatile uint8_t name;
} Input;

Voice edgar[NUMVOICES];
Input key {false, DEADNAME};
OscillatorType osctype; uint8_t osc_button_press_cnt;

/*WAVETABLES
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
const uint8_t sinetable[256] PROGMEM = {
 128,131,134,137,140,143,146,149,152,156,159,162,165,168,171,174,
 177,180,183,186,188,191,194,197,199,202,204,207,209,211,214,216,
 218,220,222,224,226,227,229,231,232,234,235,236,238,239,240,241,
 242,243,244,244,245,245,246,246,246,246,246,246,246,245,245,244,
 244,243,242,241,240,239,238,236,235,234,232,231,229,227,226,224,
 222,220,218,216,214,211,209,207,204,202,199,197,194,191,188,186,
 183,180,177,174,171,168,165,162,159,156,152,149,146,143,140,137,
 134,131,128,124,121,118,115,112,109,106,103, 99, 96, 93, 90, 87,
  84, 81, 78, 75, 73, 70, 67, 64, 62, 59, 57, 54, 52, 50, 47, 45,
  43, 41, 39, 37, 35, 34, 32, 30, 29, 27, 26, 25, 23, 22, 21, 20,
  19, 18, 17, 17, 16, 16, 15, 15, 15, 15, 15, 15, 15, 16, 16, 17,
  17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 29, 30, 32, 34, 35, 37,
  39, 41, 43, 45, 47, 50, 52, 54, 57, 59, 62, 64, 67, 70, 73, 75,
  78, 81, 84, 87, 90, 93, 96, 99,103,106,109,112,115,118,121,124
};

const uint8_t squaretable[256] PROGMEM = {
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

const uint8_t sawtable[256] PROGMEM = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
 32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
 64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
 96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
  128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
  160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};

const uint8_t triangletable[256] PROGMEM = {
  // Rising
  0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,
  64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,
  128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,
  192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254,
    // Falling
  255,253,251,249,247,245,243,241,239,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,
  191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,145,143,141,139,137,135,133,131,129,
  127,125,123,121,119,117,115,113,111,109,107,105,103,101, 99, 97, 95, 93, 91, 89, 87, 85, 83, 81, 79, 77, 75, 73, 71, 69, 67, 65,
  63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1
};

const uint8_t moogtable[256] PROGMEM = {
  255,255,255,255,255,255,255,255,255,255,255,255,253,250,248,245,
  243,240,238,236,234,232,231,229,228,227,226,225,224,224,223,223,
  223,223,223,223,224,224,225,226,227,228,229,231,232,234,236,238,
  240,243,245,248,250,253,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  253,250,248,245,243,240,238,236,234,232,231,229,228,227,226,225,
  224,224,223,223,223,223,223,223,224,224,225,226,227,228,229,231,
  232,234,236,238,240,243,245,248,250,253,255,255,255,255,255,255,
  0,0,0,0,0,0,0,0,0,0,0,0,2,5,7,10,
  12,15,17,19,21,23,24,26,27,28,29,30,31,31,32,32,
  32,32,32,32,31,31,30,29,28,27,26,24,23,21,19,17,
  15,12,10,7,5,2,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,5,7,10,12,15,17,19,21,23,24,26,27,28,29,30,
  31,31,32,32,32,32,32,32,31,31,30,29,28,27,26,24,
  23,21,19,17,15,12,10,7,5,2,0,0,0,0,0,0
};

const uint8_t sharktable[256] PROGMEM = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
  16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
  32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
  48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
  64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,
  80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
  96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,
  112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
  160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,
  176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,
  208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,
  240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};


const uint8_t formanttable[256] PROGMEM = {
  128,131,135,139,143,147,151,155,159,162,166,169,172,175,178,180,
  182,184,186,188,189,190,191,192,192,192,192,191,190,189,187,185,
  183,181,178,175,172,169,165,161,157,153,148,144,139,134,129,124,
  119,114,109,104,100,95,91,87,83,80,77,74,72,70,69,68,
  68,68,69,70,72,74,77,80,83,87,91,95,100,104,109,114,
  119,124,129,134,139,144,148,153,157,161,165,169,172,175,178,181,
  183,185,187,189,190,191,192,192,192,192,191,190,189,188,186,184,
  182,180,178,175,172,169,166,162,159,155,151,147,143,139,135,131,
  128,125,121,117,113,109,105,101,97,94,90,87,84,81,78,76,
  74,72,70,68,67,66,65,64,64,64,64,65,66,67,69,71,
  73,75,78,81,84,87,90,94,97,101,105,109,113,117,121,125,
  128,131,135,139,143,147,151,155,159,162,166,169,172,175,178,180,
  182,184,186,188,189,190,191,192,192,192,192,191,190,189,187,185,
  183,181,178,175,172,169,165,161,157,153,148,144,139,134,129,124,
  119,114,109,104,100,95,91,87,83,80,77,74,72,70,69,68,
  68,68,69,70,72,74,77,80,83,87,91,95,100,104,109,114
};

const uint8_t ms20table[256] PROGMEM = {
  0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,
  64,68,72,76,80,84,88,92,96,100,104,108,112,116,120,124,
  128,132,136,140,144,148,152,156,160,164,168,172,176,180,184,188,
  192,196,200,204,208,212,216,220,224,228,232,236,240,244,248,252,
  255,250,245,240,235,230,225,220,215,210,205,200,195,190,185,180,
  175,170,165,160,155,150,145,140,135,130,125,120,115,110,105,100,
  95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,
  15,10,5,0,5,10,15,20,25,30,35,40,45,50,55,60,
  65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,
  145,150,155,160,165,170,175,180,185,190,195,200,205,210,215,220,
  225,230,235,240,245,250,255,250,245,240,235,230,225,220,215,210,
  205,200,195,190,185,180,175,170,165,160,155,150,145,140,135,130,
  125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,
  45,40,35,30,25,20,15,10,5,0,5,10,15,20,25,30,
  35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,
  115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190
};


//asdr tables
const uint8_t Atable[WAVETABLE_SIZE] = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
  16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
  32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
  48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
  64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,
  80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
  96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,
  112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
  128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,
  144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
  160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,
  176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,
  208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,
  240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};

const uint8_t Dtable[WAVETABLE_SIZE] = {
  255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,
  239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,
  223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,
  207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,
  191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,
  175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,
  159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,
  143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,
  127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,
  111,110,109,108,107,106,105,104,103,102,
  102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,
  102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,
  102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,
  102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,
  102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,
  102,102,102,102,102,102,102,102,102,102
};

const uint8_t Rtable[WAVETABLE_SIZE] = {
  255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,
  239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,
  223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,
  207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,
  191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,
  175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,
  159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,
  143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,
  127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,
  111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,
  95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,
  79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,
  63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,
  47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,
  31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,
  15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
};


const float MIDIfreqtable[129] PROGMEM = {
  8.18,8.66,9.18,9.72,10.30,10.91,11.56,12.25,12.98,13.75,14.57,15.43,16.35, 17.32, 18.35, 19.45, 20.60, 21.83, 
  23.12, 24.50, 25.96, 27.50, 29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00, 58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 
  82.41, 87.31, 92.50, 98.00, 103.83, 110.00, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00, 233.08, 246.94, 261.63, 
  277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.26, 698.46, 739.99, 783.99, 
  830.61, 880.00, 932.33, 987.77, 1046.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760.00, 1864.66, 1975.53, 2093.00, 2217.46, 2349.32, 
  2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520.00, 3729.31, 3951.07, 4186.01, 4434.92, 4698.64, 4978.03, 5274.04, 5587.65, 5919.91, 6271.93, 6644.88, 7040.00, 
  7458.62, 7902.13, 8372.02, 8869.84, 9397.27, 9956.06, 10548.08, 11175.30, 11839.82, 12543.85, 13289.75
};

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

/*FUNCTION DECLARATIONS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

//void generate_wavetables();
void generate_env_tables(EnvelopeStage stage, float lambda, float sustain, uint8_t * table);
void init_voicearray(Voice *);
void key_release(Voice *);
void key_off(Voice * v);
void key_press(OscillatorType osc, unsigned long startcnt, float freq, Voice* v, int SAMPLE_RATE, int name);
uint8_t convertADSR(int);
void incrementADSR(Voice* v);
float midiNoteToFreq(uint8_t);
void errorTrap(int line, int idx);
void incrementADSR_OLD(Voice* v);
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


void setup() {
  //PWM and GATE outputs
  OCR0A = 0;
  OCR0B = 0;
  TCCR0A &= ~(_BV(COM0A1) | _BV(COM0B1)); // disconnect PWM outputs
  pinMode(11, OUTPUT); //mixed output
  pinMode(3, OUTPUT); pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(9, OUTPUT); //individual voices
  pinMode(A1, INPUT); pinMode(A2, INPUT); pinMode(A3, INPUT); //pot inputs adsr
  pinMode(13, INPUT_PULLUP); //button to toggle oscillator mode
  pinMode(7, OUTPUT); //error button for voices going out of bounds
  osc_button_press_cnt = 0;
  //MIDI setup
  // Set baud rate to 31,250
  UBRR0H = ((F_CPU / 16 + 31250 / 2) / 31250 - 1) >> 8;
  UBRR0L = ((F_CPU / 16 + 31250 / 2) / 31250 - 1);
  // 8 data bits
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  UCSR0B &= ~_BV(RXCIE0);  // disable Arduino's RX interrupt
  UCSR0B &= ~_BV(RXEN0);   // disable Arduino's receive engine
  // enable rx for MIDI:
  UCSR0B |= _BV(RXEN0);
  UCSR0B |= _BV(RXCIE0);

  //timer setup
  cli();

  TCCR1A = 0; // Normal operation 
  TCCR1B = 0; TCNT1 = 0; 
  TCCR1B |= (1 << WGM12); // CTC mode 
  TCCR1B |= (1 << CS10); // No prescaler 
  TIMSK1 |= (1 << OCIE1A); // Enable compare interrupt
  prescaler_timer1 = 1;
  OCR1A = (F_CPU / (prescaler_timer1 * SAMPLE_RATE)) - 1; 
  //timer 2 setup, for PWM
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);  
  TCCR2B = _BV(CS20);  // No prescaler
  
  //Serial.begin(115200);
  // initialize voices:
  init_voicearray(edgar);
  //float lambda_decay = 0.02f;
  //float lambda_release = 0.01f;
  sei();
} //setup

void loop() {
  // put your main code here, to run repeatedly:
  //input logic
  /*
  if (Serial.available()) {
      int parse_int  = Serial.parseInt();
      if (parse_int < 1 || parse_int > 8) {
        Serial.println("error! input must be between 1-8.");
        key.active = false; voicemixflag = false;
      }
      else {key.active = true; key.name = parse_int; Serial.print("Key inputted: "); Serial.println(key.name);}
  }
  */

  //input logic-- MIDI PARSING
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  if (midicompleteflag) {
    //CASE: Note-On
    if (MIDIstatus == 0x90 && MIDIvelo > 0) {
      //find off voices first
      potA = analogRead(A1); potD = analogRead(A2); potR = analogRead(A3); 
      unsigned long oldest_start = UINT32_MAX; int i_oldest = 0; bool useoldest = true; bool useidle = true;
      for (int i=0; i<NUMVOICES; i++) {
        Voice *v = &edgar[i];
        if (v->name == MIDInote) {
          useoldest = false; useidle = false; 
          key_press(osctype, samplecnt, midiNoteToFreq(MIDInote), v, SAMPLE_RATE, MIDInote);
          break;
        }
      }
      if (useidle){
        for (int i=0; i<NUMVOICES; i++) {
          Voice *v = &edgar[i];
            if (v->name == MIDInote) {useoldest = false; break;}
            if (v->stage == IDLE) {key_press(osctype, samplecnt, midiNoteToFreq(MIDInote), v, SAMPLE_RATE, MIDInote); useoldest=false; break;}
            if (v->startcnt < oldest_start) {oldest_start = v->startcnt; i_oldest = i;}
            //velocity not handled for now
        }
      }
      //if none, find oldest voice to replace
      if (useoldest) {key_press(osctype, samplecnt, midiNoteToFreq(MIDInote), &edgar[i_oldest], SAMPLE_RATE, MIDInote);}
    }
    //CASE: Note-off
    if (MIDIstatus == 0x80 || (MIDIstatus == 0x90 && MIDIvelo == 0)){
      for (int i=0; i<NUMVOICES; i++) {
        Voice * v = &edgar[i];
        if (v->name == MIDInote && v->pressed)
          {key_release(v); break;}
      }
    }
    //DEBUG: check for ZOMBIE voices
    /*
    for (int i=0; i<NUMVOICES-1; i++){
      Voice * v1 = &edgar[i];
      if(v1->name == DEADNAME) continue;
      for (int j=i+1; j<NUMVOICES; j++) {
        Voice * v2 = &edgar[j];
        if(v2->name == DEADNAME) continue;
        if (v1->name == v2->name) digitalWrite(7, HIGH);
      }
    }
    */
  midicompleteflag = false;
  } //midicompleteflag

  /* OLD
  //voice allocation logic
  if (voicemixflag) {
    unsigned long oldest_start = UINT32_MAX; int i_oldest = 0; bool foundexisting, foundoff = false;
      for (int i=0; i<NUMVOICES; i++) {
        //if a key is pressed a second time, turn it off
        if (edgar[i].name == key.name) {key_release(&edgar[i]); foundexisting = true; break;}
      }
      if (foundexisting == false) {
        for (int i=0; i<NUMVOICES; i++) {
          //else, if a key i pressed for the first time, find a place for it
          //find any off voices
          if (edgar[i].stage == OFF) {key_press(SINE, samplecnt, freq, &edgar[i], SAMPLE_RATE, key.name); foundoff=true; break;}
          if (edgar[i].startcnt < oldest_start) {oldest_start = edgar[i].startcnt; i_oldest = i;}
        }
      }
      //if no off/existing voices found, replace the oldest
      if (foundexisting==false && foundoff==false) {key_press(SINE, samplecnt, freq, &edgar[i_oldest], SAMPLE_RATE, key.name);}
    key.active = false; //reset input event flag
    voicemixflag = false; //reset ISR->voice allocation flag
  }
  */
  
  //envelope update logic
  if (envupdateflag) {
    /* I set a flag to be set active every 4 counts in ISR, and an if statement in loop() to handle envelope update logic. 
    Since all the heavy math is done in loop(), I'll have a function to convert my const int potA, potD, potS, potR; 
    to msA, msD, msS, msR; in the beginning of the if statement. I could generate a high-resolution logarithmic wavetable for each 
    attack, decay, and release, and use A, D, S, R to control how many counts i increment through each depending on how fast 
    or slow the ADSR parameters are. */

    //digitalread potentiometer values to potA, potD .....
    //convert them
    
    //increment thru ADR wavetables
    for (int i=0; i<NUMVOICES; i++) {
      incrementADSR(&edgar[i]); //updates envIndex, stage, and env_amp
    }
    envupdateflag = false;
  }
  /*
  if (printflag) {
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
    Serial.println("ON VOICES:");
    for (int i=0; i<NUMVOICES; i++) {
      if (edgar[i].pressed) {
        Serial.println((String) "     name: "+ edgar[i].name + " stage: "+ edgar[i].stage +" amp: "+ edgar[i].env_amp);
      }
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * * * * * * * ");
    Serial.println("OFF VOICES:");
    for (int i=0; i<NUMVOICES; i++) {
      if (!edgar[i].pressed) {
        Serial.println((String) "     name: "+ edgar[i].name + " stage: "+ edgar[i].stage +" pos: "+ i);
      }
    }
    Serial.print("TOTAL MIXED OUTPUT: "); Serial.println(output);
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="); Serial.println();

    printflag = false;
  } */

  if (LEDflag) {
    analogWrite(3, edgar[0].env_amp);
    
    analogWrite(5, edgar[1].env_amp); analogWrite(6, edgar[2].env_amp); analogWrite(9, edgar[3].env_amp);

    if (digitalRead(13) == LOW) {
      osc_button_press_cnt++;
      if (osc_button_press_cnt >= NUM_OSC_TYPES)
        osc_button_press_cnt = 0;
      osctype = (OscillatorType) osc_button_press_cnt;
    }
    if (safeguardflag) {PIND = (1 << 7); safeguardflag = false;}
    LEDflag = false;
  }


  
} //loop()

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


ISR(TIMER1_COMPA_vect) {
  /*
  On every sample interrupt (20 kHz):
    sample_counter++

    do oscillator phase increments
    do wavetable lookup
    mix voices
    output sample

    if sample_counter % 40 == 0:
         update envelopes

    if sample_counter % 16 == 0: 
         update voices

  */
  //SAFEGUARD CODE: if the ISR is already too late to finish in time, break that instance
  if (TCNT1 > SAFE_LIMIT) {safeguardflag = true; return;}
  samplecnt++;
  //WAVETABLE GENERATION /
  uint16_t current_output = 0;
  for (int i=0; i<NUMVOICES; i++) {
    Voice * v = &edgar[i];
    uint8_t temp_env_amp = v->env_amp;
    if (temp_env_amp == 0) continue;
    // PHASE ACCUMULATION
    uint32_t temp_phase = v->phase;
    uint32_t temp_phase_inc = v->phase_inc;
    temp_phase += temp_phase_inc;
    v->phase = temp_phase;
    //WAVETABLE--just raw oscillation
    //v->raw = pgm_read_byte(&sinetable[edgar[i].phase >> 24]);
    uint8_t idx = v->phase >> 24;
    uint8_t raw_sample = pgm_read_byte(v->wavetable + idx);
    v->raw = raw_sample;
    //WAVETABLE--envelope-scaled oscillation
    uint16_t temp_amp = (uint16_t)raw_sample * (uint16_t)temp_env_amp;
    v->amp = temp_amp >> 8;
    current_output += v->amp;
  }//for
  
  //VOICE MIXING
  OCR2A = (uint8_t)(current_output >> 2); //divide by NUMVOICES = 4
  
  //VOICE HANDLING DONE BY MIDI ISR

  //Update ENVELOPE
  if ((samplecnt & 31) == 0) envupdateflag = true; //run every 32 samples
  if ((samplecnt & 63) == 0) LEDflag = true; //run every 64 samples
} //ISR

/*ISR for MIDI
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

ISR(USART_RX_vect) {
  if (UCSR0A & _BV(DOR0)) { //in case of data overrun, reset MIDI input
    //digitalWrite(7, HIGH);
  }
  uint8_t byte = UDR0;
  //3 bytes: status, data (note and velocity)
  if (byte & 0x80){
    //status byte
    MIDIstatus = byte; MIDIbyte_index = 0;
  }
  else {
    //data byte
    if (MIDIbyte_index==0) {MIDInote = byte; MIDIbyte_index=1;}
    else if (MIDIbyte_index==1) {
      MIDIvelo = byte; midicompleteflag = true; MIDIbyte_index=0;}
  }
}

/*USER_DEFINED_FXS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

void generate_env_tables(EnvelopeStage stage, float lambda, float sustain, uint8_t * table) {
  if (stage == RELEASE){
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
            float val = 255.0f * expf(-lambda * i);
            if (val < 25) val = 0;
            table[i] = (uint8_t)val; 
    }
  }
  else if (stage == DECAY) {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      float e = expf(-lambda * i);
      float val = sustain + (1.0 - sustain) * e; 
      table[i] = (uint8_t)(255.0f * val); 
    }
  }
  else if (stage == ATTACK) {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      table[i] = (uint8_t) ((uint16_t)(i * i) / 255);
      //table[i] = (uint8_t)(255 * ( 1.0 * (float) pow((i / WAVETABLE_SIZE - 1), 2))); 
    }
  }
  else {return;}
}

void init_voicearray(Voice * edgar) {
  for (int i=0; i<NUMVOICES; i++) {
    edgar[i].pressed=false; edgar[i].amp=0; edgar[i].stage=IDLE; 
    edgar[i].name = DEADNAME; edgar[i].phase = 0; edgar[i].phase_inc= 0; edgar[i].envIndex = 0; edgar[i].f = 20; // 20 hz
    edgar[i].env_amp=0; edgar[i].wavetable = sinetable;
  } 
}//init_voicearray

void key_release(Voice * v) {
  cli(); v->pressed = false; sei();
}

void key_off(Voice * v) {
  cli(); v->stage = IDLE; v->name = DEADNAME; v->env_amp=0; sei();
}

void key_press(OscillatorType osc, unsigned long startcnt, float freq, Voice* v, int SAMPLE_RATE, int name) {
  cli();
  v->startcnt = startcnt; 
  v->f = freq; 
  v->stage = ATTACK;
  v->envIndex = 0;
  v->name = name;
  v->phase_inc = (unsigned long) ((float)v->f * (unsigned long) (1 << bits_phase_accumulator) / SAMPLE_RATE);
  v->phase = 0; 
  v-> pressed = true;
  v->a_inc = convertADSR(potA); v->d_inc = convertADSR(potD); v->r_inc = convertADSR(potR);
  switch (osc) {
        case SINE: v->wavetable = sinetable; break;
        case SQUARE: v->wavetable = squaretable; break;
        case SAW: v->wavetable = sawtable; break;
        case MOOG: v->wavetable = moogtable; break;
        case SHARK: v->wavetable = sharktable; break;
        case FORMANT: v->wavetable = formanttable; break;
        case MS20: v->wavetable = ms20table; break;
  }
  sei(); 
}

uint8_t convertADSR(uint8_t pot) {return 1 + (pot * (WAVETABLE_SIZE - 1)) / 1023;}

void incrementADSR_OLD(Voice* v) {
  //update envelope index & stages, handle overflow
  if (v->stage == ATTACK) {
    v->envIndex += v->a_inc;
    if (!v->pressed) {
      v->stage = RELEASE; v->envIndex = 0; v->amp_before_r = v->env_amp;
    }
    if (v->envIndex >= WAVETABLE_SIZE)
      {v->stage = DECAY; v->envIndex -= WAVETABLE_SIZE;}
  }
  else if (v->stage == DECAY) {
    v->envIndex += v->d_inc;
    if (v->envIndex >= WAVETABLE_SIZE)
      {
        v->envIndex -= WAVETABLE_SIZE;
        if (!v->pressed) {v->stage = RELEASE; cli(); v->envIndex = 0; sei(); v->amp_before_r = v->env_amp;}
        else {v->stage = SUSTAIN;}
      }
  }
  else if (v->stage == SUSTAIN) {
    if (!v->pressed) {v->stage = RELEASE; cli(); v->envIndex = 0; sei(); v->amp_before_r = v->env_amp;}
  }

  else if (v->stage == RELEASE) {
    v->envIndex += v->r_inc;
    if (v->envIndex >= WAVETABLE_SIZE) {key_off(v);}
  }

  //update envelope amplitude
  switch (v->stage) {
    case ATTACK: v->env_amp = Atable[v->envIndex]; break;
    case DECAY: v->env_amp = Dtable[v->envIndex]; break;
    case SUSTAIN: v->env_amp = 128; break; //keep amplitude constant
    case RELEASE: v->env_amp = (Rtable[v->envIndex] * v->amp_before_r) >> 8; break;
  }
}


void incrementADSR(Voice* v) {
  //if a key has been let go, enter release
  if (v->pressed == false && v->stage != IDLE && v->stage != RELEASE) {
    v->stage = RELEASE; v->amp_before_r = v->amp; cli(); v->envIndex = 0; sei();}
  //update envelopes & stages
  switch (v->stage) {
    case ATTACK: 
      v->envIndex += v->a_inc;
      if (v->envIndex >= WAVETABLE_SIZE)
        {v->stage = DECAY; cli(); v->envIndex = 0; sei();}
      v->env_amp = Atable[v->envIndex];
      break;
    case DECAY:
      v->envIndex += v->d_inc;
      if (v->envIndex >= WAVETABLE_SIZE)
        {v->stage = SUSTAIN; cli(); v->envIndex = 0; sei();}
      v->env_amp = Dtable[v->envIndex]; 
      break;
    case SUSTAIN: 
      v->env_amp = WAVETABLE_SIZE >> 2; 
      break;
    case RELEASE:
      v->envIndex += v->r_inc;
      if (v->envIndex >= WAVETABLE_SIZE) {key_off(v);}
      else {v->env_amp = (Rtable[v->envIndex] * v->amp_before_r) >> 8;}
      break;
  }
} //incrementADSR_NEW

float midiNoteToFreq(uint8_t note) {
	return pgm_read_float(&MIDIfreqtable[note]);
}
