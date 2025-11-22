#include <math.h>
#include <MIDI.h>



#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* POLYPHONIC LOGIC STRUCTURE:
ISR 1 (20 kHz):    update oscillators, write audio
ISR 2 (500 Hz):    update envelopes, LFOs
ISR 3:             parse MIDI
main loop:         do nothing (or UI only)
*/
const int freq = 10; //temp value of 10 hz for frequency
const int NUMVOICES = 4;
const int bits_phase_accumulator = 32;
const int samplerate = 1000; //1k Hz sample rate (for debug)
volatile unsigned long samplecnt = 0;
volatile int output = 0;
const int WAVETABLE_SIZE = 256;
typedef enum {OFF, ATTACK, DECAY, SUSTAIN, RELEASE} EnvelopeStage;
typedef enum {SINE, SQUARE, SAW, TRIANGLE} OscillatorType;

typedef struct {
  bool on; int name; uint8_t amp; EnvelopeStage stage; OscillatorType osc; 
  volatile uint32_t phase; volatile uint32_t phase_inc; unsigned long startcnt;
  volatile uint32_t f; //frequency
} Voice;

typedef struct {
  volatile bool active; int name;
} Input;

Voice edgar[NUMVOICES];
Input key {.name = 67; .active = false;}

/*WAVETABLES
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
uint8_t sinetable[WAVETABLE_SIZE];
uint8_t squaretable[WAVETABLE_SIZE];
uint8_t sawtable[WAVETABLE_SIZE];
uint8_t triangletable[WAVETABLE_SIZE];

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

/*FUNCTION DECLARATIONS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

void generate_wavetables();
void init_voicearray(Voice *);
void key_off(Voice);
void key_on(OscillatorType osc, unsigned long startcnt, int freq, Voice v);
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


void setup() {
  //Serial.begin(31250); <- for MIDI
  Serial.begin(115200);
  // initialize voices:
  init_voicearray(edgar);

    // Print first 16 values for verification
  generate_wavetables();
  Serial.println("SINE:"); for (int i=0;i<255;i++) Serial.print((int)sinetable[i]);
  Serial.println("SQUARE:"); for (int i=0;i<255;i++) Serial.print((int)squaretable[i]);
  Serial.println("SAW:"); for (int i=0;i<255;i++) Serial.print((int)sawtable[i]);
  Serial.println("TRIANGLE:"); for (int i=0;i<255;i++) Serial.print((int)triangletable[i]);
} //setup

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
      parse_int  = Serial.parseInt();
      if (parse_int <= 1 || parse_int >= 8) {
        Serial.println("error! input must be between 1-8.");
        key.active = false;
      }
      else {key.active = true; key.name = parse_int;}
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

    if sample_counter % 4 == 0:
         update envelopes

    if sample_counter % 40 == 0:
         update LFOs

    if sample_counter % 200 == 0:
         housekeeping, voice cleanup, etc.
  */
  samplecnt++;
  //WAVETABLE GENERATION /
  for (int i=0; i<NUMVOICES; i++) {
    if (edgar[i].stage != OFF) {
      // PHASE ACCUMULATION
      edgar[i].phase += edgar[i].phase_inc;
      //WAVETABLE
      switch (edgar[i].osc) {
        case SINE: edgar[i].amp = sinetable[edgar[i].phase >> 24]; break;
        case SQUARE: edgar[i].amp = squaretable[edgar[i].phase >> 24]; break;
        case SAW: edgar[i].amp = sawtable[edgar[i].phase >> 24]; break;
        case TRIANGLE: edgar[i].amp = triangletable[edgar[i].phase >> 24]; break;
      } 
    else edgar[i].amp = 0;
    }
  }//for
  //VOICE MIXING
  output = 0;
  for (int i=0; i < NUMVOICES; i++) {output += edgar[i].amp / 4;}
  //UPDATE ENVELOPES

  //VOICE HANDLING
  if ((samplecnt & 15) == 0) {
    if (key.active) {
      unsigned long oldest_start = UINT32_MAX;; int i_oldest = 0; bool foundvoice = false;
      for (int i=0; i<NUMVOICES; i++) {
        //if a key is pressed a second time, turn it off
        if (edgar[i].name == key.name) {key_off(edgar[i]); foundvoice = true; break;}
        //else, if a key i pressed for the first time, find a place for it
        //find any off voices
        if (edgar[i].on == false) {key_on(SINE, samplecnt, freq, edgar[i]); foundvoice=true; break;}
        if (edgar[i].startcnt < oldest_start) {oldest_start = edgar[i].startcnt; i_oldest = i;}
      }
      //if no off voices found, replace the oldest
      if (foundvoice==false) {key_on(SINE, samplecnt, freq, edgar[i_oldest]);}
      key.active = false;
    }
  }
  
}



/*USER_DEFINED_FXS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
void generate_wavetables(OscillatorType osc, uint8_t * table) {
  Serial.print("\n\n");
  if (osc == SINE) {
   Serial.print("SINE TABLE:\n");
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
    // Generate sine between 0 256
    table[i] = (uint8_t)(127.5 + 127.5 * sin(2.0 * PI * i / WAVETABLE_SIZE)); Serial.println((int)table[i]);} 
  }
  else if (osc == SQUARE) {
    Serial.print("SQUARE TABLE:\n");
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
    table[i] = (uint8_t)((sin(2.0 * PI * i / WAVETABLE_SIZE) >= 0) ? 255 : 0); Serial.println((int)table[i]);}
  }
  else if (osc == SAW) {
    Serial.print("SAW TABLE:\n");
    for (int i = 0; i < WAVETABLE_SIZE; i++) {table[i] = (uint8_t) i; Serial.println((int)table[i]);}
  }
  else if (osc == TRIANGLE) {
    Serial.print("TRIANGLE TABLE:\n");
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      if (i < WAVETABLE_SIZE / 2) {table[i] = (uint8_t)(i * 2);} // rising slope}
      else {table[i] = (uint8_t)((WAVETABLE_SIZE - 1 - i) * 2);} // falling slope}
      Serial.println((int)table[i]);
    }
  }
} //generate_wavetables

void init_voicearray(Voice * edgar) {
  for (int i=0; i<NUMVOICES; i++) {edgar[i].on=false; edgar[i].amp=0; edgar[i].stage=OFF; edgar[i].osc=SINE; 
  edgar[i].name = 67; edgar[i].phase = 0; edgar[i].phase_inc= 0; edgar[i].f = 20; // 20 hz
} //init_voicearray

void generate_wavetables() {
  for (int i = 0; i < WAVETABLE_SIZE; i++) {
    // ---- SINE ----
    sinetable[i] = (uint8_t)(127.5 + 127.5 * sin(2.0 * PI * i / WAVETABLE_SIZE));

    // ---- SQUARE ----
    squaretable[i] = (sin(2.0 * PI * i / WAVETABLE_SIZE) >= 0) ? 255 : 0;

    // ---- SAW ----
    sawtable[i] = (uint8_t)i; // 0 -> 255 linearly

    // ---- TRIANGLE ----
    if (i < WAVETABLE_SIZE / 2)
      triangletable[i] = (uint8_t)(i * 2);       // rising
    else
      triangletable[i] = (uint8_t)((WAVETABLE_SIZE - 1 - i) * 2); // falling
  }
}

void key_off(Voice v) {
  v->name = 67; v->stage = RELEASE; v->on = false;
}

void key_on(OscillatorType osc, unsigned long startcnt, int freq, Voice v) {
  v->startcnt = startcnt; v->f = freq; v->stage = ATTACK; v->osc = osc;
}

