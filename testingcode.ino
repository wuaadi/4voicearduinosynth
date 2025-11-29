/*
test code for LAZULUM--test different functions with serial, breadboard, etc.
*/

#include <math.h>
#include <stdio.h>
#include <MIDI.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif //not necessrary for synth

/* POLYPHONIC LOGIC STRUCTURE:
ISR (20 kHz):    update oscillators, write audio
                 send flags to update envelopes, LFOs, voice allocation

main loop:       update envelopes & voice arr, handle serial i/o 
*/
//ADSR params
int potA, potD, potR; //potentiometer values
const int freq = 10; //temp value of 10 hz for frequency
const int NUMVOICES = 4;
const int bits_phase_accumulator = 32;
const int Fs = 20000; //20k Hz sample rate
volatile unsigned long samplecnt = 0;
volatile bool voicemixflag, envupdateflag, printflag = false;
uint16_t output = 0; //0-255
int output_voltage; //0-5 volts
const int WAVETABLE_SIZE = 256;
typedef enum {OFF, ATTACK, DECAY, SUSTAIN, RELEASE} EnvelopeStage;
typedef enum {SINE, SQUARE, SAW, TRIANGLE} OscillatorType;

typedef struct {
  bool on; int name; uint16_t amp; uint8_t raw; uint8_t env_amp;
  EnvelopeStage stage; OscillatorType osc; 
  uint32_t phase; uint32_t phase_inc; unsigned long startcnt;
  uint16_t envIndex; uint32_t f; //frequency
  uint8_t a_inc, d_inc, r_inc; //attack,decay,release increments
  uint8_t amp_before_r;
} Voice;

typedef struct {
  volatile bool active; volatile int name;
} Input;

Voice edgar[NUMVOICES];
Input key {false, 67};

/*WAVETABLES
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
const uint8_t sinetable[256] PROGMEM = {
128,131,134,137,140,143,146,149,152,156,159,162,165,168,171,174,177,180,183,186,188,191,194,197,199,202,204,207,209,211,214,216,
218,220,222,224,226,227,229,231,232,234,235,236,238,239,240,241,242,243,244,244,245,245,246,246,246,246,246,246,246,245,245,244,
244,243,242,241,240,239,238,236,235,234,232,231,229,227,226,224,222,220,218,216,214,211,209,207,204,202,199,197,194,191,188,186,
183,180,177,174,171,168,165,162,159,156,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103, 99, 96, 93, 90, 87,
84, 81, 78, 75, 73, 70, 67, 64, 62, 59, 57, 54, 52, 50, 47, 45,43, 41, 39, 37, 35, 34, 32, 30, 29, 27, 26, 25, 23, 22, 21, 20,
19, 18, 17, 17, 16, 16, 15, 15, 15, 15, 15, 15, 15, 16, 16, 17,17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 29, 30, 32, 34, 35, 37,
39, 41, 43, 45, 47, 50, 52, 54, 57, 59, 62, 64, 67, 70, 73, 75,78, 81, 84, 87, 90, 93, 96, 99,103,106,109,112,115,118,121,124
};

const uint8_t squaretable[256] PROGMEM = {
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
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
//asdr tables
uint8_t Atable[WAVETABLE_SIZE];
uint8_t Dtable[WAVETABLE_SIZE];
uint8_t Rtable[WAVETABLE_SIZE];

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

/*FUNCTION DECLARATIONS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

//void generate_wavetables();
void generate_env_tables(EnvelopeStage stage, float lambda, float sustain, uint8_t * table);
void init_voicearray(Voice *);
void key_off(Voice *);
void key_on(OscillatorType osc, unsigned long startcnt, int freq, Voice* v, int Fs, int name);
uint8_t convertADSR(int);
void incrementADSR(Voice* v);
void printSerial(Voice* v);
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


void setup() {
  //PWM and GATE outputs
  pinMode(11, OUTPUT); //Audio output
  pinMode(10, OUTPUT); //light or sm idk
  //timer setup
  cli();
  TCCR1A = 0; // Normal operation 
  TCCR1B = 0; TCNT1 = 0; 
  OCR1A = 799; // For 20kHz: 16MHz / (20,000 * 1) - 1 = 799 
  TCCR1B |= (1 << WGM12); // CTC mode 
  TCCR1B |= (1 << CS10); // No prescaler 
  TIMSK1 |= (1 << OCIE1A); // Enable compare interrupt
  //timer 2 setup, for PWM
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);  
  TCCR2B = _BV(CS20);  // No prescaler
  
  Serial.println("set up timers");
  //Serial.begin(31250); <- for MIDI
  Serial.begin(115200);
  // initialize voices:
  init_voicearray(edgar);

  float lambda_decay = 0.02f;
  float lambda_release = 0.01f;
  float sustain = 0.5;
  Serial.println("\nATTACK TABLE:"); generate_env_tables(ATTACK, 0, sustain, Atable);
  Serial.println("\nDECAY TABLE"); generate_env_tables(DECAY, lambda_decay, sustain, Dtable);
  Serial.println("\nRELEASE TABLE "); generate_env_tables(RELEASE, lambda_release, sustain, Rtable);
  sei();
} //setup

void loop() {
  // put your main code here, to run repeatedly:
  //input logic
  if (Serial.available()) {
      int parse_int  = Serial.parseInt();
      if (parse_int < 1 || parse_int > 8) {
        Serial.println("error! input must be between 1-8.");
        key.active = false; voicemixflag = false;
      }
      else {key.active = true; key.name = parse_int; Serial.print("Key inputted: "); Serial.println(key.name);}
  }
  //voice allocation logic
  if (voicemixflag) {
    unsigned long oldest_start = UINT32_MAX;; int i_oldest = 0; bool foundvoice = false;
      for (int i=0; i<NUMVOICES; i++) {
        //if a key is pressed a second time, turn it off
        if (edgar[i].name == key.name) {key_off(&edgar[i]); foundvoice = true; break;}
        //else, if a key i pressed for the first time, find a place for it
        //find any off voices
        if (edgar[i].on == false) {key_on(SINE, samplecnt, freq, &edgar[i], Fs, key.name); foundvoice=true; break;}
        if (edgar[i].startcnt < oldest_start) {oldest_start = edgar[i].startcnt; i_oldest = i;}
      }
      //if no off voices found, replace the oldest
      if (foundvoice==false) {key_on(SINE, samplecnt, freq, &edgar[i_oldest], Fs, key.name);}
      key.active = false; //reset input event flag
      voicemixflag = false; //reset ISR->voice allocation flag
  }
  //envelope update logic
  if (envupdateflag) {
    //increment thru ADR wavetables
    for (int i=0; i<NUMVOICES; i++) {
      incrementADSR(&edgar[i]); //updates envIndex, stage, and env_amp
    }
    envupdateflag = false;
  }

  if (printflag) {
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
    Serial.println("ON VOICES:");
    for (int i=0; i<NUMVOICES; i++) {
      if (edgar[i].on) {
        Serial.println((String) "     name: "+ edgar[i].name + " stage: "+ edgar[i].stage +" amp: "+ edgar[i].env_amp);
      }
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * * * * * * * ");
    Serial.println("OFF VOICES:");
    for (int i=0; i<NUMVOICES; i++) {
      if (!edgar[i].on) {
        Serial.println((String) "     name: "+ edgar[i].name + " stage: "+ edgar[i].stage +" pos: "+ i);
      }
    }
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="); Serial.println();

    printflag = false;
  }


} //loop()

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


ISR(TIMER1_COMPA_vect) {
  samplecnt++;
  //WAVETABLE GENERATION /
  for (int i=0; i<NUMVOICES; i++) {
    if (edgar[i].stage != OFF) {
      // PHASE ACCUMULATION
      edgar[i].phase += edgar[i].phase_inc;
      //WAVETABLE--just raw oscillation
      switch (edgar[i].osc) {
        case SINE: edgar[i].raw = pgm_read_byte(&sinetable[edgar[i].phase >> 24]); break;
        case SQUARE: edgar[i].raw = pgm_read_byte(&squaretable[edgar[i].phase >> 24]); break;
        case SAW: edgar[i].raw = pgm_read_byte(&sawtable[edgar[i].phase >> 24]); break;
        case TRIANGLE: edgar[i].raw = pgm_read_byte(&triangletable[edgar[i].phase >> 24]); break;
      }
      //WAVETABLE--envelope-scaled oscillation
      edgar[i].amp = (uint16_t)edgar[i].raw * (uint16_t)edgar[i].env_amp >> 8; //
    }
    else {edgar[i].raw =0; edgar[i].amp = 0;}
  }//for
  //VOICE MIXING
  output = 0;
  for (int i = 0; i < NUMVOICES; i++) output += edgar[i].amp;
  output /= NUMVOICES; // Normalize mixed output
  OCR2A = output;

  //VOICE HANDLING
  if ((samplecnt & 15) == 0)
    if (key.active) voicemixflag = true;

  //Update ENVELOPE
  if ((samplecnt & 39) == 0) envupdateflag = true; //run every 40 samples

  //print for debugging
  if ((samplecnt & 59999) == 0) printflag = true;
} //ISR



/*USER_DEFINED_FXS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

void generate_env_tables(EnvelopeStage stage, float lambda, float sustain, uint8_t * table) {
  if (stage == RELEASE){
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
            float val = 255.0f * expf(-lambda * i);
            if (val < 25) val = 0;
            table[i] = (uint8_t)val; Serial.print(table[i]); Serial.print(",");
    }
  }
  else if (stage == DECAY) {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      float e = expf(-lambda * i);
      float val = sustain + (1.0 - sustain) * e; 
      table[i] = (uint8_t)(255.0f * val); Serial.print(table[i]); Serial.print(",");
    }
  }
  else if (stage == ATTACK) {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
      table[i] = (uint8_t) ((uint16_t)(i * i) / 255);
      //table[i] = (uint8_t)(255 * ( 1.0 * (float) pow((i / WAVETABLE_SIZE - 1), 2))); 
      Serial.print(table[i]); Serial.print(",");
    }
  }
  else {Serial.println("error: choose an envelope stage."); return;}
}

void init_voicearray(Voice * edgar) {
  for (int i=0; i<NUMVOICES; i++) {
    edgar[i].on=false; edgar[i].amp=0; edgar[i].stage=OFF; edgar[i].osc=SINE; 
    edgar[i].name = 67; edgar[i].phase = 0; edgar[i].phase_inc= 0; edgar[i].envIndex = 0; edgar[i].f = 20; // 20 hz
  } 
}//init_voicearray

void key_off(Voice * v) {
  v->name = 67; v->on = false;
}

void key_on(OscillatorType osc, unsigned long startcnt, int freq, Voice* v, int Fs, int name) {
  v->startcnt = startcnt; v->f = freq; v->stage = ATTACK; v->osc = osc;
  v->envIndex = 0; v->name = name;
  v->phase_inc = (unsigned long) ((double)v->f * (unsigned long) (1 << bits_phase_accumulator) / Fs);
  v->phase = 0; v-> on = true;
  v->a_inc = convertADSR(potA); v->d_inc = convertADSR(potD); v->r_inc = convertADSR(potR);
}

uint8_t convertADSR(int pot) {return 1 + (pot * (WAVETABLE_SIZE - 1)) / 1023;}

void incrementADSR(Voice* v) {
  //update envelope index & stages, handle overflow
  if (v->stage == ATTACK) {
    v->envIndex += v->a_inc;
    if (v->envIndex >= WAVETABLE_SIZE)
      {v->stage = DECAY; v->envIndex -= WAVETABLE_SIZE;}
    //if (!v->on) {v->stage = RELEASE; v->envIndex = 0; v->amp_before_r = v->env_amp;}
  }
  else if (v->stage == DECAY) {
    v->envIndex += v->d_inc;
    if (v->envIndex >= WAVETABLE_SIZE)
      {
        v->envIndex -= WAVETABLE_SIZE;
        if (!v->on) {v->stage = RELEASE; v->envIndex = 0; v->amp_before_r = v->env_amp;}
        else {v->stage = SUSTAIN;}
      }
  }
  else if (v->stage == SUSTAIN) {
    if (!v->on) {v->stage = RELEASE; v->envIndex = 0; v->amp_before_r = v->env_amp;}
  }

  else if (v->stage == RELEASE) {
    v->envIndex += v->r_inc;
    if (v->envIndex >= WAVETABLE_SIZE) {v->stage = OFF; v->envIndex = 0;}
  }

  //update envelope amplitude
  switch (v->stage) {
    case ATTACK: v->env_amp = Atable[v->envIndex]; break;
    case DECAY: v->env_amp = Dtable[v->envIndex]; break;
    case SUSTAIN: break; //keep amplitude constant
    case RELEASE: v->env_amp = (Rtable[v->envIndex] * v->amp_before_r) >> 8; break;
  }
}
