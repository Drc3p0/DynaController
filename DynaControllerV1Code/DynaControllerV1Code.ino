/* Piezo drums are disabled but could be implemented if wanted. 
   Uses a Piezo sensor (percussion / drum) to send USB MIDI note on
   messages, where the "velocity" represents how hard the Piezo was
   tapped.

   How Piezos are wired: 
   Connect a Piezo sensor to analog pin.  This example was tested
   with Murata 7BB-27-4L0.  Almost any piezo sensor (not a buzzer with
   built-in oscillator electronics) may be used.  However, Piezo
   sensors are easily damaged by excessive heat if soldering.  It
   is highly recommended to buy a Piezo with wires already attached!

   Use a 100K resistor between A0 to GND, to give the sensor a "load".
   The value of this resistor determines how "sensitive" the circuit is.

   A pair of 1N4148 diodes are recommended to protect the analog pin.
   The first diode connects to A0 with its stripe (cathode) and the other
   side to GND.  The other diode connects its non-stripe (anode) side to
   A0, and its stripe (cathode) side to 3.3V.

   Sensitivity may also be tuned with the map() function.  Uncomment
   the Serial.print lines to see the actual analog measurements in the
   Arduino Serial Monitor.

   You must select MIDI from the "Tools > USB Type" menu

This circuit uses a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required
to interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution
**********************************************************/

//This revision is modified for the Dynacontroller V1 PCB, which uses Teensy 4.0 instead of 3.2. Some pinouts are different. 
//Piezo Drum stuff not used, but still in the code (the main controller has piezos installed and hardware for tuning, but it's not implemented. 
//all control is through MPR121 touchpoints)
// Make sure to cut the ADDR traces on the back of the second MPR121 breakout board, and pull the addr pin to high/3.3V to set it to 0x5B
//helpful links: teensy wire library: https://www.pjrc.com/teensy/td_libs_Wire.html
// teensy midi library: https://www.pjrc.com/teensy/td_midi.html

//Drum stuff
int state[] = {0,0,0,0,0,0}; // 0=idle, 1=looking for peak, 2=ignore aftershocks
int peak[] = {0,0,0,0,0,0};    // remember the highest reading
elapsedMillis msec[] = {0,0,0,0,0,0}; // timer to end states 1 and 2
const int numDrumPins = 6;
const int drumNotes[] = {38,39,40,41,42,43};
const int drumPins[] = {A0, A1, A2, A7, A8, A9};
const int channel = 1;
const int thresholdMin = 12;  // minimum reading, avoid "noise"
const int aftershockMillis = 60; // time of aftershocks & vibration


#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Ticker.h>
#include <FastLED.h>

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif
//This is the duration between sends of midi signal in Milliseconds.
#define TRIGGER_DURATION 30

#define USE_CC_TIMER 0
#define USE_NOTE_ON_OFF 1

//currently using MIDI out, but might use keystrokes in the future. A toggle pin with common ground between is added to 22 to represent MIDI, and 23 to represent Keys. if implemented, use jumper to indicate what output is preferred (midi or keys)

// LED Stuff
#define DATA_PIN    8 //used to be pin 3 on teensy3.2
// clock pin wired to pin 7, but not implemented in code
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
// current leds are basically 20 per large petal and 15 per small one. so 20x6 + 15x6 = 210
#define NUM_LEDS    194
CRGB leds[NUM_LEDS];
#define BRIGHTNESS          80
#define FRAMES_PER_SECOND  100
#define LED_DECAY 5000 // leds will decay in brightness for 5 seconds before going dark
#define AMBIENT_DELAY 4000 // leds will start doing something after the keys are untouched for this duration.

#define NUM_PADS 21

bool ambient_leds = true;
bool trigger_leds = false;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns from Demo Reel 100 example

 CRGBPalette16 custom_palette_1 =
    { 0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33,
      0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF };

 CRGBPalette16 custom_palette_2 = //first variable has to be black or the colors wont continue while holding?
    {
      CRGB::Black, CRGB::Maroon, CRGB::Maroon, CRGB::DarkViolet, CRGB::Maroon, CRGB::Maroon, CRGB::DarkRed, CRGB::Maroon,
      CRGB::Maroon, CRGB::Maroon, CRGB::DarkRed, CRGB::Maroon, CRGB::Maroon, CRGB::OrangeRed, CRGB::Maroon, CRGB::Maroon
    };

     CRGBPalette16 black_palette =
    {
      CRGB::Black,CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black,
      CRGB::Black,CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black
    };

  CRGBPalette16 currentPalette( black_palette);
  CRGBPalette16 targetPalette( OceanColors_p );

// this is a set of group coordinates, one coordinate per group.  one group per zone of the controller
//.  Then each led will be given a group number. the coordinate of the group number will determine how bright group is.
// for now we will start small but the groups can be as granular as needed by increasing the array side up to the led count.

typedef struct {
    uint8_t index;
    uint16_t x;
    uint16_t y;
} CoordinatePair;

typedef struct {
    int pin_number;
    uint16_t x;
    uint16_t y;
} PadLocation;


void triggerLoop(); // hoisted, defined below.
void ledFrameLoop();
void startAmbient();
void stopPiezo();
uint8_t lightClosestLEDWithIndex(CRGB* leds,  uint16_t numLeds, const PadLocation* padCoords, CRGB color = CRGB::White);

// Ticker Library sets up timers and a function to call when the timer elapses
           //(functioncalled, timertime, number to repeat(0is forever, RESOLUTION)
Ticker repeatTimer(triggerLoop, TRIGGER_DURATION, 0, MILLIS);
Ticker ledFrameTimer(ledFrameLoop, 1000/FRAMES_PER_SECOND, 0, MILLIS);
Ticker ambientLEDs(startAmbient, AMBIENT_DELAY, 0 , MILLIS);
Ticker PiezoEffect(stopPiezo, 1000, 0 , MILLIS);

#define MPR121_TOUCH_THRESHOLD_DEFAULT 10  ///< default touch threshold value (was 12)
#define MPR121_RELEASE_THRESHOLD_DEFAULT 3 ///< default relese threshold value (was 6)
// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 capA = Adafruit_MPR121();
Adafruit_MPR121 capB = Adafruit_MPR121();
// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouchedA = 0;
uint16_t currtouchedA = 0;
uint16_t lasttouchedB = 0;
uint16_t currtouchedB = 0;

uint8_t touched_pad = 1;
const uint8_t numElectrodes = 12; //added by drc
//MPR121 MIDI output values
const uint8_t controlNumA[] = {88,88,87,87,13,13,5,5,0,0,3,3}; //Change to #'s stefan is using. 88, 87, 13, 5, 0, 3, 50 is mode shift
const uint8_t controlNumB[] = {28,29,30,31,20,21,22,23,24,25,26,27}; //26 is triggering weird.  removing it mechanically for now. This wire runs too close to the LEDs, resulting in occasional self-triggering.
const uint8_t controlValA[] = {127,0,127,0,127,0,127,0,127,0,127,0}; //Change to #'s stefan is using
const uint8_t controlValB[] = {127,127,127,127,127,127,127,127,127,127,127,127}; //Change to #'s stefan is using
uint8_t ElectrodeTouchedA[numElectrodes] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t ElectrodeTouchedB[numElectrodes] = {0,0,0,0,0,0,0,0,0,0,0,0};

//old MIDI output notes ?? can  delete?
//const uint8_t notesA[numElectrodes] = {58, 50, 48, 60, 49, 57, 46, 52, 36, 38, 55, 51}; //mapped to old large controller midi mapping
  const uint8_t notesA[numElectrodes] = {24, 36, 48, 45, 33, 57, 60, 55, 53, 41, 43, 31}; //remapped for dynacontroller 
//const uint8_t notesB[numElectrodes] = {1, 1, 1, 1, 43, 53, 40, 45, 54, 47, 56, 1}; //mapped to old large controller midi mapping
  const uint8_t notesB[numElectrodes] = {1, 2, 3, 4, 40, 29, 52, 50, 28, 38, 26, 5}; //remapped for dynacontroller
// add a map of pins to location in xy coordinates with each led
// add functiono for how close a xy vector is to the location of the pad in xy space
// peak brightness on leds closest to pressed pads



// function determines how close the pad is to the leds. ramp down brightness by closeness factor
// Initialize array of pin locations
PadLocation padCoords[NUM_PADS] = {
    {0, 388, 485},
    {1, 377, 597},
    {2, 473, 435},
    {3, 559, 501},
    {4, 472, 356},
    {5, 557, 311},
    {6, 401, 320},
    {7, 410, 195},
    {8, 304, 426},
    {9, 194, 486},
    {10, 311, 348},
    {11, 233, 284},
    {12,0,0}, // no 12 trigger
    {13,0,0}, // no 13 trigger
    {14, 394, 388},
    {15, 113, 364},
    {16, 230, 612},
    {17, 519, 652},
   {18, 670, 408},
    {19, 534, 176},
    {20, 278, 153}
};



// Initialize the array of coordinate pairs
CoordinatePair LEDcoordinates[NUM_LEDS] = {

    {0, 417, 157},
    {1, 440, 147},
    {2, 454, 134},
    {3, 475, 121},
    {4, 499, 115},
    {5, 518, 109},
    {6, 540, 102},
    {7, 562, 100},
    {8, 580, 96},
    {9, 602, 106},
    {10, 602, 125},
    {11, 606, 146},
    {12, 609, 161},
    {13, 610, 179},
    {14, 614, 195},
    {15, 620, 214},
    {16, 613, 232},
    {17, 602, 241},
    {18, 592, 260},
    {19, 610, 298},
    {20, 635, 310},
    {21, 663, 321},
    {22, 679, 331},
    {23, 695, 351},
    {24, 714, 364},
    {25, 721, 373},
    {26, 731, 382},
    {27, 739, 392},
    {28, 753, 405},
    {29, 753, 433},
    {30, 743, 442},
    {31, 731, 456},
    {32, 713, 469},
    {33, 701, 482},
    {34, 688, 491},
    {35, 667, 500},
    {36, 649, 509},
    {37, 617, 515},
    {38, 593, 545},
    {39, 590, 568},
    {40, 598, 594},
    {41, 598, 616},
    {42, 595, 634},
    {43, 591, 649},
    {44, 588, 669},
    {45, 581, 687},
    {46, 573, 708},
    {47, 576, 736},
    {48, 545, 745},
    {49, 518, 733},
    {50, 490, 725},
    {51, 467, 721},
    {52, 443, 710},
    {53, 425, 702},
    {54, 410, 682},
    {55, 400, 668},
    {56, 389, 654},
    {57, 381, 645},
    {58, 359, 641},
    {59, 349, 650},
    {60, 340, 658},
    {61, 326, 667},
    {62, 307, 680},
    {63, 291, 691},
    {64, 265, 696},
    {65, 233, 699},
    {66, 214, 707},
    {67, 190, 712},
    {68, 161, 700},
    {69, 158, 680},
    {70, 153, 662},
    {71, 150, 641},
    {72, 145, 624},
    {73, 146, 609},
    {74, 142, 591},
    {75, 145, 572},
    {76, 153, 560},
    {77, 167, 534},
    {78, 152, 495},
    {79, 142, 485},
    {80, 126, 476},
    {81, 107, 473},
    {82, 78, 459},
    {83, 60, 439},
    {84, 42, 418},
    {85, 30, 396},
    {86, 12, 367},
    {87, 29, 347},
    {88, 48, 332},
    {89, 61, 321},
    {90, 80, 309},
    {91, 95, 298},
    {92, 105, 284},
    {93, 125, 278},
    {94, 145, 274},
    {95, 167, 271},
    {96, 193, 251},
    {97, 190, 233},
    {98, 190, 217},
    {99, 188, 199},
    {100, 187, 180},
    {101, 187, 168},
    {102, 194, 148},
    {103, 201, 125},
    {104, 212, 101},
    {105, 219, 79},
    {106, 244, 65},
    {107, 259, 75},
    {108, 277, 81},
    {109, 297, 85},
    {110, 315, 93},
    {111, 337, 102},
    {112, 352, 113},
    {113, 363, 125},
    {114, 383, 144},
    {115, 214, 403},
    {116, 194, 416},
    {117, 174, 428},
    {118, 161, 454},
    {119, 154, 476},
    {120, 143, 498},
    {121, 158, 525},
    {122, 192, 536},
    {123, 221, 540},
    {124, 253, 533},
    {125, 286, 522},
    {126, 299, 510},
    {127, 313, 544},
    {128, 313, 562},
    {129, 313, 584},
    {130, 319, 600},
    {131, 328, 615},
    {132, 336, 628},
    {133, 338, 640},
    {134, 356, 653},
    {135, 372, 656},
    {136, 409, 635},
    {137, 424, 616},
    {138, 442, 585},
    {139, 445, 550},
    {140, 460, 528},
    {141, 474, 542},
    {142, 495, 550},
    {143, 511, 551},
    {144, 538, 551},
    {145, 555, 556},
    {146, 584, 550},
    {147, 597, 554},
    {148, 610, 528},
    {149, 611, 510},
    {150, 599, 495},
    {151, 596, 477},
    {152, 583, 454},
    {153, 558, 424},
    {154, 550, 398},
    {155, 568, 388},
    {156, 588, 368},
    {157, 598, 350},
    {158, 610, 330},
    {159, 619, 316},
    {160, 624, 295},
    {161, 619, 273},
    {162, 592, 269},
    {163, 571, 261},
    {164, 550, 256},
    {165, 527, 255},
    {166, 502, 264},
    {167, 463, 249},
    {168, 468, 228},
    {169, 465, 211},
    {170, 457, 189},
    {171, 445, 179},
    {172, 437, 166},
    {173, 423, 150},
    {174, 421, 142},
    {175, 395, 139},
    {176, 378, 159},
    {177, 366, 167},
    {178, 354, 175},
    {179, 342, 188},
    {180, 335, 201},
    {181, 326, 223},
    {182, 302, 247},
    {183, 283, 242},
    {184, 262, 237},
    {185, 240, 232},
    {186, 219, 238},
    {187, 205, 243},
    {188, 177, 249},
    {189, 169, 269},
    {190, 172, 304},
    {191, 171, 323},
    {192, 168, 343},
    {193, 183, 372}
};

const int NUM_GROUPS = 20;
const int ELEMENTS_PER_ARRAY = NUM_LEDS;

uint8_t calculateGroup(uint8_t elementNumber) {
  // Ensure elementNumber is within valid range
  if (elementNumber < 0 || elementNumber >= ELEMENTS_PER_ARRAY) {
    return -1;  // Invalid input
  }

  // Calculate the group number
  return (elementNumber * NUM_GROUPS / ELEMENTS_PER_ARRAY) + 1;
}

void setup() {

  Wire.begin(); //added by drc
  //Wire.begin(0x5B); //added by drc

  Serial.begin(9600); //uncomment to enable serial monitoring
  pinMode(LED_BUILTIN, OUTPUT); //added by drc
  Wire.setSDA(18); //use a 4.7k pullup resistor. same for 3.2 or 4.0
  Wire.setSCL(19); //use a 4.7k pullup resistor. same for 3.2 or 4.0
  Serial.println("Adafruit MPR121 Capacitive Touch sensor test");
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!capA.begin(0x5A)) {
    Serial.println("MPR121 0x5A not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 0x5A found!");

 if (!capB.begin(0x5B)) {
    Serial.println("MPR121 0x5B not found, check wiring?");
   while (1);
  }
  Serial.println("MPR121 0x5B found!");

  //from drum code.  not sure if its even necessary
  while (!Serial && millis() < 2500) /* wait for serial monitor */ ;
  Serial.println("Piezo Peak Capture");

  if (USE_CC_TIMER)
  {
    repeatTimer.start();
  }

  ledFrameTimer.start();

    // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {

  //drum stuff
  while (usbMIDI.read()) { } // ignore incoming messages??

  // led frame render timer
  ledFrameTimer.update();

  // turn on ambient LEDs timer
  ambientLEDs.update();

  // Get the currently touched pads
  currtouchedA = capA.touched();
  currtouchedB = capB.touched();

  if (USE_CC_TIMER) {
    repeatTimer.update();
  }
  // timer is runnig as long as this is getting called.
  checkElectrodes();

  FastLED.show();
}

void checkElectrodes(){

  for (uint8_t i=0; i<numElectrodes; i++) { //changed i<0 to i<numElectrodes
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouchedA & _BV(i)) && !(lasttouchedA & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, HIGH); //added by drc
      Serial.print(i); Serial.println(" touched of A");
      //Serial.println(currtouchedA);
      // set the array value to 1 on touch
      ElectrodeTouchedA[i] = 1;
      if (USE_NOTE_ON_OFF){
        usbMIDI.sendNoteOn(notesA[i], 127, channel); // note, velocity, channel
      }
      // on touch, turn off ambient leds and turn on trigger_leds
      touched_pad = i; 
      ambient_leds = false;
      trigger_leds = true;
      // assigns palette back to black for fade in later.
      currentPalette = black_palette;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouchedA & _BV(i)) && (lasttouchedA & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, LOW);
      Serial.print(i); Serial.println(" released of A");
      // set it back to 0 on release
      ElectrodeTouchedA[i] = 0;
      if (USE_NOTE_ON_OFF){
        usbMIDI.sendNoteOff(notesA[i], 127, channel); // note, velocity, channel
      }
      trigger_leds = false;
      ambient_leds = false;
      ambientLEDs.start();
      //start ambient w black


    }

    //For mpr121 0x5B--------------------
    // it if *is* touched and *wasnt* touched before, alert!

    if ((currtouchedB & _BV(i)) && !(lasttouchedB & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, HIGH); //added by drc
      Serial.print(i); Serial.println(" touched of B");
      // set the array value to 1 on touch
      ElectrodeTouchedB[i] = 1;
      if (USE_NOTE_ON_OFF){
        usbMIDI.sendNoteOn(notesB[i], 127, channel); // note, velocity, channel
      }
      // this should be offset by the max pin from the A set.
      //  if we have 19 active pins then the one last touched from this group should be group 1 plus i
      touched_pad = (10 + i);      // on touch, turn off ambient leds and turn on trigger_leds
			ambient_leds = false;
      trigger_leds = true;
            // assigns palette back to black for fade in later.
      currentPalette = black_palette;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouchedB & _BV(i)) && (lasttouchedB & _BV(i)) ) {
      digitalWrite (LED_BUILTIN, LOW);
      Serial.print(i); Serial.println(" released of B");
      // set it back to 0 on release
      ElectrodeTouchedB[i] = 0;
      if (USE_NOTE_ON_OFF){
        usbMIDI.sendNoteOff(notesB[i], 127, channel); // note, velocity, channel
      }
      trigger_leds = false;
      ambient_leds = false;
      ambientLEDs.start();
      //start ambient w black
      //CRGBPalette16 currentPalette( black_palette);

    }


  }

  // reset our state
  lasttouchedA = currtouchedA;
  lasttouchedB = currtouchedB;
  return; //Added back by DRC trying to debug??}
}

void triggerLoop(){
  // this fires on a timer and anything pressed triggers midi
  for (uint8_t i=0; i<numElectrodes; i++) {
    if (ElectrodeTouchedA[i]) {
      triggerMidiA(i);
    }
     if (ElectrodeTouchedB[i]) {
      triggerMidiB(i);
    }
  }
  //Serial.print(ElectrodeTouched[0]);
  //Serial.print("trigger Loop");

}

void startAmbient(){
  // this fires after the ambient timer has elapsed.
  // it will start the leds doing the ambient pattern until set to false.
  ambient_leds = true;
}

void stopPiezo(){
  // after piezo timer finishes turn off effect.
  trigger_leds = false;
}


void ledFrameLoop(){
  // calls bpm from DEMO REEL 100
  // This is a single 'frame' of leds for the whole strip.
  // it gets called based on the timer to set framerate
  // any function that writes a full strip worth of colors will work.

  gHue++;
  if (trigger_leds == true) {
    bpm();
    //pacifica_add_whitecaps();
    //Serial.println("leds triggered");
  }
  else if (ambient_leds == true && trigger_leds == false) {

    // added this modulo to slow how much the blend gets called for a slower fade in.
    if (gHue % 4 == 0 ){

      // this blends the just blacked out palette to target which is Ocean.
      //12 is default. play with this one
      nblendPaletteTowardPalette(currentPalette, targetPalette, 12);
      // form docs: a visually smoother transition: in the middle of the cross-fade your current palette will actually contain some colors from the old palette, a few blended colors, and some colors from the new palette.
      //The maximum number of possible palette changes per call is 48 (sixteen color entries time three channels each).//
      //The default 'maximim number of changes' here is 12, meaning that only approximately a quarter of the palette entries will be changed per call.


      //Serial.println("ambient mode");
    }
    // also added this sin8 that I think slows it a bit and makes it bounce, remove sin38 and it should not bounce?
    //sin8(gHue) /2 ;
    fill_palette(leds, NUM_LEDS, gHue, 15, currentPalette,50, LINEARBLEND );

  }
  else {
    fadeToBlackBy( leds, NUM_LEDS, 1);
  }

}

// Function to calculate squared Euclidean distance between two points (integer only)
uint16_t calculateDistanceSquared(PadLocation pad, CoordinatePair led) {
    // Calculate absolute differences to ensure no negative intermediate values
    uint16_t dx = (pad.x > led.x) ? (pad.x  - led.x) : (led.x - pad.x );
    uint16_t dy = (pad.y > led.y) ? (pad.y - led.y) : (led.y - pad.y);
    uint32_t dist = sqrt((dx * dx) + (dy * dy));
//    if (led.index == 1){
//      Serial.print("index:");
//      Serial.print(led.index);
//      Serial.print("padcoord:");
//      Serial.print(pad.x);
//      Serial.print(",");
//      Serial.print(pad.y);
//      Serial.print(": distance: ");
//
//      Serial.println(calculateBrightness(dist));
//
//    }
    // Square and add
    return dist;
}

uint8_t calculateBrightness(uint32_t distance) {
    
    // Function to map distance to brightness (0-255, where 255 is brightest)
    
    // Adjust these constants to control the brightness falloff
    const uint8_t MAX_BRIGHTNESS = 240;
    const uint8_t MIN_BRIGHTNESS = 2;

    static uint16_t max_dist = 10;
    max_dist = distance > max_dist ? distance : max_dist;

    // Avoid division by zero
    if (distance == 0) return MAX_BRIGHTNESS;

    // Calculate inverse square relationship
    // The larger the distance, the smaller the brightness
    uint8_t bright =  pow( sin8(map(distance, 0, max_dist, MIN_BRIGHTNESS, MAX_BRIGHTNESS)) ,2);

    if (bright > MAX_BRIGHTNESS) bright = MAX_BRIGHTNESS;
    
    return (uint8_t)bright;
}

void bpm(){
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  //uint8_t BeatsPerMinute = 6; //was 18
    CRGBPalette16 palette = custom_palette_2;

    for( int i = 0; i < NUM_LEDS; i++) { //9948  
        
        uint16_t distance = calculateDistanceSquared(padCoords[touched_pad], LEDcoordinates[i]);
    
        // uint8_t bright_scale = calculateBrightness(distance);
        //Serial.print("pad: ");
        //Serial.println(touched_pad);
        //Serial.print("distance: ");
        //Serial.println(calculateBrightness(distance));
        //Serial.print("scale: ");
        //Serial.println(beat-gHue+(i));
    
        // if we get a zero from calc brightness, then we will map the existing brightness back to the led.  
        // this will fade over time as it gets fades toward black. the idea is to keep the leds on recently touched pads from blinking out.
        
        if (distance < 200) {
            uint8_t brightness_value = calculateBrightness(distance);
      
            //Serial.print("brt: ");
            //Serial.println(brightness_value);
            
            //the third index argument (gHue + i*2) is determining the index along the palette 0-254 through the range.
            // so slowing down would probably be remove that *2 ?
            //.so changing the formula here with gHue always incrementing 0-255 can control how fast the colors scan through the palett
            leds[i] = ColorFromPalette(palette, gHue+(i*6), brightness_value);  //was gHue+(i*6)
        }
    }
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void triggerMidiA(int i){
   usbMIDI.sendControlChange(controlNumA[i], controlValA[i], channel); //(control#, controlval, channel)
   //Serial.print("triggered midi on: A ");
   //Serial.println(i);
}
void triggerMidiB(int i){
   usbMIDI.sendControlChange(controlNumB[i], controlValB[i], channel); //(control#, controlval, channel)
   //Serial.print("triggered midi on: B ");
   //Serial.println(i);
}
