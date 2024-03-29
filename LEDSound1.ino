#include <FastLED.h>

#define NUM_LEDS  20  // Number of pixels you are using
#define MIC_PIN   15  // Microphone is attached to Trinket GPIO #2/Gemma D2 (A1)
#define DATA_PIN    21  // NeoPixel LED strand is connected to GPIO #0 / D0
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     0  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (NUM_LEDS -1) // Allow dot to go slightly off scale
#define INPUT_FLOOR 10 //Lower range of analogRead input
#define INPUT_CEILING 300 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

CRGB leds[NUM_LEDS];

byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
  
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 0,     // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;
uint8_t count = 0;


void setup() {
  uint8_t  q;
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  // brightness = 75;
  FastLED.setBrightness(15);
  for(q=0; q<NUM_LEDS; q++) {  
       leds[q].setRGB(0,0,0);
    } 
    FastLED.show();
}



void loop() {
  // put your main code here, to run repeatedly:
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  
  
  n   = analogRead(MIC_PIN);    // Raw reading from mic 
  n   = abs(n - 512 - DC_OFFSET);            // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);      // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
  //Serial.println(lvl);

  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  //height = TOP * lvl;
  

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if (count < 1) peak = height;
  if (peak < height)     peak   = height; // Keep 'peak' dot at top


  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Color pixels based on rainbow gradient

  //FastLED.show();
  for(i=0; i<NUM_LEDS; i++) { 
    if(i == peak)               
       leds[i].setRGB(50,   0, 0); 
    else if (i < height)
      leds[i].setRGB(50,0,50); 
    else if (i>= height && i != peak)
      leds[i].setRGB(0,0,0); 
    } 

   FastLED.show(); // Update strip

   // Draw peak dot  
  //if(peak > 0 && peak <= NUM_LEDS-1) leds[peak].setRGB(100,0,0);
  
   //FastLED.show(); // Update strip

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
 if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
   minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
   maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
  count ++;
}

float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){
 
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;
 
 
  // condition curve parameter
  // limit range
 
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;
 
  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
 
  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */
 
  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }
 
  // Zero Refference the values
  OriginalRange = originalMax - originalMin;
 
  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }
 
  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float
 
  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }
 
  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
 
  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }
 
  return rangedValue;
}
