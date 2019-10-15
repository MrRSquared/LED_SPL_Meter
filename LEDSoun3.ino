//Decible meter logic based on Vgnesh's code found here https://www.hackster.io/vignesh-jeyanthan-n/iot-based-power-decibel-meter-2a6ec8

#include <FastLED.h>

#define NUM_LEDS  10  // Number of pixels you are using
#define MIC_PIN   15  // Microphone is attached to Trinket GPIO #2/Gemma D2 (A1)
#define DATA_PIN    05  // NeoPixel LED strand is connected to GPIO #0 / D0
#define POT_PIN_0 02
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     00  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (NUM_LEDS -1) // Allow dot to go slightly off scale
int
  vol[SAMPLES],       // Collection of prior volume samples
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 120,
  height = 0,
  oldPeak = 0;
byte  
  peak = 0,
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
  
  
  uint8_t count = 0;


CRGB leds[NUM_LEDS];
//Construct a button for a reset.
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};
Button button1 = {18, 0, false};

void IRAM_ATTR isr() {
  button1.numberKeyPresses += 1;
  button1.pressed = true;
}
void setup()
{
  // serial
  int q;
  Serial.begin(115200);
  delay(100);
  while (!Serial); // Wait untilSerial is ready - Leonardo
  Serial.println("Starting mic demo");
   FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(3.3,1000);
  // brightness = 75;
  FastLED.setBrightness(5);
  for(q=0; q<NUM_LEDS; q++) {  
       leds[q].setRGB(0,0,0);
    } 
  FastLED.show();
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
}

void loop()
{
  // what do we want to do?
  
  MeasureVolume();
  delay(1000);
  //MeasureFHT();
}

// measure basic properties of the input signal
// determine if analog or digital, determine range and average.
void MeasureVolume()
  {float dB = 0;
   float adc;
   int peak = ButtonAction();
   uint8_t  i;
   uint16_t minLvl, maxLvl;
   //int      n, height;
  adc = analogRead(MIC_PIN);
  //Added logic from Adafruit
  adc   = abs(adc - 512 - DC_OFFSET);            // Center on zero
  adc   = (adc <= (NOISE)) ? 0 : (adc - (NOISE));   
  
  int PotPin = analogRead(POT_PIN_0);
  PotPin = map(PotPin, 0, 3975, 0, 120);
  dB = 20 * log10((adc/12.00)); //Convert ADC value to dB using Regression values
  //make lights happen.
   
  int max = (120 - PotPin) ;
  int min = 15;
  int height = map(dB, min, max, 0, NUM_LEDS);
  //Adafruit's height algorithm with DB...
  //height = TOP * (dB - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if (peak< height )     peak   = height; // Keep 'peak' dot at top
  if (count < 1) {
    oldPeak = height;
    maxLvlAvg = max;
  }
  
  vol[volCount] = dB;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

 
    //FastLED.show();
  for(int i=0; i<NUM_LEDS; i++) { 
    //if(i == peak)               
    //   leds[i].setRGB(50,   0, 0); 
    if (i <= height)
     leds[i].setRGB(50,0,50); 
    else if (i > height) //&& i != peak)
      leds[i].setRGB(0,0,0); 
    } 

   FastLED.show(); // Update strip
   // Draw peak dot
  
  if(peak > 0 && peak <= NUM_LEDS && peak >= oldPeak) oldPeak = peak;
  leds[oldPeak].setRGB(60,0,0);

  
   FastLED.show(); // Update strip

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
  
  Serial.print("Analog raw value is... " +String(adc));
  Serial.print(" DB are... " +String(dB));
  Serial.print(" height is... " +String(height));
  Serial.print("the P.O.T. value is... " +String(PotPin));
  Serial.println("");
}
int ButtonAction()
  {
    if (button1.pressed) {
      Serial.printf("Button 1 has been pressed %u times\n", button1.numberKeyPresses);
      count = 0;
      oldPeak = 0;
      peak = 1;
      button1.pressed = false;
  }
  return peak;
  }
