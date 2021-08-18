//##################################################
//#  NodeMCU 
//#  Matrix 8*8 NeoPixel
//#
//# Pin 2 (D4) : Neopixel Data
//# Pin   (D2) : DHT22 Data
//# Pin   (D5) : Rotary encoder DT
//# Pin   (D6) : Rotary encoder CLK
//# Pin   (D7) : Rotary encoder SW
//# 
//#
//########################################################################
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include "RGB.h"

#include "DHT.h"  //Use for DHT22


void dispo();
void navigation();
void fire();
void colorWipe(RGB color, uint8_t wait);
void occupe();
void fadePixel(int x, int y, RGB startColor, RGB endColor, int steps, int wait);
void crossFade(RGB startColor, RGB endColor, int steps, int wait);
void ReadDHT22();



/**************************************************************************//**
 * \def     ITEMS_IN_ARRAY
 * \brief   Get number of items in array.
 ******************************************************************************/
#define ITEMS_IN_ARRAY(array)   (sizeof(array) / sizeof(*array))




//Config Matrix
#define PIN 2
#define MATRIX_HEIGHT 8
#define MATRIX_WIDTH 8

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(
  MATRIX_WIDTH, MATRIX_HEIGHT,
  PIN,
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB + NEO_KHZ800
);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.



//these values are substracetd from the generated values to give a shape to the animation
const unsigned char valueMask[MATRIX_HEIGHT][MATRIX_WIDTH]={
    {32 , 0  , 0  , 0  , 0  , 0  , 0  , 32 },
    {64 , 0  , 0  , 0  , 0  , 0  , 0  , 64 },
    {96 , 32 , 0  , 0  , 0  , 0  , 32 , 96 },
    {128, 64 , 32 , 0  , 0  , 32 , 64 , 128},
    {160, 96 , 64 , 32 , 32 , 64 , 96 , 160},
    {192, 128, 96 , 64 , 64 , 96 , 128, 192},
    {255, 160, 128, 96 , 96 , 128, 160, 255},
    {255, 192, 160, 128, 128, 160, 192, 255}
};

//these are the hues for the fire, 
//should be between 0 (red) to about 25 (yellow)
const unsigned char hueMask[MATRIX_HEIGHT][MATRIX_WIDTH]={
    {1 , 11, 19, 25, 25, 22, 11, 1 },
    {1 , 8 , 13, 19, 25, 19, 8 , 1 },
    {1 , 8 , 13, 16, 19, 16, 8 , 1 },
    {1 , 5 , 11, 13, 13, 13, 5 , 1 },
    {1 , 5 , 11, 11, 11, 11, 5 , 1 },
    {0 , 1 , 5 , 8 , 8 , 5 , 1 , 0 },
    {0 , 0 , 1 , 5 , 5 , 1 , 0 , 0 },
    {0 , 0 , 0 , 1 , 1 , 0 , 0 , 0 }
};

unsigned char matrixValue[MATRIX_HEIGHT][MATRIX_WIDTH];
unsigned char line[MATRIX_WIDTH];
int pcnt = 0;

//Converts an HSV color to RGB color
uint16_t HSVtoRGB(uint8_t ih, uint8_t is, uint8_t iv) {
  float r, g, b, h, s, v; //this function works with floats between 0 and 1
  float f, p, q, t;
  int i;

  h = (float)(ih / 256.0);
  s = (float)(is / 256.0);
  v = (float)(iv / 256.0);

  //if saturation is 0, the color is a shade of grey
  if(s == 0.0) {
    b = v;
    g = b;
    r = g;
  }
  //if saturation > 0, more complex calculations are needed
  else
  {
    h *= 6.0; //to bring hue to a number between 0 and 6, better for the calculations
    i = (int)(floor(h)); //e.g. 2.7 becomes 2 and 3.01 becomes 3 or 4.9999 becomes 4
    f = h - i;//the fractional part of h

    p = (float)(v * (1.0 - s));
    q = (float)(v * (1.0 - (s * f)));
    t = (float)(v * (1.0 - (s * (1.0 - f))));

    switch(i)
    {
      case 0: r=v; g=t; b=p; break;
      case 1: r=q; g=v; b=p; break;
      case 2: r=p; g=v; b=t; break;
      case 3: r=p; g=q; b=v; break;
      case 4: r=t; g=p; b=v; break;
      case 5: r=v; g=p; b=q; break;
      default: r = g = b = 0; break;
    }
  }
  return matrix.Color(r * 255.0, g * 255.0, b * 255.0);
}

/**
 * Randomly generate the next line (matrix row)
 */
void generateLine(){
  for(uint8_t x=0; x<MATRIX_WIDTH; x++) {
    line[x] = random(64, 255);
  }
}

/**
 * shift all values in the matrix up one row
 */
void shiftUp() {
  for (uint8_t y=MATRIX_HEIGHT-1; y>0; y--) {
    for (uint8_t x=0; x<MATRIX_WIDTH; x++) {
      matrixValue[y][x] = matrixValue[y-1][x];
    }
  }
  
  for (uint8_t x=0; x<MATRIX_WIDTH; x++) {
    matrixValue[0][x] = line[x];
  }
}

/**
 * draw a frame, interpolating between 2 "key frames"
 * @param pcnt percentage of interpolation
 */
void drawFrame(int pcnt) {
  int nextv;
  
  //each row interpolates with the one before it
  for (unsigned char y=MATRIX_HEIGHT-1; y>0; y--) {
    for (unsigned char x=0; x<MATRIX_WIDTH; x++) {
      nextv = 
          (((100.0-pcnt)*matrixValue[y][x] 
        + pcnt*matrixValue[y-1][x])/100.0) 
        - valueMask[y][x];
      uint16_t color = HSVtoRGB(
        hueMask[y][x], // H
        255, // S
        (uint8_t)max(0, nextv) // V
      );

      matrix.drawPixel(x, y, color);
    }
  }
  
  //first row interpolates with the "next" line
  for(unsigned char x=0; x<MATRIX_WIDTH; x++) {
    uint16_t color = HSVtoRGB(
      hueMask[0][x], // H
      255,           // S
      (uint8_t)(((100.0-pcnt)*matrixValue[0][x] + pcnt*line[x])/100.0) // V
    );
    matrix.drawPixel(x, 0, color);
  }
}

//Config DHT22

//DHT22 Pin
#define DHTPIN 4     // D2 on NodeMCU

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

//Tempo mise à jour
unsigned long interval = 5000; //Tempo à régler
unsigned long previousMillis = 0; //
bool fist_select=true;


// rotary encoder pins
const int CLK = 12;  // Used for generating interrupts using CLK signal D6 NodeMCU
const int DT = 14;  //Used for reading DT signal : D5 NodeMCU
const int PinSW = 13;  // Used for the push button switch : D7


// Updated by the ISR (Interrupt Service Routine)
volatile int virtualPosition = 1000;
volatile int lastPosition = 1000;

#define NumberMenu 4 // Nombre de Menu 1er niveau
int posMenu = 0; // Variable pour identifier les menus

uint8_t previous_data;
void ICACHE_RAM_ATTR enc_read();


//// ------------------------------------------------------------------
//// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
//// ------------------------------------------------------------------
//void isr ()  {
//  static unsigned long lastInterruptTime = 0;
//  unsigned long interruptTime = millis();
//
//  // If interrupts come faster than 5ms, assume it's a bounce and ignore
//  if (interruptTime - lastInterruptTime > 5) {
//    if (digitalRead(DT) == LOW)
//    {
//      virtualPosition-- ; // Could be -5 or -10
//    }
//    else {
//      virtualPosition++ ; // Could be +5 or +10
//    }
//
//    // Restrict value from 0 to +2000
//    virtualPosition = _min(2000, _max(0, virtualPosition));
//
//    // Keep track of when we were here last (no more than every 5ms)
//    lastInterruptTime = interruptTime;
//    
//  }
//}


void setup() 
{
  pinMode(DT,  INPUT_PULLUP);
  pinMode(CLK, INPUT_PULLUP);

 previous_data = digitalRead(DT) << 1 | digitalRead(CLK);
  // enable interrupts for rotary encoder pins
  attachInterrupt(digitalPinToInterrupt(DT),  enc_read, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK), enc_read, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(CLK), isr, LOW);
  

  Serial.begin(500000);

  //Start Matrix
  matrix.begin();
  randomSeed(analogRead(0));
  generateLine();
  //init all pixels to zero
  memset(matrixValue, 0, sizeof(matrixValue));
  matrix.setBrightness(10);

  //Start DHT22
   dht.begin();
}

void ICACHE_RAM_ATTR enc_read()
{
  //Serial.println("Interrupt");
  uint8_t current_data = digitalRead(DT) << 1 | digitalRead(CLK);
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  if( current_data == previous_data )
    return;
  //If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 60){
    if( bitRead(current_data, 0) == bitRead(previous_data, 1) )
     virtualPosition-=1 ;
     //quad -= 1;
    else
      virtualPosition+=1; 
      //quad += 1;
  }
  previous_data = current_data; 
  // Restrict value from 0 to +2000
  //virtualPosition = _min(2000, _max(0, virtualPosition));
  lastInterruptTime = interruptTime;
  Serial.println(virtualPosition);
  
 
}

void navigation() 
{ 
  if ( virtualPosition != lastPosition) {
  
    if  (virtualPosition > lastPosition ) {  //menu suivant
      posMenu = (posMenu + 1 ) % NumberMenu ;
    } else if (virtualPosition < lastPosition ) { //menu precedent
      if (posMenu == 0) {
        posMenu = (NumberMenu);
      }
      posMenu = (posMenu - 1) % NumberMenu;
    }
    lastPosition = virtualPosition;
    Serial.print(F("Position menu : "));
    Serial.println(posMenu);
    fist_select=true;
    //Remise a zero matrice pour preparer next screen
    matrix.fillScreen(0);
    matrix.show();
  }
}

void loop() 
{
  navigation();
  switch (posMenu){
    case 0 :
       occupe();
     break;
    case 1 :
      dispo();
     break;
    case 2 :
      fire();
     break;
    case 3 :
     unsigned long currentMillis = millis();
      if (fist_select == true){
        previousMillis = millis();
        
        ReadDHT22();
        fist_select = false;
      }
      if ((unsigned long)(currentMillis - previousMillis) >= interval ) {// tempo de mise à jour des données capteur et de l'afficheur.
        previousMillis = millis();
        ReadDHT22();
    
      } 
     break;
    //case 4 :
     //colorWipe(red,50);   
    // break;  

  }

  
}



void fire()
{
    if (pcnt >= 100) {
    shiftUp();
    generateLine();
    pcnt = 0;
  }
  drawFrame(pcnt);
  matrix.show();
  pcnt+=30;
}


// Fill the pixels one after the other with a color
void colorWipe(RGB color, uint8_t wait) 
{
  for(uint16_t row=0; row < 8; row++) {
    for(uint16_t column=0; column < 8; column++) {
      matrix.drawPixel(column, row, matrix.Color(color.r, color.g, color.b));
      matrix.show();
      delay(wait);
    }
  }
}

void dispo()
{
  uint16_t color=matrix.Color(green.r, green.g, green.b);
  int logo[8][8] = {  
   {0, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 1},
   {0, 0, 0, 0, 0, 0, 1, 1},
   {1, 0, 0, 0, 0, 1, 1, 0},
   {1, 1, 0, 0, 1, 1, 0, 0},
   {0, 1, 1, 1, 1, 0, 0, 0},
   {0, 0, 1, 1, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 0}
  };
  for(int row = 0; row < 8; row++) {
    for(int column = 0; column < 8; column++) {
     if(logo[row][column] == 1) {
       matrix.drawPixel(column,7-row, color);
       matrix.show();
       //fadePixel(column, 7-row, white, green, 120, 0);
     }
   }
  }
}

void occupe()
{
  uint16_t colorRed=matrix.Color(red.r, red.g, red.b);
  uint16_t colorWhite=matrix.Color(white.r, white.g, white.b);
  
  int logo[8][8] = {  
   {0, 0, 1, 1, 1, 1, 0, 0},
   {0, 1, 1, 1, 1, 1, 1, 0},
   {1, 1, 1, 1, 1, 1, 1, 1},
   {1, 2, 2, 2, 2, 2, 2, 1},
   {1, 2, 2, 2, 2, 2, 2, 1},
   {1, 1, 1, 1, 1, 1, 1, 1},
   {0, 1, 1, 1, 1, 1, 1, 0},
   {0, 0, 1, 1, 1, 1, 0, 0}
  };
  for(int row = 0; row < 8; row++) {
    for(int column = 0; column < 8; column++) {
     if(logo[row][column] == 1) {
       matrix.drawPixel(column,7-row, colorRed);
       //fadePixel(column, 7-row, white, green, 120, 0);
     }
     if(logo[row][column] == 2) {
       matrix.drawPixel(column,7-row, colorWhite);
       //fadePixel(column, 7-row, white, green, 120, 0);
     }     
     matrix.show();
   }
  }
}


// Fade pixel (x, y) from startColor to endColor
void fadePixel(int x, int y, RGB startColor, RGB endColor, int steps, int wait) 
{
  for(int i = 0; i <= steps; i++) 
  {
     int newR = startColor.r + (endColor.r - startColor.r) * i / steps;
     int newG = startColor.g + (endColor.g - startColor.g) * i / steps;
     int newB = startColor.b + (endColor.b - startColor.b) * i / steps;

     matrix.drawPixel(x, y, matrix.Color(newR, newG, newB));
     matrix.show();
     delay(wait);
  }
}

// Crossfade entire screen from startColor to endColor
void crossFade(RGB startColor, RGB endColor, int steps, int wait) 
{
  for(int i = 0; i <= steps; i++)
  {
     int newR = startColor.r + (endColor.r - startColor.r) * i / steps;
     int newG = startColor.g + (endColor.g - startColor.g) * i / steps;
     int newB = startColor.b + (endColor.b - startColor.b) * i / steps;

     matrix.fillScreen(matrix.Color(newR, newG, newB));
     matrix.show();
     delay(wait);
  }
}



void ReadDHT22()
{
  
    // Lecture du taux d'humidité
    float h = dht.readHumidity();
    // Lecture de la température en Celcius
    float t = dht.readTemperature();
    // Pour lire la température en Fahrenheit
    float f = dht.readTemperature(true);
    
    // Stop le programme et renvoie un message d'erreur si le capteur ne renvoie aucune mesure
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Echec de lecture !");
      return;
    }
  
    // Calcul la température ressentie. Il calcul est effectué à partir de la température en Fahrenheit
    // On fait la conversion en Celcius dans la foulée
    float hi = dht.computeHeatIndex(f, h);
    
  
    Serial.print("Humidite: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.print(" *C ");
    Serial.print("Temperature ressentie: ");
    Serial.print(dht.convertFtoC(hi));
    Serial.println(" *C");
    //********  Led pour affichage Temperature   *******
    matrix.fillScreen(0); // Clean les led
    if (t >= 16 )  {//si la température est supérieure à 16°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 0, matrix.Color(138, 0, 255));
      }
    }
   if (t >= 18 )  {//si la température est supérieure à 17°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 1, matrix.Color(84, 0, 255));
      }
    }
   if (t >= 20 )  {//si la température est supérieure à 18°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 2, matrix.Color(0, 101, 253));
      }
    }
   if (t >= 22 )  {//si la température est supérieure à 19°C
      //   Serial.println("Rang 4 allumée");
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
         matrix.drawPixel(i, 3, matrix.Color(0, 253, 253));
      }
    }
   if (t >= 24 )  {//si la température est supérieure à 20°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 4, matrix.Color(0, 253, 179));
      }
    }
   if (t >= 26 )  {//si la température est supérieure à 21°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 5, matrix.Color(50, 251, 0));
      }
    }
   if (t >= 28 )  {//si la température est supérieure à 22°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 6, matrix.Color(255, 252, 0));
      }
    }
   if (t >= 30 )  {//si la température est supérieure à 23°C
      for (int i = 0; i < 3; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
       matrix.drawPixel(i, 7, matrix.Color(248, 149, 0));
      }
    }


   //********  Led pour affichage Humidity   *******

   if (h >= 12 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 0, matrix.Color(138, 0, 255));
      }
    }
   if (h >= 24 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 1, matrix.Color(84, 0, 255));
      }
    }
   if (h >= 36 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 2, matrix.Color(0, 101, 253));
      }
    }
   if (h >= 48 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 3, matrix.Color(0, 253, 253));
      }
    }
   if (h >= 60 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 4, matrix.Color(0, 253, 179));
      }
    }
   if (h >= 72 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 5, matrix.Color(50, 251, 0));
      }
    }
   if (h >= 84 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 6, matrix.Color(255, 252, 0));
      }
    }
   if (h >= 96 )  {//
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        matrix.drawPixel(i, 7, matrix.Color(248, 149, 0));
      }
    }

    matrix.show();
    
    

 }

 
