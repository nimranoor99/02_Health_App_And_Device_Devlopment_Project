#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>     // Touch library

//                                                                  DEFINE SENSORS LIBRARIES
/*#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"*/

//                                                                   CALIBRATE VALUES 
#define SENSIBILITY 300
#define MINPRESSURE 10
#define MAXPRESSURE 1000

//                                                              THESE ARE THE PINS FOR SHIELD 
#define YP A1 
#define XM A2 
#define YM 7  
#define XP 6 

short TS_MINX=150;
short TS_MINY=120;
short TS_MAXX=920;
short TS_MAXY=940;

//                                                                INTIALIZE TOUCHSCREEN 

TouchScreen ts = TouchScreen(XP, YP, XM, YM, SENSIBILITY);

//                                                                TFTLCD_2.4_INCH_PINS 
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4 // Optional : otherwise connect to Arduino's reset pin

//                                                                   DEFINE COLORS 
#define BLACK   0x0000
#define PURPLE  0x780F
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0x5AEB
//#define PINK 0xF97F
/*
#define GRAY 0x7BEF
#define LIGHT_GRAY 0xC618
#define GREEN 0x07E0
#define LIME 0x87E0
#define BLUE 0x001F
#define RED 0xF800
#define AQUA 0x5D1C
#define YELLOW 0xFFE0
#define MAGENTA 0xF81F
#define CYAN 0x07FF
#define DARK_CYAN 0x03EF
#define ORANGE 0xFCA0
#define PINK 0xF97F
#define BROWN 0x8200
#define VIOLET 0x9199
#define SILVER 0xA510
#define GOLD 0xA508
#define NAVY 0x000F
#define MAROON 0x7800
#define PURPLE 0x780F
#define OLIVE 0x7BE0*/
//                                                                INITIALIZE LCD 

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

//                                                                 DIMENSIONS 
uint16_t width = 0;
uint16_t height = 0;
//                                                                  Checksum
uint16_t checkSum = 0;
uint16_t checkSum_Glu = 0;
//                                                                   MAX30102 
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

//                                                                 TFTLCD 2.4 INCH 
float  sx,sy, mx = 1, my = 0, hx = -1, hy = 0, sdeg = 0, mdeg = 0, hdeg = 0;    
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120, xpos; 
uint32_t targetTime = 0;
int16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0, x00 = 0, yy00 = 0;                    
uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); 
boolean initial = 1;

//                                                                      PEDOMETER 
#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

#define offsetX   -10.5       // OFFSET values
#define offsetY   -2.5
#define offsetZ   -4.5

#define gainX     257.5        // GAIN factors
#define gainY     254.5
#define gainZ     248.5

byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];                      //string buffer to transform data before sending it to the serial port

int x,y,z, X, Y;

int xavg, yavg,zavg, steps=0, flag=0;
int xval[15]={0}, yval[15]={0}, zval[15]={0};
int threshhold = 60.0;

//                                                                LED
//const int BUTTON = 12;
const int LED = 13;
int BUTTONstate = 0;

//                                                              RTC Module
#include "Wire.h"
#define DS3231_I2C_ADDRESS 0x68
//DS3231 rtc(SDA, SCL);
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val){
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val){
  return( (val/16*10) + (val%16) );
}
char d;

//                                                                 Battery Indicator
int batteryPin = A8;   // select the input pin for Battery Indicator
int Voltage = 0;
float Actual_Voltage =0;

//                                    Glucose
const int Glucose_Pin = A5; // connect ir sensor to arduino pin 2
int statusSensor = 0; 
float voltage =0.00;


void setup(void) {
  tft.reset(); 
  delay(10); 
  tft.begin(0x9341);
  tft.setRotation(3);   
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);  
  tft.fillScreen(BLACK);
  tft.setTextSize(4);
  tft.setCursor(80,100);
  tft.setTextColor(RED);
  tft.print("Glucoxy");
  delay(300);
  tft.fillRect(75, 90, 170, 70,  BLACK);
  tft.setCursor(80,100);
  tft.setTextColor(CYAN);
  tft.print("Glucoxy");
  delay(300);
  tft.fillRect(75, 90, 170, 70,  BLACK);
  tft.setCursor(80,100);
  tft.setTextColor(GREEN);
  tft.print("Glucoxy");
  delay(300);
  tft.fillRect(75, 90, 170, 70,  BLACK);
  tft.setCursor(80,100);
  tft.setTextColor(BLUE);
  tft.print("Glucoxy");
  delay(300);
  tft.fillRect(75, 90, 170, 70,  BLACK);
  tft.setCursor(80,100);
  tft.setTextColor(WHITE);
  tft.print("Glucoxy");
  waitOneTouch();
  digitalWrite(LED, LOW);
  clocksetup();
  tft.fillScreen(BLACK);
  RTCSetup();
}

void loop() { 
TSPoint p = ts.getPoint();       //checking if the user touched the screen 
pinMode(XM, OUTPUT);
pinMode(YP, OUTPUT);
if (p.z > MINPRESSURE && p.z < MAXPRESSURE) 
{ //p.z means the pressure value so if the touch wants to be detected it pressure should be in this range (it's enough)
p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());    //x and y positions of the touch so the program know the postion where the user has pressed
p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

//Oximeter Option              
if( p.x > 20 && p.x < 140 && p.y > 112 && p.y < 159  ){
          tft.drawRoundRect(17, 109, 126, 43,5,  BLUE);
          tft.fillRoundRect(20, 112, 120, 37,5,  YELLOW);
          tft.setCursor(30, 120);
          tft.setTextColor(BLUE);
          tft.setTextSize(2);
          tft.println("Oximeter");
           PulseView();
           tft.setTextColor(GREY); 
           tft.setCursor(5, 205);
           tft.fillRect(5,192,315,215,BLACK);
           tft.println("Press Anywhere to Go Back!");
           waitOneTouch();
           tft.fillRect(5,205,315,215,BLACK);
           watchOptions();         
           }
//Heart Rate Option 
else if(p.x > 165 && p.x < 305 && p.y > 112 && p.y < 159){
           tft.drawRoundRect(162, 109, 146, 43,5,  BLUE);
           tft.fillRoundRect(165, 112, 140, 37,5,  YELLOW);
           tft.setCursor(170, 120);
           tft.setTextColor(BLUE);
           tft.setTextSize(2);
           tft.println("Heart Rate");
           HRView();
           tft.setTextColor(GREY); 
           tft.setTextSize(2);
           tft.setCursor(5, 205); 
           tft.fillRect(5,192,315,215,BLACK);
           tft.println("Press Anywhere to Go Back!");
           waitOneTouch();
           tft.fillRect(5,205,315,215,BLACK);
           watchOptions();                       
           }
//Pedometer Option
else if(p.x > 20 && p.x < 140 && p.y > 162 && p.y < 199){
            tft.drawRoundRect(17, 159, 126, 43,5,  BLUE);
            tft.fillRoundRect(20, 162, 120, 37, 5, YELLOW);
            tft.setCursor(30, 170);
            tft.setTextColor(BLUE);
            tft.setTextSize(2);
            tft.println("Pedometer");
            PedoView();
            RTCSetupSensor();
for(unsigned int x = 0; x < 1; x++){
            PedoFunction();}
            tft.setTextColor(GREY); 
            tft.setTextSize(2);
            tft.setCursor(5, 205);
            tft.fillRect(5,192,315,215,BLACK); 
            tft.println("Press Anywhere to Go Back!");
            waitOneTouch();
            tft.fillRect(5,205,315,215,BLACK);
            watchOptions();
   
}

//Glucose option
else if(p.x > 165 && p.x < 305 && p.y > 162 && p.y < 199){
         tft.drawRoundRect(162, 159, 146, 43,5,  BLUE);
         tft.fillRoundRect(165, 162, 140, 37,5,  YELLOW);
         tft.setCursor(170, 170);
         tft.setTextColor(BLUE);
         tft.setTextSize(2);
         tft.println("Glucose"); 
         GLCView();
         waitOneTouch();
         tft.fillRect(5,205,315,215,BLACK);
         watchOptions();
   }
}
}



void PedoFunction(){
Wire.begin();        // join i2c bus (address optional for master)
Serial.begin(9600);
          
writeTo(DEVICE, 0x2D, 0);     //Turning on the ADXL345 
writeTo(DEVICE, 0x2D, 16);
writeTo(DEVICE, 0x2D, 8);
int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
 for(byte i=0; i<25; i++){ 
readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
//each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!! thus we are converting both bytes in to one int
x = (((int)buff[1]) << 8) | buff[0];   
y = (((int)buff[3])<< 8) | buff[2];
z = (((int)buff[5]) << 8) | buff[4];
  
//we send the x y z values as a string to the serial port 
//sprintf(str, "%d %d %d", x, y, z);
//Serial.print(str);
//Serial.print(10, byte());
 
/*Serial.print("X = ");
Serial.println(x);
Serial.print("Y = ");
Serial.println(y);
Serial.print("Z = ");
Serial.println(z);*/
  
                                  
X = ArduinoPedometerSteps();   //write steps
/*Serial.print("steps = ");
Serial.println(X);
Serial.println(" ");*/
  
Serial.print(F("X: "));
Serial.print("\t");
Serial.print(x, DEC);
Serial.print("\n");

Serial.print(F("Y: "));
Serial.print("\t");
Serial.print(y, DEC);
Serial.print("\n");

Serial.print(F("Z: "));
Serial.print("\t");
Serial.print(z, DEC);
Serial.print("\n");
Serial.print(F("steps: "));
Serial.print("\t");
Serial.print(X, DEC);
Serial.println("\n");
 
checkSum = x + y + z + X;

 Serial.print(F("Checksum Byte: "));
 Serial.print("\t");
 Serial.print(checkSum, DEC); 
 Serial.print("\t");
 if((byte)checkSum == (byte)(x + y + z + X)){Serial.print("(CHECKSUM_OK)");}
 else {Serial.print("(CHECKSUM_ERROR)");}
 Serial.println("\n");
 Serial.println("\n");
 Serial.println("");
 Serial.println("");
 Serial.println("");
 delay(150);
 
//TFT Display
uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
tft.fillScreen(BLACK);
tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);
tft.fillRect(10,200,315,215,BLACK);
tft.setTextColor(WHITE);
displayTimeSensors();
tft.setCursor(20, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Pedometer Sensor");
//tft.setTextSize(2);
//tft.setTextColor(WHITE);
//tft.setCursor(30, 110);
//tft.println("Total Averge ");
tft.setCursor(13, 120);
tft.println("STEPS: ");
tft.setTextSize(4);
tft.setCursor(139, 115);
tft.print(X);

//X DIRECTION
tft.setTextSize(2);
tft.setTextColor(WHITE);
tft.setCursor(210, 95);
tft.println("X:");
tft.setTextColor(YELLOW);
tft.setCursor(236, 95);
tft.print(x);

//Y DIRECTION
tft.setTextColor(WHITE);
tft.setCursor(210, 130);
tft.println("Y:");
tft.setTextColor(YELLOW);
tft.setCursor(236, 130);
tft.print(y);

//Z DIRECTION
tft.setTextColor(WHITE);
tft.setCursor(210, 165);
tft.println("Z:");
tft.setTextColor(YELLOW);
tft.setCursor(236, 165);
tft.print(z);

//Lines
tft.fillRect(5,75,309,7,YELLOW);
//tft.drawRect(5,170,196,4,RED);
//tft.fillRect(0,236,320,4,BLUE);
tft.drawRect(200,82,4,104,YELLOW);
tft.drawRect(204,119,108,4,YELLOW);
tft.drawRect(204,150,108,4,YELLOW);  
}
}

void PedoView(){
uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
tft.fillScreen(BLACK);
tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);
tft.setCursor(20, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Pedometer Sensor");
tft.setCursor(13, 120);
tft.println("STEPS: ");
//X DIRECTION
tft.setTextSize(2);
tft.setTextColor(WHITE);
tft.setCursor(210, 95);
tft.println("X:");
//Y DIRECTION
tft.setTextColor(WHITE);
tft.setCursor(210, 130);
tft.println("Y:");
//Z DIRECTION
tft.setTextColor(WHITE);
tft.setCursor(210, 165);
tft.println("Z:");
//to start
tft.setTextColor(GREY);
tft.setTextSize(2);
tft.setCursor(10, 205); 
tft.println("Press Anywhere to start!");
//Lines
tft.fillRect(5,75,309,7,YELLOW);
//tft.drawRect(5,170,196,4,RED);
//tft.fillRect(0,236,320,4,BLUE);
tft.drawRect(200,82,4,104,YELLOW);
tft.drawRect(204,119,108,4,YELLOW);
tft.drawRect(204,150,108,4,YELLOW);

waitOneTouch();
tft.fillRect(10,205,315,215,BLACK);
tft.setTextSize(4);
tft.setCursor(115, 200); 
tft.println("WAIT... ");
}

void PulseView(){
tft.fillScreen(BLACK);
uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);
tft.setCursor(40, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Pulse Oximeter");
tft.setTextSize(3);
tft.setCursor(2, 125);
tft.println(" SPO2: ");
//tft.setCursor(250, 120);
tft.setTextSize(2);
tft.setCursor(185, 130);
tft.println("%");
tft.setTextColor(WHITE);
tft.setCursor(206, 100);
tft.println("IR: ");
tft.setCursor(206, 146);
tft.println("RED: ");
tft.setTextColor(GREY);
tft.setTextSize(2);
tft.setCursor(10, 205); 
tft.println("Press Anywhere to start!");
tft.fillRect(5,75,309,7,YELLOW);
//tft.drawRect(5,170,309,4,WHITE);
tft.drawRect(200,82,4,103,YELLOW);
tft.drawRect(204,125,110,4,YELLOW);

waitOneTouch();
tft.fillRect(10,205,315,215,BLACK);
tft.setTextSize(4);
tft.setCursor(115, 200); 
tft.println("WAIT... ");
getPulseValues();
}
void HRView(){
tft.fillScreen(BLACK);
uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);
tft.setCursor(50, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Heart Rate");   
tft.setCursor(2, 125);
tft.println(" HR: ");
tft.setTextSize(1);
tft.setCursor(165, 130);
tft.println("BPM");
tft.setTextColor(WHITE);
tft.setCursor(206, 100);
tft.setTextSize(2);
tft.println("IR: ");
tft.setCursor(206, 146);
tft.println("RED: ");
tft.setTextColor(GREY);
tft.setTextSize(2);
tft.setCursor(10, 205); 
tft.println("Press Anywhere to start!");
tft.fillRect(5,75,309,7,YELLOW);
tft.drawRect(200,82,4,103,YELLOW);
tft.drawRect(204,125,110,4,YELLOW);
    
waitOneTouch();
tft.fillRect(10,205,315,215,BLACK);
tft.setTextSize(4);
tft.setCursor(115, 200); 
tft.println("WAIT... ");
getHRValues();
}

void getHRValues(){
Serial.begin(9600);
wait_for_Max30102();
start_signal();
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
{
Serial.println(F("MAX30105 was not found. Please check wiring/power."));
while (1);
}
//Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//while (Serial.available() == 0) ; //wait until user presses a key
Serial.read();
particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
//read the first 100 samples, and determine the signal range
for (byte i = 0 ; i < bufferLength ; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data
redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample(); //We're finished with this sample so move to next sample
//Serial.print(F("red="));
//Serial.print(redBuffer[i], DEC);
//Serial.print(F(", ir="));
//Serial.println(irBuffer[i], DEC);
}
//calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
for (int a = 0; a < 10; a++)
//while (1){
//dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
for (byte i = 25; i < 100; i++)
{
redBuffer[i - 25] = redBuffer[i];
irBuffer[i - 25] = irBuffer[i];
}
RTCSetupSensor();
//take 25 sets of samples before calculating the heart rate.
for (byte i = 75; i < 100; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data
digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample(); //We're finished with this sample so move to next sample
//send samples and calculation result to terminal program through UART
Serial.print(F("HR: "));
Serial.print("\t");
Serial.print(heartRate, DEC);
Serial.println("");

Serial.print(F("RED: "));
Serial.print("\t");
Serial.print(redBuffer[i], DEC);
Serial.println("");
      
Serial.print(F("IR: "));
Serial.print("\t");
Serial.print(irBuffer[i], DEC);
Serial.println("");



checkSum = redBuffer[i] + irBuffer[i] + heartRate;
Serial.print(F("Checksum Byte: "));
Serial.print("\t");
Serial.print(checkSum, DEC); 
Serial.print("\t");
if((byte)checkSum == (byte)(redBuffer[i] + irBuffer[i] + heartRate)){Serial.print("(CHECKSUM_OK)");}
else {Serial.print("(CHECKSUM_ERROR)");}
Serial.println("\n");
Serial.println("\n");
Serial.println("");
Serial.println("");
Serial.println("");
delay(1000);

//TFT DISPLAY
uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
tft.fillScreen(BLACK);
tft.drawRoundRect(border, border, (width - border * 2), 180, 5,YELLOW);
tft.fillRect(10,200,315,215,BLACK);
displayTimeSensors();
tft.setTextColor(WHITE);
tft.setTextSize(2);
tft.setCursor(206, 100);
tft.println("IR: ");
tft.setTextColor(YELLOW);
tft.setCursor(245, 100);
tft.println(irBuffer[i]);
tft.setTextColor(WHITE);
tft.setCursor(203, 146);
tft.println("RED:");
tft.setTextColor(YELLOW);
tft.setCursor(252, 145);
tft.println(redBuffer[i]);
tft.setCursor(50, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Heart Rate");
tft.setTextSize(3);
tft.setCursor(2, 125);
tft.println(" HR: ");
tft.setTextSize(4);
tft.setTextColor(WHITE);
tft.setCursor(87, 118);
tft.print(heartRate);
tft.setTextSize(1);
tft.setCursor(165, 130);
tft.print("BPM");
tft.fillRect(5,75,309,7,YELLOW);
//tft.drawRect(5,170,309,4,BLUE);
tft.drawRect(200,82,4,103,YELLOW);
tft.drawRect(204,125,110,4,YELLOW);
}
//delay(100);
//After gathering 25 new samples recalculate HR and SP02
 maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

 //}
}
//                                                                              Glucose 


//                                                                              CLOCK SETUP 
void clocksetup(){
tft.setTextColor(WHITE);// text color
tft.fillScreen(BLACK);// background color

xpos = tft.width() / 2; 
tft.drawCircle(xpos, 120, 125, YELLOW);
tft.fillCircle(xpos, 120, 118, BLUE); 
tft.fillCircle(xpos, 120, 110, BLACK); 
for (int a=95; a<104; a++){
tft.drawCircle(xpos, 120, a, WHITE);}  

for (int i = 0; i < 360; i += 30) {

tft.drawLine(((cos((i - 90) * 0.0174532925)) * 114 + xpos), ((sin((i - 90) * 0.0174532925)) * 114 + 120), ((cos((i - 90) * 0.0174532925)) * 100 + xpos), ((sin((i - 90) * 0.0174532925)) * 100 + 120), 0xFFE0);
tft.fillCircle(xpos, 121, 3, 0xFFFF);
}
for (int i = 0; i < 360; i += 6) {
// Draw minute markers
tft.drawPixel(((cos((i - 90) * 0.0174532925)) * 102 + xpos), ((sin((i - 90) * 0.0174532925)) * 102 + 120), 0x07E0); 
tft.drawLine(((cos((i - 90) * 0.0174532925)) * 102 + xpos), ((sin((i - 90) * 0.0174532925)) * 102 + 120), ((cos((i - 90) * 0.0174532925)) * 92 + xpos), ((sin((i - 90) * 0.0174532925)) * 92 + 120), 0x0000);
tft.drawLine(((cos((i - 90) * 0.0174532925)) * 102 + xpos)+1, ((sin((i - 90) * 0.0174532925)) * 102 + 120)+1, ((cos((i - 90) * 0.0174532925)) * 92 + xpos)+1, ((sin((i - 90) * 0.0174532925)) * 92 + 120)+1, 0x0000);
// Draw main quadrant dots
if (i == 0 || i == 180) tft.fillCircle(((cos((i - 90) * 0.0174532925)) * 102 + xpos), ((sin((i - 90) * 0.0174532925))* 102 + 120), 2, 0xFFFF); 
if (i == 90 || i == 270) tft.fillCircle(((cos((i - 90) * 0.0174532925)) * 102 + xpos), ((sin((i - 90) * 0.0174532925))* 102 + 120), 2, 0xFFFF); 

unsigned long starttime = millis();
unsigned long endtime = starttime;
while  ((endtime - starttime) <=30) // do this loop for up to 500mS=1/2 minute and 100 means 10 seconds
{
unsigned long loopcount;
clockLoop();
loopcount = loopcount+1;
endtime = millis();
}
}
tft.fillCircle(xpos, 121, 3, 0xFFFF);
targetTime = millis() + 1000;  
}

void clockLoop(){
if (targetTime < millis()) {             
targetTime = millis() + 1000;
ss++;              
    if (ss == 60) {
      ss = 0;
      mm++;            
      if (mm > 59) {
        mm = 0;
        hh++;          
        if (hh > 23) {
          hh = 0;
        }
      }
    }
hx = cos(((hh * 30 + mdeg * 0.0833333) - 90) * 0.0174532925);
hy = sin(((hh * 30 + mdeg * 0.0833333) - 90) * 0.0174532925);
mx = cos(((mm * 6 + sdeg * 0.01666667)- 90) * 0.0174532925);
my = sin(((mm * 6 + sdeg * 0.01666667) - 90) * 0.0174532925);
sx = cos(((ss * 6) - 90) * 0.0174532925);
sy = sin(((ss * 6) - 90) * 0.0174532925);
if (ss == 0 || initial) {
initial = 0;
tft.drawLine(ohx, ohy, xpos, 121, 0x0000);
ohx = hx * 62 + xpos + 1;
ohy = hy * 62 + 121;
tft.drawLine(omx, omy, xpos, 121,  0x0000);
omx = mx * 84 + xpos;
omy = my * 84 + 121;
}
tft.drawLine(osx, osy, xpos, 121,  0x0000);
osx = sx * 90 + xpos + 1;
osy = sy * 90 + 121;
tft.drawLine(osx, osy, xpos, 121, 0xF800);
tft.drawLine(ohx, ohy, xpos, 121, 0x07FF);
tft.drawLine(omx, omy, xpos, 121, 0xFFFF);
tft.drawLine(osx, osy, xpos, 121, 0xF800);
tft.fillCircle(xpos, 121, 3, 0xF800);
tft.setCursor(xpos-40, 44);
tft.setTextSize(2);
tft.print("Glucoxy");
// Draw MINI clock face "SECOND"
tft.drawCircle(xpos, 155, 20, 0xFFE0);
tft.drawCircle(xpos, 155, 18, 0x001F);
tft.drawCircle(xpos, 155, 17, 0x07FF);
tft.drawCircle(xpos, 155, 16, 0x07FF);
tft.fillRect(xpos-10, 149,22,15,0x0000); //erase
if(ss<10){tft.setCursor(xpos-10, 149); tft.setTextSize(2);
tft.print('0'); tft.setCursor(xpos+2, 149);}
else{
tft.setCursor(xpos-10, 149);}
tft.setTextSize(2);
tft.print(ss);
// Draw MINI clock face "Minutes"
tft.drawCircle(xpos+35, 117, 20, 0xFFE0);
tft.drawCircle(xpos+35, 117, 18, 0x001F);
tft.drawCircle(xpos+35, 117, 17, 0x07FF);
tft.drawCircle(xpos+35, 117, 16, 0x07FF);
tft.fillRect(xpos+25, 111,22,15, 0x0000); //erase
if(mm<10){tft.setCursor(xpos+25, 111); tft.setTextSize(2);
tft.print('0'); tft.setCursor(xpos+37, 111);}
else{
tft.setCursor(xpos+25, 111);}
tft.println(mm);
// Draw MINI clock face "Hour"
tft.drawCircle(xpos-35, 117, 20, 0xFFE0);
tft.drawCircle(xpos-35, 117, 18, 0x001F);
tft.drawCircle(xpos-35, 117, 17, 0x07FF);
tft.drawCircle(xpos-35, 117, 16, 0x07FF);
tft.fillRect(xpos-45, 111,22,15,0x0000); //erase
if(hh<10){tft.setCursor(xpos-45, 111); tft.setTextSize(2);
tft.print('0'); tft.setCursor(xpos-33, 111);}
else{
tft.setCursor(xpos-45, 111);}
tft.setTextSize(2);
tft.print(hh);
//tft.setCursor(xpos-65, 111);
//tft.println(':');
if (hh>=0 && hh<12) d='A'; else {d='P';}
tft.drawRoundRect(xpos-14,72,29,21,5,0x07FF);
tft.fillRect(xpos-11, 75,23,15,0x0000); //erase
tft.setCursor(xpos-11, 75);
tft.setTextSize(2);
tft.print(d);
tft.println('M');
}   
}

// wait 1 touch to return the point 
TSPoint waitOneTouch() {
  
TSPoint p;
  
do {
    p= ts.getPoint(); 
  
    pinMode(XM, OUTPUT); //Pins configures again for TFT control
    pinMode(YP, OUTPUT);
  
} while((p.z < MINPRESSURE )|| (p.z > MAXPRESSURE));
  
return p;
}

//Options Avaliable
void watchOptions(){
tft.fillScreen(BLACK);
tft.drawRoundRect(0, 0, 320, 240, 5,YELLOW);
tft.drawRoundRect(5, 5, 308, 68,5, YELLOW);
tft.fillRoundRect(10, 10, 298, 60,5,  BLUE);
//Brand name
tft.setCursor(15, 16);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("GLUCOXY");
//Tagline
tft.setCursor(15, 50);
tft.setTextColor(WHITE);
tft.setTextSize(2);
tft.println("Just Be Right!");
//Select Line
tft.setCursor(10, 80);
tft.setTextColor(YELLOW);
tft.setTextSize(2.5);
tft.println("Choose Your Option!!");

//option 1
tft.drawRoundRect(17, 109, 126, 43,5,  WHITE);
tft.fillRoundRect(20, 112, 120, 37,5,  BLUE);
tft.setCursor(30, 120);
tft.setTextColor(WHITE);
tft.setTextSize(2);
tft.println("Oximeter");

//option 2
tft.drawRoundRect(162, 109, 146, 43,5,  WHITE);
tft.fillRoundRect(165, 112, 140, 37,5,  BLUE);
tft.setCursor(170, 120);
tft.println("Heart Rate");
    
//option 3
tft.drawRoundRect(17, 159, 126, 43,5,  WHITE);
tft.fillRoundRect(20, 162, 120, 37, 5, BLUE);
tft.setCursor(30, 170);
tft.println("Pedometer");
    
//option 4
tft.drawRoundRect(162, 159, 146, 43,5,  WHITE);
tft.fillRoundRect(165, 162, 140, 37,5,  BLUE);
tft.setCursor(170, 170);
tft.println("Glucose"); 
delay(500);
}


//                                                                       PEDOMETER FUNCTIONS 
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
Wire.beginTransmission(device); //start transmission to device 
Wire.write(address);        // send register address
Wire.write(val);        // send value to write
Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
Wire.beginTransmission(device); //start transmission to device 
Wire.write(address);        //sends address to read from
Wire.endTransmission(); //end transmission
  
Wire.beginTransmission(device); //start transmission to device
Wire.requestFrom(device, num);    // request 6 bytes from device
  
int i = 0;
while(Wire.available())    //device may send less than requested (abnormal)
{ 
buff[i] = Wire.read(); // receive a byte
i++;
}
Wire.endTransmission(); //end transmission
}

//Get pedometer
int ArduinoPedometerSteps(){
int acc=0;
int totvect[15]={0};
int totave[15]={0};
int xaccl[15]={0};
int yaccl[15]={0};
int zaccl[15]={0};
for (int i=0;i<15;i++)
{
xaccl[i]= x;
delay(1);
yaccl[i]= y;
delay(1);
zaccl[i]= z;
delay(1);
totvect[i] = sqrt(((xaccl[i]-xavg)* (xaccl[i]-xavg))+ ((yaccl[i] - yavg)*(yaccl[i] - yavg)) + ((zval[i] - zavg)*(zval[i] - zavg)));
totave[i] = (totvect[i] + totvect[i-1]) / 2 ;
delay(50);
//cal steps 
if (totave[i]>threshhold && flag==0)
{
 steps=steps+1;
 flag=1;
}
else if (totave[i] > threshhold && flag==1)
{
//do nothing 
}
if (totave[i] <threshhold  && flag==1)
{
 flag=0;
}
// Serial.print("steps=");
// Serial.println(steps);
 return(steps);
}
delay(100); 
}


void wait_for_Max30102()
{
delay(2000);
}

void start_signal(){
pinMode(readLED, OUTPUT);
pinMode(pulseLED, OUTPUT);
digitalWrite(pulseLED, LOW); 
digitalWrite(readLED, LOW); 
delay(18);
digitalWrite(pulseLED, HIGH);
digitalWrite(readLED, HIGH);
pinMode(pulseLED, INPUT);
pinMode(readLED, INPUT);
digitalWrite(pulseLED, HIGH);
digitalWrite(readLED, HIGH);
}
//                                                                           MAX30102 FUNCTION 
void getPulseValues(){
Serial.begin(9600);
wait_for_Max30102();
start_signal();

if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
{
Serial.println(F("MAX30105 was not found. Please check wiring/power."));
while (1);
}
Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//while (Serial.available() == 0) ; //wait until user presses a key
Serial.read();
particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
//read the first 100 samples, and determine the signal range
for (byte i = 0 ; i < bufferLength ; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data
redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample(); //We're finished with this sample so move to next sample
//Serial.print(F("red="));
//Serial.print(redBuffer[i], DEC);
//Serial.print(F(", ir="));
//Serial.println(irBuffer[i], DEC);
}

//calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
for (int a = 0; a < 10; a++)
//while (1){
//dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
for (byte i = 25; i < 100; i++)
{
redBuffer[i - 25] = redBuffer[i];
irBuffer[i - 25] = irBuffer[i];
}
RTCSetupSensor();
//take 25 sets of samples before calculating the heart rate.
for (byte i = 75; i < 100; i++)
{
while (particleSensor.available() == false) //do we have new data?
particleSensor.check(); //Check the sensor for new data
digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
redBuffer[i] = particleSensor.getRed();
irBuffer[i] = particleSensor.getIR();
particleSensor.nextSample(); //We're finished with this sample so move to next sample

//send samples and calculation result to terminal program through UART
Serial.print(F("RED: "));
Serial.print("\t");
Serial.print(redBuffer[i], DEC);
Serial.println("");
      
Serial.print(F("IR: "));
Serial.print("\t");
Serial.print(irBuffer[i], DEC);
Serial.println("");

Serial.print(F("SPO2: "));
Serial.print("\t");
Serial.print(spo2, DEC);
Serial.print("\n");
Serial.println("");
      
checkSum = redBuffer[i] + irBuffer[i] + spo2; 
      
Serial.print(F("Checksum Byte: "));
Serial.print("\t");
Serial.print(checkSum, DEC); 
Serial.print("\t");
if((byte)checkSum == (byte)(redBuffer[i] + irBuffer[i] + spo2 )){Serial.print("(CHECKSUM_OK)");}
else {Serial.print("(CHECKSUM_ERROR)");}
Serial.println("\n");
Serial.println("\n");
Serial.println("");
Serial.println("");
Serial.println("");
delay(10);

uint16_t width = tft.width() - 1;
uint16_t height = tft.height() - 1;
uint8_t border = 5;
//fill box of wait to  black
tft.fillRect(10,200,315,215,BLACK);
//delay(1000);
displayTimeSensors();
tft.fillScreen(BLACK);
tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);
tft.setTextColor(WHITE);
tft.setTextSize(2);
tft.setCursor(206, 100);
tft.println("IR: ");
tft.setTextColor(YELLOW);
tft.setCursor(245, 100);
tft.println(irBuffer[i]);
delay(10);
tft.setTextColor(WHITE);
tft.setCursor(203, 146);
tft.println("RED:");
tft.setTextColor(YELLOW);
tft.setCursor(253, 145);
tft.println(redBuffer[i]);
tft.setCursor(40, 35);
tft.setTextColor(YELLOW);
tft.setTextSize(3);
tft.println("Pulse Oximeter");
tft.setTextSize(3);
tft.setCursor(2, 125);
tft.println(" SPO2: ");
tft.setTextColor(WHITE);
tft.setTextSize(4);
tft.setCursor(111, 122);
tft.print(spo2);
tft.setTextSize(2);
tft.setCursor(185, 130);
tft.print("%");
tft.fillRect(5,75,309,7,YELLOW);
//tft.drawRect(5,170,309,4,BLUE);
tft.drawRect(200,82,4,103,YELLOW);
tft.drawRect(204,125,110,4,YELLOW);
}
//delay(100);
//After gathering 25 new samples recalculate HR and SP02
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//}
}
//---------------------------------------------------------------RTC Setup----------------------------------------------------------
void RTCSetup(){
Wire.begin();
Serial.begin(9600);
setDS3231time(30,12,10,2,02,7,21);  // DS3231 seconds, minutes, hours, day, date, month, year

//battery indicator
pinMode(batteryPin,INPUT);
//TFT DEFINE
tft.setTextColor(WHITE);// text color
tft.fillScreen(BLACK);// background color
  
//Time Loop
for (int i = 0; i < 30; i += 6) {
//start loop
unsigned long starttime = millis();
unsigned long endtime = starttime;
    
while  ((endtime - starttime) <=40) // do this loop for up to 500mS=1/2 minute and 100 means 10 seconds
{
unsigned long loopcount;
tft.fillRoundRect(10, 10, 298, 60,5,  BLUE);
displayTime();
delay(500);
watchOptions();
loopcount = loopcount+1;
endtime = millis();
}
}

targetTime = millis() + 1000;
tft.fillRect(10, 10, 300, 60, BLUE);
tft.setCursor(15, 28);
tft.setTextColor(WHITE);
tft.setTextSize(3.5);
tft.println("WELCOME HERE...");
}
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year){
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year){
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void displayTime(){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10){
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10){
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  switch(dayOfWeek){
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
  }
   //tft display
   tft.fillScreen(BLACK);
   tft.drawRoundRect(0, 0, 320, 240, 5,YELLOW);
   tft.drawRoundRect(5, 5, 308, 88,5, YELLOW);
   tft.fillRoundRect(10, 10, 298, 80,5,  BLUE);
//hour
 tft.fillRect(12, 26,43,28,BLUE); //erase 
  if(hour<10)
  {
  tft.setCursor(15, 28); 
  tft.setTextSize(3);
  tft.print('0'); 
  tft.setCursor(40, 28);}
  else{
  tft.setCursor(15, 28);}
  tft.setTextSize(3);
  tft.print(hour);
  tft.setCursor(33, 30);
  tft.print(" : ");
//minute
  tft.fillRect(62, 26,40,28, BLUE); //erase 
  if(minute<10){
  tft.setCursor(64, 28); 
  tft.setTextSize(3);
  tft.print('0'); 
  tft.setCursor(82, 28);}
  else{
  tft.setCursor(64, 28);}
  tft.setTextSize(3);
  tft.println(minute);
  tft.setCursor(82, 28);
  tft.print(" : ");
//Second
  tft.fillRect(115, 26,40,28,BLUE); //erase
  if(second<10){
  tft.setCursor(117, 28); 
  tft.setTextSize(3);
  tft.print('0');
  tft.setCursor(140, 28);}
  else{
  tft.setCursor(117, 28);}
  tft.setTextSize(3);
  tft.print(second);
//PM,AM
if (hour>=0 && hour<12) d='A'; else {d='P';}
  tft.fillRect(161, 26,40,28,BLUE); //erase
  //tft.fillRect(15, 27,250,35,WHITE); 
  tft.setCursor(162, 28);
  tft.setTextSize(3);
  tft.print(d);
  tft.println('M');

//Day
  tft.fillRect(13, 61,55,26, BLUE); //erase
  tft.fillRect(68, 68,15,20, BLUE); //erase COMMA
  tft.setCursor(15, 63);
  tft.setTextSize(3);
  tft.setTextColor(YELLOW); 
  switch(dayOfWeek){
  case 1:
    Serial.println("Sunday");
    tft.print("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    tft.print("Mon");
    break;
  case 3:
    Serial.println("Tuesday");
    tft.print("Tues");
    break;
  case 4:
    Serial.println("Wednesday");
    tft.print("Wednes");
    break;
  case 5:
    Serial.println("Thursday");
    tft.print("Thurs");
    break;
  case 6:
    Serial.println("Friday");
    tft.print("Fri");
    break;
  case 7:
    Serial.println("Saturday");
    tft.print("Satur");
    break;
  }
  
  tft.setCursor(68, 64);
  tft.print(",");
//Date
tft.fillRect(84, 66,37,24, BLUE); //erase
if(dayOfWeek<10){
  tft.setCursor(86, 68);
  tft.setTextColor(YELLOW); 
  tft.setTextSize(2);
  tft.print('0');
  tft.setCursor(106, 68);}
  else{
  tft.setCursor(86, 68);}
  tft.setTextSize(2);
  tft.print(dayOfWeek);
  
  /*//tft.fillRect(90, 56,39,25, BLUE); //erase
  tft.setCursor(85, 65);
  tft.setTextSize(3);
  tft.print(dayOfWeek);*/
  tft.setCursor(122, 68);
  tft.print("/");
//Month
  tft.fillRect(136, 66,20,24, BLUE); //erase
  tft.setCursor(138, 68);
  tft.setTextSize(2);
  tft.print(month);
  tft.setCursor(155, 68);
  tft.print("/");
//Year
  tft.fillRect(174, 66,37,24 , BLUE); //erase
  tft.setCursor(176, 68);
  tft.setTextSize(2);
  tft.print(year);

Battery_Indicator();
}
void Battery_Indicator(){
  Voltage = analogRead(batteryPin);
  Actual_Voltage =( Voltage * (5.00 / 1023.00) * 2) / 2;
// >>100% 
    if(Actual_Voltage >= 6.6){ 
    tft.fillRect(277, 13, 31, 50, BLUE);
    tft.drawRect(277, 25, 25, 36, WHITE);   //large rectangle
    tft.fillRect(287, 15, 13, 10, WHITE);  //small rectangle
    tft.fillRect(280, 25, 25, 36, CYAN);
    tft.setCursor(282, 35);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("100%");
    }
// >>80%
    else if(Actual_Voltage > 6.2 && Actual_Voltage < 6.6){ 
    tft.fillRect(277, 13, 31, 50, BLUE);
    tft.drawRect(280, 25, 25, 36, WHITE);  //large rectangle
    tft.fillRect(287, 15, 13, 10, WHITE);  //small rectangle
    tft.fillRect(280, 31, 25, 31, CYAN);
    tft.setCursor(284, 38);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("80%");  
    }
// >>60%
    else if(Actual_Voltage > 5.8 && Actual_Voltage < 6.2){ 
    tft.fillRect(277, 13, 31, 50, BLUE);
    tft.drawRect(280, 25, 25, 36, WHITE);  //large rectangle
    tft.fillRect(287, 15, 13, 10, WHITE);  //small rectangle
    tft.fillRect(280, 37, 25, 25, CYAN);
    tft.setCursor(284, 42);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("60%"); 
    }
// >>40%
    else if(Actual_Voltage > 5.4 && Actual_Voltage < 5.8){ 
    tft.fillRect(277, 21, 31, 56, BLUE);
    tft.drawRect(280, 32, 25, 42, WHITE);  //large rectangle
    tft.fillRect(287, 23, 13, 10, WHITE);  //small rectangle
    tft.fillRect(281, 58, 23, 25, CYAN);
    tft.setCursor(284, 47);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("40%");  
    }
// >>20%
    else if(Actual_Voltage > 4.8 && Actual_Voltage < 5.4){ 
    tft.fillRect(277, 21, 31, 56, BLUE);    //erase
    tft.drawRect(260, 33, 26, 42, WHITE);  //large rectangle
    tft.fillRect(267, 23, 13, 10, WHITE);  //small rectangle
    tft.fillRect(261, 58, 23, 17, CYAN);
    tft.setCursor(264, 63);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("20%");
    }
// >><20%
    else if(Actual_Voltage <= 4.8){ 
    tft.fillRect(272, 21, 26, 56, BLUE); 
    tft.drawRect(265, 33, 26, 42, BLACK);  //large rectangle
    tft.fillRect(272, 23, 12, 10, BLACK);  //small rectangle
    tft.fillRect(266, 34, 23, 40, RED); 
    tft.setCursor(266, 63);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("<20%");    
    }  
}


void RTCSetupSensor(){
  Wire.begin();
  Serial.begin(9600);
  setDS3231time(30,12,10,2,02,7,21);  // DS3231 seconds, minutes, hours, day, date, month, year
  tft.setTextColor(WHITE);// text color
  targetTime = millis() + 1000;
}

void displayTimeSensors(){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10){
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10){
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  
   /*//tft display
tft.fillScreen(BLACK);
tft.drawRoundRect(0, 0, 320, 240, 5,YELLOW);
tft.drawRoundRect(5, 5, 308, 68,5, YELLOW);
tft.fillRoundRect(10, 10, 298, 60,5,  BLUE);*/
//hour
tft.fillRect(40, 193,56,40,BLACK); //erase 
  if(hour<10)
  {
  tft.setCursor(45, 198); 
  tft.setTextSize(4);
  tft.print('0'); 
  tft.setCursor(70, 198);}
  else{
  tft.setCursor(45, 198);}
  tft.setTextSize(4);
  tft.print(hour);
  tft.setCursor(82, 203);
  tft.setTextSize(3);
  tft.print(" : ");
//minute
  tft.fillRect(115, 198,43,30, BLACK); //erase 
  if(minute<10){
  tft.setCursor(120, 203); 
  tft.setTextSize(3);
  tft.print('0'); 
  tft.setCursor(137, 203);}
  else{
  tft.setCursor(120, 203);}
  tft.setTextSize(3);
  tft.println(minute);
  tft.setCursor(143, 203);
  tft.print(" : ");
//Second
  tft.fillRect(178, 198,46,30,BLACK); //erase
  if(second<10){
  tft.setCursor(182, 203); 
  tft.setTextSize(3);
  tft.print('0');
  tft.setCursor(205, 203);}
  else{
  tft.setCursor(182, 203);}
  tft.setTextSize(3);
  tft.print(second);
//PM AND AM
  if (hour>=0 && hour<12) d='A'; else {d='P';}
  tft.fillRect(235, 190,55,35,BLACK); //erase
  //tft.drawRect(15, 27,250,35,WHITE);   //loop1
  //tft.drawRect(35, 190,270,40,WHITE);   //loop2
  tft.setCursor(240, 196);
  tft.setTextSize(4);
  tft.print(d);
  tft.println('M');
  delay(1000);
}
void GLCView(){
  tft.fillScreen(BLACK);
  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 5;
  tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);  //MAIN RECTANGLE
  tft.setCursor(50, 35);
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.println("Glucose Level"); //HEADING
  tft.fillRect(5,73,309,7,YELLOW);  
  tft.setCursor(47, 100);
  tft.println("GLC");  //GLC DISPLAY
  
//LINES
  tft.fillRect(158,75,5,110,YELLOW);//seprate voltage and glucose 
  tft.fillRect(163,130,151,5,YELLOW); //seprate voltage and condition 
  tft.fillRect(164,138,145,44,BLUE); //condition Teller
//mg/dl
  tft.setCursor(92, 155); 
  tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.println("mg/dL");
//voltage level
  tft.setCursor(165, 96);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.println("Volts:");
//Start
  tft.setTextColor(GREY);
  tft.setTextSize(2);
  tft.setCursor(10, 210); 
  tft.println("Press Anywhere to start!");
  delay(1000);  
  waitOneTouch();
  tft.fillRect(10,205,315,215,BLACK);
  tft.setTextSize(4);
  tft.setCursor(115, 207); 
  tft.println("WAIT... ");
  RTCSetupSensor();
  for(int i =0; i<6; i++){
    getGLC();
    displayTimeSensors();
  }
  tft.fillRect(8,195,310,215,BLACK);
  tft.setTextColor(GREY);
  tft.setTextSize(2);
  tft.setCursor(8, 210); 
  tft.println("Press Anywhere to Go Back!");
}
void getGLC(){
  pinMode (Glucose_Pin, INPUT); // sensor pin INPUT
  Serial.begin(9600);
  statusSensor = analogRead (Glucose_Pin);
  voltage = statusSensor * (5.0 / 1023.0);
  int Glucose = (-42.77570755*voltage) + 217.01000488878688;  //From Python Model
  //Serial.println("Sensor Value is: ");
  //Serial.print(statusSensor);
  Serial.println("");
  Serial.print(F("Voltage: "));
  Serial.print("\t");
  Serial.print(voltage, DEC);
  Serial.println("");
      
  Serial.print(F("Glucose Level: "));
  Serial.print("\t");
  Serial.print(Glucose, DEC);
  Serial.println("\n");
  

  checkSum_Glu = Glucose + voltage; 
      
  Serial.print(F("Checksum Byte: "));
  Serial.print("\t");
  Serial.print(checkSum_Glu, DEC); 
  Serial.print("\t");
  if((byte)checkSum_Glu == (byte)(Glucose + voltage )){Serial.print("(CHECKSUM_OK)");}
  else {Serial.print("(CHECKSUM_ERROR)");}
  Serial.println("\n");
  Serial.println("\n");
  Serial.println("");
  
  delay(5000);
  
  //tftlcd display
  tft.fillScreen(BLACK);
  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 5;
  tft.drawRoundRect(border, border, (width - border * 2), 180,5, YELLOW);  //MAIN RECTANGLE
  tft.setCursor(50, 35);
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.println("Glucose Level"); //HEADING
  tft.fillRect(5,73,309,7,YELLOW);  
  tft.setCursor(47, 100);
  tft.println("GLC");  //GLC DISPLAY
  
//LINES
  tft.fillRect(158,75,5,110,YELLOW);//seprate voltage and glucose 
  tft.fillRect(163,130,151,5,YELLOW); //seprate voltage and condition 
  tft.fillRect(164,138,147,44,BLUE); //condition Teller
//TELL CONDITION
  tft.setCursor(167, 150); 
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  if(Glucose >54 && Glucose <70){
  tft.println("Low GLC");
  }
  else if(Glucose >= 70 && Glucose <125){
  tft.println("Normal");
  }
  else if(Glucose >= 125 && Glucose <200){
  tft.println("Pre-Diabetes");
  }
  else if(Glucose >= 200){
  tft.println("Diabetes");
  }
  else {
  tft.println("INCORRECT");
  }
  //mg/dL
  tft.setCursor(12, 140);
  tft.setTextSize(4);
  tft.setTextColor(WHITE);
  tft.println(Glucose);
  tft.setCursor(92, 155); 
  tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.println("mg/dL");

//voltage level
  tft.setCursor(165, 96);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.println("Volts:");
//volatge value
  tft.setCursor(240, 95);
  tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.println(voltage);
/*//SHOW unit
tft.setCursor(160, 160);
tft.setTextSize(1);
//tft.setTextColor(YELLOW);
tft.println("V");*/
}
