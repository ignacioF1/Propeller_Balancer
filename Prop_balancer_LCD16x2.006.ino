#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define interrupts() sei()

//CONSTANTS//
#define BALANCED_ACCEL 0.25       //Max acceleration permitted to consider prop balanced
#define SAMPLES_ANGLE_AVG 600     //Samples taken to calculate angle average
#define LCD_UPDATE_COUNT 200      //Main loop pases till the LCD is updated
#define ACCEL_ADJ 0.001           //Acceleration adjust increment/decrement
//
//Initialize library object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

//CONNECTIONS
//LCD 1602A (5V, GND, SCL(A5), SDA(A4))
//IR sensor HW-201 (5V, GND, pin 8)
//ADXL345 (3.3V, GND, SCL(A5), SDA(A4))
//ADXL345 position -> align sensor's x & y coordinates to one corner, and the IR sensor should be where y axis is pointing, in the middle of the fan. 

//GLOBAL VARIABLES
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Make custom character:
byte Check[8] =
{
0b00000,
0b00000,
0b00001,
0b00011,
0b10110,
0b11100,
0b01000,
0b00000
};
bool int_updated = true;
float xc, actualAccel;
float maxAccel = 0;
int count = 0;
int angProm = 0;
float maxAccelAngle = 0;
float actualAngle;
float promAccelAngle_x = 0;
float promAccelAngle_y = 0;
float s_numerator;
float c_denominator;
unsigned int actualPeriod = 0;
unsigned int lastPeriod = 65535;
 
//============================================================
// Timer 1 INTERRUPT
//============================================================
ISR(TIMER1_CAPT_vect) {   //Input capture interrupt ISR
  lastPeriod = ICR1;      //Get timer1 input capture register value (between 0 and 65535)
  TCNT1 = 0;              //Restart timer
  int_updated = true;     //Update flag
}
//============================================================

void setup() {
  //Serial Port start
  //Serial.begin(9600);
  
  //TIMER 1 as input capture config
  //Prescaler -> /1, /8, /64, /256, /1024
  // 16MHz / 64 => 1bit 0,000004s (4us)
  //TCCR1B b7=1(noise cancel) b6=0(falling edge) b2-0(prescaler)
  //TIMSK1 b5=1(enable input capture)
  cli();                //Disable interrupts
  TCCR1A = 0;           //Initialize timer1 mode
  TCCR1B = 0;           
  TCCR1B |= 0b10000011; //Set noise canceler, falling edge, 64 prescaler
  TIMSK1 |= 0b00100000; //Enable input capture interrupt
  TCNT1 = 0;            //Initialize timer1/counter register
  sei();                //Enable interrupts

  //IR input pin initialization
  pinMode(8,INPUT);
  
  //LCD initialization
  lcd.begin(16,2);
  lcd.backlight(); 
  lcd.clear();
  lcd.setCursor(0,0);
  //Create new character
  lcd.createChar(0, Check);
  //

  //ADXL345 initialization
  //Verify connection between sensor and controller
  if(!accel.begin()) {
    //Serial.println("The accel sensor didn't start!");
    while(1);
  }
  /* Set the data rate */
  //accel.setDataRate(ADXL345_DATARATE_400_HZ);
  accel.setDataRate(ADXL345_DATARATE_200_HZ);
  //accel.setDataRate(ADXL345_DATARATE_100_HZ);
  /* Set the range */
  //accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_2_G);
  //Calibration
  sensors_event_t event;
  accel.getEvent(&event);
  xc = event.acceleration.x;
  //
}

void loop() {
  //Read acceleration
  sensors_event_t event;
  accel.getEvent(&event);
  actualAccel = event.acceleration.x - xc;
  //Serial.println(actualAccel);
  //
  actualPeriod = TCNT1;
  if (actualAccel > maxAccel) {
    maxAccel += ACCEL_ADJ;
    actualAngle = ((float)actualPeriod/(float)lastPeriod)*360;
    updateAngle();
  } else if (actualAccel > 0 && actualAccel < maxAccel) {
    maxAccel -= ACCEL_ADJ;
  } else if (actualAccel < 0 && abs(actualAccel) > maxAccel) {
    maxAccel += ACCEL_ADJ;
    actualAngle = ((float)actualPeriod/(float)lastPeriod)*360;
    if (actualAngle >= 180) {
      actualAngle -= 180;  
    } else {
      actualAngle += 180;
    }
    updateAngle();
  } else if (actualAccel < 0 && abs(actualAccel) < maxAccel) {
    maxAccel -= ACCEL_ADJ;
  }
  //Serial.println(lastPeriod);
  
  //Update LCD after some time
  count++;
  if (count >= LCD_UPDATE_COUNT) {
    updateLCD();
    count = 0;
    int_updated = false;
  }
}

void updateAngle() {
  if (actualAngle >= 0 && actualAngle <= 360) {
    angProm++;
    if (angProm == SAMPLES_ANGLE_AVG){
      s_numerator = promAccelAngle_y / SAMPLES_ANGLE_AVG;
      c_denominator = promAccelAngle_x / SAMPLES_ANGLE_AVG;
      if (s_numerator > 0) {
        maxAccelAngle = atan2 (s_numerator , c_denominator) * 180 / M_PI;
      }else{
        maxAccelAngle = (atan2 (s_numerator , c_denominator) * 180 / M_PI) + 360;
      }
      angProm = 0;
      promAccelAngle_x = 0;
      promAccelAngle_y = 0;
    }
    promAccelAngle_x += cos (actualAngle * M_PI / 180);
    promAccelAngle_y += sin (actualAngle * M_PI / 180);
  }
}
      
void updateLCD() {
  //Update LCD
  lcd.clear();
  // Line 1
  lcd.setCursor(0,0);
  lcd.print("Max accel:");
  lcd.setCursor(11,0);
  lcd.print(maxAccel);
  if(int_updated && maxAccel <= BALANCED_ACCEL) {
    lcd.setCursor(15,0);
    lcd.write(byte(0));
  }
  // Line 2
  if (int_updated){
    lcd.setCursor(0,1);
    lcd.print("Heavy at:");
    lcd.setCursor(10,1);
    lcd.print(maxAccelAngle);
    lcd.setCursor(15,1);
    lcd.print(char(223));
  }else{
    lcd.setCursor(0,1);
    lcd.print("NO RPM!");
  }
}
