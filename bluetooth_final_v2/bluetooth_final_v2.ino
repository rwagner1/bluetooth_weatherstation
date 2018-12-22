/***************************************************************************
  
  Combined codes for Bosch sensor and UV sensor including sleep mode options

  BOSCH SENSOR
  =======================
  
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

   UV SENSOR
   =====================
   ML8511 UV Sensor Read Example
   By: Nathan Seidle
   SparkFun Electronics
   Date: January 15th, 2014
   License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  
   The ML8511 UV Sensor outputs an analog signal in relation to the amount of UV light it detects.
  
   Connect the following ML8511 breakout board to Arduino:
   3.3V = 3.3V
   OUT = A0
   GND = GND
   EN = 3.3V
   3.3V = A1
   These last two connections are a little different. Connect the EN pin on the breakout to 3.3V on the breakout.
   This will enable the output. Also connect the 3.3V pin of the breakout to Arduino pin 1.
  
   This example uses a neat trick. Analog to digital conversions rely completely on VCC. We assume
   this is 5V but if the board is powered from USB this may be as high as 5.25V or as low as 4.75V:
   http://en.wikipedia.org/wiki/USB#Power Because of this unknown window it makes the ADC fairly inaccurate
   in most cases. To fix this, we use the very accurate onboard 3.3V reference (accurate within 1%). So by doing an
   ADC on the 3.3V pin (A1) and then comparing this against the reading from the sensor we can extrapolate
   a true-to-life reading no matter what VIN is (as long as it's above 3.4V).
  
   Test your sensor by shining daylight or a UV LED: https://www.sparkfun.com/products/8662
  
   This sensor detects 280-390nm light most effectively. This is categorized as part of the UVB (burning rays)
   spectrum and most of the UVA (tanning rays) spectrum.
  
   There's lots of good UV radiation reading out there:
   http://www.ccohs.ca/oshanswers/phys_agents/ultravioletradiation.html

 
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define BAUDRATE 9600 // default baud rate 38400
#define SEALEVELPRESSURE_HPA (1013.25)
#define NDATA 4           // elements stored in data array, temp, press, hum, uv
#define NREADS 16         // number of readings for averaging

// choose communcation channel i2c for Bosch sensor
Adafruit_BME280 bme; 

int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
int wakePin = 2;                 // pin used for waking up in case interrupt at D2 is used

void setup() {
  
  Serial.begin(BAUDRATE);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);          // use board LED to monitor sleep mode
  pinMode(wakePin, INPUT);        // pin 2 is interrupt input pin
  digitalWrite(wakePin, HIGH);    // activate pull-up,pin is active low
  
  boschSetup();
 // Serial.println(F("Bosch sensor setup complete"));
  
  uvSetup();
 // Serial.println(F("UV sensor setup complete"));

}

void loop() { 
      
      sleepSetup();                                     // sleep function called here    
     
      float uvIntensity = uvCalc();                   // read and send sensor data after wake up  
      sendData(uvIntensity);
}


/* functions - Bosch sensor */

// setup Bosch sensor
void boschSetup() {

  // check if sensor is connected, if not use i2c_scanner program.
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  }

/* functions - UV sensor */

// setup UV sensor
void uvSetup() {
 
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  }

float uvCalc() {

    // read sensor ACD output at A0
    int uvLevel = averageAnalogRead(UVOUT);
    int refLevel = averageAnalogRead(REF_3V3);
 
    // use the 3.3V power pin as a reference to get a very accurate output value from sensor
    float outputVoltage = 3.3 / refLevel * uvLevel;

    // convert voltage to a UV intensity level
    float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); 

    // off-zero correction
    if (uvIntensity < 0) {
      uvIntensity = 0;
      }
      
//  Serial.print("output: ");
//  Serial.println(refLevel);
//  Serial.print("ML8511 output: ");
//  Serial.println(uvLevel);
//  Serial.print(" / ML8511 voltage: ");
//  Serial.println(outputVoltage);
//  Serial.print(" / UV Intensity (mW/cm^2): ");
//  Serial.println(uvIntensity);
//  Serial.println();

    
    return uvIntensity;
 
  }

/* send data to bluetooth */

void sendData(float uvValue) {

  String incomingByte = "";
  float data[4];
  data[0] = bme.readTemperature();
  data[1] = bme.readPressure() / 100.0F;
  data[2] = bme.readHumidity();
  data[3] = uvValue;
  
  if (Serial.available() > 0) {  
    
    while (Serial.available()) {
      // read the incoming byte:
      incomingByte += (char)Serial.read();
      delay(10);  
      }
      
    int receive = 0;
    receive = incomingByte.toInt();

    
    if (receive == 1) {                               // smartphone sends "1" to retrieve data
      
        for (int i = 0; i < NDATA; i++) {             // send four data values to smartphone
          if (i == 0) {                               // precede data package with "start" string to check for correct transmission
            Serial.print("start;");
            }          
          Serial.print(data[i]);
          Serial.print(";");
          
          if (i == NDATA-1) {                         // "stop" string to check for correct transmission
            Serial.print("stop;");
            }
        }
    }
    incomingByte = "";
 }
}
  
// create average value for analog readings
int averageAnalogRead(int pinToRead) {
  
  byte numberOfReadings = NREADS;
  unsigned int runningValue = 0; 

  for (int i = 0 ; i < numberOfReadings ; i++) {
    
    runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;
    
    }
  return(runningValue);  
}

//The Arduino Map function but for floats
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sleepSetup()        
{
    set_sleep_mode(SLEEP_MODE_IDLE);      // sleep mode is set here
    
    sleep_enable();                      // enables the sleep bit in the mcucr register - safety pin
    
    power_adc_disable();                // disable all the modules which aren't needed 
    power_spi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();
    power_twi_disable();

    digitalWrite(13, HIGH);                       // turn on LED to indicate sleep mode - just for debug purposes, otherwise we can save the power!
  //  Serial.print("Going to sleep. \n");         // Don't put any Serial.print in front of sleep_mode(), it will wake it up immediately after going to sleep
    sleep_mode();                                 // here the device is actually put to sleep!
    
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    sleep_disable();                    // first thing after waking from sleep: if not done in ISR, disable sleep after waking up    
    power_all_enable();                 // power up all modules
   // Serial.print("Just woke up \n");    // when it wakes up due to interrupt, code execution continues here
    digitalWrite(13, LOW);              // turn off LED to indicate wake-up mode  
    
}

// Interrupt Servie Routine (ISR) to wake up cpu
/*void pinInterrupt() {

    sleep_disable();      // important. interrupt could be called between attachInterrupt and sleep_cpu in the main loop. if sleep_disable() is not called before, the intterupt
                          // will be detached, cpu going to sleep mode and no longer able to be woken up except using reset
    detachInterrupt(0);   // disable interrupt, effectively debounces interrupt signal to prevent multiple interrupt calls
    }*/
