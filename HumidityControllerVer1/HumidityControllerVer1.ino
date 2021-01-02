// Written by ladyada, public domain

// DHT 22 Humidity and Temp REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

// LIBRARIES THAT NEED TO BE INCLUDED - START
#include "DHT.h"
// LIBRARIES THAT NEED TO BE INCLUDED - END


// PIN DEFINITIONS 
#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Pin 15 can work but DHT must be disconnected during program upload. DO NOT USE PIN 15

// DHT SENSOR SETUP
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//REMOTE RELAY BOARD SETUP - DEFINE RELAY PINS AND RELAY NUMBERS(NAMES)-CAN BE WIRED TO ANY DO PIN
const int relayOne = 5; //Relay One (Duty Cycle Timer #1)
const int relayTwo = 6; //Relay Two (Humidifier Transducer)
const int relayThree = 7; //Relay Three (Duty Cycle Timer #2)
const int relayFour = 8; //Relay Four (Humidifier Tank Fan)

//HUMIDIFIER TANK FLOAT SETUP - DEFINE INPUT PIN AND INPUT NAMES
const int buttonPin = 3;    // the number of the pushbutton pin Used as Humidifier Float
int buttonState = 0;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// DHT22 SENSOR WIRING NOTES 
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.

//VH400 SENSOR PIN DEFINITION
//VH400 Soil Volumetric Water Content Sensor
//VH Sensor Pin
//const int VhSensorPin = A0;

//DHT SENSOR INFORMATION
DHT dht(DHTPIN, DHTTYPE);

void setup() {

//VH-400 SETUP
  Serial.begin(9600);
  //pinMode(VhSensorPin,INPUT);
  
  //SERIAL TEXT START
  Serial.println(F("Aces..."));
  
  //REFERENCE dht.h LIBRARY
  dht.begin();

// REMOTE RELAY BOARD SETUP
  pinMode(relayOne, OUTPUT);
  digitalWrite(relayOne, HIGH);
  //digitalWrite HIGH turns the relay OFF
  
  pinMode(relayTwo, OUTPUT);
  digitalWrite(relayTwo, HIGH);
  //digitalWrite HIGH turns the relay OFF
  
  pinMode(relayThree, OUTPUT);
  digitalWrite(relayThree, HIGH);
  //digitalWrite HIGH turns the relay OFF
  
  pinMode(relayFour, OUTPUT);
  digitalWrite(relayFour, HIGH);
  //digitalWrite HIGH turns the relay OFF

//HUMIFIER TANK FLOAT SETUP -NORMALLY OPEN = TANK EMPTY
  pinMode(buttonPin, INPUT);
  //low equals tank empty switch OPEN (EMPTY)
}


void loop() {
 
 //DHT22 SENSOR LOOP -START
  // Wait a few seconds between measurements
  delay(10000);
  // Sensor readings may also be up to 2 seconds
  
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

//DHT SERIAL TEXT OUTPUT
  Serial.print(F("Humidity:% "));
  Serial.println(h);
  Serial.print(F("Temperature: "));
  //Serial.print(t);
  //Serial.print(F("°C "));
  Serial.println(f);
  // Serial.print(F("°F  Heat index: "));
  //  Serial.print(hic);
  // Serial.print(F("°C "));
  // Serial.print(hif);
  //  Serial.println(F("°F"));
 
  
  //DHT22 SENSOR LOOP -END
 
  
  //TANK FLOAT PIN
  int reading = digitalRead(buttonPin);
  
  //HUMIDIFER OPERATION SERIAL TEXT OUTPUT
  // Serial.println(reading);
  Serial.print("Humidifier Water Tank Status is ");
  Serial.println(buttonState);
  Serial.println("Humidifer Tank Status= 1=Humidifier Water Tank OK,0=Humidifier Water Tank EMPTY");
  Serial.println("");
 
  //HUMIDIFIER TRANSDUCER AND TANK FAN CONTROL LOGIC -START
  if (h <= 39 && buttonState == HIGH) {
    Serial.println(h);
    // Low output turns on a relay
    digitalWrite(relayFour, LOW);
    digitalWrite(relayTwo, LOW);


  } else if (h <= 39 && buttonState == LOW) {
    Serial.println("**ALARM:HUMIDIFIER TANK IS EMPTY-HUMIDIFIER DISABLED-FILL TANK TO CLEAR ALARM**");
    digitalWrite(relayFour, HIGH);
    digitalWrite(relayTwo, HIGH);
  }
  else if (h > 80) {
    Serial.println("humidity above 80, humidifier is off");
    //digitalWrite(relayOne, HIGH);
    //digitalWrite(relayThree, HIGH);
    digitalWrite(relayFour, HIGH);
    digitalWrite(relayTwo, HIGH);
  }
//HUMIDIFIER TANK FLOAT CONTROL LOGIC -START
  if (reading != buttonState) {
    buttonState = reading;
  }
  if (reading != lastButtonState) {
    // reset the debouncing timer
    //Serial.println(lastDebounceTime);
    lastDebounceTime = millis();
  }
//HUMIDIFER TANK SENSOR DEBOUNCE
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
  //HUMIDIFIER TANK FLOAT CONTROL LOGIC -START
       }
  }



//VH400 VEGTRONIX MOISTURE SENSOR SETUP -START
  /*
      VHC Code starts here

    float voltage;
    float Moisture;
    float VWC;
    VWC = readVH400(VhSensorPin);

    Serial.print("VWC: ");
    Serial.println(VWC);
  */

}

float readVH400(int analogPin) {
  // This function returns Volumetric Water Content by converting the analogPin value to voltage
  // and then converting voltage to VWC using the piecewise regressions provided by the manufacturer
  // at http://www.vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml

  // NOTE: You need to set analogPin to input in your setup block
  //   ex. pinMode(<analogPin>, INPUT);
  //   replace <analogPin> with the number of the pin you're going to read from.

  // Read value and convert to voltage
  int sensor1DN = analogRead(analogPin);
  float sensorVoltage = sensor1DN * (3.0 / 1023.0);
  float VWC;

  // Calculate VWC
  if (sensorVoltage <= 1.1) {
    VWC = 10 * sensorVoltage - 1;
  } else if (sensorVoltage > 1.1 && sensorVoltage <= 1.3) {
    VWC = 25 * sensorVoltage - 17.5;
  } else if (sensorVoltage > 1.3 && sensorVoltage <= 1.82) {
    VWC = 48.08 * sensorVoltage - 47.5;
  } else if (sensorVoltage > 1.82) {
    VWC = 26.32 * sensorVoltage - 7.89;
  }
  return (VWC);
}

struct VH400 {
  double analogValue;
  double analogValue_sd;
  double voltage;
  double voltage_sd;
  double VWC;
  double VWC_sd;
};

struct VH400 readVH400_wStats(int analogPin, int nMeasurements = 100, int delayBetweenMeasurements = 50) {
  // This variant calculates the mean and standard deviation of 100 measurements over 5 seconds.
  // It reports mean and standard deviation for the analog value, voltage, and WVC.

  // This function returns Volumetric Water Content by converting the analogPin value to voltage
  // and then converting voltage to VWC using the piecewise regressions provided by the manufacturer
  // at http://www.vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml

  // NOTE: You need to set analogPin to input in your setup block
  //   ex. pinMode(<analogPin>, INPUT);
  //   replace <analogPin> with the number of the pin you're going to read from.

  struct VH400 result;

  // Sums for calculating statistics
  int sensorDNsum = 0;
  double sensorVoltageSum = 0.0;
  double sensorVWCSum = 0.0;
  double sqDevSum_DN = 0.0;
  double sqDevSum_volts = 0.0;
  double sqDevSum_VWC = 0.0;

  // Arrays to hold multiple measurements
  int sensorDNs[nMeasurements];
  double sensorVoltages[nMeasurements];
  double sensorVWCs[nMeasurements];

  // Make measurements and add to arrays
  for (int i = 0; i < nMeasurements; i++) {
    // Read value and convert to voltage
    int sensorDN = analogRead(analogPin);
    double sensorVoltage = sensorDN * (3.0 / 1023.0);

    // Calculate VWC
    float VWC;
    if (sensorVoltage <= 1.1) {
      VWC = 10 * sensorVoltage - 1;
    } else if (sensorVoltage > 1.1 && sensorVoltage <= 1.3) {
      VWC = 25 * sensorVoltage - 17.5;
    } else if (sensorVoltage > 1.3 && sensorVoltage <= 1.82) {
      VWC = 48.08 * sensorVoltage - 47.5;
    } else if (sensorVoltage > 1.82) {
      VWC = 26.32 * sensorVoltage - 7.89;
    }

    // Add to statistics sums
    sensorDNsum += sensorDN;
    sensorVoltageSum += sensorVoltage;
    sensorVWCSum += VWC;

    // Add to arrays
    sensorDNs[i] = sensorDN;
    sensorVoltages[i] = sensorVoltage;
    sensorVWCs[i] = VWC;

    // Wait for next measurement
    delay(delayBetweenMeasurements);
  }

  // Calculate means
  double DN_mean = double(sensorDNsum) / double(nMeasurements);
  double volts_mean = sensorVoltageSum / double(nMeasurements);
  double VWC_mean = sensorVWCSum / double(nMeasurements);

  // Loop back through to calculate SD
  for (int i = 0; i < nMeasurements; i++) {
    sqDevSum_DN += pow((DN_mean - double(sensorDNs[i])), 2);
    sqDevSum_volts += pow((volts_mean - double(sensorVoltages[i])), 2);
    sqDevSum_VWC += pow((VWC_mean - double(sensorVWCs[i])), 2);
  }
  double DN_stDev = sqrt(sqDevSum_DN / double(nMeasurements));
  double volts_stDev = sqrt(sqDevSum_volts / double(nMeasurements));
  double VWC_stDev = sqrt(sqDevSum_VWC / double(nMeasurements));

  // Setup the output struct
  result.analogValue = DN_mean;
  result.analogValue_sd = DN_stDev;
  result.voltage = volts_mean;
  result.voltage_sd = volts_stDev;
  result.VWC = VWC_mean;
  result.VWC_sd = VWC_stDev;

  // Return the result
  return (result);
}
//VH400 VEGTRONIX MOISTURE SENSOR SETUP -END
