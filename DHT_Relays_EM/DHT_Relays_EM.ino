// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"

#define DHTPIN 4     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

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

//VH Sensor Pin
const int VhSensorPin = A0;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  
  Serial.begin(9600);
  pinMode(A0,INPUT);
  Serial.println(F("Go Gophers!!"));

  dht.begin();
  }


void loop() {
  // Wait a few seconds between measurements.
  delay(5000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
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


 /* 
  *  VHC Code starts here
  */
  float voltage;
  float Moisture;
  float VWC;
  VWC = readVH400(A0);
  
  Serial.print("VWC: ");
  Serial.println(VWC);
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
  float sensorVoltage = sensor1DN*(3.0 / 1023.0);
  float VWC;
  
  // Calculate VWC
  if(sensorVoltage <= 1.1) {
    VWC = 10*sensorVoltage-1;
  } else if(sensorVoltage > 1.1 && sensorVoltage <= 1.3) {
    VWC = 25*sensorVoltage-17.5;
  } else if(sensorVoltage > 1.3 && sensorVoltage <= 1.82) {
    VWC = 48.08*sensorVoltage-47.5;
  } else if(sensorVoltage > 1.82) {
    VWC = 26.32*sensorVoltage-7.89;
  }
  return(VWC);
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
    double sensorVoltage = sensorDN*(3.0 / 1023.0);
        
    // Calculate VWC
    float VWC;
    if(sensorVoltage <= 1.1) {
      VWC = 10*sensorVoltage-1;
    } else if(sensorVoltage > 1.1 && sensorVoltage <= 1.3) {
      VWC = 25*sensorVoltage-17.5;
    } else if(sensorVoltage > 1.3 && sensorVoltage <= 1.82) {
      VWC = 48.08*sensorVoltage-47.5;
    } else if(sensorVoltage > 1.82) {
      VWC = 26.32*sensorVoltage-7.89;
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
  double DN_mean = double(sensorDNsum)/double(nMeasurements);
  double volts_mean = sensorVoltageSum/double(nMeasurements);
  double VWC_mean = sensorVWCSum/double(nMeasurements);

  // Loop back through to calculate SD
  for (int i = 0; i < nMeasurements; i++) { 
    sqDevSum_DN += pow((DN_mean - double(sensorDNs[i])), 2);
    sqDevSum_volts += pow((volts_mean - double(sensorVoltages[i])), 2);
    sqDevSum_VWC += pow((VWC_mean - double(sensorVWCs[i])), 2);
  }
  double DN_stDev = sqrt(sqDevSum_DN/double(nMeasurements));
  double volts_stDev = sqrt(sqDevSum_volts/double(nMeasurements));
  double VWC_stDev = sqrt(sqDevSum_VWC/double(nMeasurements));

  // Setup the output struct
  result.analogValue = DN_mean;
  result.analogValue_sd = DN_stDev;
  result.voltage = volts_mean;
  result.voltage_sd = volts_stDev;
  result.VWC = VWC_mean;
  result.VWC_sd = VWC_stDev;

  // Return the result
  return(result);
}
