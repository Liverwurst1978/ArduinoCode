
//DHT TMP AND HUMIDITY SENSOR NOTES
// DHT 22 Humidity and Temp REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
// DHT22 SENSOR WIRING NOTES 
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor -NOT NEEDED


// LIBRARIES THAT NEED TO BE INCLUDED - START
#include "DHT.h"
#include <Ethernet.h>
// LIBRARIES THAT NEED TO BE INCLUDED - END


// DHT SENSOR PIN DEFINITION AND SETUP 
#define DHTPIN 2     // Digital pin connected to the DHT sensor -DO NOT USE PIN 15-
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//DHT SENSOR INFORMATION
DHT dht(DHTPIN, DHTTYPE);



//REMOTE RELAY BOARD PIN DEFINITION AND SETUP - DEFINE RELAY PINS AND RELAY NUMBERS(NAMES)-CAN BE WIRED TO ANY DO PIN
const int relayOne = 6; //Relay One (Duty Cycle Timer #1)
const int relayTwo = 7; //Relay Two (Humidifier Transducer)
const int relayThree = 8; //Relay Three (Duty Cycle Timer #2)
const int relayFour = 9; //Relay Four (Humidifier Tank Fan)




//HUMIDIFIER TANK FLOAT PIN DEFINITION AND SETUP - DEFINE INPUT PIN AND INPUT NAMES
const int buttonPin = 3;    // the number of the pushbutton pin Used as Humidifier Float
int buttonState = 0;        // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers



//VH400 PIN DEFINITION AND SETUP -VH400 Soil Volumetric Water Content Sensor
//VH DEFINE SENSOR PIN
//const int VhSensorPin = A0;


//WEB SERVER SETUP -START
// assign a MAC address for the Ethernet controller.
// fill in your address here:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
// assign an IP address for the controller:
IPAddress ip(192, 168, 1, 30);


// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(420);
//WEB SERVER SETUP -END

//----------------------------VOID SETUP START------------------------------------------
void setup() {

 Serial.begin(9600);

  
  
  //REFERENCE dht.h LIBRARY
  dht.begin();
 
//WEB SERVER SETUP
  
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields

//Start the Ethernet connection
  Ethernet.begin(mac, ip);

//WEB SERVER SETUP -START
 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start listening for clients
  server.begin();
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());

//WEB SERVER SETUP -END

//VH-400 SETUP -START
//  Serial.begin(9600);
//  pinMode(VhSensorPin,INPUT);
//VH-400 SETUP -END

 Serial.println("Aces...");
 Serial.println();
// REMOTE RELAY BOARD SETUP -START
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

// REMOTE RELAY BOARD SETUP -END


//HUMIFIER TANK FLOAT SETUP -START
  //NORMALLY OPEN CONTACTS = TANK EMPTY
  pinMode(buttonPin, INPUT);
  //low equals tank empty switch OPEN (EMPTY)
//HUMIFIER TANK FLOAT SETUP -END
}

//----------------------------VOID SETUP END------------------------------------------


//----------------------------VOID LOOP START------------------------------------------
void loop() {
 
 //WEB SERVER LOOP -START
 // listen for incoming clients
  EthernetClient client = server.available();
  //WEB SERVER LOOP -END

  
 //DHT22 SENSOR LOOP -START
  // Wait a few seconds between measurements
  delay(10000);
  // Sensor readings may also be up to 2 seconds

  //h=humidity, t=tempC, f=tempF,
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

//DHT SERIAL TEXT ERROR
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
  Serial.print(F("Humidity: %"));
  Serial.println(h);
  Serial.print(F("Temperature: "));
  Serial.print(f);
  Serial.println(F("°F "));
  //Serial.print(t);
  //Serial.print(F("°C "));
  //Serial.print(F("°F  Heat index: "));
  //Serial.println(F("°F"));
  // Serial.print(hic);
  //Serial.print(F("°C "));
  // Serial.print(hif);

//DHT22 SENSOR LOOP -END
 
//WEB SERVER SETUP CREATE WEBPAGE LOOP-START 
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
         client.println("HTTP/1.1 200 OK");
         client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 seconds
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
         
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
           //client.print("analog input ");
           //client.print(analogChannel);
           //client.print(" is ");
           //client.print(sensorReading);
           //client.println("<br />");
          
            client.print("<h2>");
            client.print(F("Humidity"));
            client.print("</h2>");
            client.print("<p>");
            client.print(h);
            client.println("%");
            client.println("</p>");

            client.print("<h2>");
            client.print(F("Temperature"));
            client.print("</h2>");
            client.print("<p>");
            client.print(f);
            client.println(" DegF");
            client.print("</p>");

            client.print("<h2>");
            client.println("Humidifier Water Tank Level");
            client.println("<p>");
            client.println("</h2>");
            client.println("<p>");
            client.print("Water Tank Level State is ");
            client.println(lastButtonState);
            client.println("</p>");
            client.print("<h4>");
            client.println("1=Water Tank OK, 0=Water Tank EMPTY");
            client.println("</h4>");
            
            
          }
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
//WEB SERVER SETUP CREATE WEBPAGE LOOP-END

//TANK FLOAT PIN LOOP -START
  int reading = digitalRead(buttonPin);
//TANK FLOAT PIN LOOP -END
  

//HUMIDIFER OPERATION SERIAL TEXT OUTPUT
  // Serial.println(reading);
  Serial.print("Humidifier Water Tank Status is ");
  Serial.println(buttonState);
  Serial.println("1=Humidifier Water Tank OK,0=Humidifier Water Tank EMPTY");
  Serial.println();
 
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
    Serial.println("Humidity above 80, Humidifier Disabled");
    Serial.println();
    //digitalWrite(relayOne, HIGH);
    //digitalWrite(relayThree, HIGH);
    digitalWrite(relayFour, HIGH);
    digitalWrite(relayTwo, HIGH);
  }
 //HUMIDIFIER TRANSDUCER AND TANK FAN CONTROL LOGIC -END

 
//HUMIDIFER TANK SENSOR DEBOUNCE LOOP -START
  if (reading != buttonState) {
    buttonState = reading;
  }
  if (reading != lastButtonState) {
    // reset the debouncing timer
    //Serial.println(lastDebounceTime);
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
 
       }
  }
//HUMIDIFER TANK SENSOR DEBOUNCE LOOP -END

//----------------------------VOID LOOP END------------------------------------------

//VH400 VEGTRONIX MOISTURE SENSOR SETUP -START
//  /*
//      VHC Code starts here
//
//    float voltage;
//    float Moisture;
//    float VWC;
//    VWC = readVH400(VhSensorPin);

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

 //VH400 SERIAL TEXT OUTPUT
 Serial.print("VWC: ");
 Serial.println(VWC);
 
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
