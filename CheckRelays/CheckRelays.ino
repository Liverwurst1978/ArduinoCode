/*
Arduino relays
*/

const int relayOne = 2; //Relay One
const int relayTwo = 7; //Relay Two
const int relayThree = 8; //Relay Three
const int relayFour = 10;
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(relayFour, OUTPUT);
  digitalWrite(relayFour, HIGH);
  pinMode(relayOne, OUTPUT);
  digitalWrite(relayOne, HIGH);
   
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(relayFour, LOW);
  delay(3000);
  digitalWrite(relayFour, HIGH);
  delay(3000);
  digitalWrite(relayOne, LOW);
  delay(3005);
  digitalWrite(relayOne, HIGH);
  delay(3005);
}
