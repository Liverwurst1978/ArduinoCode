const int btn_pin = 2;
const int led_pin = 6;
 
// We need to remember the previous button state between loops
int btn_prev = HIGH;
 
// Remember the LED state between loops
int led_state = LOW;
 
void setup() {
  
  // Set up pins
  pinMode(btn_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  
  // Set LED to default off
  digitalWrite(led_pin, led_state);
}
void loop() {
  
  int btn_state;
  
  // Read current button state
  btn_state = digitalRead(btn_pin);
  
  // If the button was previously HIGH and now LOW, it's been pressed
  if ( (btn_prev == HIGH) && (btn_state == LOW) ) {
    
    // Toggle the LED
    if ( led_state == LOW ) {
      led_state = HIGH;
    } else {
      led_state = LOW;
    }
    digitalWrite(led_pin, led_state);
  }
  
  // Remember the previous button state for the next loop iteration
  btn_prev = btn_state;
}