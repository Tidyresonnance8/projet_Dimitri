#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
int ledPin = 13; // Define the pin number for the LED

void setup() {
  Serial.begin(115200); // Initialize Serial communication at 115200 baud rate
  pinMode(ledPin, OUTPUT);
}

void loop() {

  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);

  // Call the function
  int result = myFunction(5, 10);
  Serial.println(result); // Print the result to the Serial Monitor

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}