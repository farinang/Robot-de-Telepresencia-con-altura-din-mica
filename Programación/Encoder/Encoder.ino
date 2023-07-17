#include <Encoder.h>

#define pin_A_encoder 2
#define pin_B_encoder 3

Encoder encoder(pin_B_encoder,pin_A_encoder);
long oldPosition  = -999;

void setup() {
  Serial.begin(9600);
}

void loop() {
 long newPosition = encoder.read();
 if (newPosition != oldPosition) {
   oldPosition = newPosition;
   Serial.println(newPosition);
}
}
