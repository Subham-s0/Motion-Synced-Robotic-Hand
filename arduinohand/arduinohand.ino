#define numOfValsRec 5
int valsRec[numOfValsRec];
#include <Servo.h>
#define digitsPerValRec 1
//$00000

Servo servoThumb;
Servo servoIndex;
Servo servoMiddle;
Servo servoRing;
Servo servoPinky;

int stringLength = numOfValsRec * digitsPerValRec + 1; //$00000
int counter = 0;
bool counterStart = false;
String receivedString;

void setup() {
Serial.begin(9600);


servoThumb.attach(3);
servoIndex.attach(5);
servoMiddle.attach(6);
servoRing.attach(9);
servoPinky.attach(10);
}

void receiveData() {
while (Serial.available()) {
char c = Serial.read();
if (c == '$') {
counterStart = true;
receivedString = ""; // Clear the string when the start character is detected
counter = 0; // Reset the counter
}
if (counterStart) {
if (counter < stringLength) {
receivedString += c; // Append the character to the received string
counter++;
}
if (counter >= stringLength) { // Process data when the full string is received
for (int i = 0; i < numOfValsRec; i++) {
int num = (i * digitsPerValRec) + 1; // Calculate the start index of each



valsRec[i] = receivedString.substring(num, num +

digitsPerValRec).toInt();

}
receivedString = ""; // Clear the received string
counter = 0; // Reset the counter




counterStart = false; // Reset the start flag
}
}
}
}

void loop() {
receiveData();

// Control the servos based on the received values
if (valsRec[0] == 0) { servoThumb.write(0); } else { servoThumb.write(140); }
if (valsRec[1] == 0) { servoIndex.write(0); } else { servoIndex.write(179); }
if (valsRec[2] == 0) { servoMiddle.write(0); } else { servoMiddle.write(179); }
if (valsRec[3] == 0) { servoRing.write(180); } else { servoRing.write(60); }
if (valsRec[4] == 0) { servoPinky.write(179); } else { servoPinky.write(0); }
}