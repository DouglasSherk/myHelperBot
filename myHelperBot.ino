/* Serial Hello World
 * ------------------- 
 * Simple Hello world for serial ports.
 * Print out "Hello world!" and blink pin 13 LED every second.
 *
 * Created 18 October 2006
 * copyleft 2006 Tod E. Kurt <tod@todbot.com>
 * http://todbot.com/
 */

int ledPin = 13;   // select the pin for the LED
int i=0;           // simple counter to show we're doing something

void setup() {
  pinMode(ledPin,OUTPUT);   // declare the LED's pin as output
  Serial.begin(115200);        // connect to the serial port
  randomSeed(1000);
}

void printRandom() {
  float rand = 1.0 / float(random(100));
  Serial.print(rand);
  Serial.println(" ");  
}

void loop () {
  //printRandom();
  //printRandom();
  Serial.println("0.20 0.30 0.50\n");
  //digitalWrite(ledPin, HIGH);
  //delay(50);
  //digitalWrite(ledPin, LOW);
  delay(50);
}
