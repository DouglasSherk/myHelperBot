int ledPin = 13;                 // LED connected to digital pin 13

void setup()
{
	Serial.begin(57600);
	pinMode(ledPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
	char data[256];
	int i = 0;

	while (Serial.available())
	{
		i = 1;
		data[0] = (char)Serial.read();
		
		while (data[i - 1] != '\0') {
			if (Serial.available()) {
				data[i++] = (char)Serial.read();
			}
		}

		static int toggle = 0;
		if (strstr(data, "test") != NULL) {
			digitalWrite(ledPin, toggle);   // sets the LED on
			delay(50);
			toggle = !toggle;
		}
	}
}
 