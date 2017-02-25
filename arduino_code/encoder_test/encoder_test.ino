#include <Encoder.h>

Encoder rightEncoder(18, 19);

void setup()
{
	pinMode(18, INPUT);
	pinMode(19, INPUT);	
	Serial.begin(9600);
}

void loop()
{
	static int i = 0;
	i++;
	delay(200);
	Serial.print(i);
	Serial.print("\t");
	Serial.print(digitalRead(18));
	Serial.print("\t");
	Serial.print(digitalRead(19));
	Serial.print("\t");
	Serial.print(rightEncoder.read());
	Serial.print("\t");
	Serial.println();
}