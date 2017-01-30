void setup()
{
	pinMode(12, INPUT);
	pinMode(13, INPUT);	
	Serial.begin(9600);
}

void loop()
{
	delay(50);
	Serial.print(digitalRead(12));
	Serial.print("\t");
	Serial.print(digitalRead(13));
	Serial.print("\t");
	Serial.println();
}