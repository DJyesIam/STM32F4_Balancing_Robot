int INA1 = 8;
int INA2 = 9;

void setup()
{
  Serial.begin(9600);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
}

void loop()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);

  Serial.println("DC motor test");

  Serial.println("Forward ");
  analogWrite(INA1, 255);
  analogWrite(INA2, 0);
  delay(3000);
  
  Serial.println("Backward");
  analogWrite(INA1, 0);
  analogWrite(INA2, 255);
  delay(3000);
  
  Serial.println("stop");
  analogWrite(INA1, 0);
  analogWrite(INA2, 0);
  delay(1000);
}
