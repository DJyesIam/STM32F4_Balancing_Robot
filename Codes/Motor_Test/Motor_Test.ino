int INA1 = 8;
int INA2 = 9;
int INA3 = 10;
int INA4 = 11;


void setup()
{
  Serial.begin(9600);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INA4, OUTPUT);
}

void loop()
{
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INA3, LOW);
  digitalWrite(INA4, LOW);

  Serial.println("DC motor test");

  Serial.println("Forward ");
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INA3, HIGH);
  digitalWrite(INA4, LOW);
  delay(3000);
  
  Serial.println("Backward");
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INA3, LOW);
  digitalWrite(INA4, HIGH);
  delay(3000);
  
  Serial.println("stop");
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INA3, LOW);
  digitalWrite(INA4, LOW);
  delay(1000);
}
