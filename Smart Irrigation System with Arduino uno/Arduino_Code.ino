#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

 const int RELAY_PIN = A3;  // the Arduino pin, which connects to the IN pin of relay

  //Soil Sensor
  int sensorPin = A0;
  int sensorValue;
  int limit = 550;

  // the setup function runs once when you press reset or power the board
  void setup() {
  // initialize digital pin A5 as an output.
  pinMode(RELAY_PIN, OUTPUT);

  //Soil Sensor
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  //LCD
  lcd.init();
  lcd.backlight();/*
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Welcome to our");
  lcd.setCursor(1,7);
  lcd.print("Project");
  delay(2000);*/
  }


  //Soil Sensor Code
void loop() {  
  sensorValue = analogRead(sensorPin);
  Serial.println("Analog Value : ");
  Serial.println(sensorValue);

  if (sensorValue>limit) {
  digitalWrite(13, HIGH);
  digitalWrite(RELAY_PIN, LOW); 

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MOTOR is ON");
  lcd.setCursor(1,7);
  lcd.print("DRY SOIL");
  

  }
  else {
  digitalWrite(13, LOW);
  digitalWrite(RELAY_PIN,HIGH);

   lcd.init();
  lcd.backlight();
    lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MOTOR is OFF");
  lcd.setCursor(1,7);
  lcd.print("WET SOIL");
  
  
  }
  delay(1000);
  }
/*
#include <SoftwareSerial.h>

//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(10, 11); //SIM800L Tx & Rx is connected to Arduino #11 & #10

void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  Serial.println("Initializing...");
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CFUN = 0");
  updateSerial();
  delay(5000);
  mySerial.println("AT+CFUN = 1");
  updateSerial();
  delay(1000);
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network. (0,1) means registered
  updateSerial();
}

void loop()
{
  updateSerial();
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
  */
  
