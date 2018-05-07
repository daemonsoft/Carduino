#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial


#include <Wire.h>
#include <Adafruit_MotorShield.h>



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);


float Inch=0.00;
float cm=0.00;
int SonarPin=A0;
int sensorValue;

int lightSensorPin = A3;
int lightValue;

SoftwareSerial BT(10,11);    // Definimos los pines RX y TX del Arduino conectados al Bluetooth
const int sensor = A3;       // connect a button
const int buzzer = A2;       // connect a buzzer
int status;

//sonar pw
int pw_pin=7;
int arraysize = 9;
int array[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
long inch;
int exact_cm_value;
const long A = 1000;     //Resistencia en oscuridad en KΩ
const int B = 15;        //Resistencia a la luz (10 Lux) en KΩ
const int Rc = 10;       //Resistencia calibracion en KΩ
const int LDRPin = A0;   //Pin del LDR

int V;
int ilum;

unsigned long lastPwdTime = 0;
unsigned long pwdDelay = 3000;

void setup(){

  pinMode(pw_pin, INPUT);
  pinMode(SonarPin,INPUT);
  pinMode(lightSensorPin,INPUT);

  BT.begin(9600);       // Inicializamos el puerto serie BT (Para Modo AT 2)

  Serial.begin(9600);   // Inicializamos  el puerto serie
  pinMode(sensor, INPUT); //set button as an INPUT device
  pinMode(buzzer, INPUT);   //set LED as an OUTPUT device

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(255);
  motor2->setSpeed(255);
  motor3->setSpeed(255);
  motor4->setSpeed(255);
  //myMotor->run(FORWARD);
  // turn on motor
  //myMotor->run(RELEASE);
}

void loop(){


if ((millis() - lastPwdTime) > pwdDelay) {

  lightValue = analogRead(lightSensorPin);
  Serial.println(lightValue);
  BT.print('#');
  BT.print(lightValue);
  lastPwdTime = millis();
}
  //Serial.println(sensorValue);
  //7Serial.print(Inch);
  //Serial.println("inch");
  //Serial.print(cm);
  //Serial.println("cm");
  //delay(2000);
  if(BT.available())    // Si llega un dato por el puerto BT se envía al monitor serial
  {
    status = BT.read();
  }
  sensorValue=analogRead(SonarPin);
  //delay(50);
  Inch= (sensorValue*0.497);
  cm=Inch*2.54;
  if(cm<20){
    status = 0;

    //Serial.print("  ");
    //Serial.print(cm);
  }
  switch (status) {
    case 0:
      stopAllMotors();
      break;
    case 1:
      runForward();
      break;
    case 2:
      runBackward();
      break;
    case 3:
      turnLeft();
      break;
    case 4:
      turnRight();
      break;
  }


  int btn = analogRead(buzzer); //read the status of the button
  //Serial.print("-");
  //7  Serial.print(btn);
  //  delay(1000);
    //  Serial.print("|");
    //btn = analogRead(buzzer);
    //Serial.print(btn);
    //delay(1000);


//Serial.print("tick");

// myMotor->run(FORWARD);
// delay(1000);
//
// Serial.print("tock");
//
// myMotor->run(BACKWARD);
// delay(1000);
//
// Serial.print("tech");
// myMotor->run(RELEASE);
// delay(1000);
}

void stopAllMotors() {
  motor1 -> run(RELEASE);
  motor2 -> run(RELEASE);
  motor3 -> run(RELEASE);
  motor4 -> run(RELEASE);
}
void runForward(){
  motor1 -> run(FORWARD);
  motor2 -> run(FORWARD);
  motor3 -> run(FORWARD);
  motor4 -> run(FORWARD);
}
void runBackward(){
  motor1 -> run(BACKWARD);
  motor2 -> run(BACKWARD);
  motor3 -> run(BACKWARD);
  motor4 -> run(BACKWARD);
}
void turnRight(){
  motor1 -> run(FORWARD);
  motor2 -> run(BACKWARD);
  motor3 -> run(FORWARD);
  motor4 -> run(BACKWARD);
}
void turnLeft(){
  motor1 -> run(BACKWARD);
  motor2 -> run(FORWARD);
  motor3 -> run(BACKWARD);
  motor4 -> run(FORWARD);
}
