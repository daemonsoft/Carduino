#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

int speed = 200;

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


  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor3->setSpeed(speed);
  motor4->setSpeed(speed);
}

void loop(){


if ((millis() - lastPwdTime) > pwdDelay) {

  lightValue = analogRead(lightSensorPin);
  Serial.println(lightValue);
  BT.print('#');
  BT.print(lightValue);
  lastPwdTime = millis();
}

  if(BT.available())    // Si llega un dato por el puerto BT se envía al monitor serial
  {
    status = BT.read();
  }
  sensorValue=analogRead(SonarPin);
  Inch= (sensorValue*0.497);
  cm=Inch*2.54;
  if(cm<20){
    status = 0;
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
  motor2 -> run(FORWARD);
  motor3 -> run(BACKWARD);
  motor4 -> run(BACKWARD);
}
void turnLeft(){
  motor1 -> run(BACKWARD);
  motor2 -> run(BACKWARD);
  motor3 -> run(FORWARD);
  motor4 -> run(FORWARD);
}
