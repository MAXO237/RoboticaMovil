/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
   / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
  | |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
   \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
                    (____/
   Arduino Mecanum Omni Direction Wheel Robot Car
   Se agregaron las clases: Motor, RobotCar y WifiCarServer
*/

#include "WiFiEsp.h"
#include "WiFiEspUdp.h"

// ------------------ Configuración General ------------------
#define SPEED       50
#define TURN_SPEED  80
#define SHIFT_SPEED 80

#define TURN_TIME 500
#define MOVE_TIME 500

// ------------------ Clase Motor ------------------
class Motor {
  int pinDir1;
  int pinDir2; 
  int pinPWM;

public:
  Motor(int d1, int d2, int pwm){
    pinDir1 = d1;
    pinDir2 = d2;
    pinPWM = pwm;
  }

  void begin() {
    pinMode(pinDir1, OUTPUT);
    pinMode(pinDir2, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    stop();
  }

  void forward(int speed) {
    digitalWrite(pinDir1, LOW);
    digitalWrite(pinDir2, HIGH);
    analogWrite(pinPWM, speed);
  }

  void backward(int speed) {
    digitalWrite(pinDir1, HIGH);
    digitalWrite(pinDir2, LOW);
    analogWrite(pinPWM, speed);
  }

  void stop() {
    analogWrite(pinPWM, 0);
  }
};

// ------------------ Clase RobotCar ------------------
class RobotCar {
public:
  Motor FR;
  Motor FL;
  Motor RR;
  Motor RL;

RobotCar(Motor fr, Motor fl, Motor rr, Motor rl) {
    FR = fr;
    FL = fl;
    RR = rr;
    RL = rl;
}

  void begin() {
    FR.begin(); 
    FL.begin(); 
    RR.begin(); 
    RL.begin();
    stop();
  }

  void forward(int speed) {
    RL.forward(speed); 
    RR.forward(speed);
    FR.forward(speed); 
    FL.forward(speed);
  }

  void backward(int speed) {
    RL.backward(speed); 
    RR.backward(speed);
    FR.backward(speed); 
    FL.backward(speed);
  }

  void leftTurn(int speed) {
    RL.backward(0); 
    RR.forward(speed);
    FR.forward(speed); 
    FL.backward(0);
  }

  void rightTurn(int speed) {
    RL.forward(speed); 
    RR.backward(0);
    FR.backward(0); 
    FL.forward(speed);
  }

  void diagonalIzquierdaAdelante(){
    leftShift(0, 150, 0, 150);
  }

  void diagonalDerechaAdelante(){
    rightShift(150, 0, 150, 0);
  }

  void diagonalIzquierdaAtras(){
    leftShift(150, 0, 150, 0);
  }

  void diagonalDerechaAtras(){
    rightShift(0, 130, 0, 130);
  }

  void izquierdaViendoAdelante(){
    leftShift(200, 150, 150, 200);
  }

  void derechaViendoAdelante(){
    rightShift(200, 200, 200, 200);
  }

  void stop() {
    FR.stop(); 
    FL.stop(); 
    RR.stop(); 
    RL.stop();
  }

private:
  void leftShift(int fl_bck, int rl_fwd , int rr_bck, int fr_fwd) {
    FL.backward(fl_bck); 
    RL.forward(rl_fwd);
    FR.forward(fr_fwd); 
    RR.backward(rr_bck);
  }

  void rightShift(int fl_fwd, int rl_bck , int rr_fwd, int fr_bck) {
    FL.forward(fl_fwd); 
    RL.backward(rl_bck);
    FR.backward(fr_bck); 
    RR.forward(rr_fwd);
  }
};

// ------------------ Clase WifiCarServer ------------------
class WifiCarServer {
  char ssid[20] = "Equipo Dinamita";
  unsigned int localPort = 8888;
  WiFiEspUDP Udp;
  char packetBuffer[5];
  RobotCar &car;

public:
  WifiCarServer(RobotCar &c) : car(c) {}

  void begin() {
    Serial.begin(9600);
    Serial1.begin(115200);
    Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
    delay(200);
    Serial1.write("AT+RST\r\n");
    delay(200);
    Serial1.begin(9600);

    WiFi.init(&Serial1);
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      while (true);
    }

    Serial.print("Attempting to start AP ");
    Serial.println(ssid);
    WiFi.beginAP(ssid, 10, "", 0);
    Udp.begin(localPort);

    Serial.print("Listening on port ");
    Serial.println(localPort);
    printWifiStatus();
  }

  void handleClient() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) packetBuffer[len] = 0;
      char c = packetBuffer[0];

      switch (c) {
        case 'A': car.forward(SPEED); break;  //adelante
        case 'B': car.backward(SPEED); break; // atras
        case 'L': car.leftTurn(TURN_SPEED); break;  // giro normal a la izquierda
        case 'R': car.rightTurn(TURN_SPEED); break; // giro normal a la derecha
        case 'E': car.stop(); break;  //  se detiene
        case 'F': car.diagonalIzquierdaAdelante(); break;  // diagonal izq adelante
        case 'H': car.diagonalDerechaAdelante(); break; // diagonal der adelante
        case 'I': car.diagonalIzquierdaAtras(); break;  // diagonal izq atrás
        case 'K': car.diagonalDerechaAtras(); break; // diagonal der atrás
        case 'O': car.izquierdaViendoAdelante(); break; // izquierda viendo adelante
        case 'T': car.derechaViendoAdelante(); break; // derecha viendo adelante
        default: break;
      }
    }
  }

  void printWifiStatus() {
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: "); Serial.println(ip);
    Serial.print("UDP Port: "); Serial.println(localPort);
  }
};

// ------------------ Objetos Globales ------------------
Motor motorFR(22, 24, 9);
Motor motorFL(26, 28, 10);
Motor motorRR(5, 6, 11);
Motor motorRL(7, 8, 12);

RobotCar myCar(motorFR, motorFL, motorRR, motorRL);
WifiCarServer wifiServer(myCar);

// ------------------ Setup y Loop ------------------
void setup() {
  myCar.begin();
  wifiServer.begin();
}

void loop() {
  wifiServer.handleClient();
}