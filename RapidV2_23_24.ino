#define Rpwm1 6
#define RDir1 7
#define RDir2 8
#define Lpwm1 11
#define LDir1 10
#define LDir2 9
#define ObjectSens 2
#define LQ1 A7
#define LQ2 A6
#define LQ3 A5
#define LQ4 A4
#define LQ5 A3
#define LQ6 A2
#define LQ7 A1
#define LQ8 A0

#define SW1 12
#define SW2 13

#define Led 13

float veri = 0;
float LeftBaseSpeed  = 220 ;
float RightBaseSpeed = 220 ;
float Kp = 0.07;
float Kd = 2;
float Ki = 0.0001 ;

int Q[8] = {LQ8, LQ7, LQ6, LQ5, LQ4, LQ3, LQ2, LQ1};
int SensMin[8];
int SensMax[8];
int SensVal[8];
int SensValX[8];
int FinalError = 0;
int Error = 0;
int ExtraSpeed = 100;
int RightSpeed = 0;
int LeftSpeed = 0;
int Integral = 0;
int Val = 0;
unsigned int Position = 3500;
bool turn = false;

int extraTurn = 50;
int extraStraight = 100;

byte LineOk = 0;
byte WhiteLine = 1;

void Blink() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(Led, LOW);
    delay(180);
    digitalWrite(Led, HIGH);
    delay(180);
  }
}

void MotorsSpeed(int RightSpeed, int LeftSpeed) {
  if (RightSpeed <= 0) {
    RightSpeed = abs(RightSpeed);
    analogWrite(Rpwm1, RightSpeed);
    digitalWrite(RDir1, HIGH);
    digitalWrite(RDir2, LOW);
  }
  else {
    analogWrite(Rpwm1, RightSpeed);
    digitalWrite(RDir1, LOW);
    digitalWrite(RDir2, HIGH);
  }
  if (LeftSpeed <= 0) {
    LeftSpeed = abs(LeftSpeed);
    analogWrite(Lpwm1, LeftSpeed);
    digitalWrite(LDir1, HIGH);
    digitalWrite(LDir2, LOW);
  }
  else {
    analogWrite(Lpwm1, LeftSpeed);
    digitalWrite(LDir1, LOW);
    digitalWrite(LDir2, HIGH);
  }
}

int ReadSensor() {
  unsigned long Middle = 0;
  unsigned int Total = 0;
  LineOk = 0;
  for (int i = 0; i < 8; i++) {
    int Difference = SensMax[i] - SensMin[i];
    long int Calculation = analogRead(Q[i]) - SensMin[i];
    Calculation = Calculation * 1000;
    Calculation = Calculation / Difference;
    SensVal[i] = constrain(Calculation, 0, 1000);
    SensValX[i] = SensVal[i];
  }
  for (int i = 0; i < 8; i++) {
    if (WhiteLine == 1)  SensValX[i] = 1000 - SensVal[i];
    if (SensValX[i] > 250)  LineOk = 1;
    if (SensValX[i] > 50) {
      Middle += SensValX[i] * i ;
      Total += SensValX[i];
    }
  }
  if (LineOk == 0) {
    if (Val < 3000)  return 0;
    if (Val > 5000)  return 7000;
    if (Val > 3000 && Val < 5000) return 3500;
  }
  Middle = Middle * 1000;
  Val = Middle / Total;
  return Val;
}

void detect_turn() {

  turn = true;

  for (int i = 0; i < 4; i++) {

    if (SensValX[i] < 900) {

      turn = false;

    }

  }

  for (int i = 4; i < 8; i++) {

    if (SensValX[i] > 900) {

      turn = false;

    }

  }

}

void detect_straight() {

  turn = false;

  for (int i = 0; i < 4; i++) {

    int dif = SensVal[i] - SensVal[7 - i];

    if (dif > 100 || dif < -100) {

      turn = true;

    }

  }

}

void setup() {
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(RDir1, OUTPUT);
  pinMode(RDir2, OUTPUT);
  pinMode(LDir1, OUTPUT);
  pinMode(LDir2, OUTPUT);
  pinMode(LQ1, INPUT);
  pinMode(LQ2, INPUT);
  pinMode(LQ3, INPUT);
  pinMode(LQ4, INPUT);
  pinMode(LQ5, INPUT);
  pinMode(LQ6, INPUT);
  pinMode(LQ7, INPUT);
  pinMode(LQ8, INPUT);

  pinMode(ObjectSens, INPUT);
  pinMode(Led, OUTPUT);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  MotorsSpeed(0, 0);
  // Serial.begin(9600);
  delay(2000);
  for (int i = 0; i < 8; i++) {
    SensMin[i] = 1024;
    SensMax[i] = 0;
  }
  for (int i = 0; i < 150; i++)
  {
    for (int i = 0; i < 8; i++) {
      if (SensMin[i] > analogRead(Q[i])) SensMin[i] = analogRead(Q[i]);
      if (SensMax[i] < analogRead(Q[i])) SensMax[i] = analogRead(Q[i]);
      delay(1);
    }
  }
  delay(700);
  Blink();
}



void loop() {

// MZ80 Cisim Algılama Kodları   
  
  /*
  if (digitalRead(ObjectSens) == HIGH) {
    delay(5);
    while (digitalRead(ObjectSens) == HIGH) {
      MotorsSpeed(0, 0);
    }
  }
*/




  Position = ReadSensor();

  detect_straight();

  if (turn) {ExtraSpeed = extraTurn;}
  else {ExtraSpeed = extraStraight;}

  Error = Position - 3500;
  Integral += Error;

  int Verify = Kp * Error + Kd * (Error - FinalError) + Ki * Integral;
  FinalError = Error;

  RightSpeed = RightBaseSpeed + Verify + ExtraSpeed ;
  LeftSpeed = LeftBaseSpeed - Verify +  ExtraSpeed ;

  RightSpeed = constrain(RightSpeed, -150, 150);
  LeftSpeed = constrain(LeftSpeed, -150, 150);
  MotorsSpeed(LeftSpeed, RightSpeed);
}
