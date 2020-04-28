/*
Press joystick button to change between manual and
autonomous modes. Use joystick to control platform
orientation when in manual mode
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define X1 A8
#define X2 A9
#define Y1 A10
#define Y2 A11

const int  buttonPin = 31;
unsigned long startMillis = 0;
const long interval = 300;
int xp = 0, yp = 0;
int zero[6] = {435, 260, 405, 430, 315, 355};
int nthirty[6] = {520, 195, 475, 350, 365, 285};
int pthirty[6] = {355, 315, 340, 495, 260, 420};
int val[6] = {435, 260, 405, 430, 315, 355};
int valp[6] = {435, 260, 405, 430, 315, 355};
double ang[6] = {0};
int pitch = 0, roll = 0, yaw = 0;
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 14; // analog pin connected to X output
const int Y_pin = 15; // analog pin connected to Y output
bool manual = 1;
const int SERVOMIN = 100; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor((SERVOMAX + SERVOMIN) / 2); // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 15; // 'change' pulse length count
String valInput; // Serial input var.
int x_touch = 0, y_touch = 0;

//read from touchscreen
void readTouch()
{
  int sumx = 0, sumy = 0;

  //take average of 10 readings
  for (int i = 0; i < 10; i++)
  {
    pinMode(Y1, INPUT);
    pinMode(Y2, INPUT);
    digitalWrite(Y2, LOW);
    pinMode(X1, OUTPUT);
    digitalWrite(X1, HIGH);
    pinMode(X2, OUTPUT);
    digitalWrite(X2, LOW);

    y_touch = (analogRead(Y1)); //Reads X axis touch position
    sumy = sumy + y_touch;

    pinMode(X1, INPUT);
    pinMode(X2, INPUT);
    digitalWrite(X2, LOW);
    pinMode(Y1, OUTPUT);
    digitalWrite(Y1, HIGH);
    pinMode(Y2, OUTPUT);
    digitalWrite(Y2, LOW);


    x_touch = (analogRead(X1)); //Reads Y axis touch position
    sumx = sumx + x_touch;

    //send to processing if touch position values have changed
    if ((x_touch != xp) || (y_touch != yp))
    {
      Serial.print(x_touch + 100);
      Serial.println(y_touch + 100);
      xp = x_touch;
      yp = y_touch;
    }
  }

  x_touch = sumx / 10;
  y_touch = sumy / 10;
}

void setPWM(double ang[])
{
//calibrate using PWM values at -30, 0 and +30 degrees
  for (int i = 0; i < 6; i++)
  {
    if (ang[i] <= 0)
      val[i] = zero[i] + (((nthirty[i] - zero[i]) / -30.0) * ang[i]);
    else
      val[i] = zero[i] + (((pthirty[i] - zero[i]) / 30.0) * ang[i]);
  }


  int changeSize = 10;
  int change[6] = {0};

  for (int i = 0; i < 6; i++)
  {
    change[i] = (val[i] - valp[i]) / changeSize;
  }

  for (int i = 0; i < changeSize; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      pwm.setPWM(j + 1, 0, valp[j] + change[j]);
      valp[j] = valp[j] + change[j];
    }
  }

  for (int i = 0; i < 6; i++)
  {
    pwm.setPWM(i + 1, 0, val[i]);
    valp[i] = val[i];
  }
}

void send2servo(int pitch, int roll, int yaw)
{
  calcAngle(pitch , roll, yaw);
  setPWM(ang);
}

void manualMode()
{
  pitch = map(analogRead(X_pin), 0, 1023, -7, 7);
  roll = map(analogRead(Y_pin), 0, 1023, 7, -7);
  send2servo(pitch, roll, yaw);
  readTouch();
}

void autoMode()
{
  readTouch();

  //zone 1
  if ((x_touch < 108) && (y_touch > 217))
  {
    pitch = -7;
    roll = 5;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 2
  if ((y_touch < 217) && (x_touch < 590))
  {
    pitch = -5;
    roll = -5;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 3
  if ((x_touch < 231) && (y_touch < 273) && (x_touch < 580))
  {
    pitch = -5;
    roll = -5;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 4
  if ((x_touch > 566) && (y_touch > 156) && (y_touch < 398) && (x_touch < 658))
  {
    pitch = 5;
    roll = -5;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 5
  if ((x_touch < 630) && (y_touch < 280) && (x_touch > 600) && (y_touch > 235))
  {
    moveDown();
  }

  //zone 6
  if ((x_touch < 420) && (y_touch > 459) && (y_touch < 650) && (x_touch > 217))
  {
    pitch = 3;
    roll = -3;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 7
  if ((x_touch < 420) && (y_touch < 869) && (y_touch > 711) && (x_touch > 217))
  {
    pitch = 3;
    roll = -7;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 8
  if ((x_touch < 625) && (y_touch < 610) && (y_touch > 464) && (x_touch > 510))
  {
    pitch = -7;
    roll = 7;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 9
  if ((x_touch > 566) && (y_touch > 156) && (y_touch < 398) && (x_touch < 658))
  {
    pitch = 5;
    roll = -5;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

  //zone 10
  else {
    pitch = -3;
    roll = 2;
    send2servo(pitch, roll, yaw);
    readTouch();
  }

}

void calcAngle(double x, double y, double z)
{
  double t = x * PI / 180.0; //about x
  double s = y * PI / 180.0; //about y
  double p = z * PI / 180.0; //about z

  double aa = 14.40 * PI / 180.0;
  double bb = 74.36 * PI / 180.0;
  double cc = 45.6 * PI / 180.0;

  double radiTop = 105.6;

  double P1x = radiTop * cos(aa);
  double P1y = radiTop * sin(aa);

  double P2x = +radiTop * cos(bb);
  double P2y = -1 * radiTop * sin(bb);

  double P3x = -1 * radiTop * cos(bb);
  double P3y = -1 * radiTop * sin(bb);

  double P4x = -1 * radiTop * cos(aa);
  double P4y = radiTop * sin(aa);

  double P5x = -1 * radiTop * cos(cc);
  double P5y = radiTop * sin(cc);

  double P6x = radiTop * cos(cc);
  double P6y = radiTop * sin(cc);

  double D = 4.91 * PI / 180.0;
  double E = 55.08 * PI / 180.0;
  double F = 64.92 * PI / 180.0;

  double radiBot = 104.96;

  double B1x = radiBot * cos(D);
  double B1y = -1 * radiBot * sin(D);

  double B2x = radiBot * cos(E);
  double B2y = -1 * radiBot * sin(E);

  double B3x = -1 * radiBot * cos(E);
  double B3y = -1 * radiBot * sin(E);

  double B4x = -1 * radiBot * cos(D);
  double B4y = -1 * radiBot * sin(D);

  double B5x = -1 * radiBot * cos(F);
  double B5y = radiBot * sin(F);

  double B6x = radiBot * cos(F);
  double B6y = radiBot * sin(F);

  double Tz = 160;
  //double Tz=map(analogRead(X_pin),0,1023,165,135);

  double l1x = ((P1x * cos(p) * cos(s)) + (P1y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t))))) - B1x;
  double l1y = ((P1x * sin(p) * cos(s)) + (P1y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s))))) - B1y;
  double l1z = ((-P1x * sin(s)) + (P1y * cos(s) * sin(t))) + Tz;

  double l2x = ((P2x * cos(p) * cos(s)) + (P2y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t))))) - B2x;
  double l2y = ((P2x * sin(p) * cos(s)) + (P2y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s))))) - B2y;
  double l2z = ((-P2x * sin(s)) + (P2y * cos(s) * sin(t))) + Tz;

  double l3x = ((P3x * cos(p) * cos(s)) + (P3y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t))))) - B3x;
  double l3y = ((P3x * sin(p) * cos(s)) + (P3y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s))))) - B3y;
  double l3z = ((-P3x * sin(s)) + (P3y * cos(s) * sin(t))) + Tz;

  double l4x = (P4x * cos(p) * cos(s)) + (P4y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t)))) - B4x;
  double l4y = (P4x * sin(p) * cos(s)) + (P4y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s)))) - B4y;
  double l4z = (-P4x * sin(s)) + (P4y * cos(s) * sin(t)) + Tz;

  double l5x = (P5x * cos(p) * cos(s)) + (P5y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t)))) - B5x;
  double l5y = (P5x * sin(p) * cos(s)) + (P5y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s)))) - B5y;
  double l5z = (-P5x * sin(s)) + (P5y * cos(s) * sin(t)) + Tz;

  double l6x = (P6x * cos(p) * cos(s)) + (P6y * ((-sin(p) * cos(t)) + (cos(p) * sin(s) * sin(t)))) - B6x;
  double l6y = (P6x * sin(p) * cos(s)) + (P6y * ((cos(p) * cos(t)) + (sin(p) * sin(t) * sin(s)))) - B6y;
  double l6z = (-P6x * sin(s)) + (P6y * cos(s) * sin(t)) + Tz;

  double a = 31.44;
  double e = 163;

  double Beta1 = 60 * PI / 180.0;
  double Beta2 = 240 * PI / 180.0;
  double Beta3 = 300 * PI / 180.0;
  double Beta4 = 120 * PI / 180.0;
  double Beta5 = 180 * PI / 180.0;
  double Beta6 = 0 * PI / 180.0;

  double Q1x = B1x + l1x;
  double Q1y = B1y + l1y;
  double Q1z = l1z;

  double Q2x = B2x + l2x;
  double Q2y = B2y + l2y;
  double Q2z = l2z;

  double Q3x = B3x + l3x;
  double Q3y = B3y + l3y;
  double Q3z = l3z;

  double Q4x = B4x + l4x;
  double Q4y = B4y + l4y;
  double Q4z = l4z;

  double Q5x = B5x + l5x;
  double Q5y = B5y + l5y;
  double Q5z = l5z;

  double Q6x = B6x + l6x;
  double Q6y = B6y + l6y;
  double Q6z = l6z;

  double L1 = (pow(l1x, 2) + pow(l1y, 2) + pow(l1z, 2) - (pow(e, 2) - pow(a, 2)));
  double L2 = (pow(l2x, 2) + pow(l2y, 2) + pow(l2z, 2) - (pow(e, 2) - pow(a, 2)));
  double L3 = (pow(l3x, 2) + pow(l3y, 2) + pow(l3z, 2) - (pow(e, 2) - pow(a, 2)));
  double L4 = (pow(l4x, 2) + pow(l4y, 2) + pow(l4z, 2) - (pow(e, 2) - pow(a, 2)));
  double L5 = (pow(l5x, 2) + pow(l5y, 2) + pow(l5z, 2) - (pow(e, 2) - pow(a, 2)));
  double L6 = (pow(l6x, 2) + pow(l6y, 2) + pow(l6z, 2) - (pow(e, 2) - pow(a, 2)));

  double M1 = 2 * a * Q1z;
  double M2 = 2 * a * Q2z;
  double M3 = 2 * a * Q3z;
  double M4 = 2 * a * Q4z;
  double M5 = 2 * a * Q5z;
  double M6 = 2 * a * Q6z;

  double N1 = 2 * a * ((cos(Beta1) * (Q1x - B1x)) + (sin(Beta1) * (Q1y - B1y)));
  double N2 = 2 * a * ((cos(Beta2) * (Q2x - B2x)) + (sin(Beta2) * (Q2y - B2y)));
  double N3 = 2 * a * ((cos(Beta3) * (Q3x - B3x)) + (sin(Beta3) * (Q3y - B3y)));
  double N4 = 2 * a * ((cos(Beta4) * (Q4x - B4x)) + (sin(Beta4) * (Q4y - B4y)));
  double N5 = 2 * a * ((cos(Beta5) * (Q5x - B5x)) + (sin(Beta5) * (Q5y - B5y)));
  double N6 = 2 * a * ((cos(Beta6) * (Q6x - B6x)) + (sin(Beta6) * (Q6y - B6y)));

  double Alpha1 = asin(L1 / (sqrt(pow(M1, 2) + pow(N1, 2)))) - atan(N1 / M1);
  double Alpha2 = asin(L2 / (sqrt(pow(M2, 2) + pow(N2, 2)))) - atan(N2 / M2);
  double Alpha3 = asin(L3 / (sqrt(pow(M3, 2) + pow(N3, 2)))) - atan(N3 / M3);
  double Alpha4 = asin(L4 / (sqrt(pow(M4, 2) + pow(N4, 2)))) - atan(N4 / M4);
  double Alpha5 = asin(L5 / (sqrt(pow(M5, 2) + pow(N5, 2)))) - atan(N5 / M5);
  double Alpha6 = asin(L6 / (sqrt(pow(M6, 2) + pow(N6, 2)))) - atan(N6 / M6);

  ang[0] = Alpha1 * 180.0 / PI;
  ang[1] = Alpha2 * 180.0 / PI;
  ang[2] = Alpha3 * 180.0 / PI;
  ang[3] = Alpha4 * 180.0 / PI;
  ang[4] = Alpha5 * 180.0 / PI;
  ang[5] = Alpha6 * 180.0 / PI;

}

void moveDown()
{
  readTouch();
  if (x_touch < 554)
    return;
  pitch = 5;
  roll = 1;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  unsigned long currentMillis = millis();
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }

  pitch = 5;
  roll = 2;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }

  pitch = 3;
  roll = 3;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }

  pitch = 3;
  roll = 4;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }

  pitch = 3;
  roll = 5;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }

  pitch = 0;
  roll = 9;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);
  delay(100);

  pitch = 03;
  roll = 9;
  calcAngle(pitch , roll, yaw);
  setPWM(ang);

  readTouch();
  if (x_touch < 554)
    return;

  //delay for 300ms
  startMillis = millis();
  currentMillis = millis();
  while (currentMillis - startMillis <= interval) {
    currentMillis = millis();
    readTouch();
  }
}

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  //check pushbutton
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == 0)
  {
    while (buttonState != 0) {
      continue;
    }
    manual = !manual;
  }

  //check for mode type
  if (manual) { //manual mode
    manualMode();
  }

  else {  //autonomous mode
    autoMode();
  }
}
