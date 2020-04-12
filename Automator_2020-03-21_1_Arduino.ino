/*
  Motorcycle Gear Shift Automation Code
  This example can be used to control a motorcycle's clutch, throttle and gear shifting
  automatically based on the rpm and throttle position.
  
  Created for Arduino UNO on 10 Apr, 2020
  by Tanmay Chhatbar
*/

#include <Servo.h>
#include <Wire.h>

Servo ServoGear;                    //Gear Pin 2
Servo ServoClutch;                  //Clutch Pin 3
Servo ServoThrottle;                //Throttle Pin 4

int16_t rpm;
int pos, gear, rpmmin, rpmmax;         //throttleposition, currentgear, currentrpm, calculatedminrpm, calculatedmaxrpm
int cpr, clutchtime, clutchstarttime;       //for clutch modulation calculations
unsigned long delayshifter, previousMillis = 0;

int aggcounter = 0;                         //number based counter for aggression
int redline = 10000;                        //max rpm
int intervalshift = 1000;                   //Millisecond delay between shifts
int gearmax = 6;                            //Number of gears

//rpms
int rpmminmin = 1900, rpmminmax = 4000;     //Range of minimum rpms
int rpmmaxmin = 5000, rpmmaxmax = 7000;     //Range of maximum rpms
int aggrpmoffset = 2000;                    //Offset when aggressive
int rpmgearmin;                             //Min rpm in current gear

//rpm sense
int hall = 6;
int din, dlast;
long unsigned timeold = 0;

//servos setpoints set as angle in degrees

//gear servo
int sgearmin = 45;
int sgearmid = 90;
int sgearmax = 135;

//clutch servo
int sclutchmin = 45;
int sclutchbite = 90;
int sclutchmax = 135;

//throttle servo    note: throttle moves high angle to low angle
int sthrottlemin = 142;
int sthrottlemax = 52;
int sthrottlemid = (sthrottlemin + sthrottlemax) / 2;
int blip = 135;                    //set to value to blip throttle while downshifting

//throttle sensor
int stsensemin = 170;               //throttle position sensor min reading
int stsensemax = 890;               //throttle position sensor max reading

int ledPin = 13, ledState;

int togglecounter, togglediff = 100;  //check cycle speed, toggle led after number of loops

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  gear = 1;
  pinMode(A0, INPUT);             //Throttle Position Pin A0
  pinMode(A1, INPUT);             //RPM Sensor Pin A0
  pinMode(ledPin, OUTPUT);
  ServoGear.attach(2);            //Gear Pin 2
  ServoClutch.attach(3);          //Clutch Pin 3
  ServoThrottle.attach(4);        //Throttle Pin 4
  ServoGear.write(sgearmid);
  ServoThrottle.write(sthrottlemin);
  clutchin();
  Serial.println("Setup complete");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  ledToggle();
  throttle();
  rpmsense();
  delayshifter = millis();
  if (rpm < rpmminmin && throttleread() < 6 && gear < 2)
  {
    clutchin();
    while (throttleread() < 6)
    {}
    if (gear == 1 && throttleread() >> 6)
      clutchpr();
  }
  if ((delayshifter - previousMillis) >= intervalshift)
  {
    previousMillis = delayshifter;
    throttle();
    if (rpm > rpmmax)
    {
      if (gear < gearmax) {
        shiftup();
      }
    }
    else if (rpm < rpmmin)
    {
      if (gear > 1) {
        shiftdown();
      }
    }
  }

  if (gear == 1 && rpm < rpmminmin)
    clutchin();


  if (throttleread() < sthrottlemid )
  {
    if (aggcounter > 1000)
      aggcounter = 2000;
    if (aggcounter < 2000)
      aggcounter++;
  }
  else if (aggcounter > 0)
    aggcounter--;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void rpmsense()
{
  int counter = 0;
  if (digitalRead(hall) == HIGH) {
    din = 1;
    digitalWrite(ledPin, HIGH);
  }
  else {
    din = 0;
    digitalWrite(ledPin, LOW);
  }
  while (digitalRead(hall) == din)
  {
    if (millis() - timeold > 200) {
      rpm = 0;
      Serial.println("RPM measure timeout");
      goto exitrpm;
    }
  }
  while (digitalRead(hall) != din)
  {
    if (millis() - timeold > 200) {
      rpm = 0;
      Serial.println("RPM measure timeout");
      goto exitrpm;
    }
  }
  if (digitalRead(hall) == din) {
    rpm = 120000 / ( millis() - timeold);
    Serial.println(rpm);
  }
exitrpm:
  timeold = millis();
  dlast = din;
  delay(1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int throttleread()
{
  pos = map(analogRead(A0), stsensemin, stsensemax, sthrottlemin, sthrottlemax);
  return pos;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void throttle()
{
  rpmgearmin = rpmminmin + gear * (rpmminmax - rpmminmin) / gearmax;
  throttleread();
  if (aggcounter > 1000) {
    rpmmin = rpmminmin + aggrpmoffset + pos * (rpmminmax - rpmminmin) / 180;
    rpmmax = rpmmaxmin + aggrpmoffset + pos * (rpmmaxmax - rpmmaxmin) / 180;
  }
  else {
    rpmmin = rpmminmin + pos * (rpmminmax - rpmminmin) / 360;
    rpmmax = rpmmaxmin + pos * (rpmmaxmax - rpmmaxmin) / 360;
  }
  ServoThrottle.write(pos);
  delay(5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void shiftup()
{
  clutchin();
  ServoThrottle.write(sthrottlemin);
  ServoGear.write(sgearmin);
  delay(100);
  ServoGear.write(sgearmid);
  clutchout(1);
  throttle();
  gear++;
  Serial.print("Shift up to");
  Serial.println(gear);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void shiftdown()
{
  clutchin();
  ServoThrottle.write(blip);
  ServoGear.write(sgearmax);
  delay(50);
  ServoThrottle.write(sthrottlemin);
  delay(50);
  ServoGear.write(sgearmid);
  clutchout(1);
  throttle();
  gear--;
  Serial.print("Shift down to ");
  Serial.println(gear);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void clutchin()
{
  ServoClutch.write(sclutchmin);
  //  Serial.println("Clutch in");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void clutchout(int i)           //i=1 quick letout, i=0 smooth letout, smooth not used
{
  if (i == 0)
    for (int j = 0; j = sclutchmax; j++) {
      throttle();
      ServoClutch.write(j);
      delay(2);
      //      Serial.println("Clutch out quickly");
    }
  else {
    ServoClutch.write(sclutchmax);
    //    Serial.println("Clutch out smooth");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void clutchpr()
{
  clutchstarttime = millis();
  do {
    clutchtime = millis() - clutchstarttime;
    if (clutchtime < 200)
      cpr = (sclutchbite - sclutchmin) * 0.225 * clutchtime + sclutchmin;
    else if (clutchtime < 500)
      cpr = sclutchbite;
    else if (cpr < sclutchmax)
      cpr = (sclutchmax - sclutchbite) * 0.09 * clutchtime + sclutchbite;
    else cpr = sclutchmax;
    rpmsense();
    throttle();
    if (pos < 6)
      break;
    ServoClutch.write(cpr);
    delay(15);
  } while ( cpr < sclutchmax );
  if (pos > 6)
    ServoClutch.write(sclutchmax);
}


void ledToggle()
{
  togglecounter = 1 + togglecounter;
  if (togglecounter > togglediff)
  {
    togglecounter = 1;
    if (ledState == HIGH)
    {
      digitalWrite(ledPin, LOW);
      ledState = LOW;
    }
    else {
      digitalWrite(ledPin, HIGH);
      ledState = HIGH;
    }
  }
}
