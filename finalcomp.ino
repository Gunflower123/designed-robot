#include "Arduino.h"
#include "MotorDriver.h"
#include "ReflectanceSensor.h"

//encoder ///////////////////////////////////////////////////////////////////////////////////////////////////
int pinRA = 34, pinRB = 36;
int pinLA = 39, pinLB = 35;
volatile int CounterR = 0, CounterL = 0, rotSign;
volatile int tic, dt, toc = 0;
double sampleTime = 30;

//PID class//////////////////////////////////////////////////////////////////////////////////////////
class PID{
  private:
    double Kp;
    double Ki;
    double Kd;
    double Err,Err_last = 0;
    double Cp,Ci,Cd,Ci_last = 0;
    double Cpid;
    double Sampling_frequency;
  public:
    PID (double err, double frequency, double p, double i, double d){
      Err = err;
      Kp=p;
      Ki=i;
      Kd=d;
      Sampling_frequency=frequency;
    }
    double pidcontrol(){
      Cp = Kp*Err;
      Ci = Ki*Err*Sampling_frequency+Ci_last;
      Cd = Kd*(Err-Err_last)/Sampling_frequency;
      Cpid = Cp+Ci+Cd;

      Ci_last = Ci;
      Err_last = Err;
      return Cpid;
    }
};

//servo class//////////////////////////////////////////////////////////////////////////////////////////
const int servoPin = 5;
const int freq = 50;
const int Channel = 0;
const int resolution = 8;

//sonar class//////////////////////////////////////////////////////////////////////////////////////////
class SONAR{
  private:
    int Pin, Time;
    double duration;
    double distance;
  public:
  SONAR(int sonar_pin, int pulse_time){          //sonar constructor
    Pin=sonar_pin;
    Time=pulse_time;
  }
  double sonarDistance(){       //member function 
    pinMode(Pin, OUTPUT);
    digitalWrite(Pin, HIGH);
    delayMicroseconds(Time);
    digitalWrite(Pin, LOW);
    pinMode(Pin, INPUT);
    duration = pulseIn(Pin, HIGH);    //measure the duration
    distance = (duration/2)*0.0343;
    return distance;
    delay(500);
  }
  double sonarError(){
    pinMode(Pin, OUTPUT);
    digitalWrite(Pin, HIGH);
    delayMicroseconds(Time);
    digitalWrite(Pin, LOW);
    pinMode(Pin, INPUT);
    duration = pulseIn(Pin, HIGH);    //measure the duration
    distance = 10-(duration/2)*0.0343;
    return distance;
    delay(500);
  }
};

//reflectance sensor//////////////////////////////////////////////////////////////////////////////////////////
uint8_t SensorCount = 7;                                  // Number of refectance sensors
uint8_t SensorPins[7] = {32, 25, 27, 19, 22, 23, 21};         // Sensor pins
uint32_t Timeout = 2500;                                  // Sensor reflect timeout (us)
ReflectanceSensor sensor;
int k1,k2,k3,k4,k5,k6,k7;
//motor variables//////////////////////////////////////////////////////////////////////////////////////////
int motR_pins[3] = {4, 15, 18};     
int motR_sign = -1;                
int motL_pins[3] = {2, 12, 0};
int motL_sign = 1;
MotorDriver Mr;
MotorDriver Ml;

//setup//////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  //setup the servo
  ledcSetup(Channel, freq, resolution);
  ledcAttachPin(servoPin, Channel);
  ledcWrite(Channel, 18);                                           //servo angle = 90 degree
  //Set up the Motors
  Mr.SetBaseFreq(5000);                                             //PWM base frequency setup
  Mr.SetSign(motR_sign);                                            //Setup motor sign
  Mr.DriverSetup(motR_pins[0], 3, motR_pins[1], motR_pins[2]);      //Setup motor pins and channel
  Mr.MotorWrite(0);                                                 //Write 0 velocity to the motor when initialising
  Ml.SetBaseFreq(5000);
  Ml.SetSign(motL_sign);
  Ml.DriverSetup(motL_pins[0], 4, motL_pins[1], motL_pins[2]);
  Ml.MotorWrite(0);
  //set up reflectance
  sensor.SetSensorPins(SensorPins,SensorCount);
  sensor.SetTimeout(Timeout);
  //set up encoder
  pinMode(pinRA,INPUT);
  pinMode(pinRB,INPUT);
  pinMode(pinLA,INPUT);
  pinMode(pinLB,INPUT);
  tic = millis();
  attachInterrupt(digitalPinToInterrupt(pinRA), counterR, HIGH);
  attachInterrupt(digitalPinToInterrupt(pinLA), counterL, HIGH);
}

void counterR(){
  CounterR++;
}
void counterL(){
  CounterL++;
}



volatile int Stop = 0;
volatile double servoSign=-1;
double errS,uR,uL;
double pwmSetR = 0.5;          //set initial speed
double pwmSetL = 0.5;
double SpeedR,SpeedL;
SONAR sonar1(14,5);




//main loop/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  toc = millis();
  dt = toc-tic;
  if(dt >= sampleTime){
    SpeedR = CounterR*0.13/0.03/121;                                      //encoder speed
    SpeedL = CounterL*0.13/0.03/121;
    double dis = sonar1.sonarDistance();
    
    //turnaround
    if (dis<=17){
      Mr.MotorWrite(0.7);
      Ml.MotorWrite(0.7);
      delay(710);
    
    }
    
    //common mode
    if (dis>17){
    sensor.ReadSensor();
    if (200<sensor.GetSensorValues(5)&&sensor.GetSensorValues(6)<1000){k1 = 2;}
    else{k1 = 0;}
    if (200<sensor.GetSensorValues(5)&&sensor.GetSensorValues(5)<1000){k2 = 2;}
    else{k2 = 0;}
    if (200<sensor.GetSensorValues(4)&&sensor.GetSensorValues(4)<1000){k3 = 2;}
    else{k3 = 0;}
    if (200<sensor.GetSensorValues(3)&&sensor.GetSensorValues(3)<1000){k4 = 2;}
    else{k4 = 0;}
    if (200<sensor.GetSensorValues(2)&&sensor.GetSensorValues(2)<1000){k5 = 2;}
    else{k5 = 0;}
    if (200<sensor.GetSensorValues(1)&&sensor.GetSensorValues(1)<1000){k6 = 2;}
    else{k6 = 0;}
    if (200<sensor.GetSensorValues(0)&&sensor.GetSensorValues(0)<1000){k7 = 2;}
    else{k7 = 0;}
    if(k1+k2+k3+k4+k5+k6+k7 != 0){
      Stop=0;
      errS = ((k1*1+k2*2+k3*3+k4*4+k5*5+k6*6+k7*7)/((k1+k2+k3+k4+k5+k6+k7)/2))-8;
      PID sensor(errS, sampleTime, 0.1, 0, 0);
      pwmSetR = 0.5 - sensor.pidcontrol();                          
      pwmSetL = 0.5 + sensor.pidcontrol();
      double errR = pwmSetR - SpeedR;
      double errL = pwmSetL - SpeedL;
      PID wheelR(errR, sampleTime, 1, 0.01, 0.02);
      PID wheelL(errL, sampleTime, 1, 0.01, 0.02);
      uL = pwmSetL+wheelL.pidcontrol();
      uR = pwmSetR+wheelR.pidcontrol();
      Mr.MotorWrite(uR);                               
      Ml.MotorWrite(-uL);
   
    }
    else{
      if (Stop==0){
      Mr.MotorWrite(0.7);                               
      Ml.MotorWrite(-0.7);   
      delay(400);
      Stop++;
      }
      
      pwmSetR = 0;                          
      pwmSetL = 0;
      double errRR = pwmSetR - SpeedR;
      double errLL = pwmSetL - SpeedL;
      PID wheelRR(errRR, sampleTime, 0.0001, 0, 0);
      PID wheelLL(errLL, sampleTime, 0.0001, 0, 0);
      uL = pwmSetL+wheelLL.pidcontrol();
      uR = pwmSetR+wheelRR.pidcontrol();
      Mr.MotorWrite(uR);                               
      Ml.MotorWrite(-uL);         
   
    }

    }
    
    Serial.printf("errS: ");
    Serial.println(errS);
    Serial.printf("L speed: ");
    Serial.println(SpeedL);
    Serial.printf("R speed: ");
    Serial.println(SpeedR);
    Serial.println("");
    CounterR = CounterL = 0;
    tic = millis();
  }
  else{
    toc = millis();
  } 

}
