// https://osoyoo.com/2019/07/30/introduction-of-osoyoo-2wd-balance-car-robot-kit/  includes a link to the schematic
// Schematic: https://github.com/osoyoo/Osoyoo-development-kits/blob/master/OSOYOO%202WD%20Balance%20Car%20Robot/Osoyoo%20Balance%20Car%20Robot%20digram.pdf
// Osoyoo Bluetooth module:https://osoyoo.com/2017/10/25/arduino-lesson-hc-06-bluetooth-module/
#include <PinChangeInt.h>
#include <MsTimer2.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <MemoryFree.h>

extern void setupAll(bool initial=true);

int freeMemMin=9999;
void checkFreeMem() {
  int fm=freeMemory();
  if (fm<freeMemMin) freeMemMin=fm;
}

// SPEED CONTROL SECTION: ALLOWS TO CONTROL THE WHEEL speedMeas //////////////////////////////////////////////////////////////////////////////////////
//TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

float pwmExtreme;

void setupSetSpeed(bool initial=true)
{
  if (initial) {  
    pinMode(IN1M, OUTPUT);
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);
    pinMode(IN4M, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);//enable TB6612FNG
    //初始化电机驱动模块
    digitalWrite(IN1M, 0);
    digitalWrite(IN2M, 1);
    digitalWrite(IN3M, 1);
    digitalWrite(IN4M, 0);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(STBY, 1);
  }
  pwmExtreme=0;
}

int deadOffset=10; //this is about the PWM value needed to start turning the wheels
int deadOk=2; //below this value, accept that the motors are not turning
void setSpeedDetails(float pwm1,float pwm2,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB)
//globals modified: pwmExtreme. Not mission critical, accept very rare scrambled outputs (from reading bytes before/during/after change)
//depending on modified globals: deadOk, deadOffset. Not critical, todo: change to 1 byte type uint_8
{ 
  if (abs(pwmExtreme)<abs(pwm1)) pwmExtreme=pwm1;
  if (pwm1>+deadOk) pwm1+=deadOffset;
  if (pwm1<-deadOk) pwm1-=deadOffset;
  pwm1=constrain(pwm1,-255,255);
  if (pwm1 >= 0) 
  {
  digitalWrite(Pin2, 0);
  digitalWrite(Pin1, 1);
  analogWrite(PinPWMA, pwm1);
  } 
  else 
  {
  digitalWrite(Pin2, 1);
  digitalWrite(Pin1, 0);
  analogWrite(PinPWMA, -pwm1);
  }

  if (abs(pwmExtreme)<abs(pwm2)) pwmExtreme=pwm2;
  if (pwm2>+deadOk) pwm2+=deadOffset;
  if (pwm2<-deadOk) pwm2-=deadOffset;
  pwm2=constrain(pwm2,-255,255);
  if (pwm2 >= 0) 
  {
  digitalWrite(Pin4, 0);
  digitalWrite(Pin3, 1);
  analogWrite(PinPWMB, pwm2);
  } 
  else
  {
  digitalWrite(Pin4, 1);
  digitalWrite(Pin3, 0);
  analogWrite(PinPWMB, -pwm2);
  }
  checkFreeMem();
}

void setSpeedSimple(float pwm1, float pwm2) {setSpeedDetails(pwm1,pwm2,IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);}

// POSITION MEASUREMENT SECTION ////////////////////////////////////////////////////////////////////////////////////////////////////

//Encoder count signal
#define PinALeft 2  //Interrupt 0
#define PinBLeft 3  //sign
#define PinARight 4 //Interrupt 1
#define PinBRight 5 //sign

//both variables are modified within ISR and are multi-byte. The must be accessed outside ISRs only when interrupts are blocked. checked ok.
volatile long countRight = 0;
volatile long countLeft = 0;
void CodeLeft() 
{
  if (1-(digitalRead(PinBLeft) ^ digitalRead(PinALeft))==0) countLeft++; else countLeft--;
} 

void CodeRight() 
{
  if (digitalRead(PinBRight) ^ digitalRead(PinARight)==0) countRight++; else countRight--;
}

long pr,pl;
float xpos,ypos;
float xyangle;
float speedFilterPrev,rotFilterPrev;

void xyangleZero() {noInterrupts(); xpos=0; ypos=0; xyangle=0; interrupts();}
void setupPositionMeasurement(bool initial=true)
{
  if (initial) {
    pinMode(PinALeft, INPUT);  
    pinMode(PinARight, INPUT);
    pinMode(PinBLeft, INPUT);  
    pinMode(PinBRight, INPUT);
    attachInterrupt(0, CodeLeft, CHANGE);
    attachPinChangeInterrupt(PinARight, CodeRight, CHANGE);
  }
  noInterrupts();  //arduino.h: #define interrupts() sei(), #define noInterrupts() cli()
  countLeft=0;
  countRight=0;
  interrupts();
  pr=0;
  pl=0;
  xpos=0;
  ypos=0;
  speedFilterPrev=0;
  rotFilterPrev=0;
  xyangleZero();
  checkFreeMem();
}

float buildup = 0.03;  
float tickrad=1600/90*180/PI; //C1600 rotMeas ~90 degrees. todo2: tune later
float speedMeas,speedFiltered; 
float rotMeas,rotFiltered; 
float xposGet() {noInterrupts(); float x=xpos; interrupts(); return x;}
float yposGet() {noInterrupts(); float y=ypos; interrupts(); return y;}
float xyangleGet() {noInterrupts(); float xy=xyangle; interrupts(); return xy;}

void getSpeedRotFiltered()
//note: pr,pl, speedmeas, xpos, ... are modified and used within inter() ISR, including logging, which happens straightforwardly and consistently. 
//However, when accessing these values from outside/main routine, multi-byte values may be inconsistent. Use OnOff mechanism or noInterrupts()/interrupts() helps to measure correctly. todo: check that this is implemented correctly where necessary, e.g. similar to xposGet()
{
    noInterrupts();
    long lpulse = countLeft;  //long readout is a multi-step process in a 8bit controller. therefore must disable interrupts to change the value between byte transfers.
    countLeft = 0;
    interrupts();
    noInterrupts();
    long rpulse = countRight;
    countRight = 0;
    interrupts();
    pr+=rpulse;
    pl+=lpulse;

    speedMeas =(float) (rpulse + lpulse);
    rotMeas = (float) (rpulse - lpulse);
    
    xyangle=(pl-pr)/tickrad; //convert to angle in radians
    xpos+=speedMeas*cos(xyangle);
    ypos+=speedMeas*sin(xyangle);
    
    speedFiltered = speedFilterPrev + speedMeas * buildup;  //"return" value
    speedFilterPrev = speedFiltered * (1-buildup);
    
    rotFiltered = rotFilterPrev + rotMeas * buildup;    //"return" value
    rotFilterPrev = rotFiltered * (1-buildup);
}

// MOTION MEASUREMENT SECTION ///////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050 mpu; 

void setupMotionMeasurement(bool initial=true)
{
  if (initial) {
    delay(1500);
    mpu.initialize();//初始化MPU6050
    delay(2);
  }
}

int16_t ax, ay, az, gx, gy, gz;
void measureMotion()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);//IIC获取MPU6050六轴数据 ax ay az gx gy gz
}

// KALMAN FILTER SECTION ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//updated each cycle:
float angle_KF=0.0; // The output value of the Kalman filter, the best estimated angle
float gyroMeas=0.0; // The output value of the Kalman filter, the optimal estimated angular velocity
float q_bias=0; //Q_bias is gyroscope drift
float P[2][2] = {{1, 0 },{ 0, 1} }; // P matrix in the formula, the covariance of X

//constant parameters:
float Q_angle=0.001; // The covariance of the gyroscope noise (the error covariance of the estimation process)
float Q_gyro=0.005; // Covariance of gyroscope drift noise (error covariance of estimation process)
float R_angle=0.5; // Covariance of noise measured by accelerometer
float dt=0.005; //Integration time, dt is the filter sampling time (seconds)
float C_0 = 1; // A number of H matrix

//intermediary variables:
float angle_err=0; // intermediary
float PCt_0=0, PCt_1=0, E=0; //Intermediate variable
float K_0=0, K_1=0, t_0=0, t_1=0; //K is the Kalman gain, t is the intermediate variable
float Pdot[4] = {0,0,0,0}; //Calculate the intermediate variables of the P matrix

float KalmanFilter(float angleMeas, float gyroMeas)
{
  angle_KF += (gyroMeas - q_bias) * dt;  //Angle measurement model equation, angle estimation value = the last optimal angle + (angular velocity-the last optimal zero drift)*dt 
                                  // In terms of drift, consider each time Are all the same Q_bias=Q_bias
  angle_err = angleMeas - angle_KF;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];  //derivative of the covariance matrix: Pdot = A*P + P*A' + Q
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];  // Intermediate variables of matrix multiplication
  PCt_1 = C_0 * P[1][0];  //C_0=1
  E = R_angle + C_0 * PCt_0; // compute the error estimate: S = C P C' + R 
  K_0 = PCt_0 / E;  // Kalman gain, two, one is Angle, one is Q_bias. // Compute the kalman filter gains: K = P C' inv(S) 
  K_1 = PCt_1 / E;
  t_0 = PCt_0; // Matrix calculation intermediate variable, equivalent to a
  t_1 = C_0 * P[0][1]; // Matrix calculation intermediate variable, equivalent to b
  P[0][0] -= K_0 * t_0; // t_0=C_0 * P[0][0] // Update covariance matrix: P = P - K C P 
  P[0][1] -= K_0 * t_1; // t_1=C_0 * p[0][1]
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;

  // Update the state (new)estimates (Correct the prediction of the state). Also adjust the bias on the gyro at every iteration. 
  // x = x + K * y  
  angle_KF += K_0 * angle_err; // Calculate the optimal angle
  q_bias += K_1 * angle_err; //Calculate the optimal zero drift
  //angle_dot = gyroMeas - q_bias; // Calculate the optimal angular velocity
  checkFreeMem();
  return angle_KF;
}

// https://www.programmersought.com/article/24884671436/
// https://os.mbed.com/users/cdonate/code/kalman/file/8a460b0d6f09/kalman.c/
// https://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2
// http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
// https://www.intechopen.com/books/kalman-filters-theory-for-advanced-applications/kalman-filter-for-moving-object-tracking-performance-analysis-and-filter-design  H=(1,0,0,1)
// https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
// C:\a_\py\jupyter\balanced robot Kalman matrix simplifications.ipynb

// CONTROL LOOP SECTION ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int echo=1;
String errstr=""; //not reset by setup*. Changed via addErr also within ISRs. Should bracket in noInterrupts() when accessing. 
void addErr(String add) {
  noInterrupts();
  if (errstr.indexOf(add) == -1) errstr=errstr+add; //avoid that same error is recorded multiple times, potentially risking memory overflow
  interrupts();
}
void printNonemptyErrstr() {
    const int bufsize=20;
    char buff[bufsize];
    noInterrupts();
    if (errstr!="") {
      errstr.toCharArray(buff,bufsize); //always returns a (potentially truncated) 0-terminated string within buff size
      errstr="";
      interrupts();
      Serial.print(F("! ")); Serial.println(buff);
    } else {
      interrupts();    
    }
  }

int sgn(float x) {return (x>0)-(x<0);}

// build trajectory for control loop to execute

const int maxTbl=5;
int useTbl=0; //number of valid values
bool activateMotion=false;
long timeTbl[maxTbl]; //loop number (exclusive) until which the ..IncTbl[] value is to be used
float speedIncTbl[maxTbl];  //value to increase target speed each loop
float rotIncTbl[maxTbl]; //value to increase target rotation each loop
int repeatCount;

float calcSum(char title[],float incTbl[],long tTbl[],int useTbl, bool verb) {
  float sumis=0;
  float v=0;
  for(long i=0;i<tTbl[useTbl-1];i++) {
    int j=0;
    while (j<useTbl-1 and tTbl[j]<=i) j++; //now j is the index of tTbl:   tTbl[j-1]<=i< tTbl[j]
    v+=incTbl[j];
    sumis+=v;
  }
  if (verb) {
    Serial.print(title); 
    Serial.print(F(" sum:")); Serial.print(sumis); 
    if (v!=0) {Serial.print(F(" end:")); Serial.print(v);}
  }
  return sumis;
}

void printTbls() {
    Serial.print(F(" useTbl:")); Serial.print(useTbl);
    for (int i=0; i<useTbl; i++) {  
      Serial.print(F("  time:")); Serial.print(timeTbl[i]);
      if (speedIncTbl[i]!=0) {Serial.print(F(" speedInc:")); Serial.print(speedIncTbl[i]); }  //simplify output by not printing 0 values
      if (rotIncTbl[i]!=0) {Serial.print(F(" rotInc:")); Serial.print(rotIncTbl[i]); }
      Serial.print(F(" "));
    }
    calcSum(" speed>",speedIncTbl,timeTbl,useTbl,true);
    calcSum(" rot>",rotIncTbl,timeTbl,useTbl,true);
    Serial.println();
    Serial.flush();
}
  
void buildTrajectorySimple(float incTbl[], long tTbl[], float target, float slope, float top) {
  /*           --------------------------
   *        /          value: top         \
   *      / slope                           \ -slope
   *    /        total area A = target        \
   *   -----------------------------------------------> time in loop-count steps
   *   0       t1                      t2      t3=t2+t1
   *   
   *   t = time measured in loop-count steps
   *   upward slope: v(t) = slope * t
   *   area in both sloped domains: 
   *   - large target: 
   *        t1 = roundUp(top/slope). As = t1**2 * slope.  
   *        At = target - As. => t2 = t1 +At/top
   *        must take care of rounding of t1, t2 => adjust scale of v(t)
   *   - small target, does not have a flat "top" section
   *        target= A = t1 * vmax. t1 = vmax/slope => target = vmax**2/slope => vmax=sqrt(target*slope), t1=roundUp(vmax/slope)=roundUp(sqrt(target/slope)) 
   *   - start with "small target" case, when vmax>top, switch to large target case
   */
  bool verb=false;
  if (verb) {Serial.print(F(" target:")); Serial.print(target); Serial.print(F(" slope:")); Serial.print(slope); Serial.print(F(" top:")); Serial.print(top); }
  float vmax = sqrt(abs(target)*slope);
  incTbl[0]=slope*sgn(target); //0 for target==0
  if (vmax<=top) {
    if (verb) {Serial.print(F(" vmax:")); Serial.print(vmax); Serial.print(F(" small>"));}
    long t1 = trunc(vmax/slope+1);  //so even for target==1, we have t1=1. So no special cases to consider, especially useTbl is nonzero
    useTbl=2;
    tTbl[0]=t1;
    incTbl[1]=-incTbl[0];
    tTbl[1]=t1*2;
  } else {
    long t1 = trunc(top/slope+0.99999999);
    float As = t1*t1 * slope;
    long dt2=max(0,trunc((abs(target)-As)/top+0.99999999));
    if (verb) {Serial.print(F(" t1:")); Serial.print(t1); Serial.print(F(" As:")); Serial.print(As); Serial.print(F(" dt2=")); Serial.print(dt2); }
    useTbl=3;
    tTbl[0]=t1;
    incTbl[1]=0;
    tTbl[1]=t1+dt2;
    incTbl[2]=-incTbl[0];
    tTbl[2]=2*t1+dt2;
  }
  for (int i=0; i<useTbl; i++) {  
    if (verb) {Serial.print(F(" inc:")); Serial.print(incTbl[i]); Serial.print(F(" time:")); Serial.print(tTbl[i]); }
  }
  float sumis=calcSum("x>",incTbl,tTbl,useTbl,false);
  if (verb) {Serial.print(F(" target/sumis:")); Serial.print(target/sumis);}
  if (sumis!=0) for(int i=0;i<useTbl;i++) incTbl[i]*=target/sumis;
  for (int i=0; i<useTbl; i++) {  
    if (verb) {Serial.print(F(" incTbl:")); Serial.print(incTbl[i]); Serial.print(F(" tTbl:")); Serial.print(tTbl[i]); }
  }
  if (verb) Serial.println();
}

void buildTrajectory(float posTarget, float posSlope, float posTop, float rotTarget, float rotSlope, float rotTop) {
  long timeTbl2[maxTbl]; //dummy
  if (posTarget!=0) {  //in this case let the position define the timing, then scale the rotation to this. this is a shortcut, todo!
    buildTrajectorySimple(speedIncTbl, timeTbl, posTarget, posSlope, posTop);
    int save=useTbl;
    buildTrajectorySimple(rotIncTbl, timeTbl2, rotTarget, rotSlope, rotTop); 
    if (save==2 and useTbl==3) {addErr("err1"); useTbl=0; return;};
    if (save==3 and useTbl==2) {rotIncTbl[2]=rotIncTbl[1]; rotIncTbl[1]=0;};  //careful! indices are not checked vs. overflow!
    useTbl=save;
    float sumis=calcSum("rot>",rotIncTbl,timeTbl,useTbl,false);
    if (sumis!=0) for(int i=0;i<useTbl;i++) rotIncTbl[i]*=rotTarget/sumis;
  } else {
    if (rotTarget!=0) {
      buildTrajectorySimple(rotIncTbl, timeTbl, rotTarget, rotSlope, rotTop); 
      for(int i=0;i<useTbl;i++) speedIncTbl[i]=0;
    } else {
      useTbl=0;
    }
  }
}

enum OnOff {STOPPED,STARTED,STOPPING};  //phases: 0: start program in STOPPED. 1. outside ISR: set to STARTED. 
//2.&4. ISR performs in STARTED/STOPPING, at end of ISR moves STOPPING into STOPPED. 3. outside ISR: set to STOPPING, wait for STOPPED (then in phase 5)
OnOff addDeviMeas=STOPPED; //OnOff-phase 0
float posDeviMeas, posDeviMax, posDeviMin, dirDeviMeas, dirDeviMax, dirDeviMin, angleDeviMeas;
long numDeviMeas;
void initDeviMeasurement() {
  posDeviMeas=posDeviMax=posDeviMin=dirDeviMeas=dirDeviMax=dirDeviMin=angleDeviMeas=0.0;
  numDeviMeas=0;
  addDeviMeas=STARTED; //OnOff-phase 1
}
void reportDeviMeasurement() {
  addDeviMeas=STOPPING; //OnOff-phase 3
  delay(11); //should suffice for 1 ISR call needed for state transition to STOPPING
  interrupts(); //this is required, if "Serial.print(addDeviMeas);" not present in next line while statement. Unclear why. Todo3
  while (addDeviMeas==STOPPING) {delay(5);}; //wait for ISR to terminate and switch to STOPPED 
  Serial.print(F(" n:")); Serial.print(numDeviMeas);
  Serial.print(F(" posDeviMin:")); Serial.print(posDeviMin);
  Serial.print(F(" posDeviMax:")); Serial.print(posDeviMax);
  Serial.print(F(" posDevi_RMS:")); Serial.print(sqrt(posDeviMeas/numDeviMeas));
  Serial.print(F(" dirDeviMin:")); Serial.print(dirDeviMin);
  Serial.print(F(" dirDeviMax:")); Serial.print(dirDeviMax);
  Serial.print(F(" dirDevi_RMS:")); Serial.print(sqrt(dirDeviMeas/numDeviMeas));
  Serial.print(F(" angleDeviMeas_avg:")); Serial.println(angleDeviMeas/numDeviMeas);
}

float amplifySmall(float posDevi, float linearAbove, float amplify) {
  return (abs(posDevi)>linearAbove) ? posDevi : 
        ((abs(posDevi)*amplify>linearAbove) ? linearAbove*sgn(posDevi) : amplify*posDevi );
}

float angleMeas=0;  //output every loop
long loopNumber=0;

float posDevi;
float dirDevi;
float kp,kd,kpSpeed,kiSpeed,kiRot; 
float Imax, Iforce, Iticks, Integral;

enum Control {STOP,RUN};
Control controlMode;  

const int logLen = 250;
int logTbl[logLen]; //may change type to what fits recordings
int logCount = logLen+1;
int logSkip = 1;

float pwm1 = 0, pwm2 = 0; //just output
enum RecordMode {OFF,DYNAMICS,TIMING,PRINTING}; //TIMING is a deprecated measurement mode, some parts are in comments at end of code
RecordMode recordMode=OFF;

float angleFiltered; //loop output
float speedPID =0;
float positionPI; //global for debugging/printing

unsigned long usmax=0;
float angleVertical=5.5; //optimize manually by 'a'/'A' commands, want angleDeviMeas_avg to be close to 0.  5.5 is good for additional USB cable/ferrite connected, 2.1 if USB is connected to raspberry pi
float speedInc,rotInc;
long useLoops;

void setupControlLoop(bool initial=true)
{
  if (initial) {
    MsTimer2::set(5, inter);
    MsTimer2::start();
  }
  controlMode=STOP;
  kp = 90; //new values with Raspberry pi
  kd = 1.0;
  kpSpeed =50.0; 
  kiSpeed = 0.25; 
  kiRot =0.15;
  Imax=2.0; //PWM values
  Iforce=4; //PWM values per second
  Iticks=50; 
  Integral=0;
  posDevi=0;
  dirDevi=0;
  repeatCount=-1;
  useLoops=-1;
  addDeviMeas=STOPPED;  //more direct than STOPPING
  speedInc=0;
  rotInc=0;
}

float posLoc=400;
float posAmp=1;
float dirLoc=40;
float dirAmp=1;

bool loopActive() {return repeatCount>0;}; //note that "loopnumber<=useloops" is bad, as loopNumber is multi-byte and changed in inter() [and after sei(), therefore noInterrupts() does not help]

void inter()
{
  sei();//allow interrupts again within this routine. this is necessary for the wheel rotation counters
  unsigned long us=micros();
  loopNumber+=1; 
  if (activateMotion) {
    activateMotion=false;
    if (useTbl>0) useLoops=loopNumber+timeTbl[useTbl-1]; else useLoops=-1; //this is what activates the motion (if loopNumber<useLoops)
    speedInc=0;
    rotInc=0;
  }
  if (loopNumber==useLoops and repeatCount>1) {repeatCount--; useLoops+=timeTbl[useTbl-1];}  //compare repeatCount>1 not 0, because at loopNumber==useLoops, 1 run already happened
  if (loopNumber<useLoops) {
    long loopsPassed=loopNumber+timeTbl[useTbl-1]-useLoops; //max is timeTbl[useTbl-1]-1
    int timeindex=0;
    while (timeindex<useTbl-1 and timeTbl[timeindex]<=loopsPassed) timeindex++; //now timeindex is the index of timeTbl:   timeTbl[timeindex-1] <= loopsPassed < timeTbl[timeindex]
    speedInc+=speedIncTbl[timeindex];
    rotInc+=rotIncTbl[timeindex];
  } else {
    if (loopNumber==useLoops) {repeatCount--; speedInc=0; rotInc=0;} //this only happens at the end of the last loop. reduces repeatCount to 0, and ensures no runaway
  }
  
  measureMotion(); //measure accelerations ay, az, and gyro gx
  float angleMeas = atan2(ay , az) * 180 / PI + angleVertical;       
  gyroMeas = (gx - 128.1) / 131;            
  angleFiltered=KalmanFilter(angleMeas, gyroMeas);
  if (abs(angleFiltered) > 30) {
    setupAll(false);
    addErr("angle>30. ");
    //command=""; //avoid. This could hang if Rpi waits for command response. Also would require masking interrrupt when working with command outside ISR
  }
  float anglePD = kp * angleFiltered  + kd * gyroMeas;

  getSpeedRotFiltered(); //read out wheel positions. values are stored in pl, pr, speedMeas, rotMeas, speedFiltered, rotFiltered  
  posDevi += speedMeas + speedInc; 
  float addto=sgn(posDevi)*Iforce;
  if (abs(posDevi)<Iticks) addto*=abs(posDevi)/Iticks;
  Integral = constrain(Integral+addto*dt,-Imax,Imax);
  float posDevi2= amplifySmall(posDevi,posLoc,posAmp); //when deviation is already low, then amplify response. Avoids overshooting at higher values and improves precision of target achievement
  positionPI = kiSpeed * posDevi2 + kpSpeed * speedFiltered;
  speedPID = -anglePD + positionPI + Integral;

  dirDevi += rotMeas + rotInc; 
  float dirDevi2= amplifySmall(dirDevi,dirLoc,dirAmp);
  float dirAct=dirDevi2*kiRot; 
  
  if (addDeviMeas!=STOPPED) {
    posDeviMeas+=posDevi*posDevi; dirDeviMeas+=dirDevi*dirDevi; angleDeviMeas+=angleMeas;
    if(posDeviMax<posDevi) posDeviMax=posDevi;
    if(dirDeviMax<dirDevi) dirDeviMax=dirDevi;
    if(posDeviMin>posDevi) posDeviMin=posDevi;
    if(dirDeviMin>dirDevi) dirDeviMin=dirDevi;
    numDeviMeas+=1;
    if (addDeviMeas==STOPPING) addDeviMeas=STOPPED;
  }

  pwm1=speedPID+dirAct;
  pwm2=speedPID-dirAct;
  if (controlMode==STOP) pwm1=pwm2=0;
  setSpeedSimple(pwm1,pwm2);

  if (recordMode==DYNAMICS and ((loopNumber % logSkip) ==0)) {
    if (logCount<logLen) logTbl[logCount++]=pl+pr-posDevi;
    if (logCount<logLen) logTbl[logCount++]=posDevi;
    if (logCount<logLen) logTbl[logCount++]=(int) (angleFiltered*10);
    if (logCount<logLen) logTbl[logCount++]=speedMeas;
    if (logCount<logLen) logTbl[logCount++]=speedPID;
  }
  if (recordMode!=OFF and logCount>=logLen) recordMode=PRINTING; //only executed when data collection is active and data is fully recorded
  us=micros()-us;
  if (us>usmax) usmax=us;
  checkFreeMem();
}

int repeatMax=2;
void activateRepeatMotion(int repeats) {
  repeatCount=max(1,repeats); 
  //controlMode=RUN; //start motion only with "s" command, no others.
  if (useTbl>0) activateMotion=true;
}  //careful: mind the sequence to start actions

// PRINT STATUS //////////////////////////////////////////////////////////////////////////////////////////////////////

// convention for first character in each output line, so recipient can filter:
// ".": info about program start (no more "... " to indicate waiting times)
// ">": repeat of console input = commands
// " ": data follows. stdandard data output: " "+name+":"+value. 
//                    Exception1: spaces between ":" and value. Rule: after each ":" spaces are allowed but then a nbr must follow.
//                    Exception 2: if there is no ":", then this is independent text without a following number
// "|": tabular data line follows 

void setupPrint(bool initial=true)
{ 
  if (initial) {
    Serial.begin(115200); //9600); 
    Serial.setTimeout(100); //100 ms, default is 1000ms
    Serial.println();
    Serial.println(F(".started..."));
    Serial.flush(); //wait until data is sent, dont want it to accumulate
  }
}

void printFloat(const __FlashStringHelper * title, float f, int total=7, int post=2) {
    char chartemp[20];
    dtostrf(f, total, post, chartemp); Serial.print(title); Serial.print(chartemp);
}
void printLong(const __FlashStringHelper * title, long l, int total=4) {
    String str = String(l);
    Serial.print(title);
    for (int i=0; i<total-str.length(); i++) Serial.print(F(" "));
    Serial.print(str);
}
void printStatus() {
  if (recordMode==PRINTING) {
    recordMode=OFF;
    //avoid table headings. Complexifies pickup. Serial.println(F(".  trgt posDv ang10  spdM   pwm"));
    for (int i=0; i<logLen; i++) {
      printLong(F("|"),logTbl[i], 5); 
      //if ((i+1)%logRows==0) Serial.println(); //no more multiline, complexifies data pickup
    }
    Serial.println();
    Serial.flush();
  }
}

void printControlParams1() {
    printFloat(F(" kp:"),kp);
    printFloat(F(" kd:"),kd);
    printFloat(F(" kiSpeed:"),kiSpeed);
    printFloat(F(" kpSpeed:"),kpSpeed);
    printFloat(F(" kiRot:"),kiRot);
    printFloat(F(" buildup:"),buildup,7,3);
    printFloat(F(" Q_angle:"),Q_angle);
    printFloat(F(" Q_gyro:"),Q_gyro);
    printFloat(F(" R_angle:"),R_angle);
    Serial.println(F("  modify_via=PDipjb,ka,kg,kr"));
    Serial.flush(); //wait until data is sent, dont want it to accumulate
}

void printControlParams2() {
    printFloat(F(" posLoc:"),posLoc);
    printFloat(F(" posAmp:"),posAmp);
    printFloat(F(" dirLoc:"),dirLoc);
    printFloat(F(" dirAmp:"),dirAmp);
    printFloat(F(" deadOk:"),deadOk);
    printFloat(F(" deadOffset:"),deadOffset);
    printFloat(F(" tickrad:"),tickrad);
    Serial.println(F("  modify_via=yp,yP,yd,yD,z,Z,T"));
    Serial.flush(); //wait until data is sent, dont want it to accumulate
}

void printController1() {  
    printLong (F(" xpos:"),xposGet(),5);
    printLong (F(" ypos:"),yposGet(),5);
    printFloat(F(" xyangle/deg:"),xyangleGet()/PI*180);
    Serial.println();
    Serial.flush();
}

void printController2() {  
    printFloat(F(" posDevi:"),posDevi);
    printFloat(F(" dirDevi:"),dirDevi);
    printFloat(F(" positionPI:"),positionPI);
    printFloat(F(" speedPID:"),speedPID);
    printLong (F(" pwm1:"),pwm1);
    printLong (F(" pwm2:"),pwm2);
    printLong (F(" extreme_wheel_PWM:"),pwmExtreme); //". should not exceed 255 a lot regularly."); 
    pwmExtreme=0;
    Serial.println();
    Serial.flush();
}

void printLoopInfo() {
    printLong(F(" useLoops:"),useLoops,6);
    printLong(F(" loopNumber:"),loopNumber,6);
    printLong(F(" repeatCount:"),repeatCount);
    printLong(F(" repeatMax:"),repeatMax);
    printLong(F(" active:"),loopActive());
    Serial.println();
    Serial.flush(); 
}

void printIntegralInfo() {
    printFloat(F(" Imax:"),Imax,10,3);
    printFloat(F(" Iforce:"),Iforce,10,3);
    printFloat(F(" Iticks:"),Iticks,10,3);
    printFloat(F(" Integral:"),Integral,10,3);
    Serial.println();
    Serial.flush(); 
}

int freeMemReport=100;
int usmaxReport=3500;
void checkHealth() {
  checkFreeMem();
  if (freeMemMin<freeMemReport) {freeMemReport=freeMemMin; Serial.print(F("! freeMemMin:")); Serial.println(freeMemMin);}
  if (usmax>usmaxReport) {usmaxReport=usmax; Serial.print(F("! max_ISR_time:")); Serial.print(usmax);}  //must be well below 5000us
  printNonemptyErrstr();
}

// MAIN ROUTINES //////////////////////////////////////////////////////////////////////////////

String command;

void setupAll(bool initial=true) {
  setupSetSpeed(initial);
  setupPositionMeasurement(initial);
  setupMotionMeasurement(initial);
  setupControlLoop(initial);
  setupPrint(initial);
}

void setup() {setupAll();}

String numberSubstring() {
  int i=0;
  while (i<command.length()) {
    char ch=command.charAt(i);
    if (ch=='-' or ch=='.' or isDigit(ch)) i++; else break;
  }
  String ret=command.substring(0,i);
  command=command.substring(i);
  return ret;
}

float getValueFloat() {return numberSubstring().toFloat();}
float getCommaFloat() {expectcomma(); return getValueFloat();}
int getValueInt() {return numberSubstring().toInt();}
int getCommaInt() {expectcomma(); return getValueInt();}
long getValueLong() {return (long) numberSubstring().toFloat();}
void expectcomma() {
  if (command.length()>0 and command.charAt(0)==',') command=command.substring(1); else {addErr(F("comma_missing")); command="";}
}

void addTbl(float posTarget, float rotTarget, long timeSteps) {
  if (useTbl<maxTbl) {
    timeTbl[useTbl] = (useTbl>0) ? timeTbl[useTbl-1]+timeSteps : timeSteps;
    speedIncTbl[useTbl]=posTarget/(timeSteps*timeSteps); //note this acceleration is additive to previously created speeds/rotations
    rotIncTbl[useTbl]=rotTarget/(timeSteps*timeSteps);
    useTbl++;
  } else {addErr("addTBL_err");};
}

float posSlope=0.1;
float posTop=10;
float rotSlope=0.5; //about the speed of the control loop, which is not very aggresive
float rotTop=10;  //20 causes pwm values of ca. 250

void zigzag(float posTarget, float rotTarget, long fullTimePerCycle) {
  useTbl=4; 
  long t1=fullTimePerCycle/6;
  timeTbl[0]=t1; timeTbl[1]=3*t1; timeTbl[2]=4*t1; timeTbl[3]=6*t1; 
  float f1=posTarget/(t1*t1);
  speedIncTbl[0]=f1; speedIncTbl[1]=-f1; speedIncTbl[2]=f1; speedIncTbl[3]=0; 
  f1=rotTarget/(t1*t1);
  rotIncTbl[0]=f1; rotIncTbl[1]=-f1; rotIncTbl[2]=f1; rotIncTbl[3]=0;   
}

char getCommandChar() {if (command.length()>0) {char getcmd = command.charAt(0); command = command.substring(1); return getcmd;} else {return ' ';}}

void loop() 
{
  command=Serial.readString();
  command.replace("\n","");
  command.replace("\r","");
  if (echo>0 and command.length()>0) Serial.println(">"+command);  
  while (command.length()>0) {
    char getcmd = getCommandChar();
    switch(getcmd){
      case 'E': {echo=getValueInt(); break;}
      case 'o': {Serial.println(F(" ok")); break;}
      case 'e': {setupAll(false); break;} //end. Robot will fall to either side.
      
      case 's': {controlMode=RUN; break;} //start
      case 'f': {float f1=getValueFloat(); float f2=getCommaFloat(); int i3=getCommaInt(); zigzag(f1,f2, i3); activateRepeatMotion(repeatMax); break;} //unfortunately, c++ function parameters are evaluated right to left
      case 'F': {buildTrajectory(getValueFloat(),posSlope,posTop,0,rotSlope,rotTop); activateRepeatMotion(1); break;}
      case 'C': {buildTrajectory(0,posSlope,posTop,getValueFloat(),rotSlope,rotTop); activateRepeatMotion(1); break;}
      case 'W': {buildTrajectory(getValueFloat(),posSlope,posTop,getCommaFloat(),rotSlope,rotTop); activateRepeatMotion(1); break;}
      case '0': {noInterrupts(); posDevi=0; dirDevi=0; interrupts();}
      case 'T': {tickrad=getValueFloat(); break;}
      case 'v': {posSlope=getValueFloat(); break;} 
      case 'V': {posTop=getValueFloat(); break; }
      case 'r': {rotSlope=getValueFloat(); break;} 
      case 'R': {rotTop=getValueFloat(); break; }
      case 'q': {if (addDeviMeas!=STOPPED) reportDeviMeasurement(); else initDeviMeasurement(); break;} // 'q' like rootmeansQuare deviation
      case 'l': {while (loopActive()) {delay(5);};  break; } // use "lo" to get " ok" when done
      case 'd': {delay(getValueLong()); break; }

      case 'u': {useTbl=max(0,min(maxTbl-1,getValueLong())); break;}
      case 'U': {float f1=getValueFloat(); float f2=getCommaFloat(); int i3=getCommaInt(); addTbl(f1,f2,i3); break;}
      case 'x': {repeatMax=getValueLong(); break; }
      case 'X': activateRepeatMotion(repeatMax); break;

      case 'n': {switch (getCommandChar()) {  //i"n"fo
                    case 't': printTbls(); break;
                    case 'l': printLoopInfo(); break;      
                    case 'c': printController1(); break;
                    case 'C': printController2(); break;
                    case 'p': printControlParams1(); break;
                    case 'P': printControlParams2(); break;
                    case 'I': printIntegralInfo(); break;
                    default: addErr(F("err1")); command=""; break;
                } break;}
      case 'y': {switch (getCommandChar()) {  //amplif"y" 
                    case 'p': posLoc=getValueFloat(); break;
                    case 'P': posAmp=getValueFloat(); break;
                    case 'd': dirLoc=getValueFloat(); break;
                    case 'D': dirAmp=getValueFloat(); break;
                    default: addErr(F("err4")); command=""; break;
                } break;}
      case 'k': {switch (getCommandChar()) {  //set Kalman Filter noise values
                    case 'a': Q_angle=getValueFloat(); break;
                    case 'g': Q_gyro=getValueFloat(); break;
                    case 'r': R_angle=getValueFloat(); break;
                    default: addErr(F("err3")); command=""; break;
                } break;}
      case 'I': {switch (getCommandChar()) {  //set integration parameters
                    case 'm': Imax=getValueFloat(); break;
                    case 'f': Iforce=getValueFloat(); break;
                    case 't': Iticks=getValueFloat(); break;
                    default: addErr(F("err3")); command=""; break;
                } break;}

      case 'm': logSkip=getValueInt(); if (logSkip==0) logSkip=1; logCount=0; recordMode=DYNAMICS; break; //does not print on its own, must use M for this
      case 'M': {while (recordMode!=OFF and recordMode!=PRINTING) {delay(5);};  break; }

      case 'a': initDeviMeasurement(); delay(getValueLong()); reportDeviMeasurement(); break;  //standard: use 5000 to measure vertical alignment
      case 'A': angleVertical=getValueFloat(); break; //manually set to value of angleDeviMeas_avg output of previous line
      case 'P': kp=getValueFloat(); break;
      case 'D': kd=getValueFloat(); break;
      case 'i': kiSpeed=getValueFloat(); break;
      case 'p': kpSpeed=getValueFloat(); break;
      case 'j': kiRot=getValueFloat(); break;
      case 'b': buildup=getValueFloat(); break;
      case 'z': deadOk=getValueLong(); break;
      case 'Z': deadOffset=getValueLong(); break;
      case 'N': xyangleZero(); break;
      case '#': command=""; break; //the rest is commented out
      case ';': break; //use e.g. for command "F10;3"
      
      case 10 : break; //newline
      default:  Serial.print(F("! unknown_input_key_number:")); Serial.println((int)getcmd); command=""; break;
    }  
    printStatus();
    checkHealth();
  }
}

//commands for re-use:
//m10;F-1000;
//q;F-1000;l;d500;q
//x1;q;f-1000,500,400;l;d500;q
//x1;q;f-2000,1600,1000;l;d1;q

/* todos:
 *  the problems I want to fix: 1. unprecise positioning, 2. combine speed+rotation, 3. performance
 *  prio2: 
 *  - check which parameters matter and then optimize for 
 *    1. reducing oscillations around 0, 
 *    2. improving target achievement precision/time. 
 *  - check for String mem use, try Reserve()
 *  - update Kalman-Filter to full 2 dim?
 *  prio3 = don't do:
 *  - fail-safe mode, i.e. acting on PWM when speed or angle get close to critical. -> not main lever to improve performance 
 *    (better optimize flow logic, delay/stabilization times, when to take 1 or 2 images...)
 *  - Improving measurement of tilt angle, including known information on intended longitudinal acceleration
 *  - 2 point controller to start robot from lying state (tried it early, but vibrations matter a lot)
*/

/* most painfilly sought bugs:
 * - recordMode==DYNAMICS  //unused comparison instead of assignment
 * - tbl[3]=tbl[2] //overwriting other data at array size of 3
 * - forgetting return-statement at end of function. Function will still return some random value, difficult to detect
 */
/*void measureTiming() {  //note: there can be interrupts inbetween time measurements
  logCount=0;
  unsigned long us;
  us = micros();                                    logTbl[logCount++]=(micros()-us); //result: typically 4
  us = micros(); float x=random(1000)/1000;         logTbl[logCount++]=(micros()-us); //result: typically 52, slower than I thought...
  us = micros(); x=sin(x);                          logTbl[logCount++]=(micros()-us); //result: typically 4, much faster than I thought...
  us = micros(); Serial.print(".");                 logTbl[logCount++]=(micros()-us); //result: typically 12 or 16. This just initiates the sending
  us = micros();                    Serial.flush(); logTbl[logCount++]=(micros()-us); //result: typically around 1040, reasonable for 8 bits at 9600 baud
  us = micros(); Serial.print("."); Serial.flush(); logTbl[logCount++]=(micros()-us); //result: typically around 1050
  
  logTbl[logCount++]=0; //marker, to show beginning of recording for each loop 
  recordMode=TIMING; //must be at end of this fuction, as otherwise interrupts may already occur and add data
}*/
