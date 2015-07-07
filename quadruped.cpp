#include "Arduino.h"
#include <Servo.h>

//Initialize Servo Objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

char state;            // Stores the value from the Bluetooth module 



//Distance Formula
float distance(float x1, float y1, float x2, float y2) {
  float result = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  return result;
}

//Determines angle of a line connecting two points relative to positive x axis
float angle(float x1, float y1, float x2, float y2) {
  float result = atan2(y2 - y1, x2 - x1);
  return result;
}

//Finds the intersection of two circles and returns the x coordinate
//Basically the only function you need to solve a mechanism's kinematic state
float XcircleFind(float x1, float y1, float r1, float x2, float y2, float r2, float sign) {

  float D = distance(x1, y1, x2, y2);
  float xr = (pow(r1, 2) - pow(r2, 2) + pow(D, 2)) / (2 * D);
  float yr = sign * sqrt(pow(r1, 2) - pow(xr, 2));
  float thetaR = atan2(yr, xr);

  float thetaD = angle(x1, y1, x2, y2);
  float thetaT = thetaD - thetaR;
  float x3 = r1 * cos(thetaT) + x1;
  float y3 = r1 * sin(thetaT) + y1;
  return x3;
  // return array of x,y coordinates
}

//Finds the intersection of two circles and returns the y coordinate
//This is only split into two functions because I dont know how to return 
//multiple variables from a single function
float YcircleFind(float x1, float y1, float r1, float x2, float y2, float r2, float sign) {

  float D = distance(x1, y1, x2, y2);
  float xr = (pow(r1, 2) - pow(r2, 2) + pow(D, 2)) / (2 * D);
  float yr = sign * sqrt(pow(r1, 2) - pow(xr, 2));
  float thetaR = atan2(yr, xr);

  float thetaD = angle(x1, y1, x2, y2);
  float thetaT = thetaD - thetaR;
  float x3 = r1 * cos(thetaT) + x1;
  float y3 = r1 * sin(thetaT) + y1;
  return y3;
}


//Performs inverse kinematics
//Just like the previous function, IKBACKSERVO and IKFRONTSERVO
//could be merged into one function but I don't know how to return 
//two variables at once
float IKBACKSERVO (float x3 = 0.5, float y3 = -5)
{
  x3 += .5; //Makes it act like x=0 at crotch center

  float x1 = 0.0;         //back hip perm location
  float x2 = 1.0;         //front hip perm location
  //float x3 = 0.5;    //assigned foot location
  float x4;                 //back knee solved
  float x5;                 //front knee solved
  float x6 = 0.0;         //back servo perm location
  float x7 = 1.0;         //front servo perm location
  float x8;                 //rear springtop solved
  float x9;                 //front springtop solved

  float y1 = 0.0;         //back hip perm location
  float y2 = 0.0;         //front hip perm location
  //float y3 = -4;   //assigned foot location
  float y4;                 //back knee solved
  float y5;                 //front knee solved
  float y6 = 1.25;      //back servo perm location
  float y7 = 1.25;      //front servo perm location
  float y8;                 //rear springtop solved
  float y9;                 //front springtop solved

  float r1 = 3.5; //back thigh length
  float r2 = 3.5; //front thigh length
  //float r3 = 0; //nothing to assign here
  float r4 = 4.0; //backcalf length
  float r5 = 4.0; //frontcalf length
  float r6 = 1.0; //backcrank length
  float r7 = 1.0; //frontcrank length
  float r8 = 4.25; //frontspring length  (kinda fudged)
  float r9 = 4.25; //backspring length  (kinda fudged)
  float sign; //determines is top or bottom solution is chosed for circle intersection




  x4 = XcircleFind(x1, y1, r1, x3, y3, r4, 1); //
  y4 = YcircleFind(x1, y1, r1, x3, y3, r4, 1);

  x5 = XcircleFind(x2, y2, r2, x3, y3, r5, -1);
  y5 = YcircleFind(x2, y2, r2, x3, y3, r5, -1);

  x8 = XcircleFind(x6, y6, r6, x4, y4, r8, 1);
  y8 = YcircleFind(x6, y6, r6, x4, y4, r8, 1);

  x9 = XcircleFind(x7, y7, r7, x5, y5, r9, -1);
  y9 = YcircleFind(x7, y7, r7, x5, y5, r9, -1);

  float thetas1 = angle(x6, y6, x8, y8);
  float thetas2 = angle(x7, y7, x9, y9);

  float backservoangle = 180 * thetas1 / PI - 45;
  float frontservoangle = 180 * thetas2 / PI + 45;

  if (backservoangle < 0)
  {
    backservoangle += 360;
  }
  if (frontservoangle < 0)
  {
    frontservoangle += 360;
  }

  return backservoangle;
}


//Same as last function, but needed because I dont know how to return multiple variables
float IKFRONTSERVO (float x3 = 0.5, float y3 = -5)
{
  x3 += .5; //Makes it act like x=0 at crotch center

  float x1 = 0.0;         //back hip perm location
  float x2 = 1.0;         //front hip perm location
  //float x3 = 0.5;    //assigned foot location
  float x4;                 //back knee solved
  float x5;                 //front knee solved
  float x6 = 0.0;         //back servo perm location
  float x7 = 1.0;         //front servo perm location
  float x8;                 //rear springtop solved
  float x9;                 //front springtop solved

  float y1 = 0.0;         //back hip perm location
  float y2 = 0.0;         //front hip perm location
  //float y3 = -4;   //assigned foot location
  float y4;                 //back knee solved
  float y5;                 //front knee solved
  float y6 = 1.25;      //back servo perm location
  float y7 = 1.25;      //front servo perm location
  float y8;                 //rear springtop solved
  float y9;                 //front springtop solved

  float r1 = 3.5; //back thigh length
  float r2 = 3.5; //front thigh length
  //float r3 = 0; //nothing to assign here
  float r4 = 4.0; //backcalf length
  float r5 = 4.0; //frontcalf length
  float r6 = 1.0; //backcrank length
  float r7 = 1.0; //frontcrank length
  float r8 = 4.5; //frontspring length  (kinda fudged)
  float r9 = 4.5; //backspring length  (kinda fudged)
  float sign; //determines is top or bottom solution is chosed for circle intersection




  x4 = XcircleFind(x1, y1, r1, x3, y3, r4, 1); //lower-left joint
  y4 = YcircleFind(x1, y1, r1, x3, y3, r4, 1);

  x5 = XcircleFind(x2, y2, r2, x3, y3, r5, -1); //lower-right joint
  y5 = YcircleFind(x2, y2, r2, x3, y3, r5, -1);

  x8 = XcircleFind(x6, y6, r6, x4, y4, r8, 1); //upper-left joint
  y8 = YcircleFind(x6, y6, r6, x4, y4, r8, 1);

  x9 = XcircleFind(x7, y7, r7, x5, y5, r9, -1); //upper-right joint
  y9 = YcircleFind(x7, y7, r7, x5, y5, r9, -1);

  float thetas1 = angle(x6, y6, x8, y8);
  float thetas2 = angle(x7, y7, x9, y9);

  float backservoangle = 180 * thetas1 / PI - 45;
  float frontservoangle = 180 * thetas2 / PI + 45;

  if (backservoangle < 0)
  {
    backservoangle += 360;
  }
  if (frontservoangle < 0)
  {
    frontservoangle += 360;
  }

  return frontservoangle;
}



//////////////////////////////FUNCTIONS WHICH INTERPRET SERIAL COMMUNICATION 
//Basically boiled all the gaits into this single function. It can be called
//To invoke a left or right turn, a creep gait, a trot gait, and more.

void Move (int gaitSelection = 1, float xamp = 2.3, float tstep=0.03,float yoffset = 4.5, float ylift = 4, int turn = 0){          //1=creep, 2=trot
 //keep xamp below 2-3 maximum amplitude of stride
 //tstep should be between 0.01 and 0.1 
 //yoffset sets the distance between the hips and the ground. y below hip is negative. yoffset is SUBTRACTED from the y coordinate, so it should be a positive number
 //square root it to see how much the foot is lifted off the ground. ylift is ADDED to the y coordinate during the liftoff phase
 
 if (gaitSelection == 0) //trot turning function
 {
   
   
 
 if (turn == -1)//go left
 {
   //t=0
  for (float t = 0; t < 1; t += tstep)
  {
    //front left
    servo1.write(IKBACKSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo2.write(IKFRONTSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    //front right
    servo3.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), -yoffset));
    servo4.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));
    //back right
    servo5.write(IKFRONTSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo6.write(IKBACKSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    //back left
    servo7.write(IKBACKSERVO(0*xamp * (1.0 - 2.0 * t), - yoffset));
    servo8.write(IKFRONTSERVO(0*xamp * (1.0 - 2.0 * t), - yoffset));

  }

  //t=1
  for (float t = 0; t < 1; t += tstep)
  {

    servo1.write(IKBACKSERVO(0*xamp * (1.0 - 2.0 * t), - yoffset));
    servo2.write(IKFRONTSERVO(0*xamp * (1.0 - 2.0 * t), - yoffset));

    servo3.write(IKFRONTSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo4.write(IKBACKSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    servo5.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));
    servo6.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));

    servo7.write(IKBACKSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo8.write(IKFRONTSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));

  }
  //t=2
 }
 
 if (turn == 1)//go right
 {
  
 //t=0
  for (float t = 0; t < 1; t += tstep)
  {
    //front left
    servo1.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo2.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    //front right
    servo3.write(IKFRONTSERVO(0*xamp * (-1.0 + 2.0 * t), -yoffset));
    servo4.write(IKBACKSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset));
    //back right
    servo5.write(IKFRONTSERVO(0*xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo6.write(IKBACKSERVO(0*xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    //back left
    servo7.write(IKBACKSERVO(xamp * (1.0 - 2.0 * t), - yoffset));
    servo8.write(IKFRONTSERVO(xamp * (1.0 - 2.0 * t), - yoffset));

  }

  //t=1
  for (float t = 0; t < 1; t += tstep)
  {

    servo1.write(IKBACKSERVO(xamp * (1.0 - 2.0 * t), - yoffset));
    servo2.write(IKFRONTSERVO(xamp * (1.0 - 2.0 * t), - yoffset));

    servo3.write(IKFRONTSERVO(0*xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo4.write(IKBACKSERVO(0*xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    servo5.write(IKFRONTSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset));
    servo6.write(IKBACKSERVO(0*xamp * (-1.0 + 2.0 * t), - yoffset));

    servo7.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo8.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));

  }
  //t=2
 }
 } 
 
 if (gaitSelection == 1)
 {
   // Creep Gait
   
   //t=0

   for (float t = 0; t < 1; t += tstep)
   {
         //front left
     servo1.write(IKBACKSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
     servo2.write(IKFRONTSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
         //front right
     servo3.write(IKFRONTSERVO(xamp*(-1.0/3.0+2.0/3.0*t),-yoffset));
     servo4.write(IKBACKSERVO(xamp*(-1.0/3.0+2.0/3.0*t),- yoffset));
         //back right
     servo5.write(IKFRONTSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));
     servo6.write(IKBACKSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));
         //back left
     servo7.write(IKBACKSERVO(xamp*(1.0-2.0/3.0*t),- yoffset));
     servo8.write(IKFRONTSERVO(xamp*(1.0-2.0/3.0*t),- yoffset));

   }

   //t=1
     for (float t = 0; t < 1; t += tstep)
   {

     //front left
     servo1.write(IKBACKSERVO(xamp*(1.0-(2.0/3.0)*t),- yoffset));
     servo2.write(IKFRONTSERVO(xamp*(1.0-(2.0/3.0)*t),- yoffset));

     //front right
     servo3.write(IKFRONTSERVO(xamp*(1.0/3.0+2.0/3.0*t),-yoffset));
     servo4.write(IKBACKSERVO(xamp*(1.0/3.0+2.0/3.0*t),- yoffset));

     //back right
     servo5.write(IKFRONTSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
     servo6.write(IKBACKSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));

     //back left
     servo7.write(IKBACKSERVO(xamp*((1.0/3.0)-(2.0/3.0)*t),- yoffset));
     servo8.write(IKFRONTSERVO(xamp*((1.0/3.0)-(2.0/3.0)*t),- yoffset));

   }
   //t=2
     for (float t = 0; t < 1; t += tstep)
   {

     servo1.write(IKBACKSERVO(xamp*(1.0/3.0-2.0/3.0*t),- yoffset));
     servo2.write(IKFRONTSERVO(xamp*(1.0/3.0-2.0/3.0*t),- yoffset));

     servo3.write(IKFRONTSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));
     servo4.write(IKBACKSERVO(xamp*(-(-1.0+2.0*t)),- yoffset +ylift*t-ylift*t*t));

     servo5.write(IKFRONTSERVO(xamp*(-1.0+(2.0/3.0)*t),- yoffset));
     servo6.write(IKBACKSERVO(xamp*(-1.0+(2.0/3.0)*t),- yoffset));

     servo7.write(IKBACKSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));
     servo8.write(IKFRONTSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));

   }
   //t=3
     for (float t = 0; t < 1; t += tstep)
   {

     servo1.write(IKBACKSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));
     servo2.write(IKFRONTSERVO(xamp*(-1.0/3.0-2.0/3.0*t),- yoffset));

     servo3.write(IKFRONTSERVO(xamp*(-1.0+2.0/3.0*t),-yoffset));
     servo4.write(IKBACKSERVO(xamp*(-1.0+2.0/3.0*t),-yoffset));

     servo5.write(IKFRONTSERVO(xamp*(-1.0/3.0+2.0/3.0*t) , - yoffset));
     servo6.write(IKBACKSERVO(xamp*(-1.0/3.0+2.0/3.0*t) , - yoffset));

     servo7.write(IKBACKSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));
     servo8.write(IKFRONTSERVO(xamp*(-1.0+2.0*t),- yoffset +ylift*t-ylift*t*t));

   }
   //t=4
   //back to t=0
 }
  
  if (gaitSelection == 2)
  
  {//trot gait

  //t=0
  for (float t = 0; t < 1; t += tstep)
  {
    //front left
    servo1.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo2.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    //front right
    servo3.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), -yoffset));
    servo4.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));
    //back right
    servo5.write(IKFRONTSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo6.write(IKBACKSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    //back left
    servo7.write(IKBACKSERVO(xamp * (1.0 - 2.0 * t), - yoffset));
    servo8.write(IKFRONTSERVO(xamp * (1.0 - 2.0 * t), - yoffset));

  }

  //t=1
  for (float t = 0; t < 1; t += tstep)
  {

    servo1.write(IKBACKSERVO(xamp * (1.0 - 2.0 * t), - yoffset));
    servo2.write(IKFRONTSERVO(xamp * (1.0 - 2.0 * t), - yoffset));

    servo3.write(IKFRONTSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));
    servo4.write(IKBACKSERVO(xamp * (-(-1.0 + 2.0 * t)), - yoffset + ylift * t - ylift * t * t));

    servo5.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));
    servo6.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset));

    servo7.write(IKBACKSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));
    servo8.write(IKFRONTSERVO(xamp * (-1.0 + 2.0 * t), - yoffset + ylift * t - ylift * t * t));

  }
  //t=2
  }
  
}


//Code that runs when you turn on the arduino
void setup() {
  
  Serial.begin(9600);   
  servo1.attach(30);  // attaches the servo on pin 30 to the servo object
  servo2.attach(31);  // attaches the servo on pin 31 to the servo object
  servo3.attach(32);  // attaches the servo on pin 32 to the servo object
  servo4.attach(33);  // attaches the servo on pin 33 to the servo object
  servo5.attach(34);  // attaches the servo on pin 34 to the servo object
  servo6.attach(35);  // attaches the servo on pin 35 to the servo object
  servo7.attach(36);  // attaches the servo on pin 36 to the servo object
  servo8.attach(37);  // attaches the servo on pin 37 to the servo object

}

//Code that runs as long as the arduino is on

void loop() {

  float tstep = 0.03;
  float xamp = 2.3; //
  float yoffset = 4.5; //coordinate system, y below hip is negative. yoffset is SUBTRACTED from the y coordinate, so it should be a positive number
  float ylift = 4; //how much the foot is lifted off the ground. ylift is ADDED to the y coordinate during the liftoff phase
  
      if(Serial.available() > 0)         //Checks if Searial Data is ready to read
    {state = Serial.read();}         //Sets Variable to Serial Input


          
   
      if (state == 'w')           //standard forwards trot          
    {Move(2, 2.3, 0.03,4.5,4,0);}              
    
      if (state == 's')           //standard backwards trot          
    {Move(2, -2.3, 0.03,4.5,4,0);}               
    
      if (state == 'a')           //trot Left          
    {Move(0, 2.3, 0.03,4.5,4,-1);}               
    
      if (state == 'd')           //trot Right          
    {Move(0, 2.3, 0.03,4.5,4,1);}               
    
    
    //if (state == 'i')
    //{Longboard()}
    
      if (state == 'c')             
    {Move(1, 1.5,0.015,5,16);}     //crawl forward, obsticle avoidance profile

    
      if (state == 'h')                         //If serial input is 2, execute {the code}
    {Move(2, 0.0, 0.5,2,0.0);}              // sets the servo to another position 
      if (state == 'm')                         //If serial input is 2, execute {the code}
    {Move(2, 0.0, 0.5,4.5,0.0);}              // sets the servo to another position 
      if (state == 'l')                         //If serial input is 2, execute {the code}
    {Move(2, 0.0, 0.5,6,0.0);}              // sets the servo to another position 

      
  delay(15);        //Delay is typical of all programs to limit power consumption in ms


}