#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

//driver for I2C bridge used for touchscreen conversion 
#include "Adafruit_STMPE610.h"

//I2C Address used for STMPE610 bridge module
#define I2C_ADDR 0x27

//MIN and MAX PWM pulse sizes, they can be found in servo documentation
#define MAX 2200
#define MIN 800 //450us operating pulse width

//Positions of servos mounted in opposite direction
#define INV1 1
#define INV2 3
#define INV3 5

//constants for computation of positions of connection points
#define pi  3.14159
#define deg2rad 180/pi
#define deg30 pi/6

// PID set point and servo maximum angles 
#define  PIDANGLE  90
#define  SP_X      2000   //set point for X-coordinate (10" resistive touchscreen)
#define  SP_Y      2500   //set point for Y-coordinate (10" resistive touchscreen)

//Resistive Platform object
Adafruit_STMPE610 touch = Adafruit_STMPE610();

/**************************************************************************/
/*
    Stewart Platform 
    =======================================================================
    https://github.com/ThomasKNR/RotaryStewartPlatform.git
    
    The basis for this project was inspired by the ThomasKNR source code 
    for a mechanically driven Stewart platform which operates on inverse 
    kinematics. This project modifies the code with the intention 
    for an autonomous system independent of user interface for a servo 
    driven input. 
    This was achieved with the use of a resistive touch screen and the 
    STMPE610 bridge used for a closed loop feedback system. 
*/
/**************************************************************************/ 
//Array of servo objects
Servo servo[6];
//Zero positions of servos, in this positions their arms are perfectly horizontal, in us
//static int zero[6]={1475,1470,1490,1480,1460,1490};
static int zero[6]={1500,1500,1500,1500,1500,1500}; //1500 neutral for hs311
//translation range: -.9 (-90%) to .9
//In this array is stored requested position for platform - x,y,z,rot(x),rot(y),rot(z)
static float arr[6]={0.0,0.0,0.0, radians(0),radians(0),radians(0)};

//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6]={0.0,0.0,0.0, 0.0,0.0,0.0};
//Array of current servo positions in us
static int servo_pos[6];
//rotation of servo arms in respect to axis x
const float beta[] = {pi/2,-pi/2,-pi/6, 5*pi/6,-5*pi/6,pi/6},
//maximum servo positions, 0 is horizontal position
      servo_min=radians(-80),servo_max=radians(80),
      
//servo_mult - multiplier used for conversion radians->servo pulse in us
//L1-effective length of servo arm(in.), L2 - length of base and platform connecting rod arm(in.)
//z_home - height of platform above base, 0 is height of servo arms
      servo_mult=400/(pi/4),L1 = 0.748 ,L2 = 6.57, z_home = 5.63;

//PD distance from center of platform to attachment points (arm attachment point)
//RD distance from center of base to center of servo rotation points (servo axis)
//theta_p-angle between two servo axis points, theta_r - between platform attachment points
//theta_angle-helper variable
//p[][]=x y values for servo rotation points
//re[]{}=x y z values of platform attachment points positions
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position
const float RD = 2.42,PD =2.99,theta_p = radians(37.5),
theta_angle=(pi/3-theta_p)/2, theta_r = radians(8),
      p[2][6]={
          {
            -PD*cos(deg30-theta_angle),-PD*cos(deg30-theta_angle),
            PD*sin(theta_angle),PD*cos(deg30+theta_angle),
            PD*cos(deg30+theta_angle),PD*sin(theta_angle)
         },
         {
            -PD*sin(deg30-theta_angle),PD*sin(deg30-theta_angle),
            PD*cos(theta_angle),PD*sin(deg30+theta_angle),
            -PD*sin(deg30+theta_angle),-PD*cos(theta_angle)
         }
      },
      re[3][6] = {
          {
              -RD*sin(deg30+theta_r/2),-RD*sin(deg30+theta_r/2),
              -RD*sin(deg30-theta_r/2),RD*cos(theta_r/2),
              RD*cos(theta_r/2),-RD*sin(deg30-theta_r/2),
          },{
              -RD*cos(deg30+theta_r/2),RD*cos(deg30+theta_r/2),
              RD*cos(deg30-theta_r/2),RD*sin(theta_r/2),
              -RD*sin(theta_r/2),-RD*cos(deg30-theta_r/2),
          },{
              0,0,0,0,0,0
          }
};
//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move
static float M[3][3], rxp[3][6], T[3], H[3] = {0,0,z_home};

void setup(){

//attachment of servos to PWM digital pins of arduino
   servo[0].attach(3, MIN, MAX);
   servo[1].attach(5, MIN, MAX);
   servo[2].attach(6, MIN, MAX);
   servo[3].attach(9, MIN, MAX);
   servo[4].attach(10, MIN, MAX);
   servo[5].attach(11, MIN, MAX);
//begin of serial communication
   Serial.begin(9600);
//putting into base position
   setPos(arr);
}

//function calculating needed servo rotation value
float getAlpha(int *i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[*i];
   while(n<20){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = L1*cos(th)*cos(beta[*i]) + p[0][*i];
      q[1] = L1*cos(th)*sin(beta[*i]) + p[1][*i];
      q[2] = L1*sin(th);
    //calculation of distance between according platform attachment point and base attachment point
      dl[0] = rxp[0][*i] - q[0];
      dl[1] = rxp[1][*i] - q[1];
      dl[2] = rxp[2][*i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);
    //if this distance is the same as leg length, value of theta_a is corrent, we return it
      if(abs(L2-dl2)<0.01){
         return th;
      }
    //if not, we split the searched space in half, then try next value
      if(dl2<L2){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
}

//function calculating rotation matrix
void getmatrix(float pe[])
{
   float psi=pe[5];
   float theta=pe[4];
   float phi=pe[3];
   M[0][0] = cos(psi)*cos(theta);
   M[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   M[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);

   M[0][1] = sin(psi)*cos(theta);
   M[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   M[2][1] = cos(theta)*sin(phi);

   M[0][2] = -sin(theta);
   M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
   M[2][2] = cos(theta)*cos(phi);
}
//calculates wanted position of platform attachment poins using calculated rotation matrix
//and translation vector
/*Note: 
  Gets odd error if computed within a single loop, must be broken down into 
  3 separate loops to avoid possible register overflow?
*/
void getrxp()
{  
   for(int i=0;i<6;i++)
      rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i]);
   
   for(int i=0;i<6;i++)
      rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i]);
      
   for(int i=0;i<6;i++)
      rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i]);   
   
}
//function calculating translation vector - desired move vector + home translation vector
void getT(float pe[])
{
   T[0] = pe[0]+H[0];
   T[1] = pe[1]+H[1];
   T[2] = pe[2]+H[2];
}

unsigned char setPos(float pe[]){
    unsigned char errorcount;
    errorcount=0;
    for(int i = 0; i < 6; i++)
    {
        getT(pe);
        getmatrix(pe);
        getrxp();
        theta_a[i]=getAlpha(&i);
        if(i==INV1||i==INV2||i==INV3){
            servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, MIN,MAX);
        }
        else{
            servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, MIN,MAX);
        }
    }

    for(int i = 0; i < 6; i++)
    {
        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MIN||servo_pos[i]==MAX){
            errorcount++;
        }
        servo[i].writeMicroseconds(servo_pos[i]);
    }
    return errorcount;
}


void loop()
{
     Serial.write("Welcome"); 
     //Integral vars
     int IGRL_N = 3;     
     
     float  i_xerr, i_yerr;
     float igrlx[5] = {0};
     float igrly[5] = {0}; 

     //Derivative vars
     float x_old, y_old; 
     float xerr, yerr, xerr_old, yerr_old;
     float d_xerr, d_yerr, d_xerr2, d_yerr2, v_x, v_y, v_xerr, v_yerr;

     //Proportional vars    
     float p_xerr, p_yerr, x_radians, y_radians;                     
     int angle, th; 
     uint16_t x,y;
     uint8_t z;
     
     float sp_x = SP_X; 
     float sp_y = SP_y; 

     //shape perimeters
     int shape; 
     int liney[2] = {750, 3500}; 
     int trix[3]  = {750, 750, 3500}; 
     int triy[3]  = {750,3500, 2125};      
     int btinput, inputx, inputy;  
     
     // Initialize error values
     xerr = 0; 
     yerr = 0;
     x_old = 0; 
     y_old = 0;  
     xerr_old = 0; 
     yerr_old = 0; 
     d_xerr2 = 0; 
     d_yerr2 = 0;
     i_xerr = 0; 
     i_yerr = 0;
     memset(igrlx, 0, IGRL_N); 
     memset(igrly, 0, IGRL_N); 
     
     if (! touch.begin()) {
        Serial.println("STMPE not found!");
        while(1);
     }

     
     Serial.println("Enter S to change set point");
     while(1){
      
/**************************************************************************/
/*
    User Interface 
    =======================================================================
    Allows a user to manipulate the set point of the platform using 
    a phone app with blue tooth capabilities. (Blueterm was used for testing)
    The set point is manipulated by receiving a value from 1-9. The ASCII 
    value is converted to its numerical value to represent a percentage. The 
    percentage is used to determine the new set point by multiplying it by the 
    maximum values possible on the platform. 
    If no input is recieved, the default setpoint is set at the middle of the
    platform. 
*/
/**************************************************************************/    
       if(Serial.available()>=1) {
          btinput = Serial.read(); 
          switch(btinput) { 
            case 's': 
                  
                 //Configure set point 
                 Serial.println("Enter set point (1 - 9) : "); 
                  while(Serial.available() <=2) {};  //Wait for 2 inputs from user
                  
                  inputx = Serial.read(); 
                  inputy = Serial.read(); 
                  if(inputx >48 && inputx<58) {
                    sp_x = 200+3500*(inputx-48)/10;
                    Serial.write(inputx); 
                  }
                  else {
                    sp_x = SP_X; 
                    Serial.write('*'); 
                  }
                  if(inputy >48 && inputy<58) {
                    sp_y = 200+3500*(inputy-48)/10;
                    Serial.write(inputy); 
                  }
                  else{
                    sp_y = SP_y; 
                    Serial.write('*'); 
                  } 

            break; 
            
            default: 
                  sp_x = SP_X; 
                  sp_y = SP_y;
            break;
          }                             
       }
       
     

/**************************************************************************/
/*
    PID Manipulation    
    =======================================================================
    Proportional: A linear relationship representing the error between 
                  the Set point and feedback variable. 
                  
    Derivative: The rate of change between the proportional error and the 
                velocity of the feedback variable. The Derivative factor 
                is calculated by taking the difference between the past 
                state location and the current state location. Velocity can 
                be determined by taking the difference in the rate of change 
                of the error. 

    Integral: The integral is the sum of all errors for up to N samples collected. 
*/
/**************************************************************************/
        touch.readData(&x, &y, &z);
        
        //Proportional 
        p_xerr = (float)x - sp_x ; 
        p_yerr = (float)y - sp_y ; 
        
       //Velocity 
        v_x = ((float)x - x_old); 
        v_y = ((float)y - y_old); 
        x_old = (float)x; 
        y_old = (float)y; 
        v_x = abs(v_x);
        v_y = abs(v_y);
        //Derivative 
        d_xerr = (p_xerr - xerr_old); 
        d_yerr = (p_yerr - yerr_old); 
        xerr_old = p_xerr;
        yerr_old = p_yerr;
        
        v_xerr = d_xerr - d_xerr2; 
        v_yerr = d_yerr - d_yerr2; 
        d_xerr2 = d_xerr;
        d_yerr2 = d_yerr;    
        
        //Integral               
        igrlx[IGRL_N-1] = p_xerr; 
        igrly[IGRL_N-1] = p_yerr; 
        
        for(int i=0; i<IGRL_N; i++) { 
          i_xerr += igrlx[i]; 
          i_yerr += igrly[i];
           
          if(i>0) {
            igrlx[i-1] = igrlx[i]; 
            igrly[i-1] = igrly[i];
          }
        }         
        
/**************************************************************************/
/*
    Process Plant   
    =======================================================================
    Each factor is accumulated to determine a final error in the system. 
    The constant values used are referred to as tuning parameters and can be 
    altered based on your preference for response time of the system. According to what 
    I found, the Proportional parameter will give the system a larger response based on 
    location of the ball. It should be the largest of all parameters in order to 
    observe a significant response of the platform. 
    
    The derivative parameter will respond based upon the speed of the ball. If it is too
    large the platform will overshoot, especially toward the center as the error 
    gets smaller. If it is too small, the platform will not respond dramatic enough as 
    the ball approaches the edge. 
    
    The integral parameter will smooth out overshoot however it retains past errors
    which can be irrelevant in this time critical system. It can be neglected by
    setting it to zero. 
    
    x/y radian: The x/y radian variable will provide a magnitude for the overall 
                angle required for the system. The maximum error possible for the 
                platform varies on dimensional parameters; for the 10" used there can 
                only be a max ERROR of 1350 found by testing.
                By using the PID computed error, a percentage can be determined 
                for the required angle adjustment. 
*/
/**************************************************************************/
        xerr = .015*p_xerr + .000001*i_xerr + .0035*d_xerr*v_x; 
        yerr = .015*p_yerr + .000001*i_yerr + .0035*d_yerr*v_y; 
       
        x_radians = -(xerr/1350)*(PIDANGLE); 
        y_radians = (yerr/1350)*(PIDANGLE);
         
        //Set angles                              
        arr[3] = radians(x_radians);
        arr[4] = radians(y_radians);
        setPos(arr);

     }       
        
}
