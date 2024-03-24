#ifndef ROBOT_H_
#define ROBOT_H_

#include "Common_Variables.h"

#define PI 3.14

// Simple vector -3 struct to cleanly handle variable transfer and storage
typedef struct  {
  float e1;
  float e2;
  float e3;
} VEC3 ;

// Struct holding information relevant to the entire robot
typedef struct  {
  float bodyWidth;
  float bodyLength;

  float hipLength;
  float thighLength;
  float shinLength;

  float sideAngleMin;
  float hipAngleMin;
  float kneeAngleMin;

  float sideAngleMax;
  float hipAngleMax;
  float kneeAngleMax;

  float sideAngleRest;
  float hipAngleRest;
  float kneeAngleRest;

  VEC3 bodyTilt;
  VEC3 footRotation;
} BODY;

// Struct holding information relevant to a specific servo
typedef struct {
  int channelSide;
  int channelHip;
  int channelKnee;
} Servo;

// Struct holding information relevant to a specific leg of the robot
typedef struct {
  Servo servoSide;
  Servo servoHip;
  Servo servoKnee;
  VEC3 posOrigin;
  VEC3 posFoot;
  VEC3 coordInverter;
  VEC3 angles;
  VEC3 degreeMin;
  VEC3 degreeMax;
  VEC3 degreeRest;
} LEG;

#define constrain(Val,low,high) ((Val)<(low)?(low):((Val)>(high)?(high):(Val)))

// Declare all body parts
BODY body;
LEG leg1;
LEG leg2;
LEG leg3;
LEG leg4;

// Declare foot target positions to ease variable handling and transfer
VEC3 pos1;
VEC3 pos2;
VEC3 pos3;
VEC3 pos4;

const int MPU_addr=0x68;
uint8_t dataArray[12];


/*!
 * @file initBody.h
 * @brief  Initializes a BODY struct at given memory adress with robot construction parameters.
 * -----------------  (in milimeters and degrees)  -----------------
 * @param body	Pointer to the BODY struct to be initialized.
 * @param bodyWidth	Width of the body.
 * @param bodyLength	Length of the body.
 * @param hipLength	Length of the hip.
 * @param thighLength	Length of the thigh.
 * @param shinLength	Length of the shin.
 *
 * @param sideAngleMin	Minimum allowed angle for the side servo.
 * @param hipAngleMin	Minimum allowed angle for the hip servo.
 * @param kneeAngleMin	Minimum allowed angle for the knee servo.
 *
 * @param sideAngleMax	Maximum allowed angle for the side servo.
 * @param hipAngleMax	Maximum allowed angle for the hip servo.
 * @param kneeAngleMax	Maximum allowed angle for the knee servo.
 * @return	none
 */
void initBody( BODY * body , float bodyWidth , float bodyLength , float hipLength , float thighLength , float shinLength , float sideAngleMin , float hipAngleMin , float kneeAngleMin , float sideAngleMax , float hipAngleMax , float kneeAngleMax, float sideAngleRest, float hipAngleRest, float kneeAngleRest)
{
  body -> bodyWidth = bodyWidth ;
  body -> bodyLength = bodyLength ;
  body -> hipLength = hipLength ;
  body -> thighLength = thighLength ;
  body -> shinLength = shinLength ;
  body -> sideAngleMin = sideAngleMin *PI /180;
  body -> hipAngleMin = hipAngleMin *PI /180;
  body -> kneeAngleMin = kneeAngleMin *PI /180;
  body -> sideAngleMax = sideAngleMax *PI /180;
  body -> hipAngleMax = hipAngleMax *PI /180;
  body -> kneeAngleMax = kneeAngleMax *PI /180;
  body -> bodyTilt .e1 =0;
  body -> bodyTilt .e2 =0;
  body -> bodyTilt .e3 =0;
  body -> footRotation .e1 =0;
  body -> footRotation .e2 =0;
  body -> footRotation .e3 =0;
  body -> sideAngleRest = sideAngleRest;
  body -> hipAngleRest = hipAngleRest;
  body -> kneeAngleRest = kneeAngleRest;
}

/*!
 * @file initLeg.h
 * @brief  Initializes a LEG struct at given memory adress with robot construction parameters , which are partly given from a BODY struct .
 * @param leg	Pointer to the LEG struct to be initialized.
 * @param body	Pointer to the BODY struct containing the leg's construction parameters.
 * @param xInverter	Variable marking legs positioned in different quadrants (x).
 * @param yInverter	Variable marking legs positioned in different quadrants (y).
 * @param zInverter	Variable marking legs positioned in different quadrants (z).
 *
 * @param usSideMin	Microsecond values for the servos corresponding to minimum allowed angle (side).
 * @param usHipMin	Microsecond values for the servos corresponding to minimum allowed angle (hip).
 * @param usKneeMin	Microsecond values for the servos corresponding to minimum allowed angle (knee).
 *
 * @param usSideMax	Microsecond values for the servos corresponding to maximum allowed angle (side).
 * @param usHipMax	Microsecond values for the servos corresponding to maximum allowed angle (hip).
 * @param usKneeMax	Microsecond values for the servos corresponding to maximum allowed angle (knee).
 * @return	none
 */
void initLeg ( LEG * leg , BODY * body , int xInverter ,int yInverter ,int zInverter , float degSideMin , float degHipMin , float degKneeMin , float degSideMax , float degHipMax , float degKneeMax, float degSideRest, float degHipRest, float degKneeRest)
{
  leg -> coordInverter.e1 = xInverter ; // Sets X quadrant sign
  leg -> coordInverter.e2 = yInverter ; // Sets Y quadrant sign
  leg -> coordInverter.e3 = zInverter ; // Sets Z quadrant sign
  leg -> posOrigin.e1 = body -> bodyWidth /2* xInverter ; // fixes the X cordinate of leg origin
  leg -> posOrigin.e2 = body -> bodyLength /2* yInverter ; // fixes the Y cordinate of leg origin
  leg -> posOrigin.e3 = 0; // Z cordinate is 0 since leg posOrigin is in the frame of center of body
  leg -> posFoot.e1 = (body -> bodyWidth /2+ body -> hipLength )* xInverter ; // fixes the X cordinate of leg foot
  leg -> posFoot.e2 = (body -> bodyLength /2) * yInverter ; // fixes the Y cordinate of leg foot
  leg -> posFoot.e3 = -1/ sqrt (2) *( body -> thighLength +body -> shinLength );

  // Initiate angles to " safe " values to not leave the variables empty .
  leg -> angles.e1 = 0;
  leg -> angles.e2 = -PI /4;
  leg -> angles.e3 = PI /2;

  leg -> degreeMin.e1 = degSideMin ;
  leg -> degreeMin.e2 = degHipMin ;
  leg -> degreeMin.e3 = degKneeMin ;

  leg -> degreeMax.e1 = degSideMax ;
  leg -> degreeMax.e2 = degHipMax ;
  leg -> degreeMax.e3 = degKneeMax ;

  leg -> degreeRest.e1 = degSideRest;
  leg -> degreeRest.e2 = degHipRest;
  leg -> degreeRest.e3 = degKneeRest;

  // printLeg (leg );
}

/*!
 * @file updateLegAngles.h
 * @brief  Calculates leg angles to place the foot at the target position and stores these in the LEG struct .
 * @brief  Does not change existing leg angles if the target position is out of reach or would require exceeding joint limits .
 * @param leg	Pointer to the LEG struct to be updated.
 * @param body	Pointer to the BODY struct containing the leg's construction parameters.
 * @return	none
 */
void updateLegAngles (LEG * leg , BODY * body )
{
  // Calculations to get leg angles
  float dx = ( leg -> posFoot.e1 - leg -> posOrigin.e1 ) * leg -> coordInverter.e1;
  float dy = leg -> posFoot.e2 - leg -> posOrigin.e2;
  float dz = - (leg -> posFoot.e3 - leg -> posOrigin.e3);

  float nS = pow(dx ,2) + pow(dz ,2); // delta x² + delta z²
  float mS = nS - pow(body -> hipLength ,2); // delta x² + delta z² - hipLength²
  float m = pow(mS ,0.5) ; // sqrt(delta x² + delta z² - hipLength²)

  float pS = mS + pow(dy ,2);
  float p = pow(pS ,0.5) ;
  float thighLengthS = pow(body -> thighLength ,2);
  float shinLengthS = pow(body -> shinLength ,2);

  float side = -( atan (dx/dz) - atan (body -> hipLength /m));
  float hip = atan (dy/m) - acos ( ( shinLengthS - thighLengthS -pS)/( -2* body -> thighLength *p) );
  float knee = acos ( (pS - thighLengthS - shinLengthS )/( -2* body -> thighLength *body -> shinLength ) );


  // Check to see that calculated values are allowed
  if( isnan(side) == false && isnan(hip) == false && isnan(knee) == false )
  {
    if( side >= body -> sideAngleMin && side <= body -> sideAngleMax )
    {
      if( hip >= body -> hipAngleMin && hip <= body -> hipAngleMax )
      {
        if( knee >= body -> kneeAngleMin && knee <= body -> kneeAngleMax )
        {
        leg -> angles.e1 = side ;
        leg -> angles.e2 = hip ;
        leg -> angles.e3 = knee ;
        }
      }
    }
  }
}

/*!
 * @file moveLeg.h
 * @brief  Writes microsecond signals to the joint servos to move them to the angles stored in the LEG struct .
 * @param leg	Pointer to the LEG struct to be moved.
 * @param body	Pointer to the BODY struct containing the leg's construction parameters.
 * @return	none
 */
void moveLeg (LEG * leg , BODY * body )
{
  float degreeSideAngle = round(((leg -> angles.e1) / 0.01745329252f) + (leg -> degreeRest.e1));
  float degreeHipAngle = round(((leg -> angles.e2) / 0.01745329252f) + (leg -> degreeRest.e2));
  float degreeKneeAngle = round(((leg -> angles.e3) / 0.01745329252f) + (leg -> degreeRest.e3));

  PCA9685_SetServoAngle(leg -> servoSide.channelSide, degreeSideAngle);
  PCA9685_SetServoAngle(leg -> servoHip.channelHip, degreeHipAngle);
  PCA9685_SetServoAngle(leg -> servoKnee.channelKnee, degreeKneeAngle);
}

/*!
 * @file rotatePoint.h
 * @brief  Applies a rotational transform on a foot point to rotate it around the center of the suport base .
 * @param point	VEC3 struct with point to rotate.
 * @param rotation	VEC3 struct with rotational angles in radians.
 * @param stanceHeight	float with the base height of the robot stance.
 * @return	VEC3 struct with rotated point
 */
VEC3 rotatePoint ( VEC3 point , VEC3 rotation , float stanceHeight )
{
  float x= rotation.e1;
  float y= rotation.e2;
  float z= rotation.e3;

  VEC3 out ;

  // Rotate around x>y>z using a rotational " matrix " transform
  out.e1 = point.e1 *( cos(z)*cos(y)) + point .e2 *( cos(z)*sin(y)*sin(x)-sin(z)*cos (x)) + ( point .e3+ stanceHeight )*( cos(z)*sin(y)*cos(x)+sin(z)*sin(x));
  out.e2 = point.e1 *( sin(z)*cos(y)) + point .e2 *( sin(z)*sin(y)*sin(x)+cos(z)*cos (x)) + ( point .e3+ stanceHeight )*( sin(z)*sin(y)*cos(x)-cos(z)*sin(x));
  out.e3 = point.e1 *(- sin(y)) + point .e2 *( cos(y)*sin(x)) + ( point .e3+ stanceHeight )*( cos(y)*cos(x)) - stanceHeight ;
  return ( out );
}

/*!
 * @file updateFootPosition.h
 * @brief  Updates the target position variables in a given leg struct with new positions .
 * @param leg	Pointer to the LEG struct to be updated.
 * @param pos	VEC3 struct with body-centered target position (x,y,z).
 * @return	none
 */
void updateFootPosition (LEG * leg , VEC3 pos ){
  leg -> posFoot.e1 = pos.e1;
  leg -> posFoot.e2 = pos.e2;
  leg -> posFoot.e3 = pos.e3;
}

void robot_init(void)
{
  leg1.servoSide.channelSide = ServoFRTS;
  leg2.servoSide.channelSide = ServoFLTS;
  leg3.servoSide.channelSide = ServoBLTS;
  leg4.servoSide.channelSide = ServoBRTS;

  leg1.servoHip.channelHip = ServoFRT1;
  leg1.servoKnee.channelKnee = ServoFRB2;
  leg2.servoHip.channelHip = ServoFLT1;
  leg2.servoKnee.channelKnee = ServoFLB2;
  leg3.servoHip.channelHip = ServoBLT1;
  leg3.servoKnee.channelKnee = ServoBLB2;
  leg4.servoHip.channelHip = ServoBRT1;
  leg4.servoKnee.channelKnee = ServoBRB2;


  initBody(&body, 6.5, 12.7, 2.525, 6.5, 6.5, -27.5, -60, 45, 30, 60 ,135, 90, 50, 138);
  //initBody(&body, 7.9, 130, LST, LTW, LWF, ShoulderMin, ArmMin, WristMin, ShoulderMax, ArmMax , WristMax); //TODO: Verify values
  initLeg(&leg1, &body, 1, 1, 1, ShoulderMin, ArmMin, WristMin, ShoulderMax, ArmMax, WristMax, ShoulderRest, ArmRest, WristRest); //TODO: Set the correct values
  initLeg(&leg2, &body, -1, 1, 1, ShoulderMin, ArmMin, WristMin, ShoulderMax, ArmMax, WristMax, ShoulderRest, ArmRest, WristRest); //TODO: Set the correct values
  initLeg(&leg3, &body, -1, -1, 1, ShoulderMin, ArmMin, WristMin, ShoulderMax, ArmMax, WristMax, ShoulderRest, ArmRest, WristRest); //TODO: Set the correct values
  initLeg(&leg4, &body, 1, -1, 1, ShoulderMin, ArmMin, WristMin, ShoulderMax, ArmMax, WristMax, ShoulderRest, ArmRest, WristRest); //TODO: Set the correct values

  // Adopt base position for all legs
  pos1.e1 =+ stanceWidth;
  pos1.e2 =+ stanceLength;
  pos1.e3 =- stanceHeight;

  pos2.e1 =-stanceWidth;
  pos2.e2 =+ stanceLength;
  pos2.e3 =- stanceHeight;

  pos3.e1 =-stanceWidth;
  pos3.e2 =- stanceLength;
  pos3.e3 =- stanceHeight;

  pos4.e1 =+ stanceWidth;
  pos4.e2 =- stanceLength;
  pos4.e3 =- stanceHeight;
}

void robot_balance(void)
{
  body.bodyTilt.e1 = sensorData.gx;
  body.bodyTilt.e2 = sensorData.gy;
  body.bodyTilt.e3 = sensorData.gz;

  // Control loop for the foot positions
  body.footRotation.e1 = constrain(body.footRotation.e1+Kp* body.bodyTilt.e1, -xLim /180* PI, xLim /180* PI);
  body.footRotation.e2 = constrain(body.footRotation.e2+Kp* body.bodyTilt.e2, -yLim /180* PI, yLim /180* PI);
  body.footRotation.e3 = constrain(body.footRotation.e3+Kp* body.bodyTilt.e3, -zLim /180* PI, zLim /180* PI);

  // Update foot position variables with rotational transform
  updateFootPosition(&leg1, rotatePoint(pos1, body.footRotation, stanceHeight));
  updateFootPosition(&leg2, rotatePoint(pos2, body.footRotation, stanceHeight));
  updateFootPosition(&leg3, rotatePoint(pos3, body.footRotation, stanceHeight));
  updateFootPosition(&leg4, rotatePoint(pos4, body.footRotation, stanceHeight));

  // Update leg angles to reach the new positions
  updateLegAngles(&leg1, &body);
  updateLegAngles(&leg2, &body);
  updateLegAngles(&leg3, &body);
  updateLegAngles(&leg4, &body);

  // Move the legs to the new positions
  moveLeg(&leg1, &body);
  moveLeg(&leg2, &body);
  moveLeg(&leg3, &body);
  moveLeg(&leg4, &body);
}


#endif /* ROBOT_H_ */
