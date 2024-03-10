#ifndef ROBOT_H_
#define ROBOT_H_

#include "structs.h"
#include "initBody.h"
#include "initLeg.h"
#include "updateFootPosition.h"
#include "updateLegAngles.h"
#include "moveLeg.h"
#include "rotatePoint.h"
#include "Common_Variables.h"


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



void robot_init(void)
{
  leg1.servoSide.channelSide = 0;
  leg1.servoHip.channelHip = 1;
  leg1.servoKnee.channelKnee = 2;

  leg2.servoSide.channelSide = 4;
  leg2.servoHip.channelHip = 5;
  leg2.servoKnee.channelKnee = 6;

  leg3.servoSide.channelSide = 8;
  leg3.servoHip.channelHip = 9;
  leg3.servoKnee.channelKnee = 10;

  leg4.servoSide.channelSide = 12;
  leg4.servoHip.channelHip = 13;
  leg4.servoKnee.channelKnee = 14;


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
    printf("\n");
}

#endif /* ROBOT_H_ */
