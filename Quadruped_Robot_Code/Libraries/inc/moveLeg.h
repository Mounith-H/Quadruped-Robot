// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// moveLeg ()
//
// DESCRIPTION :
// Writes microsecond signals to the joint servos to move them to the angles stored in the LEG struct .
// This is done with simple linear interpolation ( note that this assumes that the target angles are inbetween minimum and maximum ).
//
// IN:
// LEG struct pointer
// BODY struct pointer
//
// OUT:
// none

/*!
 * @file moveLeg.h
 * @brief  Writes microsecond signals to the joint servos to move them to the angles stored in the LEG struct .
 * @param leg	Pointer to the LEG struct to be moved.
 * @param body	Pointer to the BODY struct containing the leg's construction parameters.
 * @return	none
 */
void moveLeg (LEG * leg , BODY * body )
{
  // int degreeSide = round(leg -> degreeMin.e1 +(leg -> angles.e1 - body -> sideAngleMin )*(leg -> degreeMax.e1 - leg -> degreeMin.e1)/(body -> sideAngleMax - body -> sideAngleMin));
  // int degreesHip = round(leg -> degreeMin.e2 +(leg -> angles.e2 - body -> hipAngleMin )*(leg -> degreeMax.e2 - leg -> degreeMin.e2)/(body -> hipAngleMax - body -> hipAngleMin));
  // int degreeKnee = round(leg -> degreeMin.e3 +(leg -> angles.e3 - body -> kneeAngleMin )*(leg -> degreeMax.e3 - leg -> degreeMin.e3)/(body -> kneeAngleMax - body -> kneeAngleMin));

  float degreeSideAngle = round(((leg -> angles.e1) / 0.01745329252f) + (leg -> degreeRest.e1));
  float degreeHipAngle = round(((leg -> angles.e2) / 0.01745329252f) + (leg -> degreeRest.e2));
  float degreeKneeAngle = round(((leg -> angles.e3) / 0.01745329252f) + (leg -> degreeRest.e3));

  PCA9685_SetServoAngle(leg -> servoSide.channelSide, degreeSideAngle);
  PCA9685_SetServoAngle(leg -> servoHip.channelHip, degreeHipAngle);
  PCA9685_SetServoAngle(leg -> servoKnee.channelKnee, degreeKneeAngle);

  printf("%d : %d : %d ||",(int)degreeSideAngle, (int)degreeHipAngle, (int)degreeKneeAngle);


}
