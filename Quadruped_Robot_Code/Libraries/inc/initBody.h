// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// initBody ()
//
// DESCRIPTION :
// Initializes a BODY struct at given memory adress with robot construction parameters .
//
// IN:
// BODY struct pointer
// float body dimensions ( length and width )
// float leg dimensions (hip -, thigh -, shin - length )
// float minimum allowed angles in degrees (side , hip , knee )
// float maximum allowed angles in degrees (side , hip , knee )
//
// OUT:
// none

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

#define PI 3.14

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
