// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// initLeg ()
//
// DESCRIPTION :
// Initializes a LEG struct at given memory adress with robot construction parameters , which are partly given from a BODY struct .
//
// IN:
// LEG struct pointer
// BODY struct pointer
// inverter variables which marks legs positioned in different quadrants (x, y, z)
// microsecond values for the servos corresponding to minimum allowed angle ( side , hip , knee )
// microsecond values for the servos corresponding to maximum allowed angle ( side , hip , knee )
//
// OUT:
// none

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
