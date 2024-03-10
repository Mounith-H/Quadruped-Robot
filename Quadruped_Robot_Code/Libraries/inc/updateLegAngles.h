// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// updateLegAngles ()
//
// DESCRIPTION :
// Calculates leg angles to place the foot at the target position and stores these in the LEG struct .
// Does not change existing leg angles if the target position is out of reach or would require exceeding joint limits .
//
// IN:
// LEG struct pointer
// BODY struct pointer
//
// OUT:
// none

/*!
 * @file updateLegAngles.h
 * @brief  Calculates leg angles to place the foot at the target position and stores these in the LEG struct .
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
  // else
  // {
  //   printf("Out of reach\n");
  // }
  //printf("%f", leg -> angles.e1);

}
