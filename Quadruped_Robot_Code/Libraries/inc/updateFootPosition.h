// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// updateFootPosition ()
//
// DESCRIPTION :
// Updates the target position variables in a given leg struct with new positions .
//
// IN:
// LEG struct pointer
// VEC3 struct with body - centered target position (x,y,z)
//
// OUT:
// none

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
