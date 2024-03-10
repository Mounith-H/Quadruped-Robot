// Algot Lindestam , David Lorang
// 03/05 -2021
//
// NAME :
// rotatePoint ()
//
// DESCRIPTION :
// Applies a rotational transform on a foot point to rotate it around the center of the suport base .
// ---------------------------------  Rotation matrix ---------------------------------
//
// IN:
// VEC3 struct with point to rotate
// VEC3 struct with rotational angles in radians
// float with the base height of the robot stance
//
// OUT:
// VEC3 struct with rotated point

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
