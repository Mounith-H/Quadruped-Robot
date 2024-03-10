// Mounith H
// 10-03-2024
//
// NAME :
// structs .h
//
// DESCRIPTION :
// Header file holding defenitions for structs used in the project .
//


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
