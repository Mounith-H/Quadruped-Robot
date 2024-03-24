// Set the pin numbers for the servos
#define ServoFRT1 0 // Front Right Leg Top
#define ServoFRB2 1 // Front Right Leg Bottom
#define ServoFLT1 2 // Front Left Leg Top
#define ServoFLB2 3 // Front Left Leg Bottom
#define ServoBLT1 4 // Back Left Leg Top
#define ServoBLB2 5 // Back Left Leg Bottom
#define ServoBRT1 6 // Back Right Leg Top
#define ServoBRB2 7 // Back Right Leg Bottom


// Set the pin numbers for the servos on Sholder // TODO : For future use correnty not used
#define ServoFRTS 8 // Front Left Leg Bottom
#define ServoFLTS 9 // Front Left Leg Top
#define ServoBLTS 10 // Front Right Leg Bottom
#define ServoBRTS 11 // Front Right Leg Top

// Set base stance
float stanceWidth = 5.775; //TODO: Set the correct values
float stanceLength = 6.35; //TODO: Set the correct values
float stanceHeight = 9; //TODO: Set the correct values

// These variables describe a leg
float upperLegLength = 11;   // Length between the Arms joint and the Wrist joint (Thigh)
float lowerLegLength = 13;   // Length between the Wrist joint and the foot (shin)
float sholderLength = 5.5;    // Length between the Shoulder joint and the center line of the Arm (hip)

float ShoulderMin = 50.0;    // Minimum angle for the shoulder joint (Side)
float ShoulderMax = 130.0;   // Maximum angle for the shoulder joint (Side)

float ArmMin = 15.0;         // Minimum angle for the arm joint (Hip)
float ArmMax = 165.0;        // Maximum angle for the arm joint (Hip)

float WristMin = 50.0;       // Minimum angle for the wrist joint (Knee)
float WristMax = 180.0;      // Maximum angle for the wrist joint (Knee)

float ShoulderRest = 90.0;  // Resting angle for the shoulder joint (Side)
float ArmRest = 138.0;       // Resting angle for the arm joint (Hip)
float WristRest = 50.0;      // Resting angle for the wrist joint (Knee)

// Declare control loop parameters
float Kp = 0.05;
float xLim = 30;
float yLim = 30;
float zLim = 45;
