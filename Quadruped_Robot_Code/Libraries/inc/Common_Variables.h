
// Set base stance
float stanceWidth = 5.775; //TODO: Set the correct values
float stanceLength = 6.35; //TODO: Set the correct values
float stanceHeight = 9; //TODO: Set the correct values

// These variables describe a leg
float LWF = 12.4;   // Length between the Wrist joint and the foot (Hip)
float LTW = 11.0;   // Length between the Arms joint and the Wrist joint (Thigh)
float LST = 5.5;    // Length between the Shoulder joint and the center line of the Arm (Shin)

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
