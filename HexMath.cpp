#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

using namespace std;

const float rotationVectorMultiplier = 1;   // controls how much of the vector addition is dedicated to rotation vs field movement
// 0 < x < float limit idk
const float speedLimit = 1;   // limit motor speed output
// 0 < x <= 1

// mehh just input coords

float sideLength = 19;

float relativeCoordFL[2] = {-sideLength/2,sideLength*(sqrt(3)/2)};
float relativeCoordFR[2] = {sideLength/2,sideLength*(sqrt(3)/2)};
float relativeCoordL[2] = {-sideLength,0};
float relativeCoordR[2] = {sideLength,0};
float relativeCoordBL[2] = {-sideLength/2,-sideLength*(sqrt(3)/2)};
float relativeCoordBR[2] = {sideLength/2,-sideLength*(sqrt(3)/2)};

// controller input
float xLeft = 0; //controller left joystick X
float yLeft = 0; //controller left joystick Y
float xRight = 1; //controller right joystick X (rotation)
float gyro = 0; //robot gyro reading (rad)

float magnitude(float x, float y) // magnitude of vector
{
    return sqrt(x*x + y*y);
}

float findMax(float arr[], int a) // edward was mad at c++ libraries
{
    float sus = max(arr[0],arr[1]);
    for(int i=2; i<a; i++){
        sus = max(sus,arr[i]);
    }
    return sus;
}

int main()
{
    float rotationScalar = rotationVectorMultiplier*xRight/sideLength;
    float posVect[2];
    if(magnitude(xLeft, yLeft)){
        posVect[0] = magnitude(xLeft,yLeft)*sin(atan2(xLeft,yLeft)-gyro);
        posVect[1] = magnitude(xLeft,yLeft)*cos(atan2(xLeft,yLeft)-gyro); // math notation of vector
    }
    else{
        posVect[0] = 0;
        posVect[1] = 0;
    }
    float frontLeft[2] = {(rotationScalar*relativeCoordFL[1])+posVect[0], -(rotationScalar*relativeCoordFL[0])+posVect[1]};
    float frontRight[2] = {(rotationScalar*relativeCoordFR[1])+posVect[0], -(rotationScalar*relativeCoordFR[0])+posVect[1]};
    float left[2] = {(rotationScalar*relativeCoordL[1])+posVect[0], -(rotationScalar*relativeCoordL[0])+posVect[1]};
    float right[2] = {(rotationScalar*relativeCoordR[1])+posVect[0], -(rotationScalar*relativeCoordR[0])+posVect[1]};
    float backLeft[2] = {(rotationScalar*relativeCoordBL[1])+posVect[0], -(rotationScalar*relativeCoordBL[0])+posVect[1]};
    float backRight[2] = {(rotationScalar*relativeCoordBR[1])+posVect[0], -(rotationScalar*relativeCoordBR[0])+posVect[1]}; // declare rotation vector directions and add positional
    
    float physFL[2] = {magnitude(frontLeft[0], frontLeft[1]), atan2(frontLeft[0], frontLeft[1])}; // converts to physics notation
    float physFR[2] = {magnitude(frontRight[0], frontRight[1]), atan2(frontRight[0], frontRight[1])};
    float physL[2] = {magnitude(left[0], left[1]), atan2(left[0], left[1])};
    float physR[2] = {magnitude(right[0], right[1]), atan2(right[0], right[1])};
    float physBL[2] = {magnitude(backLeft[0], backLeft[1]), atan2(backLeft[0], backLeft[1])};
    float physBR[2] = {magnitude(backRight[0], backRight[1]), atan2(backRight[0], backRight[1])};

    float magnitudes[6] = {physFL[0],physFR[0],physBL[0],physBR[0],physL[0],physR[0]};
    float MAX = findMax(magnitudes, sizeof(magnitudes)/sizeof(magnitudes[0]));
    if(MAX>1){
        physFL[0] = physFL[0]*speedLimit/MAX;
        physFR[0] = physFR[0]*speedLimit/MAX;
        physL[0] = physL[0]*speedLimit/MAX; 
        physR[0] = physR[0]*speedLimit/MAX;
        physBL[0] = physBL[0]*speedLimit/MAX;
        physBR[0] = physBR[0]*speedLimit/MAX;
    }

    //*** phys[F/B][R/L] [0] = motor strength; [1] = direction relative to motor front [-pi, pi]

    // TEST IF WORK:
    printf("Front Left Mag %lf\n",physFL[0]);
    printf("Front Right Mag %lf\n",physFR[0]);
    printf("Left Mag %lf\n",physL[0]);
    printf("Right Mag %lf\n",physR[0]);
    printf("Back Left Mag %lf\n",physBL[0]);
    printf("Back Right Mag %lf\n",physBR[0]);
    printf("\n");
    printf("Front Left Arg degree: %lf\n",physFL[1]);
    printf("Front Right Arg degree: %lf\n",physFR[1]);
    printf("Left Arg degree: %lf\n",physL[1]);
    printf("Right Arg degree: %lf\n",physR[1]);
    printf("Back Left Arg degree: %lf\n",physBL[1]);
    printf("Back Right Arg degree: %lf\n",physBR[1]);
}