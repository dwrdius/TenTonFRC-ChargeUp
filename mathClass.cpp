#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

using namespace std;

const double rotationVectorMultiplier = 1;   // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
const double speedLimit = 1;   // limit motor speed output   0 < x <= 1
const double baseWidth = 14.5;   // inches
const double baseLength = 23.25;

// For determining motor positions on an x,y grid
double relativeX = baseWidth / 2;
double relativeY = baseLength / 2;

double xLeft = -0.69; //controller left joystick X
double yLeft = 0.96; //controller left joystick Y
double xRight = -0.420; //controller right joystick X (rotation)
double gyro = 3.14; //robot gyro reading (rad)

//CANCoder readings for old angles
double FLTheta = 0;
double FRTheta = 0;
double BLTheta = 0;
double BRTheta = 0; 

double magnitude(double x, double y) // magnitude of vector
{
    return sqrt(x*x + y*y);
}

int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle-initialAngle, M_PI*2));
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        return -1;
    }
    return 1;
}
double angleOptimisation(double initialAngle, double finalAngle)
{
    double diff = fmod(finalAngle-initialAngle, M_PI*2);
    double posDiff = fabs(diff);
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        diff = fmod(diff + M_PI, M_PI*2);
    }
    posDiff = fabs(diff);
    
    if(posDiff<=M_PI_2){
        return diff;
    }
    else if(diff>=3*M_PI_2){
        return diff-(2*M_PI);
    }
    else{
        return diff+(2*M_PI);
    }
}

double findMax(double arr[], int a) // finds max value in array
{
    double sus = max(arr[0],arr[1]);
    for(int i=2; i<a; i++){
        sus = max(sus,arr[i]);
    }
    return sus;
}

int getDegree(double x)
{
    double a = x*180/M_PI;
    int b = static_cast<int>(a);
    return b;
}

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double rotationScalar = rotationVectorMultiplier*xRight/magnitude(relativeX,relativeY);
    double posVect[2];
    if(magnitude(xLeft, yLeft)){
        posVect[0] = magnitude(xLeft,yLeft)*sin(atan2(xLeft,yLeft)-gyro);
        posVect[1] = magnitude(xLeft,yLeft)*cos(atan2(xLeft,yLeft)-gyro); // math notation of vector
    }
    else{
        if(rotationScalar){
            return {{0,0}, {0,0}, {0,0}, {0,0}};
        }
        posVect[0] = 0;
        posVect[1] = 0;
    }
    double frontLeft[2] = {(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double frontRight[2] = {(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]};
    double backLeft[2] = {-(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double backRight[2] = {-(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]}; // declare rotation vector directions and add positional
    // the most sketchy thing ever.
    double magnitudes[4] = {magnitude(frontLeft[0], frontLeft[1]), magnitude(frontRight[0], frontRight[1]), magnitude(backLeft[0], backLeft[1]), magnitude(backRight[0], backRight[1])};
    double limitingScalar = speedLimit/max(1.0, findMax(magnitudes, sizeof(magnitudes)/sizeof(magnitudes[0])));

    double physFL[2] = {limitingScalar*magnitudes[0], atan2(frontLeft[0], frontLeft[1])}; // converts to physics notation
    double physFR[2] = {limitingScalar*magnitudes[1], atan2(frontRight[0], frontRight[1])};
    double physBL[2] = {limitingScalar*magnitudes[2], atan2(backLeft[0], backLeft[1])};
    double physBR[2] = {limitingScalar*magnitudes[3], atan2(backRight[0], backRight[1])};
    //-----
    double FL[2] = {magnitudeOptimization(FLTheta, physFL[1])*physFL[0], angleOptimisation(FLTheta, physFL[1])};
    double FR[2] = {magnitudeOptimization(FRTheta, physFR[1])*physFR[0], angleOptimisation(FRTheta, physFR[1])};
    double BL[2] = {magnitudeOptimization(BLTheta, physBL[1])*physBL[0], angleOptimisation(BLTheta, physBL[1])};    
    double BR[2] = {magnitudeOptimization(BRTheta, physBR[1])*physBR[0], angleOptimisation(BRTheta, physBR[1])};

    //*** [][0] = [Module][motor strength]; [][1] = [Module][angle to turn]
    //[Module] order: [0,1,2,3] = [FL,FR,BL,BR]
    double *swerveNumbers[4] = {FL, FR, BL, BR};
    // TEST IF WORK:

    printf("Front Left Mag %lf\n",swerveNumbers[0][0]);
    printf("Front Right Mag %lf\n",swerveNumbers[1][0]);
    printf("Back Left Mag %lf\n",swerveNumbers[2][0]);
    printf("Back Right Mag %lf\n",swerveNumbers[3][0]);
    printf("\n");
    printf("Front Left Arg degree: %d\n",getDegree(swerveNumbers[0][1]));
    printf("Front Right Arg degree: %d\n",getDegree(swerveNumbers[1][1]));
    printf("Back Left Arg degree: %d\n",getDegree(swerveNumbers[2][1]));
    printf("Back Right Arg degree: %d\n",getDegree(swerveNumbers[3][1]));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
}