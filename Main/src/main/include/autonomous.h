#ifndef AUTO_H
#define AUTO_H

#include <cmath>
namespace autonomous {
    double kAutoCommandList[][3]={
        {0, 160, 180}, //0
        {-10, 220, 180}, //1
        {-10, 220, 180}, //2
        {-10, 220, 180}, //3
        {-10, 220, 180}, //4
        // {70, 190, 0}. // COMMENT OUT UNLESS BALANCING
        {0, 10, 0}, //5
        {43, 0, 0} //6
    }; 
}
#endif