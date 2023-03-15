#ifndef AUTO_H
#define AUTO_H

#include <cmath>
namespace autonomous {
    double kAutoCommandList[][3]={
        {50, 50, 12},
        // ...
        {0, 0, 0}
    }; 
    double (*auto_ptr)[][3] = &kAutoCommandList;
}
#endif