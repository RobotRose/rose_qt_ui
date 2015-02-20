/*
 * UnitConverter.h
 *
 *  Created on: Nov 22, 2010
 *      Author: freddy
 */

#ifndef UNITCONVERTER_H_
#define UNITCONVERTER_H_

#include <math.h>

class UnitConverter
{
public:
    UnitConverter()
    {
    }
    ~UnitConverter()
    {
    }
    
    static double DegreesToRadians(double degrees)
    {
        return degrees * M_PI / 180;
    }
    static double RadiansToDegrees(double radians)
    {
        return radians * 180 / M_PI;
    }
};

#endif // UNITCONVERTER_H_
