//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * SensorLocation.h
 *
 *  Created on: Dec 20, 2010
 *      Author: martijn
 */

#ifndef SENSORLOCATION_H_
#define SENSORLOCATION_H_

#include <vector>

struct SensorLocation
{
	SensorLocation()
	{
		m_PosX = 0;
		m_PosY = 0;
		m_Angle = 0.0;
	};

	SensorLocation( const SensorLocation& other )
	{
		m_PosX = other.m_PosX;
		m_PosY = other.m_PosY;
		m_Angle = other.m_Angle;
	};

	SensorLocation( int posx, int posy, double angle )
	{
		m_PosX = posx;
		m_PosY = posy;
		m_Angle = angle;
	};

	int m_PosX;
	int m_PosY;
	double m_Angle; // radians
};

typedef std::vector<SensorLocation> SensorLocationList;

#endif /* SENSORLOCATION_H_ */
