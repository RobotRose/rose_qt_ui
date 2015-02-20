//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * CameraDisplay.cpp
 *
 *  Created on: Oct 1, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#include "CameraDisplay.h"
//---------------------------------------------------------------------------------
//
//
using namespace std;
#include <iostream>
//
CameraDisplay::CameraDisplay( unsigned int componentselection, QWidget * parent  )
	: InteractiveImageDisplay( componentselection, parent )
{
//    setStyleSheet("background: transparant");
    //m_Scene.setBackgroundBrush(Qt::transparent);
    SetColorOfLineItems( Qt::white );
}
//---------------------------------------------------------------------------------
//
//
void CameraDisplay::ShowWebCamImage( const std::vector<unsigned char>& data, const int width, const int height )
{
	QRectF rect = sceneRect();
	m_OverlayItems.m_ImageDisplay->ShowImageFromBuffer( data, width, height, rect);
}
//---------------------------------------------------------------------------------
//
//
