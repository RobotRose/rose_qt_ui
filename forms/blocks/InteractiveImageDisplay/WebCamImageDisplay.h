//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * WebCamImageDisplay.h
 *
 *  Created on: Nov 2, 2010
 *      Author: martijn
 */

#ifndef WEBCAMIMAGEDISPLAY_H_
#define WEBCAMIMAGEDISPLAY_H_


class WebCamImageDisplay
{
public:
	virtual void ShowWebCamImage( const std::vector<unsigned char>& data, const int width, const int height ) = 0;
};


#endif /* WEBCAMIMAGEDISPLAY_H_ */
