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

#ifndef MAPIMAGEDISPLAY_H_
#define MAPIMAGEDISPLAY_H_


class MapImageDisplay
{
public:
        virtual void ShowMapImage( const QImage& image ) = 0;
};

#endif /* MAPIMAGEDISPLAY_H_ */
