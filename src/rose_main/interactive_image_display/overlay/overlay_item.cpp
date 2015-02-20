/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/17
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"

OverlayItem::OverlayItem()
	:m_LineColor( Qt::white )
{
        SetEnabled( false );
        setAcceptHoverEvents(true);
}
//---------------------------------------------------------------------------------
//
//
void OverlayItem::SetLineColor( QColor color )
{
	m_LineColor = color;
}
//---------------------------------------------------------------------------------
//
// An overlayitem is a QGraphicsItem. Although this QGraphicsItem
// has a setEnabled, its not declared virtual so it can't be overridden.
// A QGraphicsItem also lacks the even function, so there is no way
// of getting to know if an OverlayItem was enabled or not.
// It is able to get the enabled state (isEnabled()) but there is no way
// to get events of change of this fact. For this reason
// we implemented the SetEnabled (with captial Set) to make this work.
void OverlayItem::SetEnabled( bool enable )
{
	// call Qt enable
	setEnabled( enable );
	//
	update();
}
//---------------------------------------------------------------------------------
//
//
