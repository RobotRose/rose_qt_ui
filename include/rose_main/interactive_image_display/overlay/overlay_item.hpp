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
#ifndef OVERLAYITEM_HPP
#define OVERLAYITEM_HPP
//
#include <QtGui/QGraphicsObject>
#include <QtGui/QColor>
//
class OverlayItem : public QGraphicsObject
{
public:
	OverlayItem();
	//
	virtual void SetLineColor( QColor color );
	virtual void SetEnabled( bool enable );
	//
protected:
	//
	QColor m_LineColor;
	//
};
//
#endif /* OVERLAYITEM_HPP */
