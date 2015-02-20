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
#include "rose_main/interactive_image_display/overlay/cam_indicator.hpp"

CamIndicator::CamIndicator()
	:OverlayItem()
	,m_Width( 20 )
    ,m_CameraSelected( LeftCamera )
{
};
//---------------------------------------------------------------------------------
//
//
QRectF CamIndicator::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void CamIndicator::SetCurrentCamera(LRCameraSelector camera)
{
	m_CameraSelected = camera;
	update();
}
//---------------------------------------------------------------------------------
//
//
void CamIndicator::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );
	QFont font("Trebuchet", 10);
	painter->setFont(font);

	QString text;
	if (m_CameraSelected == LeftCamera)
	{
		text = "L";
	}
	else
	{
		text = "R";
	}
	QRectF textArea(0, 0, m_Width, m_Width);
	painter->drawText(textArea, Qt::AlignHCenter | Qt::AlignVCenter, text);
}
