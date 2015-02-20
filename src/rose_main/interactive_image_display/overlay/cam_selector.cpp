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
#include "rose_main/interactive_image_display/overlay/cam_selector.hpp"

CamSelector::CamSelector(LRCameraSelector selector)
	:OverlayItem()
	,m_Width( 50 )
    ,m_HoveredOver( false )
	,m_WhichCameraSelector( selector )
{
	setAcceptHoverEvents(true);
}
//---------------------------------------------------------------------------------
//
//
QRectF CamSelector::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void CamSelector::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	if( isEnabled() && isVisible() )
	{
		//painter->setRenderHint(QPainter::Antialiasing);
		//
		QPen pen = painter->pen();
		if( m_HoveredOver )
		{
			pen.setColor( QColor(0, 142, 211) );
		}
		else
		{
			pen.setColor( m_LineColor );
		}
		pen.setWidth( OVERLAY_PEN_WIDTH );
		painter->setPen( pen );
		QFont font("Trebuchet", 5, QFont::Bold);
		painter->setFont(font);
		//

		QBrush brush = painter->brush();
		brush.setColor( m_LineColor );
		brush.setStyle( Qt::NoBrush );
		painter->setBrush( brush );

		QPainterPath path_triangle;

		QPoint p1, p2, p3;
		InitialiseTrianglePoints(p1, p2, p3);

		path_triangle.moveTo( p1 );
		path_triangle.lineTo( p2 );
		path_triangle.lineTo( p3 );
		path_triangle.lineTo( p1 );

		painter->drawPath( path_triangle );

		// tekst
		QString text;
		InitialiseText(text);
		QRectF textArea(0, m_TriangleHeight, m_Width, m_Width-m_TriangleHeight);
		painter->drawText(textArea, Qt::AlignHCenter | Qt::AlignVCenter, text);
	}
}
//---------------------------------------------------------------------------------
//
//
void CamSelector::InitialiseTrianglePoints(QPoint &p1, QPoint &p2, QPoint &p3)
{

	// LeftCamera:              RightCamera:
	//    p1                                   p1
	//    |\                                   /|
	//    |  \                               /  |
	//    |    \                           /    |
	//    |      \                       /      |
	//    |        \                   /        |
	//    |          \               /          |
	//    |          / p2         p2 \          |
    //    |        /                   \        |
    //    |      /                       \      |
    //    |    /                           \    |
    //    |  /                               \  |
    //    |/                                   \|
    //    p3                                   p3
	p1.setY(0);
	p2.setY(m_TriangleHeight/2);
	p3.setY(m_TriangleHeight);
	if (m_WhichCameraSelector == RightCamera)
	{
		p1.setX(m_Width/2-m_TriangleWidth/2);
		p2.setX(m_Width/2+m_TriangleWidth/2);
		p3.setX(m_Width/2-m_TriangleWidth/2);
	}
	else
	{
		p1.setX(m_Width/2+m_TriangleWidth/2);
		p2.setX(m_Width/2-m_TriangleWidth/2);
		p3.setX(m_Width/2+m_TriangleWidth/2);
	}
}
//---------------------------------------------------------------------------------
//

void CamSelector::InitialiseText(QString &text)
{   
	if (m_WhichCameraSelector == RightCamera)
	{
		text = "RIGHT HAND";
	}
	else
	{
		text = "LEFT HAND";
	}
}
//---------------------------------------------------------------------------------
//
//
void CamSelector::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
	if( isEnabled() && isVisible())
	{
		setCursor( QCursor( Qt::PointingHandCursor ) );
		m_HoveredOver = true;
		update();
	}
}
//---------------------------------------------------------------------------------
//
//
void CamSelector::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
	setCursor( QCursor( Qt::ArrowCursor ) );
	m_HoveredOver = false;
	update();
}
//---------------------------------------------------------------------------------
//
//
void CamSelector::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	if( isEnabled() && isVisible())
	{
        std_msgs::String msg;
        std::stringstream ss_left;
        std::stringstream ss_right;
        ros::NodeHandle n;
        left_arm_select = n.advertise<std_msgs::String>("/gui/left_arm_select", 100);
        right_arm_select = n.advertise<std_msgs::String>("/gui/right_arm_select", 100);
		if (m_WhichCameraSelector == RightCamera) {
			//            std::cout<<"!!!!!!!!!Going to publish disable[left]"<<std::endl;
			//            ROS_INFO("%s", msg.data.c_str());
			ss_left << "disable";
			ss_right << "enable";
			msg.data = ss_left.str();
			left_arm_select.publish(msg);
			ROS_INFO("Right is going to be enabled, send first to left arm: %s", msg.data.c_str());
			msg.data = ss_right.str();
			right_arm_select.publish(msg);
			ROS_INFO("Right is going to be enabled, send next to right arm: %s", msg.data.c_str());
		} else {
			ss_left << "enable";
			ss_right << "disable";
			msg.data = ss_right.str();
			right_arm_select.publish(msg);
			ROS_INFO("Left is going to be enabled, send first to right arm: %s", msg.data.c_str());
			msg.data = ss_left.str();
			left_arm_select.publish(msg);
			ROS_INFO("Left is going to be enabled, send next to left arm: %s", msg.data.c_str());
		}
		Q_EMIT Clicked( m_WhichCameraSelector );
		update();
	}
}
//---------------------------------------------------------------------------------
//
//
