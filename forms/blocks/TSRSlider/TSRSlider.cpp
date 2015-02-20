//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#include <QtGui/QColor>
#include <QtGui/QImage>
#include <QtGui/QRadialGradient>
#include <QtGui/QPainter>
// 2011/04/05    nv this seems tobe Qt3 #include <QtCore/QCustomEvent>
#include <QtGui/QApplication>
#include <QtCore/QRect>
#include <iostream>
#include <QtGui/QMouseEvent>
//------------------------------------------------------------------------------
//
//
#include "TSRSlider.h"
//---------------------------------------------------------------------------------
//
//
TSRSlider::TSRSlider( QWidget *parent, SliderSlave::Ptr sliderslave  )
	:QWidget ( parent )
	,Image( NULL )
    ,m_Painter( NULL )
    ,m_HooveredOver( false )
    ,m_DraggingSlider( false )
    ,m_SliderSlave( sliderslave )
    ,m_SliderMode( Slider::smWillReiveUpdates )
    ,m_TicksEnable( true )
{
	m_MaximumValue = 100;  // %
	m_MinimumValue = 0;  // %
	m_CurrentValue = 25; // %
	m_UserSetpoint = 25; // %
	//
	ReCalcConstants();
	//
	UpdateSliderRectWithMidSliderPos( CalcMidSliderPosFromUserValue( m_UserSetpoint ) );
	//
	ReConfigureImage( 400, 80 );
	RedrawOnImage();

    //! @todo MichielVanOsch: is this syncing-behavior really the right way?
	// sync real world with initial value of setpoint as initialized in gui component (aka, here.)
    m_SliderSlave->NewSetpointAvailable( m_UserSetpoint);

	update();

	setMouseTracking( true );
}
//------------------------------------------------------------------------------
//
//
TSRSlider::~TSRSlider()
{
	if( m_Painter )
	{
		delete m_Painter;
		m_Painter = NULL;
	}
	if( Image )
	{
		delete Image;
		Image = NULL;
	}
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::SetCurrentValue( int value )
{
	m_CurrentValue = value;

    UpdateSliderRectWithMidSliderPos(CalcMidSliderPosFromUserValue(m_CurrentValue));

	RedrawOnImage();
	update();
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::SetSliderMode( SliderMode mode )
{
	m_SliderMode = mode;
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::SetTickEnable( bool enable )
{
	m_TicksEnable = enable;
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::resizeEvent( QResizeEvent *event )
{
	Q_UNUSED(event);
	ReConfigureImage( width(), height() );
	ReCalcConstants();
	UpdateSliderRectWithMidSliderPos( CalcMidSliderPosFromUserValue( m_UserSetpoint ) );
	RedrawOnImage();
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::RedrawOnImage()
{
	m_mutex.lock();

	// Redraw
	if( !m_Painter )
		m_Painter = new QPainter();
	//
	Image->fill( Qt::transparent );
	//
	ReCalcConstants();
	//
	FillSliderRectangle();
	DrawProgress();
	DrawSliderRectangle();
	//
	if( m_TicksEnable )
		DrawTicks();
	//
	DrawSlider();
	//
	m_mutex.unlock();
	update();
}
//------------------------------------------------------------------------------
//
void TSRSlider::ReCalcConstants()
{
	m_OutherLineColor = Qt::black;
	m_SliderColor = Qt::gray;

	m_OutherLineWidth = 2;

	// From within the total area we have available, we keep some space from the edges...
	// These values define this space.
	m_LeftMargin = 3;
	m_rightMargin = 3;
	m_TopMargin = 3;
	m_BtmMargin = 3;

	// Define the radius of the rounded corners of the main rectangle.
	// This will also become a base line for the width of the slider.
	m_Radius = 4;
	m_SliderWidth = (2*m_Radius)+1;

	// See what space we have left (total available area - margins left, right, top, down.
	m_WorkingWidth = width() - m_LeftMargin - m_rightMargin;
	m_WorkingHeight = height() - m_TopMargin- m_BtmMargin;

	// Define the main rectangle.
	m_MainRect = QRect( m_LeftMargin, m_TopMargin, m_WorkingWidth, m_WorkingHeight );

	// Calculate how far the progress bar should be filled based on the currentvalue.
	double range = m_MaximumValue - m_MinimumValue;
	double scale = m_CurrentValue / range;
	int progresswidth = ( m_MinPos + ( scale * ( m_MaxPos - m_MinPos ) ) );
	m_ProgressRect = QRect( m_LeftMargin, m_TopMargin, progresswidth, m_WorkingHeight );

	// Define the positions the slider should always be in between of.
	m_MinPos = m_LeftMargin + ceil(m_SliderWidth/2.0);
	m_MaxPos = width() - m_rightMargin - ceil(m_SliderWidth/2.0);
}
//------------------------------------------------------------------------------
//
int TSRSlider::CalcMidSliderPosFromUserValue( int value )
{
	double range = m_MaximumValue - m_MinimumValue;
	double scale = value / range;

	return ( m_MinPos + ( scale * ( m_MaxPos - m_MinPos ) ) );
}
//------------------------------------------------------------------------------
//
void TSRSlider::UpdateSliderRectWithMidSliderPos( int midsliderpos )
{
	m_SliderRect = QRect( midsliderpos - (m_SliderWidth/2.0), m_TopMargin, m_SliderWidth, m_WorkingHeight );
}
//------------------------------------------------------------------------------
//
int TSRSlider::CalcUserValueFromMidSliderPosition( int midsliderposition )
{
	if( midsliderposition > m_MaxPos )
		midsliderposition = m_MaxPos;
	if( midsliderposition < m_MinPos )
		midsliderposition = m_MinPos;
	double range = m_MaxPos - m_MinPos;
	double scale = (midsliderposition - m_MinPos) / range;

	int user = ( m_MinimumValue + ( scale * ( m_MaximumValue - m_MinimumValue ) ) );

	return user;
}
//------------------------------------------------------------------------------
//
void TSRSlider::DrawProgress()
{
	m_Painter->begin( Image );
	//
	QPainterPath rounded_rect_path;
	rounded_rect_path.addRect( m_ProgressRect );
	//
	if( isEnabled () )
		m_Painter->fillPath( rounded_rect_path , QBrush( QColor(0, 142, 210 )) );
	else
		m_Painter->fillPath( rounded_rect_path , QBrush( Qt::gray ) );
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void TSRSlider::FillSliderRectangle()
{
	m_Painter->begin( Image );
	//
	m_Painter->setRenderHint(QPainter::Antialiasing);
	QPen pen;
	pen.setWidth( m_OutherLineWidth );
	pen.setColor( m_OutherLineColor );
	//
	m_Painter->setPen( pen );
	//
	QPainterPath rounded_rect_path;
	rounded_rect_path.addRect( m_MainRect );

	m_Painter->fillPath( rounded_rect_path , QBrush( Qt::white ) );
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void TSRSlider::DrawSliderRectangle()
{
	m_Painter->begin( Image );
	//
	m_Painter->setRenderHint(QPainter::Antialiasing);
	QPen pen;
	pen.setWidth( m_OutherLineWidth );
	pen.setColor( m_OutherLineColor );
	//
	m_Painter->setPen( pen );
	//
	m_Painter->drawRoundedRect( m_MainRect , m_Radius, m_Radius );
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void TSRSlider::DrawTicks()
{
	m_Painter->begin( Image );
	//
	QPen pen;

	// Draw the 10% ticks.
	// These ticks have 2 parts, one black on top, and one part at the bottom in gray.

	// This loop, draws the upper parts of the 10% ticks.
	pen.setWidth( 2 );
	pen.setColor( m_OutherLineColor );
	m_Painter->setPen( pen );
	for( unsigned int i = 0; i < 11; i++ )
	{
		int pos = m_MinPos + (((i*10)/100.0)*(m_MaxPos - m_MinPos));
		m_Painter->drawLine( pos, m_TopMargin, pos,  m_TopMargin+( m_WorkingHeight/2 ) );
	}

	// This loop, draws the lower parts of the 10% ticks.
	pen.setWidth( 1 );
	pen.setColor( Qt::gray );
	m_Painter->setPen( pen );
	for( unsigned int i = 0; i < 11; i++ )
	{
		int pos = m_MinPos + (((i*10)/100.0)*(m_MaxPos - m_MinPos));
		m_Painter->drawLine( pos, m_TopMargin+( m_WorkingHeight/2 ), pos,  m_TopMargin + m_WorkingHeight - m_OutherLineWidth);
	}

	// This loop, draws smaller 5% ticks.
	pen.setWidth( 1 );
	pen.setColor( Qt::black );
	m_Painter->setPen( pen );
	for( unsigned int i = 0; i < 20; i++ )
	{
		int pos = m_MinPos + (((i*5)/100.0)*(m_MaxPos - m_MinPos));
		m_Painter->drawLine( pos, m_TopMargin, pos, m_TopMargin+( m_WorkingHeight/4 )  );
	}
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void TSRSlider::DrawSlider()
{
	m_Painter->begin( Image );
	//
	m_Painter->setRenderHint(QPainter::Antialiasing);

	QPen pen;
	pen.setWidth( m_OutherLineWidth );
	pen.setColor( m_OutherLineColor );
	//
	QBrush brush( m_SliderColor );
	m_Painter->setBrush( brush );
	m_Painter->setPen( pen );
	//
	m_Painter->drawRoundedRect( m_SliderRect, m_Radius, m_Radius);

	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void TSRSlider::ReConfigureImage( int width, int height )
{
	m_mutex.lock();
	// Clean up previous image.
	if( Image )
		delete Image;
	Image = new QImage( width, height, QImage::Format_ARGB32 );
	// Redraw
	if( !m_Painter )
		m_Painter = new QPainter();
	//
	Image->fill( Qt::transparent );
	m_mutex.unlock();
}
//------------------------------------------------------------------------------
//
//
void TSRSlider::paintEvent( QPaintEvent *event )
{
	Q_UNUSED(event);
	if( !Image )
	{
		std::cout << "TSRSlider : paintEvent and Image == NULL !" << std::endl ;
		return;
	}
	//
	m_mutex.lock();
	//
	QPainter painter(this);
	painter.drawImage(0,0,*Image);
	//
	m_mutex.unlock();
}
//---------------------------------------------------------------------------------
//
//
void TSRSlider::mouseMoveEvent ( QMouseEvent * event )
{
	if( !m_DraggingSlider )
	{
		if( m_SliderRect.contains( event->pos() ) )
		{
			m_HooveredOver = true;
			setCursor( QCursor( Qt::OpenHandCursor ) );
			update();
		}
		else
		{
			m_HooveredOver = false;
			setCursor( QCursor( Qt::ArrowCursor ) );
			update();
		}
	}
	else
	{
		int xpos = event->pos().x();
		//
		if( xpos > m_MaxPos )
			xpos = m_MaxPos;
		if( xpos < m_MinPos )
			xpos = m_MinPos;
		//
		int newsetpoint = CalcUserValueFromMidSliderPosition( xpos );

		if( m_SliderMode == Slider::smWillReiveUpdates )
		{
			// Wait for update....
		}
		else
		{
			// m_SliderMode == Slider::smDirectFollow
			m_CurrentValue = newsetpoint;
		}
		//
        m_SliderSlave->NewSetpointAvailable( newsetpoint);
		//
		UpdateSliderRectWithMidSliderPos( xpos );
		ReCalcConstants();
		RedrawOnImage();
		update();
	}
}
//---------------------------------------------------------------------------------
//
//
void TSRSlider::mousePressEvent ( QMouseEvent * event )
{
	if( m_HooveredOver && ( event->buttons() & Qt::LeftButton == Qt::LeftButton ) )
	{
		m_DraggingSlider = true;
		setCursor( QCursor( Qt::ClosedHandCursor ) );
		update();
	}
}
//---------------------------------------------------------------------------------
//
//
void TSRSlider::mouseReleaseEvent ( QMouseEvent * event )
{
	m_DraggingSlider = false;
    m_SliderSlave->ChangeFinished();
}
//---------------------------------------------------------------------------------
//
//
