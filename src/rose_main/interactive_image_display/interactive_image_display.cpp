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
#include "rose_main/interactive_image_display/interactive_image_display.hpp"

const int PANTILTZOOM_X_MARGIN = 30;
const int PANTILTZOOM_Y_MARGIN = 30;
const int TOP_BUTTONS_X_MARGIN = 80;
const int TOP_BUTTONS_Y_MARGIN = 30;
const int CAMSELECT_X_MARGIN = 30;
const int CAMSELECT_Y_MARGIN = 30;
const int PANTILTWIDTH = 40;
const int BUTTONSPACING = 40;
//---------------------------------------------------------------------------------
//
//
InteractiveImageDisplay::InteractiveImageDisplay( unsigned int componentselection, QWidget * parent )
	:QGraphicsView( parent )
	,ButtonPosition( 1 )
{
    setEnabled( false );
	//
	setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
	setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
	//
	setScene( &m_Scene );
	//
	CreateImageDisplay();
	InitOverlay( componentselection );
	//
	RePosition(); // also forces update();
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::SetInteractionObject( boost::shared_ptr<InteractionObject> interface )
{
    m_InteractionCallBack = interface;
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RePosition()
{
	int w = width();
	int h = height();

	m_UsedWidth  = w;
	m_UsedHeight = h;

	m_Scene.setSceneRect( 0, 0, m_UsedWidth, m_UsedHeight );

	RepositionImageDisplay();
	RepositionPanTilt();
	RepositionCamSelectors();
	RepositionCamIndicator();
	RepositionMidrose();
	RepositionZoombar();
	ButtonPosition = 1;
	RepositionDefaultPositionIndicator();
	RepositionVideoControl();
	RepositionTagControl();

	if( m_OverlayItems.m_CurrentRobotPosition )
	{
		m_OverlayItems.m_CurrentRobotPosition->setPos( w/2, h/2 );
	}
	update();
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionImageDisplay()
{
	m_OverlayItems.m_ImageDisplay->setPos( 0, 0 );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateImageDisplay()
{
	m_OverlayItems.m_ImageDisplay = new ImageDisplay();
	m_Scene.addItem( m_OverlayItems.m_ImageDisplay );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateCamSelector()
{
	if( ! m_OverlayItems.m_CamIndicator )
	{
		m_OverlayItems.m_CamIndicator = new CamIndicator();
		m_Scene.addItem( m_OverlayItems.m_CamIndicator );
	}
	if( !m_OverlayItems.m_LeftCamSelector )
	{
		m_OverlayItems.m_LeftCamSelector = new CamSelector(LeftCamera);
		m_Scene.addItem( m_OverlayItems.m_LeftCamSelector );
		connect( m_OverlayItems.m_LeftCamSelector, SIGNAL( Clicked(LRCameraSelector) ), this, SLOT( SelectCamPrivate(LRCameraSelector) )  );
	}
	if( !m_OverlayItems.m_RightCamSelector )
	{
		m_OverlayItems.m_RightCamSelector = new CamSelector(RightCamera);
		m_Scene.addItem( m_OverlayItems.m_RightCamSelector );
		connect( m_OverlayItems.m_RightCamSelector, SIGNAL( Clicked(LRCameraSelector) ), this, SLOT( SelectCamPrivate(LRCameraSelector) )  );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateDefaultPositionControl()
{
	if( !m_OverlayItems.m_DefaultPosition )
	{
		m_OverlayItems.m_DefaultPosition = new DefaultPosControl();
                m_Scene.addItem( m_OverlayItems.m_DefaultPosition );
                if(m_OverlayItems.m_ImageDisplay != NULL)
                    m_OverlayItems.m_DefaultPosition->setParent(m_OverlayItems.m_ImageDisplay);
		connect( m_OverlayItems.m_DefaultPosition, SIGNAL( Clicked() ), this, SLOT( HouseClicked() )  );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateVideoControl()
{
	if( !m_OverlayItems.m_VideoControl )
	{
		m_OverlayItems.m_VideoControl = new VideoControl();
		m_Scene.addItem( m_OverlayItems.m_VideoControl );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateTagControl()
{
	if( !m_OverlayItems.m_TagControl )
	{
		m_OverlayItems.m_TagControl = new TagControl();
		m_Scene.addItem( m_OverlayItems.m_TagControl );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateCurrentRobotPosition()
{
	if( !m_OverlayItems.m_CurrentRobotPosition )
	{
		m_OverlayItems.m_CurrentRobotPosition = new CurrentRobotPosition();
		m_Scene.addItem( m_OverlayItems.m_CurrentRobotPosition );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateWaypointsOverlay()
{
    ROS_DEBUG_NAMED(ROS_NAME, "InteractiveImageDisplay::CreateWaypointsOverlay");
    if( !m_OverlayItems.m_WaypointsOverlay )
    {
        ROS_DEBUG_NAMED(ROS_NAME, "InteractiveImageDisplay::CreateWaypointsOverlay: Creating new");
        m_OverlayItems.m_WaypointsOverlay = new WaypointsOverlay();
        m_Scene.addItem( m_OverlayItems.m_WaypointsOverlay );
    }
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreatePanTiltControl()
{
	if( !m_OverlayItems.m_PanTiltMiddle )
	{
		m_OverlayItems.m_ArrowUp = new PanTiltArrow();
		m_Scene.addItem( m_OverlayItems.m_ArrowUp );
		connect( m_OverlayItems.m_ArrowUp, SIGNAL( ButtonClicked() ), this, SLOT( MoveUpClicked() )  );
		connect( m_OverlayItems.m_ArrowUp, SIGNAL( ButtonDownLongStart() ), this, SLOT( MoveUpStart() )  );
		connect( m_OverlayItems.m_ArrowUp, SIGNAL( ButtonDownLongStop() ), this, SLOT( MoveUpStop() )  );
		//
                m_OverlayItems.m_ArrowDown = new PanTiltArrow();
                m_Scene.addItem( m_OverlayItems.m_ArrowDown );
		connect( m_OverlayItems.m_ArrowDown, SIGNAL( ButtonClicked() ), this, SLOT( MoveDownClicked() )  );
		connect( m_OverlayItems.m_ArrowDown, SIGNAL( ButtonDownLongStart() ), this, SLOT( MoveDownStart() )  );
		connect( m_OverlayItems.m_ArrowDown, SIGNAL( ButtonDownLongStop() ), this, SLOT( MoveDownStop() )  );
		//
		m_OverlayItems.m_ArrowLeft = new PanTiltArrow();
		m_Scene.addItem( m_OverlayItems.m_ArrowLeft );
		connect( m_OverlayItems.m_ArrowLeft, SIGNAL( ButtonClicked() ), this, SLOT( MoveLeftClicked() )  );
		connect( m_OverlayItems.m_ArrowLeft, SIGNAL( ButtonDownLongStart() ), this, SLOT( MoveLeftStart() )  );
		connect( m_OverlayItems.m_ArrowLeft, SIGNAL( ButtonDownLongStop() ), this, SLOT( MoveLeftStop() )  );
		//
		m_OverlayItems.m_ArrowRight = new PanTiltArrow();
		m_Scene.addItem( m_OverlayItems.m_ArrowRight );
		connect( m_OverlayItems.m_ArrowRight, SIGNAL( ButtonClicked() ), this, SLOT( MoveRightClicked() )  );
		connect( m_OverlayItems.m_ArrowRight, SIGNAL( ButtonDownLongStart() ), this, SLOT( MoveRightStart() )  );
		connect( m_OverlayItems.m_ArrowRight, SIGNAL( ButtonDownLongStop() ), this, SLOT( MoveRightStop() )  );
		//
		m_OverlayItems.m_PanTiltMiddle = new PanTiltMiddle();
		m_Scene.addItem( m_OverlayItems.m_PanTiltMiddle );
		//
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateZoomBar()
{
	if( !m_OverlayItems.m_ZoomBar )
	{
		m_OverlayItems.m_ZoomBar = new ZoomBar();
                m_Scene.addItem( m_OverlayItems.m_ZoomBar );
		connect( m_OverlayItems.m_ZoomBar, SIGNAL( Zoom( int )  ), this, SLOT( Zoom( int ) )  );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::CreateCenterDisplay()
{
	if( !m_OverlayItems.m_MidRose )
	{
                m_OverlayItems.m_MidRose = new CenterOfDisplayDraw();
		m_Scene.addItem( m_OverlayItems.m_MidRose );
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::InitOverlay( unsigned int componentselection  )
{
	std::cout << "Init overlay" << std::endl;
    // These go first, so that everything else draws over it ?
    if( componentselection & ocWaypointsOverlay )
        CreateWaypointsOverlay();
	//
	if( componentselection & ocCamSelectors )
		CreateCamSelector();
	//
	if( componentselection & ocDefaultPosControl )
		CreateDefaultPositionControl();
	//
	if( componentselection & ocVideoControl )
		CreateVideoControl();
	//
	if( componentselection & ocTagControl )
		CreateTagControl();
	//
	if( componentselection & ocPanTilt )
		CreatePanTiltControl();
	//
	if( componentselection & ocZoomBar )
		CreateZoomBar();
	//
	if( componentselection & ocCenterDisplay )
		CreateCenterDisplay();
	//
	if( componentselection & ocCurrentRobotPosition )
		CreateCurrentRobotPosition();
	//
	std::cout << "RePosition" << std::endl;
	RePosition();
	//
}

//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionMidrose()
{
	if( !m_OverlayItems.m_MidRose )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;
	//
	m_OverlayItems.m_MidRose->setPos( w/2 - m_OverlayItems.m_MidRose->boundingRect().width()/2, height()/2 - m_OverlayItems.m_MidRose->boundingRect().height()/2  );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionZoombar()
{
	if( !m_OverlayItems.m_ZoomBar )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	int zoombar_y_margin = 120;
	//
	m_OverlayItems.m_ZoomBar->setPos( w - PANTILTZOOM_X_MARGIN - PANTILTWIDTH, zoombar_y_margin );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionDefaultPositionIndicator()
{
	if( !m_OverlayItems.m_DefaultPosition )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	int dpcw = m_OverlayItems.m_DefaultPosition->boundingRect().width();
	m_OverlayItems.m_DefaultPosition->setPos( w - TOP_BUTTONS_X_MARGIN - ( ButtonPosition * BUTTONSPACING), TOP_BUTTONS_Y_MARGIN );
	ButtonPosition++;
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionVideoControl()
{
	if( !m_OverlayItems.m_VideoControl )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	int vcw = m_OverlayItems.m_VideoControl->boundingRect().width();
	m_OverlayItems.m_VideoControl->setPos( w - TOP_BUTTONS_X_MARGIN - ( ButtonPosition * BUTTONSPACING) , TOP_BUTTONS_Y_MARGIN );
	m_OverlayItems.m_VideoControl->disable();
	ButtonPosition++;
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionTagControl()
{
	if( !m_OverlayItems.m_TagControl )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	m_OverlayItems.m_TagControl->setPos( w -TOP_BUTTONS_X_MARGIN - ( ButtonPosition * BUTTONSPACING), TOP_BUTTONS_Y_MARGIN );
	ButtonPosition++;
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionCamIndicator()
{
	if( !m_OverlayItems.m_CamIndicator )
		return;
    //
	m_OverlayItems.m_CamIndicator->setPos( CAMSELECT_X_MARGIN, CAMSELECT_Y_MARGIN );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionCamSelectors()
{
	if( !m_OverlayItems.m_LeftCamSelector || !m_OverlayItems.m_LeftCamSelector )
		return;
	//
	int csw = m_OverlayItems.m_LeftCamSelector->boundingRect().width();
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	m_OverlayItems.m_LeftCamSelector->setPos( CAMSELECT_X_MARGIN, h - CAMSELECT_Y_MARGIN - csw );
	m_OverlayItems.m_RightCamSelector->setPos( w - CAMSELECT_X_MARGIN - csw, h - CAMSELECT_Y_MARGIN - csw );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::Resize()
{
	RePosition();
	update();
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::resizeEvent ( QResizeEvent * event )
{
	Resize();
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::RepositionPanTilt()
{
	if( !m_OverlayItems.m_PanTiltMiddle )
		return;
	//
	int w = m_UsedWidth;
	int h = m_UsedHeight;

	int ptw = m_OverlayItems. m_PanTiltMiddle->boundingRect().width();
	int aw = m_OverlayItems.m_ArrowDown->boundingRect().width();
	int ah = m_OverlayItems.m_ArrowDown->boundingRect().height();
	int pantilt_y_margin = PANTILTZOOM_Y_MARGIN + ah;

	// the center of pantiltmiddle on its actual location on screen
	int center_ptw = w - PANTILTZOOM_X_MARGIN - ptw/2;
    int left_ptw = w - PANTILTZOOM_X_MARGIN - ptw;

	// beware: the position to set for each arrow is always the top left position of the original drawing (ArrowUp)
	//         e.g. for ArrowRight, which is rotated 90 degrees, the top right position is set.
    m_OverlayItems.m_ArrowUp->setRotation(0);
    m_OverlayItems.m_ArrowUp->setPos( center_ptw - aw/2, pantilt_y_margin - ah );

    m_OverlayItems.m_ArrowDown->setRotation(180);
    m_OverlayItems.m_ArrowDown->setPos( center_ptw + aw/2, pantilt_y_margin + ptw +ah );

    m_OverlayItems.m_ArrowLeft->setRotation(270);
    m_OverlayItems.m_ArrowLeft->setPos( left_ptw - ah, pantilt_y_margin + ptw/2 + aw/2  );

    m_OverlayItems.m_ArrowRight->setRotation(90);
    m_OverlayItems.m_ArrowRight->setPos( w - PANTILTZOOM_X_MARGIN + ah , pantilt_y_margin + ptw/2 - aw/2 );

    m_OverlayItems.m_PanTiltMiddle->setPos( left_ptw, pantilt_y_margin );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::SelectCamPrivate( LRCameraSelector cam )
{
	if( cam == LeftCamera )
	{
		m_OverlayItems.m_LeftCamSelector->setVisible( false );
		m_OverlayItems.m_RightCamSelector->setVisible( true );
	}
	else
	{
		m_OverlayItems.m_LeftCamSelector->setVisible( true );
		m_OverlayItems.m_RightCamSelector->setVisible( false );
	}
	m_OverlayItems.m_CamIndicator->SetCurrentCamera( cam );
	//
	if( m_InteractionCallBack.get() )
		m_InteractionCallBack->SelectCam( cam );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::changeEvent ( QEvent * event )
{
	if( event->type() == QEvent::EnabledChange )
	{
		EnableOverlayItems( isEnabled() );
        update();
	}
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::EnableOverlayItem( OverlayItem* item, bool enable )
{
    if( item )
        item->SetEnabled( enable );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::EnableOverlayItems( bool enable )
{
	// TODO: When overlay items are added to the overlay,
	// pointers to these objects are stored in an object.
	// It would also be useful to have these pointers in a list, so
	// the code below can become a for loop.
	EnableOverlayItem( m_OverlayItems.m_ArrowDown, enable );
	EnableOverlayItem( m_OverlayItems.m_ArrowLeft, enable );
	EnableOverlayItem( m_OverlayItems.m_ArrowRight, enable );
	EnableOverlayItem( m_OverlayItems.m_ArrowUp, enable );
	EnableOverlayItem( m_OverlayItems.m_CurrentRobotPosition, enable );
	EnableOverlayItem( m_OverlayItems.m_PanTiltMiddle, enable );
	EnableOverlayItem( m_OverlayItems.m_ZoomBar, enable );
	EnableOverlayItem( m_OverlayItems.m_TagControl, enable );
	EnableOverlayItem( m_OverlayItems.m_VideoControl, enable );
	EnableOverlayItem( m_OverlayItems.m_DefaultPosition, enable );
	EnableOverlayItem( m_OverlayItems.m_LeftCamSelector, enable );
	EnableOverlayItem( m_OverlayItems.m_RightCamSelector, enable );
    EnableOverlayItem( m_OverlayItems.m_CamIndicator, enable );
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::SetColorOfLineItem( OverlayItem* item, QColor color )
{
	if( item )
		item->SetLineColor(color);
}
//---------------------------------------------------------------------------------
//
//
void InteractiveImageDisplay::SetColorOfLineItems( QColor color )
{
	// TODO: When overlay items are added to the overlay,
	// pointers to these objects are stored in an object.
	// It would also be useful to have these pointers in a list, so
	// the code below can become a for loop.
	SetColorOfLineItem( m_OverlayItems.m_ArrowDown, color );
	SetColorOfLineItem( m_OverlayItems.m_ArrowLeft, color );
	SetColorOfLineItem( m_OverlayItems.m_ArrowRight, color );
	SetColorOfLineItem( m_OverlayItems.m_ArrowUp, color );
	SetColorOfLineItem( m_OverlayItems.m_CurrentRobotPosition, color );
	SetColorOfLineItem( m_OverlayItems.m_PanTiltMiddle, color );
	SetColorOfLineItem( m_OverlayItems.m_ZoomBar, color );
	SetColorOfLineItem( m_OverlayItems.m_TagControl, color );
	SetColorOfLineItem( m_OverlayItems.m_VideoControl, color );
	SetColorOfLineItem( m_OverlayItems.m_DefaultPosition, color );
	SetColorOfLineItem( m_OverlayItems.m_LeftCamSelector, color );
	SetColorOfLineItem( m_OverlayItems.m_RightCamSelector, color );
	SetColorOfLineItem( m_OverlayItems.m_CamIndicator, color );
}
