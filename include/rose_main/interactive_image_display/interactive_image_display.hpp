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
#ifndef INTERACTIVEIMAGEDISPLAY_HPP
#define INTERACTIVEIMAGEDISPLAY_HPP

#include <iostream>
#include <QtCore/QEvent>
#include <QtGui/QGraphicsItem>
#include <QtGui/QGraphicsView>
#include <QtGui/QScrollBar>

#include "rose_main/interactive_image_display/interaction_object.hpp"

#include "rose_main/interactive_image_display/overlay/cam_selector.hpp"
#include "rose_main/interactive_image_display/overlay/cam_indicator.hpp"
#include "rose_main/interactive_image_display/overlay/center_of_display_draw.hpp"
#include "rose_main/interactive_image_display/overlay/current_robot_position.hpp"
#include "rose_main/interactive_image_display/overlay/default_pos_control.hpp"
#include "rose_main/interactive_image_display/overlay/image_display.hpp"
#include "rose_main/interactive_image_display/overlay/pan_tilt_arrow.hpp"
#include "rose_main/interactive_image_display/overlay/pan_tilt_middle.hpp"
#include "rose_main/interactive_image_display/overlay/tag_control.hpp"
#include "rose_main/interactive_image_display/overlay/video_control.hpp"
#include "rose_main/interactive_image_display/overlay/waypoints_overlay.hpp"
#include "rose_main/interactive_image_display/overlay/zoombar.hpp"

#include <boost/shared_ptr.hpp>

using namespace std;

class OverlayItems
{
public:
	//
	OverlayItems()
        :m_WaypointsOverlay( NULL )
        ,m_MidRose( NULL )
		,m_ZoomBar( NULL )
		,m_PanTiltMiddle( NULL )
		,m_ArrowUp( NULL )
		,m_ArrowDown( NULL )
		,m_ArrowLeft( NULL )
		,m_ArrowRight( NULL )
		,m_TagControl( NULL )
		,m_VideoControl( NULL )
		,m_DefaultPosition( NULL )
		,m_LeftCamSelector( NULL )
		,m_RightCamSelector( NULL )
		,m_CamIndicator( NULL )
	    ,m_ImageDisplay( NULL )
        ,m_CurrentRobotPosition( NULL )
	{
	}
	//
	CenterOfDisplayDraw* m_MidRose;
    //
	ZoomBar* m_ZoomBar;
	//
	PanTiltMiddle* m_PanTiltMiddle;
	//
	PanTiltArrow* m_ArrowUp;
	PanTiltArrow* m_ArrowDown;
	PanTiltArrow* m_ArrowLeft;
	PanTiltArrow* m_ArrowRight;
	//
	TagControl* m_TagControl;
	//
	VideoControl* m_VideoControl;
	//
	DefaultPosControl* m_DefaultPosition;
	//
	CamSelector* m_LeftCamSelector;
	CamSelector* m_RightCamSelector;
	//
	CamIndicator* m_CamIndicator;
	//
	ImageDisplay* m_ImageDisplay;
	//
	CurrentRobotPosition* m_CurrentRobotPosition;
    //
    WaypointsOverlay* m_WaypointsOverlay;
};

class InteractiveImageDisplay : public QGraphicsView
{
	Q_OBJECT

public:
	//
	enum OverlayComponent { ocCamSelectors 			= 0x0001,
							ocDefaultPosControl 	= 0x0002,
							ocVideoControl 			= 0x0004,
							ocTagControl 			= 0x0008,
							ocPanTilt 				= 0x0010,
							ocZoomBar				= 0x0020,
							ocCenterDisplay			= 0x0040,
                            ocCurrentRobotPosition	= 0x0080,
                            ocWaypointsOverlay      = 0x0100
						  };
	//
	InteractiveImageDisplay( unsigned int componentselection, QWidget * parent );
	//
	void SetInteractionObject( boost::shared_ptr<InteractionObject> interface );
	//
	void Resize();
	//
	void SelectCam( LRCameraSelector target ){ SelectCamPrivate(target); };
        //
protected:
	//
	virtual void resizeEvent ( QResizeEvent * event );
    virtual void changeEvent ( QEvent * event );

    void SetColorOfLineItem( OverlayItem* item, QColor color );
        void SetColorOfLineItems( QColor color );
	//
	OverlayItems m_OverlayItems;
	//
	QGraphicsScene m_Scene;
	//
private:
	//
	void InitOverlay( unsigned int componentselection  );
	void EnableOverlayItem( OverlayItem* item, bool enable );
	void EnableOverlayItems( bool enable );
	//
	void RePosition();
	void RepositionImageDisplay();
	void RepositionPanTilt();
	void RepositionCamSelectors();
	void RepositionCamIndicator();
	void RepositionMidrose();
	void RepositionZoombar();
	void RepositionDefaultPositionIndicator();
	void RepositionVideoControl();
	void RepositionTagControl();
	//
	void CreateImageDisplay();
	void CreateCamSelector();
	void CreateDefaultPositionControl();
	void CreateVideoControl();
	void CreateTagControl();
	void CreatePanTiltControl();
	void CreateZoomBar();
	void CreateCenterDisplay();
	void CreateCurrentRobotPosition();
    void CreateWaypointsOverlay();
	//
	int ButtonPosition;
	//

	int m_UsedWidth;
	int m_UsedHeight;
	//
	//
	boost::shared_ptr<InteractionObject> m_InteractionCallBack;
	//
	bool TagAvailableAtPosition( QPoint& pos );
	//
private Q_SLOTS:
    //
	virtual void MoveUpClicked() { m_InteractionCallBack->MoveStepUp(); };
	virtual void MoveUpStart() { m_InteractionCallBack->MoveUpStart(); };
	virtual void MoveUpStop() { m_InteractionCallBack->MoveUpStop(); };
	//
	virtual void MoveDownClicked() { m_InteractionCallBack->MoveStepDown(); };
	virtual void MoveDownStart() { m_InteractionCallBack->MoveDownStart(); };
	virtual void MoveDownStop(){ m_InteractionCallBack->MoveDownStop(); };
	//
	virtual void MoveLeftClicked() { m_InteractionCallBack->MoveStepLeft(); };
	virtual void MoveLeftStart() { m_InteractionCallBack->MoveLeftStart(); };
	virtual void MoveLeftStop() { m_InteractionCallBack->MoveLeftStop(); };
	//
	virtual void MoveRightClicked() { m_InteractionCallBack->MoveStepRight(); };
	virtual void MoveRightStart() { m_InteractionCallBack->MoveRightStart(); };
	virtual void MoveRightStop() { m_InteractionCallBack->MoveRightStop(); };
	//
	virtual void HouseClicked() { m_InteractionCallBack->MoveHome(); };
	//
	virtual void SelectCamPrivate( LRCameraSelector target );

	virtual void Zoom( int percentage ) {};
	//
};


#endif /* INTERACTIVEIMAGEDISPLAY_HPP */
