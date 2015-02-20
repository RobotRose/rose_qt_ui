/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/17
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#ifndef MAPDISPLAY_H_
#define MAPDISPLAY_H_

#include <iostream>
#include <QtGui/QWidget>

#include "MapImageDisplay.h"
#include "MapDisplayNode.h"

#include "rose_ui_map_display/selection.h"
#include "rose_main/interactive_image_display/interactive_image_display.hpp"

class MapDisplay : public InteractiveImageDisplay, MapImageDisplay
{
    Q_OBJECT

//
public:
        //Constructor
        MapDisplay( unsigned int componentselection, QWidget * parent );
        //
        virtual void ShowMapImage( const QImage& image );
        void set2DPositioning();
        void setNavigation();
        void setNoNavigation();

Q_SIGNALS:
        void setNavGoalStarted();
        void setNavGoalDone();

        void setPose2dStarted();
        void setPose2dDone();

        void setPoseOrNavSelectionStopped();

protected:
        virtual void mousePressEvent(QMouseEvent *event);
        virtual void mouseReleaseEvent(QMouseEvent *event);
        virtual void mouseMoveEvent( QMouseEvent * event);
        virtual void wheelEvent(QWheelEvent *event);

private Q_SLOTS:
        void onMapChanged();
        void onWaypointRequested();
        void onWaypointRequestCancelled();

        virtual void MoveUpClicked();
        virtual void MoveDownClicked();
        virtual void MoveLeftClicked();
        virtual void MoveRightClicked();
        virtual void Zoom( int percentage );
        virtual void HouseClicked();

private:
        void transformToPanAndZoom();
        void drawArrow(QPointF a, QPointF b);

        //For map retrieval
        QImage m_CurMap;
        boost::mutex m_MapMutex;
        MapDisplayNode m_MapDisplayNode;

        //For click handling
        ros::Publisher geoPub;
        ros::Publisher posPub;
        ros::Publisher navGoal;
        bool isLMousePressed;
        QGraphicsLineItem *dirLine;
        QGraphicsLineItem *arrowLine1;
        QGraphicsLineItem *arrowLine2;
        QGraphicsEllipseItem *ellipse;

        ros::NodeHandle nh;

        bool onClickPositioning2D;
        bool onClickNavigation;

        //Zoom factor is the times zoomed, with 1 being the complete map fitting in the display.
        double zoomFactor;

        //The translation set by the overlayitems
        QPointF offset;

};
//
#endif /* MAPDISPLAY_H_ */
