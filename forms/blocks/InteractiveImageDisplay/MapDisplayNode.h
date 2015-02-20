//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/**
 * @file /include/qtest/MapDisplayNode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtest_MapDisplayNode_HPP_
#define qtest_MapDisplayNode_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QtCore/QThread>
#include <QtGui/QStringListModel>
#include <math.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"


//required message files:
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <rose_ui_map_display/colored_polygon_stamped.h>
#include <rose_ui_map_display/waypoint.h>
#include <rose_ui_map_display/waypoint_array.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include <tf/transform_listener.h>
//
#include "rose_common/common.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Class
*****************************************************************************/

class MapDisplayNode : public QThread {
        Q_OBJECT

Q_SIGNALS:
        void mapChanged();
        void waypointRequested();
        void waypointRequestCanceled();

public:
        MapDisplayNode(int argc, char** argv );
        ~MapDisplayNode();
	void init(const std::string &topic_name);
	void init(const std::string &master_url, const std::string &host_url, const std::string &topic_name);
	void run();

	QStringListModel* loggingModel() { return &logging_model; }
        const QImage& getmap(bool doDrawRobotFootprint, bool doDrawNavigationPath);
        const std::vector< std::tuple<float, float, float, float, QString> > getWaypoints();

        QPoint transformScaledPixelsToFullPixels(QPoint point);
        std::pair<float,float> fullPixelsToMeters(QPoint point);
        std::pair<float,float> metersToPixels(std::pair<float,float> pixel, bool applyYmirroring=false);

        bool img_initialized;
        bool map_initialized;

        QPolygonF getRobotFootprint();
        bool isKnownLocation(int x, int y);
        boost::timed_mutex syncroot;

private:    //members
        QPolygonF robot_footprint;
        std::vector<QColor> robot_footprint_colors;
        QPolygonF navigationPath;

	int init_argc;
	char** init_argv;
        QStringListModel logging_model;

        ros::Subscriber map_subscriber;
        ros::Subscriber waypointrequest_subscriber;
        ros::Subscriber waypointRequestCancel_subscriber;
        ros::Subscriber obstacle_subscriber;
        ros::Subscriber inflation_subscriber;
        ros::Subscriber robot_footprint_subscriber;
        ros::Subscriber navigation_plan_subscriber;
        ros::Subscriber waypoints_subscriber;

        float mapscale;
        std::pair<float,float> origin;
        QImage mapimg;
        QImage emptyMapImg;
        QImage finalMapImg;
        int mapminx,mapminy,mapmaxx,mapmaxy;

        //colors for painting
        QRgb unknownspace;
        QRgb obstacle;
        QRgb emptyspace;
        QRgb costobstacle;
        QRgb costinflation;

        //Stored values
        nav_msgs::GridCells::ConstPtr gridObstacle;
        nav_msgs::GridCells::ConstPtr gridInflatedObstacle;
        int m_HalfMapWidth;
        int m_HalfMapHeight;

        rose_ui_map_display::waypoint_array::ConstPtr m_WaypointArrayMsg;
        std::vector< std::tuple<float, float, float, float, QString> > m_Waypoints;

private:    //methods

        /* Create an empty image and set MapDisplayNode color-members (unknownspace, obstacle, etc.) */
        void setup_image(int width, int height);

        /* Calculate the size of the published map */
        void getsize(const nav_msgs::OccupancyGrid::ConstPtr& map, int * minx, int * maxx, int * miny, int * maxy);

        /*  */
        void spiral(int* x,int* y,bool reset);

        /* */
        void drawRobotFootprint(QImage &img, QPolygonF footprint);
        void drawNavigationPath(QImage &img, QPolygonF path);

        /* Colors a single pixel of mapimg with the specified color. */
        void setPixel(QImage &img, int x, int y, QRgb color);

        /* Callback function for the /map-topic. */
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        /* Callback function a request to get a waypoint from the operator */
        void waypointRequestCallback(const std_msgs::String::ConstPtr& msg);

        /* Callback function to _cancel_ a request to get a waypoint from the operator */
        void waypointRequestCancelCallback(const std_msgs::Empty::ConstPtr& msg);

        void drawObstaclesWithInflation(QImage &img);

        /* Callback function for the /move_base/local_costmap/obstacles-topic. */
        void obstacleCallback(const nav_msgs::GridCells::ConstPtr& msg);

        /* Callback function for the /move_base/local_costmap/inflated_obstacles-topic. */
        void inflationCallback(const nav_msgs::GridCells::ConstPtr& msg);

        /* Callback function for the /move_base/local_costmap/robot_footprint-topic. */
        void robotfootprintCallback(const rose_ui_map_display::colored_polygon_stamped::ConstPtr &msg);

        /* */
        void robotnavigationplanCallback(const nav_msgs::Path::ConstPtr &msg);

        /* Receive and draw waypoints on the maps*/
        void waypointsCallback(const rose_ui_map_display::waypoint_array::ConstPtr &msg);

        /* Not implemented */
        void updateRobot();

        /* Not implemented */
        void transform_2dpoint(int* x, int* y, float angle);


};


#endif /* qtest_MapDisplayNode_HPP_ */
