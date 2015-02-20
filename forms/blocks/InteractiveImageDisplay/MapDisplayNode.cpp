//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/**
 * @file /src/MapDisplayNode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <rose_ui_map_display/colored_polygon_stamped.h>
#include <sstream>
#include <QImage>
#include "MapDisplayNode.h"
#include <QtGui>
#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Implementation
*****************************************************************************/

MapDisplayNode::MapDisplayNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    m_HalfMapWidth(0),
    m_HalfMapHeight(0)
{
    mapscale = 0.0;

    img_initialized = false;
    map_initialized = false;

    setup_image(10,10);
    mapimg.fill(qRgba(0,0,0,0));

	boost::shared_ptr<nav_msgs::GridCells> tempGC1(new nav_msgs::GridCells);
	boost::shared_ptr<nav_msgs::GridCells> tempGC2(new nav_msgs::GridCells);
	gridObstacle 			= tempGC1;
	gridInflatedObstacle 	= tempGC2;

    m_Waypoints = std::vector< std::tuple<float, float, float, float, QString> >();
    boost::shared_ptr<rose_ui_map_display::waypoint_array> tempGC3(new rose_ui_map_display::waypoint_array);
    m_WaypointArrayMsg =  tempGC3;
}

MapDisplayNode::~MapDisplayNode()
{
    ros::shutdown(); // explicitly needed since we use ros::start();
    std::cout << "Waiting for ros thread to finish." << std::endl;
    wait();
}

void MapDisplayNode::init(const std::string &topic_name)
{
    ros::init(init_argc,init_argv,"qtest");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    //subscribers needed to get all the topics
    //obstacle_subscriber = n.subscribe("move_base/local_costmap/obstacles",1,&MapDisplayNode::obstacleCallback,this);
   // inflation_subscriber = n.subscribe("move_base/local_costmap/inflated_obstacles",1,&MapDisplayNode::inflationCallback,this);
    map_subscriber                      = n.subscribe( "/map_display/map",              1, &MapDisplayNode::mapCallback,                 this);
    waypointrequest_subscriber          = n.subscribe( "/map_display/request",          1, &MapDisplayNode::waypointRequestCallback,     this);
    waypointRequestCancel_subscriber    = n.subscribe( "/map_display/request_cancel",   1, &MapDisplayNode::waypointRequestCancelCallback,     this);

    robot_footprint_subscriber  = n.subscribe( "/map_display/robot_footprint",  1, &MapDisplayNode::robotfootprintCallback,      this);
    ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::init subscribed to /map_display/robot_footprint");

    navigation_plan_subscriber  = n.subscribe( "/map_display/navigation_path",  1, &MapDisplayNode::robotnavigationplanCallback, this);
    waypoints_subscriber        = n.subscribe( "/map_display/waypoints",        1, &MapDisplayNode::waypointsCallback, this);

    start();
}

void MapDisplayNode::init(const std::string &master_url, const std::string &host_url, const std::string &topic_name)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"qtest");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    //subscribers needed to get all the topics
    //obstacle_subscriber = n.subscribe("move_base/local_costmap/obstacles",1,&MapDisplayNode::obstacleCallback,this);
    //inflation_subscriber = n.subscribe("move_base/local_costmap/inflated_obstacles",1,&MapDisplayNode::inflationCallback,this);
    map_subscriber              = n.subscribe( "/map_display/map",              1, &MapDisplayNode::mapCallback,                 this);

    robot_footprint_subscriber  = n.subscribe( "/map_display/robot_footprint",  1, &MapDisplayNode::robotfootprintCallback,      this);
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplayNode::init subscribed to /map_display/robot_footprint");

    navigation_plan_subscriber  = n.subscribe( "/map_display/navigation_path",  1, &MapDisplayNode::robotnavigationplanCallback, this);

    start();
}

void MapDisplayNode::run()
{
    //spin to receive messages
    ros::Rate loop_rate(10);
    while ( ros::ok() ) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

const QImage& MapDisplayNode::getmap(bool doDrawRobotFootprint, bool doDrawNavigationPath)
{
//    ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::getmap(%i, %i)", (int)doDrawRobotFootprint, (int)doDrawNavigationPath);

    //Reset the original map without obstacles
    mapimg = emptyMapImg.copy(emptyMapImg.rect());

    //Draw obstacles
    drawObstaclesWithInflation(mapimg);

    //Draw footprint
    QPolygonF tempFootprint = robot_footprint;
    if(doDrawRobotFootprint)
    {
        drawRobotFootprint(mapimg, tempFootprint);
    }

    //Draw navigation path (if any)
    if(doDrawNavigationPath)
        drawNavigationPath(mapimg, navigationPath);

    //Flip the image to convert between coordinate systems (y[0] == top VS y[0] == bottom)
    finalMapImg = mapimg.mirrored(false, true);

    return finalMapImg;
}

/*
 Get waypoints, transformed to image coordinates!

 */
const std::vector< std::tuple<float, float, float, float, QString> > MapDisplayNode::getWaypoints()
{
//    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplayNode::getWaypoints");
    m_Waypoints.clear();

//    ROS_DEBUG_NAMED(ROS_NAME, "There are now %i waypoints",(int) m_WaypointArrayMsg->waypoints.size());

    for(auto &wp_msg : m_WaypointArrayMsg->waypoints)
    {
//        ROS_DEBUG_NAMED(ROS_NAME, "Processing %s", wp_msg.name.c_str());

        auto mapCoords = metersToPixels(std::pair<float,float>(wp_msg.x, wp_msg.y), true); //true means to apply the vertical flip already here.
        float map_x = mapCoords.first;
        float map_y = mapCoords.second;

        float theta = wp_msg.theta * (180 / M_PI); //From radians to degrees

//        ROS_DEBUG_NAMED(ROS_NAME, "%s x: %f, y: %f, theta %f", wp_msg.name.c_str(), map_x, map_y, theta);
        auto wpTuple = std::make_tuple(map_x, map_y, 20, theta, QString(wp_msg.name.c_str()));
        m_Waypoints.push_back(wpTuple);
    }

    return m_Waypoints;
}

bool MapDisplayNode::isKnownLocation(int x, int y)
{
    return ( mapimg.pixel(x,y) != unknownspace );
}

void MapDisplayNode::drawRobotFootprint(QImage &img, QPolygonF footprint)
{
//    ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::drawRobotFootprint. Count = %i", footprint.count());
    if(footprint.count() >= 4)
    {
//        ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::drawRobotFootprint. Going to draw...", footprint.count());
        //Draw the robot footprint
        QPainter painter(&img);

        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::HighQualityAntialiasing, true);

        painter.setPen(Qt::green);
//        painter.drawPolyline(footprint.toPolygon());

        for(int i; i<footprint.count() -1; i++) //Size -1 because we don't want to also draw the last point.
        {
//            ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::drawRobotFootprint. Drawing line %i of %i", i, footprint.count());
            QPointF startPoint = footprint[i];
            QPointF endPoint = footprint[i+1];
            QColor lineColor = robot_footprint_colors[i];

//            ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::drawRobotFootprint drawing line with color (%i,%i, %i)", lineColor.red(), lineColor.green(), lineColor.blue());

            QPen pen = QPen();
            pen.setWidth(2);
            pen.setColor(lineColor);
            painter.setPen(pen);
            QLineF line = QLineF(startPoint, endPoint);
            painter.drawLine(line);
        }

        painter.setPen(Qt::green);
        //draw the direction
//        QPolygonF dir;
//        QPointF p30((footprint[3].x() + footprint[0].x()) / 2,
//                   (footprint[3].y() + footprint[0].y()) / 2);
//        QPointF p01((footprint[0].x() + footprint[1].x()) / 2,
//                   (footprint[0].y() + footprint[1].y()) / 2);
//        QPointF p12((footprint[1].x() + footprint[2].x()) / 2,
//                   (footprint[1].y() + footprint[2].y()) / 2);
//        QPointF p23((footprint[2].x() + footprint[3].x()) / 2,
//                   (footprint[2].y() + footprint[3].y()) / 2);

//        //Draw filled arrowhead
//        dir << p30 << p01 << p12;
//        painter.setBrush(Qt::green);
//        painter.drawPolygon(dir);

        painter.end();
    }
}

void MapDisplayNode::drawNavigationPath(QImage &img, QPolygonF path)
{
    QPainter painter(&img);
    painter.setPen(Qt::green);
    painter.drawPolyline(path.toPolygon());
}

void MapDisplayNode::setup_image(int width, int height)
{
    ROS_INFO("setup_image width: %d, height: %d", width, height);

    //check if there is a map, if so delete it before making a new one
    emptyMapImg = QImage(width, height, QImage::Format_ARGB32);
    mapimg = QImage(width,height, QImage::Format_ARGB32);

    //setting up colors to use later
    emptyspace = qRgba(190,190,190,255);     //gray, empty space
    obstacle = qRgba(0,0,0,255);             //black, obstacle
    unknownspace = qRgba(255,255,255,255);   //white, unknown space
    costobstacle = qRgba(255,0,0,255);       //red, costmap obstacle
    costinflation = qRgba(255,120,0,255);      //costmap inflation

    img_initialized = true;
}

//small function to make setting pixels easier
void MapDisplayNode::setPixel(QImage &img, int x, int y, QRgb color)
{  
    // Set color
    img.setPixel((m_HalfMapWidth+x)-mapminx,(m_HalfMapHeight+y)-mapminy,color);
}

//function to calculate size of the published map
void MapDisplayNode::getsize(const nav_msgs::OccupancyGrid::ConstPtr& map, int * minx, int * maxx, int * miny, int * maxy)
{
    bool done = false;
    bool top,bot,left,right;
    top = bot = left = right = true;
    int loopx = 0, loopy = 0;
    spiral(&loopx,&loopy,true);

    int centerx = m_HalfMapWidth, centery = m_HalfMapHeight;
    int mapwidth = map->info.width;

    while(map->data[centerx+loopx + ((centery+loopy) * mapwidth)] == -1)
    {
        spiral(&loopx,&loopy,false);
    }
    ROS_INFO("first x: %d, y: %d",loopx,loopy);

    *minx = *maxx = loopx+m_HalfMapWidth;
    *miny = *maxy = loopy+m_HalfMapHeight;

    //check if the row still contains usefull pixels, if so increase the size in that direction
    while(!done)
    {
        if(!(top || bot || left || right))
        {
            done = true;
            top = bot = left = right = true;
        }

        if(top)
        {
            top = false;
            for(int i = 0; i< (*maxx-*minx +1); i++)
            {
                if(map->data[(*miny) * mapwidth + (*minx)+i] != -1)
                {
                    done = false;
                    // top = true;
                    top = bot = left = right = true;
                    (*miny)--;
                    break;
                }
            }
        }

        if(bot)
        {
            bot = false;
            for(int i = 0; i< ((*maxx)-(*minx) +1); i++)
            {
                if(map->data[(*maxy)*mapwidth + (*minx)+i] != -1)
                {
                    done = false;
                    //bot = true;
                    top = bot = left = right = true;
                    (*maxy)++;
                    break;
                }
            }
        }

        if(left)
        {
            left = false;
            for(int i = 0; i< ((*maxy)-(*miny) +1); i++)
            {
                if(map->data[((*miny)+i)*mapwidth + (*minx)] != -1)
                {
                    done = false;
                    // left = true;
                    top = bot = left = right = true;
                    (*minx)--;
                    break;
                }
            }
        }

        if(right)
        {
            right = false;
            for(int i = 0; i< ((*maxy)-(*miny) +1); i++)
            {
                if(map->data[((*miny)+i)*mapwidth + (*maxx)] != -1)
                {
                    done = false;
                    // right = true;
                    top = bot = left = right = true;
                    (*maxx)++;
                    break;
                }
            }
        }

    }//end while

    ROS_INFO("done: %d, %d, %d, %d", (*minx), (*maxx), (*miny), (*maxy));

}

//function make a spiral
void MapDisplayNode::spiral(int *retx, int *rety, bool reset)
{
    static int x = 0,y = 0, dx = 1, dy = 0, size = 0;

    //check if the spiral has to be reset;
    if(reset)
    {
        x = 0;
        y = 0;
        dx = 1;
        dy = 0;
        size = 0;
        return;
    }
    //return the last calculated value
    *retx = x; *rety = y;

    // begin spiral
    if(x == size && y == size)
    {
        size ++;
    }
    else if(x == size && y == (size*-1))
    {
        dx = -1;
        dy = 0;
    }
    else if(x == size && y == (size-1))
    {
        dx = 0;
        dy = -1;
    }
    else if(x == (size*-1) && y == (size*-1))
    {
        dx = 0;
        dy = 1;
    }
    else if(x == (size*-1) && y == size)
    {
        dx = 1;
        dy = 0;
    }
    // end spiral
    x += dx;
    y += dy;
}


void MapDisplayNode::drawObstaclesWithInflation(QImage &img)
{
    float x=0,y=0;

    //Return without drawing if map is not drawn.
	if(!map_initialized) return;
    //if(!map_initialized || !gridObstacle || !gridInflatedObstacle) return;

    // ROS_INFO("number of obstacle points: %d", gridObstacle->cells.size());

    for(unsigned int i = 0; i < gridObstacle->cells.size(); i++)
    {
        //calculate the x and y to for the map
        x = ((float)gridObstacle->cells[i].x / mapscale);
        y = ((float)gridObstacle->cells[i].y / mapscale);

        //paint the obstacles pixels to the map
        setPixel(img, (int)x,(int)y,costobstacle);
    }

    //ROS_INFO("number of inflation points: %d",gridInflatedObstacle->cells.size());
    for(unsigned int i = 0; i < gridInflatedObstacle->cells.size(); i++)
    {
        //calculate the x and y to for the map
        x = ((float)gridInflatedObstacle->cells[i].x / mapscale);
        y = ((float)gridInflatedObstacle->cells[i].y / mapscale);

        //paint the inflation pixels to the map
        setPixel(img, (int)x,(int)y,costinflation);
    }
}


QPoint MapDisplayNode::transformScaledPixelsToFullPixels(QPoint point)
{
    return QPoint(point.x() + mapminx, point.y() + mapminy);
}

std::pair<float,float>  MapDisplayNode::fullPixelsToMeters(QPoint point)
{
    std::pair<float,float> meters;
    meters.first = origin.first + ((float)point.x() * mapscale);
    meters.second = origin.second + ((float)point.y() * mapscale);

    return meters;
}

std::pair<float,float>  MapDisplayNode::metersToPixels(std::pair<float,float> meters, bool applyYmirroring)
{
    // ROS_INFO("meters x: %f, y: %f", meters.first, meters.second);
    std::pair<float,float> pixel;

    //calculate the x and y to for the map
    auto x = meters.first/mapscale - origin.first/mapscale;
    auto y = meters.second/mapscale - origin.second/mapscale;

    pixel.first  = x - mapminx;
    pixel.second = y - mapminy;

    if(applyYmirroring)
    {
        pixel.second = mapimg.height() - pixel.second;
    }

    // ROS_INFO("meterstoPixels x: %f, y: %f", pixel.first, pixel.second);
    return pixel;
}


QPolygonF MapDisplayNode::getRobotFootprint()
{
    return robot_footprint;
}


/*****************************************************************************
 * Callbacks
 ****************************************************************************/

//callback to get the map
void MapDisplayNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    syncroot.lock();

    QRgb pixelcolor;

    m_HalfMapWidth = msg->info.width/2;
    m_HalfMapHeight = msg->info.height/2;

    mapscale = msg->info.resolution;
    origin = std::pair<float,float>(msg->info.origin.position.x, msg->info.origin.position.y);

    int maxx=0,minx=0,maxy=0,miny=0;
    getsize(msg,&minx,&maxx,&miny,&maxy);

    //add a bit to the size of the map to make sure everything fits;
    // minx -= 10;
    // miny -= 10;
    // maxx += 10;
    // maxy += 10;

    ROS_INFO("setting up map");
    ROS_INFO("minx: %d, miny %d, maxx: %d, maxy: %d",minx,miny,maxx,maxy);
    // if(maxx-minx>maxy-miny)
    // {
        
    //     maxy += ((maxx-minx) - (maxy-miny))/2;
    // }
    // else
    // {
    //     setup_image(maxy-miny,maxy-miny);
    //     maxx += ((maxy-miny) - (maxx-minx))/2;
    // }

    setup_image(maxx-minx,maxy-miny);

    //set correction values to convert origional coordinates to this maps coordinates
    mapminx = minx;
    mapminy = miny;
    ROS_INFO("minx: %d, miny %d",minx,miny);

    mapimg.fill(unknownspace);

    for(int i = miny; i < maxy; i++)
    {
        for(int j = minx; j < maxx; j++)
        {
            // ROS_DEBUG("[%d,%d] : %d", i, j, j + i*msg->info.width);
            // ROS_DEBUG("j + i*msg->info.width >= 0");
            // ROS_DEBUG("data : %d", msg->data[j + i*msg->info.width]);
            switch(msg->data[j + i*msg->info.width])
            {
                    case 100: pixelcolor = obstacle; break;
                    case 0: pixelcolor = emptyspace; break;
                    default: pixelcolor = unknownspace; break;
            }
            
            mapimg.setPixel(j-minx,i-miny,pixelcolor);
        }
    }

    emptyMapImg = mapimg.copy(mapimg.rect());

    map_initialized = true;

    Q_EMIT mapChanged();
    syncroot.unlock();
}

void MapDisplayNode::waypointRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplayNode::waypointRequestCallback received request: '%s'", msg->data.c_str());

    Q_EMIT waypointRequested();
}

void MapDisplayNode::waypointRequestCancelCallback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplayNode::waypointRequestCancelCallback received cancel request");

    Q_EMIT waypointRequestCanceled();
}

//callback to get the obstacles
void MapDisplayNode::obstacleCallback(const nav_msgs::GridCells::ConstPtr &msg)
{
    syncroot.lock();
    gridObstacle = msg;
    Q_EMIT mapChanged();
    syncroot.unlock();
}

//callback to get the inflation of the obstacles
void MapDisplayNode::inflationCallback(const nav_msgs::GridCells::ConstPtr &msg)
{
    syncroot.lock();
    gridInflatedObstacle = msg;
    Q_EMIT mapChanged();
    syncroot.unlock();
}

//callback to get the robot footprint
void MapDisplayNode::robotfootprintCallback(const rose_ui_map_display::colored_polygon_stamped::ConstPtr &msg)
{
    //ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::robotfootprintCallback");
    if(!map_initialized ) return;

    //ROS_INFO_NAMED(ROS_NAME, "polygon size: %d", (int)msg->polygon.points.size());

    syncroot.lock();
    QPolygonF oldFootprint(robot_footprint);
    robot_footprint.clear();

    float x = 0, y = 0;
    for(unsigned int i = 0; i < msg->polygon.points.size(); i++)
    {
        //calculate the x and y to for the map
        // x = msg->polygon.points[i].x/mapscale;
        // y = msg->polygon.points[i].y/mapscale;

        auto mapCoords = metersToPixels(std::pair<float,float>(msg->polygon.points[i].x, msg->polygon.points[i].y));
        // auto mapCoords = metersToPixels(std::pair<float,float>(0,0));
        x = mapCoords.first;
        y = mapCoords.second;

        // ROS_DEBUG_NAMED(ROS_NAME, "x: %f, y: %f", x, y);

        //adding the point for the QPolygon
        //robot_footprint << QPoint((m_HalfMapWidth+x)-mapminx,(m_HalfMapHeight+y)-mapminy);
        // robot_footprint << QPoint(y,x);
        // 
        robot_footprint << QPoint(x,y);
    }

    robot_footprint_colors.clear();
    for(unsigned int i = 0; i < msg->colors.size(); i++)
    {
        std_msgs::ColorRGBA rosColor = msg->colors[i];

        //ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::robotfootprintCallback receiving color (%f,%f, %f, %f)", rosColor.r, rosColor.g, rosColor.b, rosColor.a);
        //Convert ROS color to QColor:
        QColor qColor = QColor::fromRgbF(rosColor.r, rosColor.g, rosColor.b, rosColor.a);
        //ROS_INFO_NAMED(ROS_NAME, "MapDisplayNode::robotfootprintCallback converted color (%i,%i, %i, %i)", qColor.red(), qColor.green(), qColor.blue(), qColor.alpha());

        //ROS_DEBUG_NAMED(ROS_NAME, "Converting a color...");
        robot_footprint_colors.push_back(qColor);
    }

    //if(oldFootprint.data() != robot_footprint.data())
//    if(!qEqual(oldFootprint.begin(), oldFootprint.end(), robot_footprint.begin()))
    Q_EMIT mapChanged();

    syncroot.unlock();
}

void MapDisplayNode::robotnavigationplanCallback(const nav_msgs::Path::ConstPtr &msg)
{
    //ROS_INFO("Robot navigation path received\n");

    if(!map_initialized ) return;
    
    syncroot.lock();

    QPolygonF oldNavigationPath(navigationPath);
    navigationPath.clear();

    float x = 0, y = 0;
    for(unsigned int i = 0; i < msg->poses.size(); i++)
    {
        auto mapCoords = metersToPixels(std::pair<float,float>(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y));
        // auto mapCoords = metersToPixels(std::pair<float,float>(0,0));
        x = mapCoords.first;
        y = mapCoords.second;

        //adding the point for the QPolygon
        navigationPath << QPoint(x,y);
    }

    if(!qEqual(oldNavigationPath.begin(), oldNavigationPath.end(), navigationPath.begin()))
        Q_EMIT mapChanged();
    
    syncroot.unlock();
}

void MapDisplayNode::waypointsCallback(const rose_ui_map_display::waypoint_array::ConstPtr &msg)
{
    ROS_INFO_NAMED(ROS_NAME, "Received %i waypoints", (int)msg->waypoints.size());
    m_WaypointArrayMsg = msg;

    Q_EMIT mapChanged();
}
