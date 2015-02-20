//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <string>
#include <list>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class Waypoint
{
public:
    string name;
    geometry_msgs::PoseStamped pose;

    /*************************************************************************
    ToXml: Convert a list of waypoints into an xml-string like the following:
        <Waypoints>
            <Waypoint name="Kitchen">
                <PoseStamped>
                    <Header frame_id="/map"/>
                    <Pose>
                        <Position x="1" y="2" z="3" />
                        <Orientation x="0" y="0" z="0" w="0" />
                    </Pose>
                </PoseStamped>
            </Waypoint>
        </Waypoints>
    *************************************************************************/
    static void ToXml(list<Waypoint> waypoints, const std::string filename);

    /*************************************************************************
    FromXml: Convert an xml-string into a list of waypoints
    *************************************************************************/
    static list<Waypoint> FromXml(const std::string filename);
};

#endif //WAYPOINT_H
