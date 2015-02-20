//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#include "Waypoint.h"
#include <tinyxml.h>

list<Waypoint> Waypoint::FromXml(const std::string filename)
{
    list<Waypoint> waypoints;

    TiXmlDocument doc(filename.c_str());
    if(doc.LoadFile())
    {
        TiXmlElement *wps, *wp, *ps, *hdr, *pose, *position, *orientation;

        wps = doc.FirstChildElement( "Waypoints" );

        for(wp = wps->FirstChildElement( "Waypoint" ); wp != 0; wp = (TiXmlElement*)(wp->NextSibling()))
        {
            Waypoint w;

            //Get all children
            ps = wp->FirstChildElement( "PoseStamped" );
            hdr = ps->FirstChildElement("Header");
            pose = ps->FirstChildElement("Pose");
            position = pose->FirstChildElement("Position");
            orientation = pose->FirstChildElement("Orientation");

            //Get all values
            w.name = wp->Attribute( "name" );
            w.pose.header.seq = 0;
            w.pose.header.stamp = ros::Time(0);
            w.pose.header.frame_id = hdr->Attribute("frame_id");
            position->Attribute("x", &w.pose.pose.position.x);
            position->Attribute("y", &w.pose.pose.position.y);
            position->Attribute("z", &w.pose.pose.position.z);
            orientation->Attribute("x", &w.pose.pose.orientation.x);
            orientation->Attribute("y", &w.pose.pose.orientation.y);
            orientation->Attribute("z", &w.pose.pose.orientation.z);
            orientation->Attribute("w", &w.pose.pose.orientation.w);

            waypoints.push_back(w);
        }
    }

    return waypoints;
}

void Waypoint::ToXml(list<Waypoint> waypoints, const std::string filename)
{
    //stream for converting to string
    std::ostringstream os;

    TiXmlDocument doc;

    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );

    TiXmlElement * root = new TiXmlElement( "Waypoints" );
    doc.LinkEndChild( root );

    for(std::list<Waypoint>::iterator it = waypoints.begin(); it != waypoints.end(); it++){
        TiXmlElement* wp = new TiXmlElement( "Waypoint" );
        wp->SetAttribute("name", it->name.c_str());
        root->LinkEndChild( wp );

        TiXmlElement * ps = new TiXmlElement( "PoseStamped" );
        wp->LinkEndChild( ps );

        TiXmlElement * hdr = new TiXmlElement( "Header" );
        hdr->SetAttribute("frame_id", it->pose.header.frame_id.c_str());
        ps->LinkEndChild( hdr );

        TiXmlElement * pose = new TiXmlElement( "Pose" );
        ps->LinkEndChild( pose );

        TiXmlElement * pos = new TiXmlElement( "Position" );
        pos->SetDoubleAttribute("x", it->pose.pose.position.x);
        pos->SetDoubleAttribute("y", it->pose.pose.position.y);
        pos->SetDoubleAttribute("z", it->pose.pose.position.z);
        pose->LinkEndChild( pos );

        TiXmlElement * ori = new TiXmlElement( "Orientation" );
        ori->SetDoubleAttribute("x", it->pose.pose.orientation.x);
        ori->SetDoubleAttribute("y", it->pose.pose.orientation.y);
        ori->SetDoubleAttribute("z", it->pose.pose.orientation.z);
        ori->SetDoubleAttribute("w", it->pose.pose.orientation.w);
        pose->LinkEndChild( ori );
    }

    doc.SaveFile(filename.c_str());
}
