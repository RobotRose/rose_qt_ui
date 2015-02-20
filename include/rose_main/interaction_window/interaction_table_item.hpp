/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/15
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#ifndef INTERACTIONTABLEITEM_HPP
#define INTERACTIONTABLEITEM_HPP

#include <QTableWidget>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

class InteractionTableItem
{
	
public:
    InteractionTableItem( QString id, QString name );
    ~InteractionTableItem();

    void    set_selected( bool selected );
    bool    get_selected();

    QString get_name();
    QString get_id();

private:
    QString name_;
    QString id_;
    bool    selected_;

};

#endif // INTERACTIONTABLEITEM_HPP
