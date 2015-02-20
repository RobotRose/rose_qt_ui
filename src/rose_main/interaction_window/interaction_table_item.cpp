/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/15
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "rose_main/interaction_window/interaction_table_item.hpp"

InteractionTableItem::InteractionTableItem( QString id, QString name )
    :id_ ( id )
    ,name_( name )
{

}

InteractionTableItem::~InteractionTableItem()
{

}

void InteractionTableItem::set_selected( bool selected )
{
    selected_ = selected;
}

bool InteractionTableItem::get_selected()
{
    return selected_;
}

QString InteractionTableItem::get_id()
{
    return id_;
}

QString InteractionTableItem::get_name()
{
    return name_;
}
