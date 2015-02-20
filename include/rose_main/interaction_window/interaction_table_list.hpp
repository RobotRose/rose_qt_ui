/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/12
* 		- File created.
*
* Description:
*	Contains a tablelist of scripts/items/paramaters. Used by the
*	Interactive window in the GUI.
* 
***********************************************************************************/

#ifndef INTERACTIONTABLELIST_H
#define INTERACTIONTABLELIST_H

#include <iostream>
#include <QHeaderView>
#include <QScrollBar>
#include <QtGui/QTableWidget>

#include <ros/ros.h>

#include "rose_main/interaction_window/interaction_table_item.hpp"
#include "rose_ui_item_selector/selection_mode.hpp"

class InteractionTableList : public QTableWidget
{
	Q_OBJECT

  public:
    enum eListType{
        eScriptList,
        eParamList
    };

    InteractionTableList( QWidget* parent, eListType listtype, int position = 0 );
    ~InteractionTableList();

    void        addItem(InteractionTableItem* item);
    void        clearTable();

    int         get_position();
    eListType   get_listtype();

    void        setTableSelectionMode( eSelectionMode mode );
    void        setRowSelected(int row);
    void        scrollTo(int row);

  private:
    void setTableNotSelectable();
    void setTableOneSelectable();
    void setTableMultipleSelectable();

    eListType       listtype_;
    QWidget*        parent_;
    intptr_t        position_;
    eSelectionMode  selection_mode_;
	/* data */
};

#endif // INTERACTIONTABLELIST_H
