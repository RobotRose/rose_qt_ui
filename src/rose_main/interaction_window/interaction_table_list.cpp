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

#include "rose_main/interaction_window/interaction_table_list.hpp"

InteractionTableList::InteractionTableList( QWidget* parent, eListType listtype, int position )
	: parent_ ( parent )
    , listtype_ ( listtype )
    , position_ ( position )
{
    setColumnCount(1);

    horizontalHeader()->setHidden(true);
    verticalHeader()->setHidden(true);

    horizontalScrollBar()->setHidden(true);
    verticalScrollBar()->setHidden(false);
    verticalScrollBar()->setEnabled(true);

    setMouseTracking(true);
    setParent(parent_);

    // Allow only selection of one cell in a table
    setTableSelectionMode(eSingleSelection);

    connect(this, SIGNAL(itemClicked(QTableWidgetItem*)), parent_, SLOT(itemActivated(QTableWidgetItem*)));
    connect(this, SIGNAL(itemEntered(QTableWidgetItem*)), parent_, SLOT(itemEntered(QTableWidgetItem*)));
    show();
}

InteractionTableList::~InteractionTableList()
{
    ROS_INFO("InteractionTableList::~InteractionTableList()");
}

void InteractionTableList::addItem(InteractionTableItem* item)
{
    // Create QWidget item
    QTableWidgetItem* new_item = new QTableWidgetItem();
    new_item->setText(item->get_name());

    // Add row and item to that row
    setRowCount(rowCount()+1);
    setColumnWidth(rowCount()-1, width());
    setItem(rowCount()-1, columnCount()-1, new_item);
    new_item->setFlags(new_item->flags()  ^ Qt::ItemIsEditable);
}

void InteractionTableList::clearTable()
{
    while ( rowCount() > 0 )
    {
        removeRow(0);
        setRowCount(rowCount()-1);
    }
}

int InteractionTableList::get_position()
{
    return position_;
}

InteractionTableList::eListType InteractionTableList::get_listtype()
{
    return listtype_;
}

void InteractionTableList::setTableNotSelectable()
{
    setSelectionMode(QAbstractItemView::NoSelection);
    setDisabled(true);
}

void InteractionTableList::setTableOneSelectable()
{
    setSelectionMode(QAbstractItemView::SingleSelection);
    setDisabled(false);
}

void InteractionTableList::setTableMultipleSelectable()
{
    setSelectionMode(QAbstractItemView::MultiSelection);
    setDisabled(false);
}

void InteractionTableList::setTableSelectionMode( eSelectionMode mode )
{
    selection_mode_ = mode;

    if ( selection_mode_ == eNoSelection )
        setTableNotSelectable();
    if ( selection_mode_ == eSingleSelection )
        setTableOneSelectable();
    if ( selection_mode_ == eMultiSelection )
        setTableMultipleSelectable();
}

void InteractionTableList::setRowSelected(int row)
{
    item(row, 0)->setSelected(true);
    item(row, 0)->setIcon(QIcon(":resources/arrowhead-right.png"));
}

void InteractionTableList::scrollTo(int row)
{
    scrollToItem(item(row,0));
}