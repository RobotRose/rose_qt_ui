/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/11/22
*         - File created.
*
* Description:
*    The 'operation successful window apears when an operation is finished.
* 
***********************************************************************************/
#ifndef OPERATION_SUCCESSFUL_HPP
#define OPERATION_SUCCESSFUL_HPP

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <QtGui/QApplication>
#include <QtGui/QDesktopWidget>
#include <QtGui/QWidget>
#include <QtGui/QCloseEvent>
#include <ros/ros.h>

#include "std_msgs/String.h"

#include "GuiDefinitions.h"

#include "ui_OperationSuccessful.h"

/**
 * The 'operation successful' window class. This window appears when the robots
 * wants to ask the operation if the operation was successful
 */
class OperationSuccessful : public QWidget
{
    Q_OBJECT

public:
    /**
     * Constructor.
     * @param[in] *parent
     */
    OperationSuccessful( QWidget *parent = 0 );

    /**
     * Destructor.
     */
    ~OperationSuccessful();

private:
    /**
     * Initializes the GUI.
     */
    void initGui();

    /**
    * Sets the background color.
    * @param[in] w QWidget, colorpalette
    * @param[in] c QColor, the background color
    */
    void setBackGroundColor( QWidget* w, QColor c );

    void informUserOnEmptyString();

    Ui::OperationSuccessfulClass ui; //!< OperationSuccessful window

    ros::Publisher save_item_publisher_;

private Q_SLOTS:
    /**
    * callBack function when the power off button is clicked
    */
	void on_m_ButtonPowerOff_clicked();

    void on_m_ButtenYes_clicked();
    void on_m_ButtenNo_clicked();
    void m_CheckBoxSaveItem_clicked();

};

//---------------------------------------------------------------------------------
#endif // OPERATION_SUCCESSFUL_HPP
