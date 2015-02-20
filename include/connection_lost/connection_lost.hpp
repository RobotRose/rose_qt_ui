/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/11/22
*         - File created.
*
* Description:
*    The 'connection lost´ window apears when the connection to Rose is lost.
* 
***********************************************************************************/
#ifndef CONNECTION_LOST_HPP
#define CONNECTION_LOST_HPP

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <QtGui/QApplication>
#include <QtGui/QDesktopWidget>
#include <QtGui/QWidget>
#include <QtGui/QCloseEvent>

#include "GuiDefinitions.h"

#include "ui_ConnectionLost.h"

/**
 * The 'connection lost' window class. This window appears when the connection to 
 * Rose is lost.
 */
class ConnectionLost : public QWidget
{
    Q_OBJECT

public:
    /**
     * Constructor.
     * @param[in] *parent
     */
    ConnectionLost( QWidget *parent = 0 );

    /**
     * Destructor.
     */
    ~ConnectionLost();

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

    Ui::ConnectionLostClass ui; //!< ConnectionLost window

private Q_SLOTS:
    /**
    * callBack function when the power off button is clicked
    */
	void on_m_ButtonPowerOff_clicked();

};

//---------------------------------------------------------------------------------
#endif // CONNECTION_LOST_HPP
