/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/11/22
* 		- File created.
*
* Description:
*	The 'connection lost´ window apears when the connection to Rose is lost. 
*	Implements connection_lost.hpp.
* 
***********************************************************************************/
#include "connection_lost/connection_lost.hpp"

ConnectionLost::ConnectionLost( QWidget *parent)
	:QWidget(parent)
{
	ui.setupUi(this);
	
	initGui();
	show();
}

ConnectionLost::~ConnectionLost()
{

}

void ConnectionLost::setBackGroundColor( QWidget* w, QColor c )
{
	QPalette pal = w->palette();
	pal.setColor( w->backgroundRole(), c );
	w->setPalette(pal);
}

void ConnectionLost::initGui()
{
	setWindowFlags( Qt::SplashScreen | Qt::WindowStaysOnTopHint | Qt::X11BypassWindowManagerHint );

	adjustSize();
	move(QApplication::desktop()->screen()->rect().center() - rect().center());

	setBackGroundColor( this, Qt::red );
	setBackGroundColor( ui.m_TopLayer, Qt::white );
}

void ConnectionLost::on_m_ButtonPowerOff_clicked()
{
	close();
}