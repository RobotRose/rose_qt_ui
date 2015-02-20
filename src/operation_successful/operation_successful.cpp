/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/11/22
* 		- File created.
*
* Description:
*    The 'operation successful window apears when an operation is finished.
*	Implements operation_successful.hpp.
* 
***********************************************************************************/
#include "operation_successful/operation_successful.hpp"

OperationSuccessful::OperationSuccessful( QWidget *parent)
	:QWidget(parent)
{
	ui.setupUi(this);

	// save_item_publisher_ = n_.advertise<operation_successful_checker::store_item>("operation_successful_checker/store", 5);
	initGui();
	show();
}

OperationSuccessful::~OperationSuccessful()
{

}

void OperationSuccessful::setBackGroundColor( QWidget* w, QColor c )
{
	QPalette pal = w->palette();
	pal.setColor( w->backgroundRole(), c );
	w->setPalette(pal);
}

void OperationSuccessful::initGui()
{
	setWindowFlags( Qt::SplashScreen | Qt::WindowStaysOnTopHint | Qt::X11BypassWindowManagerHint );

	adjustSize();
	move(QApplication::desktop()->screen()->rect().center() - rect().center());

	setBackGroundColor( this, Qt::red );
	setBackGroundColor( ui.m_TopLayer, Qt::white );
}

void OperationSuccessful::on_m_ButtonPowerOff_clicked()
{
	close();
}

void OperationSuccessful::on_m_ButtenYes_clicked()
{
	// operation_successful_checker::store_item store_item_msg;

	if ( ui.m_LineItemName->text().isEmpty() )
	{	
		informUserOnEmptyString();
		return;
	}

	// store_item_msg.store_item = (ui.m_CheckBoxSaveItem->checkState() != Qt::Checked);

	// save_item_publisher_.publish(store_item_msg);
}

void OperationSuccessful::on_m_ButtenNo_clicked()
{

}

void OperationSuccessful::m_CheckBoxSaveItem_clicked()
{
	// std_msgs::String text;
	// text.data = ui.m_LineItemName->text().toLocal8Bit().constData();
}

void OperationSuccessful::informUserOnEmptyString()
{
	ui.m_LineItemName->setText("Please fill in a name");
}