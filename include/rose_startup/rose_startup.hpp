/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/16
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/

#ifndef ROSE_STARTUP_HPP
#define ROSE_STARTUP_HPP

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
//#include <QFocusEvent>
#include <QMouseEvent>
#include <QRgb>
#include <QScrollBar>
#include <QShortcut>
#include <QTableWidgetItem>
#include <QtCore/QEvent>
#include <QtGui/QWidget>
#include <string>
#include <vector>

#include "ApplicationServices/Customers.h"
#include "DependencyInjector.h"
#include "GuiDefinitions.h"
#include "TSRSlider/TSRSlider.h"

#include "rose_main/rose_main.hpp"

#include "ui_RoseStartup.h"

using namespace boost;
using namespace std;

#define CALL_EVENT         ((QEvent::Type)(QEvent::User + 2))
#define QUIT_EVENT         ((QEvent::Type)(QEvent::User + 4))

class RoseStartup : public QWidget
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<RoseStartup> Ptr ;
    RoseStartup( DependencyInjector::Ptr dependencies, QWidget *parent = 0);
    ~RoseStartup();    

    RoseMain *m_mainform;
private:
    enum {CustomerID=0, FullName=1, FullAddress=2, Photo=3 };
    static const int CUSTOMER_PHOTO_SIZE = 90 ;
    static const std::string DATABASE_FILENAME ;
    static const std::string GET_ALL_CUSTOMERS_QUERY ;
    void SetupCustomerTable();
    void SetNameAddressLabelText(QTableWidgetItem * item);
    void SetNameAddressLabelText(int column);
    QTableWidgetItem* CreatePhotoItem(const std::vector<unsigned char>& blob);
    void AppendPhotoItemToTheTable(QTableWidgetItem* item);
    void AddPhotoToTheCustomerTable(const std::vector<unsigned char>& blob);
    void AddPhotosOfCustomers();
    void InitGui();

    Ui::RoseStartupClass ui;
    boost::shared_ptr<DependencyInjector> m_dependencies;

    void CreateSliderForOperatorVolume();
    void CreateSliderForClientVolume();
    TSRSlider::Ptr m_OperatorVolumeSlider;
    TSRSlider::Ptr m_ClientVolumeSlider;

    EmptySliderSlave::Ptr m_EmptySliderSlaveOperator;
    EmptySliderSlave::Ptr m_EmptySliderSlaveClient;

    void loginToSelectedCustomer();

public Q_SLOTS:
    void mainform_destroyed();

private Q_SLOTS:

    void on_m_facesScrollTable_itemEntered ( QTableWidgetItem * item );
    void on_m_facesScrollTable_itemClicked ( QTableWidgetItem * item );
    void on_m_facesScrollTable_itemDoubleClicked ( QTableWidgetItem * item );

    void on_m_ButtonClose_clicked();

    void on_m_buttonNext_clicked();
    void on_m_buttonPrevious_clicked();
    void on_m_buttonCall_clicked();

	void on_m_buttonOperatorVolumeMute_toggled( bool checked );
	void on_m_buttonOperatorVolumeStepPlus_clicked();
	void on_m_buttonOperatorVolumeStepMin_clicked();

	void on_m_buttonClientVolumeMute_toggled( bool checked );
	void on_m_buttonClientVolumeStepPlus_clicked();
	void on_m_buttonClientVolumeStepMin_clicked();
};

#endif // ROSE_STARTUP_HPP
