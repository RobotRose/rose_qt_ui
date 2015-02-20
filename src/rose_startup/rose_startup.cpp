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
#include "rose_startup/rose_startup.hpp"

RoseStartup::RoseStartup( DependencyInjector::Ptr dependencies, QWidget *parent)
    : QWidget(parent)
    , m_dependencies(dependencies)
{
    ui.setupUi(this);
    InitGui();
    AddPhotosOfCustomers();
    CreateSliderForOperatorVolume();
    CreateSliderForClientVolume();
}
//
//---------------------------------------------------------------------------------------------------------------------
//
RoseStartup::~RoseStartup()
{	
    std::cout << "RoseStartup::~RoseStartup " << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_ButtonClose_clicked()
{    
	close();
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::SetNameAddressLabelText(QTableWidgetItem * item)
{
    SetNameAddressLabelText(ui.m_facesScrollTable->column(item));
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::CreateSliderForOperatorVolume()
{
	m_EmptySliderSlaveOperator = EmptySliderSlave::Ptr( new EmptySliderSlave("Operator volume") );

    m_OperatorVolumeSlider = TSRSlider::Ptr( new TSRSlider( ui.m_ControlsPanelWidget, m_EmptySliderSlaveOperator ) );
    m_OperatorVolumeSlider->move( 47, 330 );
    m_OperatorVolumeSlider->resize( 271, 33 );
    m_OperatorVolumeSlider->SetSliderMode( Slider::smDirectFollow );
    m_OperatorVolumeSlider->SetTickEnable( false );
    m_OperatorVolumeSlider->show();
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::CreateSliderForClientVolume()
{
	m_EmptySliderSlaveClient = EmptySliderSlave::Ptr( new EmptySliderSlave("Client volume") );

	m_ClientVolumeSlider = TSRSlider::Ptr( new TSRSlider( ui.m_ControlsPanelWidget, m_EmptySliderSlaveClient ) );
	m_ClientVolumeSlider->move( 47, 400 );
	m_ClientVolumeSlider->resize( 271, 33 );
	m_ClientVolumeSlider->SetSliderMode( Slider::smDirectFollow );
	m_ClientVolumeSlider->SetTickEnable( false );
	m_ClientVolumeSlider->show();
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::SetNameAddressLabelText(int column)
{
	// TODO: Hey man: this is unreadable - refactor !
    QString name = QString((*(m_dependencies->TSRCustomers))[column].GetName().c_str());
    QString address = QString((*(m_dependencies->TSRCustomers))[column].GetAddress().c_str());
    address.replace(",", "\n");
    ui.m_labelCustomerName->setText(name + "\n" + address);
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::on_m_facesScrollTable_itemEntered ( QTableWidgetItem * item )
{
    SetNameAddressLabelText(item);
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::on_m_facesScrollTable_itemClicked ( QTableWidgetItem * item )
{
    SetNameAddressLabelText(item);
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::on_m_facesScrollTable_itemDoubleClicked ( QTableWidgetItem * item )
{
    SetNameAddressLabelText(item);

    loginToSelectedCustomer();
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::loginToSelectedCustomer()
{
    ROS_DEBUG_NAMED(ROS_NAME, "RoseStartup::loginToSelectedCustomer");

    // Check if customer is selected
    if(ui.m_facesScrollTable->currentItem() != NULL)
    {
        ROS_DEBUG_NAMED(ROS_NAME, "RoseStartup::loginToSelectedCustomer: Instantiating new RoseMain...");
        m_mainform = new RoseMain(m_dependencies, (*(m_dependencies->TSRCustomers))[ui.m_facesScrollTable->currentColumn()]);

        ROS_DEBUG_NAMED(ROS_NAME, "RoseStartup::loginToSelectedCustomer: Connecting signals");
        connect(m_mainform, SIGNAL(destroyed()), this, SLOT(mainform_destroyed()));

        ROS_DEBUG_NAMED(ROS_NAME, "RoseStartup::loginToSelectedCustomer: Showing new mainform");
        m_mainform->show();

        ROS_DEBUG_NAMED(ROS_NAME, "RoseStartup::loginToSelectedCustomer: Paranoid close?");
        m_mainform->setAttribute(Qt::WA_DeleteOnClose); // paranoid close
    }
    else
    {
        ui.m_labelCustomerName->setText("No customer selected.");
    }
}

//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::InitGui()
{

	ui.m_buttonNext->setIcon( QIcon(":resources/next.png") );
	ui.m_buttonPrevious->setIcon( QIcon(":resources/previous.png") );
	ui.m_labelRoseLogoSmallTopLeft->setPixmap( QPixmap(":resources/rose.png" ) );
	ui.m_labelRoseLogoSmallTopLeft->setScaledContents( true );
	ui.m_labelFulllogo->setPixmap( QPixmap(":resources/rose_toplogo.png" ) );
	ui.m_labelFulllogo->setScaledContents( true );
    QPalette palette;
    QColor color;
    color.setRgb( 244, 244, 244 );
    palette.setColor(QPalette::Background, color );
	ui.m_controlsPanel->setPalette(palette);
   
    this->showMaximized();

	ui.m_labelRoseNavigator->setText( "<font color=\"#008ED3\">ROSE Navigator</font>" );

	ui.m_facesScrollTable->setStyleSheet( ui.m_facesScrollTable->styleSheet() +  " QTableWidget::item:selected { background-color:transparent;  border-color:#008ED3;  border:4px inset #008ED3;  border-style: solid; } QTableWidget::item::icon { selection-color:transparent;   }    "  );

	ui.m_facesScrollTable->setMouseTracking (true) ;
	this->setMouseTracking (true) ;

	ui.m_labelCustomerName->setText("");

    ui.m_buttonCall->setStyleSheet( TsrDefinitions::Instance()->GetDefaultButtonStyleSheet().c_str() );
    ui.m_buttonCancel->setStyleSheet( TsrDefinitions::Instance()->GetDefaultButtonStyleSheet().c_str() );
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::SetupCustomerTable()
{
    ui.m_facesScrollTable->setRowCount(1);
    ui.m_facesScrollTable->setColumnCount(0);
    ui.m_facesScrollTable->horizontalHeader()->setHidden(true);
    ui.m_facesScrollTable->verticalHeader()->setHidden(true);
    ui.m_facesScrollTable->setIconSize(QSize(CUSTOMER_PHOTO_SIZE-15, CUSTOMER_PHOTO_SIZE-15));
    ui.m_facesScrollTable->horizontalScrollBar()->setHidden(true);
    ui.m_facesScrollTable->verticalScrollBar()->setHidden(true);
    ui.m_facesScrollTable->setRowHeight(0, CUSTOMER_PHOTO_SIZE);
}
//
//---------------------------------------------------------------------------------------------------------------------
//
QTableWidgetItem* RoseStartup::CreatePhotoItem(const vector<unsigned char>& blob)
{
    QTableWidgetItem *item = new QTableWidgetItem();
    QPixmap pixmap;
    pixmap.loadFromData(&blob[0], blob.size());
    item->setIcon(QIcon(pixmap));
    return item;
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::AppendPhotoItemToTheTable(QTableWidgetItem* item)
{
    ui.m_facesScrollTable->setColumnCount(ui.m_facesScrollTable->columnCount() + 1);
    ui.m_facesScrollTable->setColumnWidth(ui.m_facesScrollTable->columnCount() - 1, CUSTOMER_PHOTO_SIZE);
    ui.m_facesScrollTable->setItem(0, ui.m_facesScrollTable->columnCount() - 1, item);
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::AddPhotoToTheCustomerTable(const vector<unsigned char>& blob)
{
    AppendPhotoItemToTheTable( CreatePhotoItem(blob) );
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::AddPhotosOfCustomers()
{
    SetupCustomerTable();

    m_dependencies->TSRCustomers->Discover();

    BOOST_FOREACH(ApplicationServices::Customer customer, *(m_dependencies->TSRCustomers) )
    {
        AddPhotoToTheCustomerTable(customer.GetPicture());
    }
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::on_m_buttonNext_clicked()
{
    int cidx = ui.m_facesScrollTable->currentColumn();
    if( ++cidx < ui.m_facesScrollTable->columnCount() )
    {
        ui.m_facesScrollTable->setCurrentCell( 0, cidx );
        SetNameAddressLabelText(cidx);
        // ui.m_buttonPrevious->setDisabled(false);

        // if( cidx == ui.m_facesScrollTable->columnCount()-1 )
        // {
        //    ui.m_buttonNext->setDisabled(true);
        // }
    }
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseStartup::on_m_buttonPrevious_clicked()
{
    int cidx = ui.m_facesScrollTable->currentColumn();
    if( --cidx >= 0 )
    {
        ui.m_facesScrollTable->setCurrentCell( 0, cidx );
        SetNameAddressLabelText(cidx);
        // ui.m_buttonNext->setDisabled(false);

        // if(cidx==0)
        // {
        //     ui.m_buttonPrevious->setDisabled(true);
        // }
    }
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonCall_clicked()
{
    loginToSelectedCustomer();
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::mainform_destroyed()
{
    m_mainform = NULL;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonOperatorVolumeMute_toggled ( bool checked )
{
	std::cout << "on_m_buttonOperatorVolumeMute_toggled to " << ( checked ? "checked" : "unchecked" ) << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonOperatorVolumeStepPlus_clicked()
{
	std::cout << "on_m_buttonOperatorVolumeStepPlus_pressed" << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonOperatorVolumeStepMin_clicked()
{
	std::cout << "on_m_buttonOperatorVolumeStepMin_pressed" << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonClientVolumeMute_toggled( bool checked )
{
	std::cout << "on_m_buttonClientVolumeMute_toggled to " << ( checked ? "checked" : "unchecked" ) << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonClientVolumeStepPlus_clicked()
{
	std::cout << "on_m_buttonClientVolumeStepPlus_clicked" << std::endl;
}
//---------------------------------------------------------------------------------
//
//
void RoseStartup::on_m_buttonClientVolumeStepMin_clicked()
{
	std::cout << "on_m_buttonClientVolumeStepMin_clicked" << std::endl;
}
//---------------------------------------------------------------------------------
//
//
