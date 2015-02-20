/********************************************************************************
** Form generated from reading UI file 'RoseMain.ui'
**
** Created: Fri May 2 09:59:14 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROSEMAIN_H
#define UI_ROSEMAIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RoseMainClass
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *m_labelRoseLogoSmallTopLeft;
    QWidget *widget_4;
    QWidget *widget_2;
    QLabel *m_labelRoseNavigator;
    QWidget *m_widgetConnectionStatus;
    QLabel *m_labelDateTime;
    QLabel *m_labelAlignment;
    QWidget *m_mainContainer;
    QHBoxLayout *horizontalLayout_2;
    QWidget *m_leftContainer;
    QVBoxLayout *verticalLayout_4;
    QTabWidget *m_GeneralControlsTabWidget;
    QWidget *m_GeneralControlsPage;
    QPushButton *m_ButtonHelp;
    QPushButton *m_ButtonClose;
    QPushButton *m_ButtonPause;
    QLabel *label_2;
    QFrame *line;
    QPushButton *m_ButtonTelepresence;
    QTabWidget *m_CommunicationsTabWidget;
    QWidget *m_tabCommunication;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_4;
    QPushButton *m_buttonOperatorVolumeStepMin;
    QHBoxLayout *m_OperatorVoiceSliderLayout;
    QPushButton *m_buttonOperatorVolumeStepPlus;
    QPushButton *m_buttonOperatorVolumeMute;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_7;
    QPushButton *m_buttonClientVolumeStepMin;
    QHBoxLayout *m_ClientVoiceSliderLayout;
    QPushButton *m_buttonClientVolumeStepPlus;
    QPushButton *m_buttonClientVolumeMute;
    QHBoxLayout *horizontalLayout_14;
    QLabel *labelClientName;
    QHBoxLayout *horizontalLayout_24;
    QLabel *labelClientImage;
    QTextEdit *textClientInfo;
    QTabWidget *m_askUserTextWindow;
    QWidget *m_askUserTextWidget;
    QVBoxLayout *verticalLayout_16;
    QVBoxLayout *verticalLayout_15;
    QTabWidget *m_interactionWindow;
    QWidget *m_interactionWidget;
    QVBoxLayout *verticalLayout_14;
    QVBoxLayout *verticalLayout_11;
    QWidget *m_midContainer;
    QVBoxLayout *verticalLayout_3;
    QTabWidget *m_tabCameras;
    QWidget *tab_6;
    QVBoxLayout *verticalLayout_8;
    QWidget *m_cameraOverView;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_6;
    QSpacerItem *horizontalSpacer_8;
    QCheckBox *m_showOperatorCamCheckBox;
    QTabWidget *m_HouseLayoutTabWidget;
    QWidget *m_tabHouseLayout;
    QVBoxLayout *verticalLayout_10;
    QWidget *m_HouseLayoutView;
    QVBoxLayout *verticalLayout_21;
    QVBoxLayout *m_HouseLayoutViewLayout;
    QWidget *widget_26;
    QVBoxLayout *verticalLayout_19;
    QHBoxLayout *horizontalLayout_9;
    QRadioButton *navigationCheckbox;
    QRadioButton *posEstimateCheckbox;
    QSpacerItem *horizontalSpacer_25;
    QWidget *m_rightContainer;
    QVBoxLayout *verticalLayout_2;
    QTabWidget *m_tabGripperControl;
    QWidget *m_tabHandControl;
    QVBoxLayout *verticalLayout_6;
    QWidget *m_cameraHand;
    QVBoxLayout *verticalLayout_9;
    QFrame *line_4;
    QLabel *label_9;
    QWidget *m_HandGripperOpenCloseControls;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_6;
    QLabel *m_OpenGripperLabel;
    QSpacerItem *horizontalSpacer_11;
    QWidget *widget_5;
    QHBoxLayout *horizontalLayout_16;
    QHBoxLayout *m_OpenCloseSliderLayout;
    QLabel *m_ClosedGripperLabel;
    QSpacerItem *horizontalSpacer;
    QFrame *line_3;
    QLabel *label_10;
    QWidget *m_HandGripperPressureControls;
    QHBoxLayout *horizontalLayout_5;
    QSpacerItem *horizontalSpacer_7;
    QLabel *m_PressureGripperLabel;
    QSpacerItem *horizontalSpacer_9;
    QVBoxLayout *verticalLayout_13;
    QVBoxLayout *verticalLayout_5;
    QWidget *widget_6;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_2;
    QLabel *m_EggLabel;
    QSpacerItem *horizontalSpacer_3;
    QLabel *m_CupLabel;
    QSpacerItem *horizontalSpacer_4;
    QLabel *m_WeightLabel;
    QSpacerItem *horizontalSpacer_10;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *m_PressureSliderLayout;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *m_ButtonStop;
    QTabWidget *m_messagesWindow;
    QWidget *m_messagesWidget;
    QVBoxLayout *verticalLayout_18;
    QVBoxLayout *verticalLayout_17;
    QHBoxLayout *horizontalLayout_25;
    QTabWidget *m_manual_platform_controlWindow;
    QWidget *m_manual_platform_controlWidget;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;

    void setupUi(QWidget *RoseMainClass)
    {
        if (RoseMainClass->objectName().isEmpty())
            RoseMainClass->setObjectName(QString::fromUtf8("RoseMainClass"));
        RoseMainClass->resize(1800, 1000);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(RoseMainClass->sizePolicy().hasHeightForWidth());
        RoseMainClass->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(RoseMainClass);
        verticalLayout->setSpacing(0);
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        m_labelRoseLogoSmallTopLeft = new QLabel(RoseMainClass);
        m_labelRoseLogoSmallTopLeft->setObjectName(QString::fromUtf8("m_labelRoseLogoSmallTopLeft"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(m_labelRoseLogoSmallTopLeft->sizePolicy().hasHeightForWidth());
        m_labelRoseLogoSmallTopLeft->setSizePolicy(sizePolicy1);
        m_labelRoseLogoSmallTopLeft->setMinimumSize(QSize(25, 25));
        m_labelRoseLogoSmallTopLeft->setMaximumSize(QSize(25, 25));

        horizontalLayout->addWidget(m_labelRoseLogoSmallTopLeft);

        widget_4 = new QWidget(RoseMainClass);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        sizePolicy1.setHeightForWidth(widget_4->sizePolicy().hasHeightForWidth());
        widget_4->setSizePolicy(sizePolicy1);
        widget_4->setMinimumSize(QSize(4, 2));
        widget_4->setMaximumSize(QSize(4, 25));

        horizontalLayout->addWidget(widget_4);

        widget_2 = new QWidget(RoseMainClass);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy2);
        m_labelRoseNavigator = new QLabel(widget_2);
        m_labelRoseNavigator->setObjectName(QString::fromUtf8("m_labelRoseNavigator"));
        m_labelRoseNavigator->setGeometry(QRect(0, 0, 150, 25));
        m_labelRoseNavigator->setMinimumSize(QSize(150, 0));
        m_labelRoseNavigator->setMaximumSize(QSize(150, 16777215));
        QFont font;
        font.setFamily(QString::fromUtf8("Arial"));
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        m_labelRoseNavigator->setFont(font);

        horizontalLayout->addWidget(widget_2);

        m_widgetConnectionStatus = new QWidget(RoseMainClass);
        m_widgetConnectionStatus->setObjectName(QString::fromUtf8("m_widgetConnectionStatus"));
        m_widgetConnectionStatus->setMinimumSize(QSize(60, 0));
        m_widgetConnectionStatus->setMaximumSize(QSize(60, 16777215));

        horizontalLayout->addWidget(m_widgetConnectionStatus);

        m_labelDateTime = new QLabel(RoseMainClass);
        m_labelDateTime->setObjectName(QString::fromUtf8("m_labelDateTime"));
        m_labelDateTime->setMinimumSize(QSize(200, 0));
        m_labelDateTime->setMaximumSize(QSize(200, 16777215));
        m_labelDateTime->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout->addWidget(m_labelDateTime);

        m_labelAlignment = new QLabel(RoseMainClass);
        m_labelAlignment->setObjectName(QString::fromUtf8("m_labelAlignment"));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(m_labelAlignment->sizePolicy().hasHeightForWidth());
        m_labelAlignment->setSizePolicy(sizePolicy3);
        m_labelAlignment->setMinimumSize(QSize(10, 0));
        m_labelAlignment->setMaximumSize(QSize(10, 16777215));

        horizontalLayout->addWidget(m_labelAlignment);


        verticalLayout->addLayout(horizontalLayout);

        m_mainContainer = new QWidget(RoseMainClass);
        m_mainContainer->setObjectName(QString::fromUtf8("m_mainContainer"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(m_mainContainer->sizePolicy().hasHeightForWidth());
        m_mainContainer->setSizePolicy(sizePolicy4);
        horizontalLayout_2 = new QHBoxLayout(m_mainContainer);
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        m_leftContainer = new QWidget(m_mainContainer);
        m_leftContainer->setObjectName(QString::fromUtf8("m_leftContainer"));
        verticalLayout_4 = new QVBoxLayout(m_leftContainer);
        verticalLayout_4->setSpacing(5);
        verticalLayout_4->setContentsMargins(5, 5, 5, 5);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        m_GeneralControlsTabWidget = new QTabWidget(m_leftContainer);
        m_GeneralControlsTabWidget->setObjectName(QString::fromUtf8("m_GeneralControlsTabWidget"));
        QSizePolicy sizePolicy5(QSizePolicy::Expanding, QSizePolicy::Minimum);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(m_GeneralControlsTabWidget->sizePolicy().hasHeightForWidth());
        m_GeneralControlsTabWidget->setSizePolicy(sizePolicy5);
        m_GeneralControlsTabWidget->setMinimumSize(QSize(0, 180));
        m_GeneralControlsTabWidget->setMaximumSize(QSize(16777215, 16777215));
        m_GeneralControlsPage = new QWidget();
        m_GeneralControlsPage->setObjectName(QString::fromUtf8("m_GeneralControlsPage"));
        m_ButtonHelp = new QPushButton(m_GeneralControlsPage);
        m_ButtonHelp->setObjectName(QString::fromUtf8("m_ButtonHelp"));
        m_ButtonHelp->setGeometry(QRect(30, 50, 80, 26));
        m_ButtonClose = new QPushButton(m_GeneralControlsPage);
        m_ButtonClose->setObjectName(QString::fromUtf8("m_ButtonClose"));
        m_ButtonClose->setGeometry(QRect(30, 10, 80, 26));
        m_ButtonPause = new QPushButton(m_GeneralControlsPage);
        m_ButtonPause->setObjectName(QString::fromUtf8("m_ButtonPause"));
        m_ButtonPause->setGeometry(QRect(120, 10, 80, 26));
        label_2 = new QLabel(m_GeneralControlsPage);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(250, 10, 57, 15));
        line = new QFrame(m_GeneralControlsPage);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(220, 10, 20, 131));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        m_ButtonTelepresence = new QPushButton(m_GeneralControlsPage);
        m_ButtonTelepresence->setObjectName(QString::fromUtf8("m_ButtonTelepresence"));
        m_ButtonTelepresence->setGeometry(QRect(32, 90, 161, 27));
        m_GeneralControlsTabWidget->addTab(m_GeneralControlsPage, QString());

        verticalLayout_4->addWidget(m_GeneralControlsTabWidget);

        m_CommunicationsTabWidget = new QTabWidget(m_leftContainer);
        m_CommunicationsTabWidget->setObjectName(QString::fromUtf8("m_CommunicationsTabWidget"));
        m_CommunicationsTabWidget->setMaximumSize(QSize(16777215, 500));
        m_tabCommunication = new QWidget();
        m_tabCommunication->setObjectName(QString::fromUtf8("m_tabCommunication"));
        verticalLayout_12 = new QVBoxLayout(m_tabCommunication);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_4 = new QLabel(m_tabCommunication);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMinimumSize(QSize(100, 0));
        label_4->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_11->addWidget(label_4);

        m_buttonOperatorVolumeStepMin = new QPushButton(m_tabCommunication);
        m_buttonOperatorVolumeStepMin->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeStepMin"));
        m_buttonOperatorVolumeStepMin->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeStepMin->setMaximumSize(QSize(30, 30));

        horizontalLayout_11->addWidget(m_buttonOperatorVolumeStepMin);

        m_OperatorVoiceSliderLayout = new QHBoxLayout();
        m_OperatorVoiceSliderLayout->setSpacing(6);
        m_OperatorVoiceSliderLayout->setObjectName(QString::fromUtf8("m_OperatorVoiceSliderLayout"));

        horizontalLayout_11->addLayout(m_OperatorVoiceSliderLayout);

        m_buttonOperatorVolumeStepPlus = new QPushButton(m_tabCommunication);
        m_buttonOperatorVolumeStepPlus->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeStepPlus"));
        m_buttonOperatorVolumeStepPlus->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeStepPlus->setMaximumSize(QSize(30, 30));

        horizontalLayout_11->addWidget(m_buttonOperatorVolumeStepPlus);

        m_buttonOperatorVolumeMute = new QPushButton(m_tabCommunication);
        m_buttonOperatorVolumeMute->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeMute"));
        m_buttonOperatorVolumeMute->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeMute->setMaximumSize(QSize(30, 30));
        m_buttonOperatorVolumeMute->setCheckable(true);

        horizontalLayout_11->addWidget(m_buttonOperatorVolumeMute);


        verticalLayout_12->addLayout(horizontalLayout_11);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_7 = new QLabel(m_tabCommunication);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setMinimumSize(QSize(100, 0));
        label_7->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_13->addWidget(label_7);

        m_buttonClientVolumeStepMin = new QPushButton(m_tabCommunication);
        m_buttonClientVolumeStepMin->setObjectName(QString::fromUtf8("m_buttonClientVolumeStepMin"));
        m_buttonClientVolumeStepMin->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeStepMin->setMaximumSize(QSize(30, 30));

        horizontalLayout_13->addWidget(m_buttonClientVolumeStepMin);

        m_ClientVoiceSliderLayout = new QHBoxLayout();
        m_ClientVoiceSliderLayout->setSpacing(6);
        m_ClientVoiceSliderLayout->setObjectName(QString::fromUtf8("m_ClientVoiceSliderLayout"));

        horizontalLayout_13->addLayout(m_ClientVoiceSliderLayout);

        m_buttonClientVolumeStepPlus = new QPushButton(m_tabCommunication);
        m_buttonClientVolumeStepPlus->setObjectName(QString::fromUtf8("m_buttonClientVolumeStepPlus"));
        m_buttonClientVolumeStepPlus->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeStepPlus->setMaximumSize(QSize(30, 30));

        horizontalLayout_13->addWidget(m_buttonClientVolumeStepPlus);

        m_buttonClientVolumeMute = new QPushButton(m_tabCommunication);
        m_buttonClientVolumeMute->setObjectName(QString::fromUtf8("m_buttonClientVolumeMute"));
        m_buttonClientVolumeMute->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeMute->setMaximumSize(QSize(30, 30));
        m_buttonClientVolumeMute->setCheckable(true);

        horizontalLayout_13->addWidget(m_buttonClientVolumeMute);


        verticalLayout_12->addLayout(horizontalLayout_13);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        labelClientName = new QLabel(m_tabCommunication);
        labelClientName->setObjectName(QString::fromUtf8("labelClientName"));
        labelClientName->setMinimumSize(QSize(100, 0));
        labelClientName->setMaximumSize(QSize(1200, 16777215));
        labelClientName->setAlignment(Qt::AlignCenter);

        horizontalLayout_14->addWidget(labelClientName);


        verticalLayout_12->addLayout(horizontalLayout_14);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        labelClientImage = new QLabel(m_tabCommunication);
        labelClientImage->setObjectName(QString::fromUtf8("labelClientImage"));
        labelClientImage->setMinimumSize(QSize(148, 148));
        labelClientImage->setMaximumSize(QSize(148, 148));

        horizontalLayout_24->addWidget(labelClientImage);

        textClientInfo = new QTextEdit(m_tabCommunication);
        textClientInfo->setObjectName(QString::fromUtf8("textClientInfo"));
        textClientInfo->setMinimumSize(QSize(0, 0));
        textClientInfo->setMaximumSize(QSize(1200, 1200));

        horizontalLayout_24->addWidget(textClientInfo);


        verticalLayout_12->addLayout(horizontalLayout_24);

        m_CommunicationsTabWidget->addTab(m_tabCommunication, QString());

        verticalLayout_4->addWidget(m_CommunicationsTabWidget);

        m_askUserTextWindow = new QTabWidget(m_leftContainer);
        m_askUserTextWindow->setObjectName(QString::fromUtf8("m_askUserTextWindow"));
        QSizePolicy sizePolicy6(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(m_askUserTextWindow->sizePolicy().hasHeightForWidth());
        m_askUserTextWindow->setSizePolicy(sizePolicy6);
        m_askUserTextWindow->setMinimumSize(QSize(0, 80));
        m_askUserTextWidget = new QWidget();
        m_askUserTextWidget->setObjectName(QString::fromUtf8("m_askUserTextWidget"));
        verticalLayout_16 = new QVBoxLayout(m_askUserTextWidget);
        verticalLayout_16->setSpacing(6);
        verticalLayout_16->setContentsMargins(11, 11, 11, 11);
        verticalLayout_16->setObjectName(QString::fromUtf8("verticalLayout_16"));
        verticalLayout_15 = new QVBoxLayout();
        verticalLayout_15->setSpacing(6);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));

        verticalLayout_16->addLayout(verticalLayout_15);

        m_askUserTextWindow->addTab(m_askUserTextWidget, QString());

        verticalLayout_4->addWidget(m_askUserTextWindow);

        m_interactionWindow = new QTabWidget(m_leftContainer);
        m_interactionWindow->setObjectName(QString::fromUtf8("m_interactionWindow"));
        m_interactionWindow->setMinimumSize(QSize(0, 0));
        m_interactionWindow->setMaximumSize(QSize(16777215, 16777215));
        m_interactionWindow->setContextMenuPolicy(Qt::DefaultContextMenu);
        m_interactionWidget = new QWidget();
        m_interactionWidget->setObjectName(QString::fromUtf8("m_interactionWidget"));
        m_interactionWidget->setEnabled(true);
        sizePolicy.setHeightForWidth(m_interactionWidget->sizePolicy().hasHeightForWidth());
        m_interactionWidget->setSizePolicy(sizePolicy);
        m_interactionWidget->setMinimumSize(QSize(0, 0));
        m_interactionWidget->setMaximumSize(QSize(16777215, 16777215));
        m_interactionWidget->setContextMenuPolicy(Qt::DefaultContextMenu);
        m_interactionWidget->setLayoutDirection(Qt::LeftToRight);
        m_interactionWidget->setAutoFillBackground(false);
        verticalLayout_14 = new QVBoxLayout(m_interactionWidget);
        verticalLayout_14->setSpacing(6);
        verticalLayout_14->setContentsMargins(11, 11, 11, 11);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));

        verticalLayout_14->addLayout(verticalLayout_11);

        m_interactionWindow->addTab(m_interactionWidget, QString());

        verticalLayout_4->addWidget(m_interactionWindow);


        horizontalLayout_2->addWidget(m_leftContainer);

        m_midContainer = new QWidget(m_mainContainer);
        m_midContainer->setObjectName(QString::fromUtf8("m_midContainer"));
        verticalLayout_3 = new QVBoxLayout(m_midContainer);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        m_tabCameras = new QTabWidget(m_midContainer);
        m_tabCameras->setObjectName(QString::fromUtf8("m_tabCameras"));
        sizePolicy5.setHeightForWidth(m_tabCameras->sizePolicy().hasHeightForWidth());
        m_tabCameras->setSizePolicy(sizePolicy5);
        m_tabCameras->setMinimumSize(QSize(640, 0));
        m_tabCameras->setMaximumSize(QSize(660, 16777215));
        tab_6 = new QWidget();
        tab_6->setObjectName(QString::fromUtf8("tab_6"));
        verticalLayout_8 = new QVBoxLayout(tab_6);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        m_cameraOverView = new QWidget(tab_6);
        m_cameraOverView->setObjectName(QString::fromUtf8("m_cameraOverView"));
        sizePolicy1.setHeightForWidth(m_cameraOverView->sizePolicy().hasHeightForWidth());
        m_cameraOverView->setSizePolicy(sizePolicy1);
        m_cameraOverView->setMinimumSize(QSize(640, 480));
        m_cameraOverView->setMaximumSize(QSize(640, 480));
        verticalLayout_7 = new QVBoxLayout(m_cameraOverView);
        verticalLayout_7->setSpacing(0);
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));

        verticalLayout_8->addWidget(m_cameraOverView);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_8);

        m_showOperatorCamCheckBox = new QCheckBox(tab_6);
        m_showOperatorCamCheckBox->setObjectName(QString::fromUtf8("m_showOperatorCamCheckBox"));
        QSizePolicy sizePolicy7(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy7.setHorizontalStretch(0);
        sizePolicy7.setVerticalStretch(0);
        sizePolicy7.setHeightForWidth(m_showOperatorCamCheckBox->sizePolicy().hasHeightForWidth());
        m_showOperatorCamCheckBox->setSizePolicy(sizePolicy7);
        m_showOperatorCamCheckBox->setMinimumSize(QSize(150, 0));
        m_showOperatorCamCheckBox->setMaximumSize(QSize(200, 16777215));
        m_showOperatorCamCheckBox->setLayoutDirection(Qt::LeftToRight);
        m_showOperatorCamCheckBox->setChecked(true);

        horizontalLayout_6->addWidget(m_showOperatorCamCheckBox);


        verticalLayout_8->addLayout(horizontalLayout_6);

        m_tabCameras->addTab(tab_6, QString());

        verticalLayout_3->addWidget(m_tabCameras);

        m_HouseLayoutTabWidget = new QTabWidget(m_midContainer);
        m_HouseLayoutTabWidget->setObjectName(QString::fromUtf8("m_HouseLayoutTabWidget"));
        sizePolicy.setHeightForWidth(m_HouseLayoutTabWidget->sizePolicy().hasHeightForWidth());
        m_HouseLayoutTabWidget->setSizePolicy(sizePolicy);
        m_HouseLayoutTabWidget->setMinimumSize(QSize(320, 400));
        m_HouseLayoutTabWidget->setMaximumSize(QSize(660, 550));
        m_tabHouseLayout = new QWidget();
        m_tabHouseLayout->setObjectName(QString::fromUtf8("m_tabHouseLayout"));
        m_tabHouseLayout->setMinimumSize(QSize(0, 300));
        m_tabHouseLayout->setMaximumSize(QSize(16777215, 550));
        verticalLayout_10 = new QVBoxLayout(m_tabHouseLayout);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        m_HouseLayoutView = new QWidget(m_tabHouseLayout);
        m_HouseLayoutView->setObjectName(QString::fromUtf8("m_HouseLayoutView"));
        QSizePolicy sizePolicy8(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy8.setHorizontalStretch(0);
        sizePolicy8.setVerticalStretch(0);
        sizePolicy8.setHeightForWidth(m_HouseLayoutView->sizePolicy().hasHeightForWidth());
        m_HouseLayoutView->setSizePolicy(sizePolicy8);
        m_HouseLayoutView->setMaximumSize(QSize(16777215, 380));
        verticalLayout_21 = new QVBoxLayout(m_HouseLayoutView);
        verticalLayout_21->setSpacing(0);
        verticalLayout_21->setContentsMargins(0, 0, 0, 0);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        m_HouseLayoutViewLayout = new QVBoxLayout();
        m_HouseLayoutViewLayout->setSpacing(6);
        m_HouseLayoutViewLayout->setObjectName(QString::fromUtf8("m_HouseLayoutViewLayout"));

        verticalLayout_21->addLayout(m_HouseLayoutViewLayout);


        verticalLayout_10->addWidget(m_HouseLayoutView);

        widget_26 = new QWidget(m_tabHouseLayout);
        widget_26->setObjectName(QString::fromUtf8("widget_26"));
        QSizePolicy sizePolicy9(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy9.setHorizontalStretch(0);
        sizePolicy9.setVerticalStretch(0);
        sizePolicy9.setHeightForWidth(widget_26->sizePolicy().hasHeightForWidth());
        widget_26->setSizePolicy(sizePolicy9);
        widget_26->setMinimumSize(QSize(0, 0));
        widget_26->setMaximumSize(QSize(16777215, 25));
        verticalLayout_19 = new QVBoxLayout(widget_26);
        verticalLayout_19->setSpacing(6);
        verticalLayout_19->setContentsMargins(0, 0, 0, 0);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        navigationCheckbox = new QRadioButton(widget_26);
        navigationCheckbox->setObjectName(QString::fromUtf8("navigationCheckbox"));

        horizontalLayout_9->addWidget(navigationCheckbox);

        posEstimateCheckbox = new QRadioButton(widget_26);
        posEstimateCheckbox->setObjectName(QString::fromUtf8("posEstimateCheckbox"));

        horizontalLayout_9->addWidget(posEstimateCheckbox);

        horizontalSpacer_25 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_25);


        verticalLayout_19->addLayout(horizontalLayout_9);


        verticalLayout_10->addWidget(widget_26);

        m_HouseLayoutTabWidget->addTab(m_tabHouseLayout, QString());

        verticalLayout_3->addWidget(m_HouseLayoutTabWidget);


        horizontalLayout_2->addWidget(m_midContainer);

        m_rightContainer = new QWidget(m_mainContainer);
        m_rightContainer->setObjectName(QString::fromUtf8("m_rightContainer"));
        m_rightContainer->setEnabled(true);
        verticalLayout_2 = new QVBoxLayout(m_rightContainer);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        m_tabGripperControl = new QTabWidget(m_rightContainer);
        m_tabGripperControl->setObjectName(QString::fromUtf8("m_tabGripperControl"));
        m_tabGripperControl->setMinimumSize(QSize(0, 225));
        m_tabGripperControl->setMaximumSize(QSize(16777215, 16777215));
        m_tabHandControl = new QWidget();
        m_tabHandControl->setObjectName(QString::fromUtf8("m_tabHandControl"));
        verticalLayout_6 = new QVBoxLayout(m_tabHandControl);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        m_cameraHand = new QWidget(m_tabHandControl);
        m_cameraHand->setObjectName(QString::fromUtf8("m_cameraHand"));
        sizePolicy1.setHeightForWidth(m_cameraHand->sizePolicy().hasHeightForWidth());
        m_cameraHand->setSizePolicy(sizePolicy1);
        m_cameraHand->setMinimumSize(QSize(320, 240));
        m_cameraHand->setMaximumSize(QSize(320, 240));
        verticalLayout_9 = new QVBoxLayout(m_cameraHand);
        verticalLayout_9->setSpacing(0);
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));

        verticalLayout_6->addWidget(m_cameraHand);

        line_4 = new QFrame(m_tabHandControl);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout_6->addWidget(line_4);

        label_9 = new QLabel(m_tabHandControl);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setMaximumSize(QSize(16777215, 22));

        verticalLayout_6->addWidget(label_9);

        m_HandGripperOpenCloseControls = new QWidget(m_tabHandControl);
        m_HandGripperOpenCloseControls->setObjectName(QString::fromUtf8("m_HandGripperOpenCloseControls"));
        m_HandGripperOpenCloseControls->setMaximumSize(QSize(16777215, 75));
        horizontalLayout_4 = new QHBoxLayout(m_HandGripperOpenCloseControls);
        horizontalLayout_4->setSpacing(0);
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_6);

        m_OpenGripperLabel = new QLabel(m_HandGripperOpenCloseControls);
        m_OpenGripperLabel->setObjectName(QString::fromUtf8("m_OpenGripperLabel"));
        sizePolicy3.setHeightForWidth(m_OpenGripperLabel->sizePolicy().hasHeightForWidth());
        m_OpenGripperLabel->setSizePolicy(sizePolicy3);
        m_OpenGripperLabel->setMinimumSize(QSize(55, 55));
        m_OpenGripperLabel->setMaximumSize(QSize(55, 55));

        horizontalLayout_4->addWidget(m_OpenGripperLabel);

        horizontalSpacer_11 = new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_11);

        widget_5 = new QWidget(m_HandGripperOpenCloseControls);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        sizePolicy3.setHeightForWidth(widget_5->sizePolicy().hasHeightForWidth());
        widget_5->setSizePolicy(sizePolicy3);
        widget_5->setMinimumSize(QSize(300, 33));
        widget_5->setMaximumSize(QSize(300, 33));
        horizontalLayout_16 = new QHBoxLayout(widget_5);
        horizontalLayout_16->setSpacing(0);
        horizontalLayout_16->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        m_OpenCloseSliderLayout = new QHBoxLayout();
        m_OpenCloseSliderLayout->setSpacing(6);
        m_OpenCloseSliderLayout->setObjectName(QString::fromUtf8("m_OpenCloseSliderLayout"));

        horizontalLayout_16->addLayout(m_OpenCloseSliderLayout);


        horizontalLayout_4->addWidget(widget_5);

        m_ClosedGripperLabel = new QLabel(m_HandGripperOpenCloseControls);
        m_ClosedGripperLabel->setObjectName(QString::fromUtf8("m_ClosedGripperLabel"));
        sizePolicy3.setHeightForWidth(m_ClosedGripperLabel->sizePolicy().hasHeightForWidth());
        m_ClosedGripperLabel->setSizePolicy(sizePolicy3);
        m_ClosedGripperLabel->setMinimumSize(QSize(55, 0));
        m_ClosedGripperLabel->setMaximumSize(QSize(55, 55));

        horizontalLayout_4->addWidget(m_ClosedGripperLabel);

        horizontalSpacer = new QSpacerItem(41, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);


        verticalLayout_6->addWidget(m_HandGripperOpenCloseControls);

        line_3 = new QFrame(m_tabHandControl);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout_6->addWidget(line_3);

        label_10 = new QLabel(m_tabHandControl);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setMaximumSize(QSize(16777215, 22));

        verticalLayout_6->addWidget(label_10);

        m_HandGripperPressureControls = new QWidget(m_tabHandControl);
        m_HandGripperPressureControls->setObjectName(QString::fromUtf8("m_HandGripperPressureControls"));
        m_HandGripperPressureControls->setMaximumSize(QSize(16777215, 75));
        horizontalLayout_5 = new QHBoxLayout(m_HandGripperPressureControls);
        horizontalLayout_5->setSpacing(0);
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_7);

        m_PressureGripperLabel = new QLabel(m_HandGripperPressureControls);
        m_PressureGripperLabel->setObjectName(QString::fromUtf8("m_PressureGripperLabel"));
        m_PressureGripperLabel->setMinimumSize(QSize(55, 55));
        m_PressureGripperLabel->setMaximumSize(QSize(55, 55));

        horizontalLayout_5->addWidget(m_PressureGripperLabel);

        horizontalSpacer_9 = new QSpacerItem(10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_9);

        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));

        horizontalLayout_5->addLayout(verticalLayout_13);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        widget_6 = new QWidget(m_HandGripperPressureControls);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        widget_6->setMinimumSize(QSize(300, 0));
        widget_6->setMaximumSize(QSize(300, 16777215));
        horizontalLayout_3 = new QHBoxLayout(widget_6);
        horizontalLayout_3->setSpacing(0);
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        m_EggLabel = new QLabel(widget_6);
        m_EggLabel->setObjectName(QString::fromUtf8("m_EggLabel"));
        QSizePolicy sizePolicy10(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy10.setHorizontalStretch(0);
        sizePolicy10.setVerticalStretch(0);
        sizePolicy10.setHeightForWidth(m_EggLabel->sizePolicy().hasHeightForWidth());
        m_EggLabel->setSizePolicy(sizePolicy10);
        m_EggLabel->setMinimumSize(QSize(0, 33));
        m_EggLabel->setMaximumSize(QSize(16777215, 33));

        horizontalLayout_3->addWidget(m_EggLabel);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        m_CupLabel = new QLabel(widget_6);
        m_CupLabel->setObjectName(QString::fromUtf8("m_CupLabel"));
        sizePolicy10.setHeightForWidth(m_CupLabel->sizePolicy().hasHeightForWidth());
        m_CupLabel->setSizePolicy(sizePolicy10);
        m_CupLabel->setMinimumSize(QSize(0, 33));
        m_CupLabel->setMaximumSize(QSize(16777215, 33));

        horizontalLayout_3->addWidget(m_CupLabel);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_4);

        m_WeightLabel = new QLabel(widget_6);
        m_WeightLabel->setObjectName(QString::fromUtf8("m_WeightLabel"));
        sizePolicy10.setHeightForWidth(m_WeightLabel->sizePolicy().hasHeightForWidth());
        m_WeightLabel->setSizePolicy(sizePolicy10);
        m_WeightLabel->setMinimumSize(QSize(0, 33));
        m_WeightLabel->setMaximumSize(QSize(16777215, 33));

        horizontalLayout_3->addWidget(m_WeightLabel);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_10);


        verticalLayout_5->addWidget(widget_6);

        widget_3 = new QWidget(m_HandGripperPressureControls);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        sizePolicy2.setHeightForWidth(widget_3->sizePolicy().hasHeightForWidth());
        widget_3->setSizePolicy(sizePolicy2);
        widget_3->setMinimumSize(QSize(300, 33));
        widget_3->setMaximumSize(QSize(300, 33));
        horizontalLayout_8 = new QHBoxLayout(widget_3);
        horizontalLayout_8->setSpacing(0);
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        m_PressureSliderLayout = new QHBoxLayout();
        m_PressureSliderLayout->setSpacing(6);
        m_PressureSliderLayout->setObjectName(QString::fromUtf8("m_PressureSliderLayout"));

        horizontalLayout_8->addLayout(m_PressureSliderLayout);


        verticalLayout_5->addWidget(widget_3);


        horizontalLayout_5->addLayout(verticalLayout_5);

        horizontalSpacer_5 = new QSpacerItem(100, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_5);


        verticalLayout_6->addWidget(m_HandGripperPressureControls);

        m_tabGripperControl->addTab(m_tabHandControl, QString());

        verticalLayout_2->addWidget(m_tabGripperControl);

        m_ButtonStop = new QPushButton(m_rightContainer);
        m_ButtonStop->setObjectName(QString::fromUtf8("m_ButtonStop"));
        QSizePolicy sizePolicy11(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy11.setHorizontalStretch(0);
        sizePolicy11.setVerticalStretch(0);
        sizePolicy11.setHeightForWidth(m_ButtonStop->sizePolicy().hasHeightForWidth());
        m_ButtonStop->setSizePolicy(sizePolicy11);
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(true);
        font1.setWeight(75);
        m_ButtonStop->setFont(font1);

        verticalLayout_2->addWidget(m_ButtonStop);

        m_messagesWindow = new QTabWidget(m_rightContainer);
        m_messagesWindow->setObjectName(QString::fromUtf8("m_messagesWindow"));
        sizePolicy6.setHeightForWidth(m_messagesWindow->sizePolicy().hasHeightForWidth());
        m_messagesWindow->setSizePolicy(sizePolicy6);
        m_messagesWindow->setMinimumSize(QSize(0, 180));
        m_messagesWindow->setMaximumSize(QSize(16777215, 140));
        m_messagesWidget = new QWidget();
        m_messagesWidget->setObjectName(QString::fromUtf8("m_messagesWidget"));
        verticalLayout_18 = new QVBoxLayout(m_messagesWidget);
        verticalLayout_18->setSpacing(6);
        verticalLayout_18->setContentsMargins(11, 11, 11, 11);
        verticalLayout_18->setObjectName(QString::fromUtf8("verticalLayout_18"));
        verticalLayout_17 = new QVBoxLayout();
        verticalLayout_17->setSpacing(6);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));

        verticalLayout_18->addLayout(verticalLayout_17);

        m_messagesWindow->addTab(m_messagesWidget, QString());

        verticalLayout_2->addWidget(m_messagesWindow);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        horizontalLayout_25->setContentsMargins(-1, 0, -1, -1);
        m_manual_platform_controlWindow = new QTabWidget(m_rightContainer);
        m_manual_platform_controlWindow->setObjectName(QString::fromUtf8("m_manual_platform_controlWindow"));
        sizePolicy.setHeightForWidth(m_manual_platform_controlWindow->sizePolicy().hasHeightForWidth());
        m_manual_platform_controlWindow->setSizePolicy(sizePolicy);
        m_manual_platform_controlWidget = new QWidget();
        m_manual_platform_controlWidget->setObjectName(QString::fromUtf8("m_manual_platform_controlWidget"));
        sizePolicy8.setHeightForWidth(m_manual_platform_controlWidget->sizePolicy().hasHeightForWidth());
        m_manual_platform_controlWidget->setSizePolicy(sizePolicy8);
        gridLayout_2 = new QGridLayout(m_manual_platform_controlWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        m_manual_platform_controlWindow->addTab(m_manual_platform_controlWidget, QString());

        horizontalLayout_25->addWidget(m_manual_platform_controlWindow);


        verticalLayout_2->addLayout(horizontalLayout_25);


        horizontalLayout_2->addWidget(m_rightContainer);


        verticalLayout->addWidget(m_mainContainer);


        retranslateUi(RoseMainClass);

        m_askUserTextWindow->setCurrentIndex(0);
        m_tabGripperControl->setCurrentIndex(0);
        m_messagesWindow->setCurrentIndex(0);
        m_manual_platform_controlWindow->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(RoseMainClass);
    } // setupUi

    void retranslateUi(QWidget *RoseMainClass)
    {
        RoseMainClass->setWindowTitle(QApplication::translate("RoseMainClass", "RoseMain", 0, QApplication::UnicodeUTF8));
        m_labelRoseLogoSmallTopLeft->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_labelRoseNavigator->setText(QApplication::translate("RoseMainClass", "ROSE Navigator", 0, QApplication::UnicodeUTF8));
        m_labelDateTime->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_labelAlignment->setText(QString());
        m_ButtonHelp->setText(QApplication::translate("RoseMainClass", "HELP", 0, QApplication::UnicodeUTF8));
        m_ButtonClose->setText(QApplication::translate("RoseMainClass", "CLOSE", 0, QApplication::UnicodeUTF8));
        m_ButtonPause->setText(QApplication::translate("RoseMainClass", "PAUSE", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("RoseMainClass", "Battery", 0, QApplication::UnicodeUTF8));
        m_ButtonTelepresence->setText(QApplication::translate("RoseMainClass", "TELEPRESENCE ON", 0, QApplication::UnicodeUTF8));
        m_GeneralControlsTabWidget->setTabText(m_GeneralControlsTabWidget->indexOf(m_GeneralControlsPage), QApplication::translate("RoseMainClass", "General Controls", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("RoseMainClass", "OPERATOR", 0, QApplication::UnicodeUTF8));
        m_buttonOperatorVolumeStepMin->setText(QApplication::translate("RoseMainClass", "---", 0, QApplication::UnicodeUTF8));
        m_buttonOperatorVolumeStepPlus->setText(QApplication::translate("RoseMainClass", "++", 0, QApplication::UnicodeUTF8));
        m_buttonOperatorVolumeMute->setText(QApplication::translate("RoseMainClass", "MU", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("RoseMainClass", "CLIENT", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeStepMin->setText(QApplication::translate("RoseMainClass", "---", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeStepPlus->setText(QApplication::translate("RoseMainClass", "++", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeMute->setText(QApplication::translate("RoseMainClass", "MU", 0, QApplication::UnicodeUTF8));
        labelClientName->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        labelClientImage->setText(QString());
        m_CommunicationsTabWidget->setTabText(m_CommunicationsTabWidget->indexOf(m_tabCommunication), QApplication::translate("RoseMainClass", "Communication", 0, QApplication::UnicodeUTF8));
        m_askUserTextWindow->setTabText(m_askUserTextWindow->indexOf(m_askUserTextWidget), QApplication::translate("RoseMainClass", "Text input", 0, QApplication::UnicodeUTF8));
        m_interactionWindow->setTabText(m_interactionWindow->indexOf(m_interactionWidget), QApplication::translate("RoseMainClass", "Tasks and Operations", 0, QApplication::UnicodeUTF8));
        m_showOperatorCamCheckBox->setText(QApplication::translate("RoseMainClass", "Show webcam", 0, QApplication::UnicodeUTF8));
        m_tabCameras->setTabText(m_tabCameras->indexOf(tab_6), QApplication::translate("RoseMainClass", "Cameras", 0, QApplication::UnicodeUTF8));
        navigationCheckbox->setText(QApplication::translate("RoseMainClass", "Navigation", 0, QApplication::UnicodeUTF8));
        posEstimateCheckbox->setText(QApplication::translate("RoseMainClass", "Pos. estimation", 0, QApplication::UnicodeUTF8));
        m_HouseLayoutTabWidget->setTabText(m_HouseLayoutTabWidget->indexOf(m_tabHouseLayout), QApplication::translate("RoseMainClass", "House Layout", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("RoseMainClass", "Open and close", 0, QApplication::UnicodeUTF8));
        m_OpenGripperLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_ClosedGripperLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("RoseMainClass", "Pressure", 0, QApplication::UnicodeUTF8));
        m_PressureGripperLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_EggLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_CupLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_WeightLabel->setText(QApplication::translate("RoseMainClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_tabGripperControl->setTabText(m_tabGripperControl->indexOf(m_tabHandControl), QApplication::translate("RoseMainClass", "Hand Control", 0, QApplication::UnicodeUTF8));
        m_ButtonStop->setText(QApplication::translate("RoseMainClass", "STOP", 0, QApplication::UnicodeUTF8));
        m_messagesWindow->setTabText(m_messagesWindow->indexOf(m_messagesWidget), QApplication::translate("RoseMainClass", "Messages", 0, QApplication::UnicodeUTF8));
        m_manual_platform_controlWindow->setTabText(m_manual_platform_controlWindow->indexOf(m_manual_platform_controlWidget), QApplication::translate("RoseMainClass", "Movement", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class RoseMainClass: public Ui_RoseMainClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROSEMAIN_H
