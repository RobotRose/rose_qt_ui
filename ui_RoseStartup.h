/********************************************************************************
** Form generated from reading UI file 'RoseStartup.ui'
**
** Created: Fri May 2 09:59:14 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROSESTARTUP_H
#define UI_ROSESTARTUP_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RoseStartupClass
{
public:
    QVBoxLayout *m_MainVerticalLayout;
    QHBoxLayout *m_MainLayoutTop;
    QLabel *m_labelRoseLogoSmallTopLeft;
    QWidget *m_FillSpaceWidget1;
    QLabel *m_labelRoseNavigator;
    QWidget *m_FillSpaceWidget2;
    QSpacerItem *m_MainSpacerTop;
    QHBoxLayout *m_MainLayoutMid;
    QSpacerItem *m_MainSpacerLeft;
    QVBoxLayout *m_MainLayoutMidVertical;
    QWidget *m_RoseLogoWidget;
    QHBoxLayout *m_RoseLogoWidgetLayout;
    QSpacerItem *m_RoseLogoSpacerLeft;
    QLabel *m_labelFulllogo;
    QSpacerItem *m_RoseLogoSpacerRight;
    QWidget *m_controlsPanel;
    QHBoxLayout *m_controlsPanelLayout;
    QSpacerItem *m_CenterPanelSpacerLeft;
    QWidget *m_ControlsPanelWidget;
    QLabel *m_ClientLabel;
    QPushButton *m_buttonCall;
    QFrame *m_HorizontalLine;
    QPushButton *m_buttonCancel;
    QLabel *m_OperatorLabel;
    QLabel *m_labelCustomerName;
    QLabel *m_labelStatusMessage;
    QTableWidget *m_facesScrollTable;
    QPushButton *m_buttonPrevious;
    QPushButton *m_buttonNext;
    QPushButton *m_buttonOperatorVolumeStepMin;
    QPushButton *m_buttonOperatorVolumeStepPlus;
    QPushButton *m_buttonOperatorVolumeMute;
    QPushButton *m_buttonClientVolumeStepMin;
    QPushButton *m_buttonClientVolumeStepPlus;
    QPushButton *m_buttonClientVolumeMute;
    QSpacerItem *m_CenterPanelSpacerRight;
    QSpacerItem *m_MainSpacerRight;
    QHBoxLayout *m_MainLayoutBottom;
    QSpacerItem *m_MainSpacerBottomLeft;
    QVBoxLayout *m_CloseButtonLayout;
    QSpacerItem *m_MainSpacerBottomTop;
    QPushButton *m_ButtonClose;
    QSpacerItem *m_MainSpacerBottomBottom;
    QSpacerItem *m_MainSpacerBottomRight;

    void setupUi(QWidget *RoseStartupClass)
    {
        if (RoseStartupClass->objectName().isEmpty())
            RoseStartupClass->setObjectName(QString::fromUtf8("RoseStartupClass"));
        RoseStartupClass->resize(905, 840);
        m_MainVerticalLayout = new QVBoxLayout(RoseStartupClass);
        m_MainVerticalLayout->setSpacing(0);
        m_MainVerticalLayout->setContentsMargins(0, 0, 0, 0);
        m_MainVerticalLayout->setObjectName(QString::fromUtf8("m_MainVerticalLayout"));
        m_MainLayoutTop = new QHBoxLayout();
        m_MainLayoutTop->setSpacing(0);
        m_MainLayoutTop->setObjectName(QString::fromUtf8("m_MainLayoutTop"));
        m_labelRoseLogoSmallTopLeft = new QLabel(RoseStartupClass);
        m_labelRoseLogoSmallTopLeft->setObjectName(QString::fromUtf8("m_labelRoseLogoSmallTopLeft"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(m_labelRoseLogoSmallTopLeft->sizePolicy().hasHeightForWidth());
        m_labelRoseLogoSmallTopLeft->setSizePolicy(sizePolicy);
        m_labelRoseLogoSmallTopLeft->setMinimumSize(QSize(25, 25));
        m_labelRoseLogoSmallTopLeft->setMaximumSize(QSize(25, 25));

        m_MainLayoutTop->addWidget(m_labelRoseLogoSmallTopLeft);

        m_FillSpaceWidget1 = new QWidget(RoseStartupClass);
        m_FillSpaceWidget1->setObjectName(QString::fromUtf8("m_FillSpaceWidget1"));
        sizePolicy.setHeightForWidth(m_FillSpaceWidget1->sizePolicy().hasHeightForWidth());
        m_FillSpaceWidget1->setSizePolicy(sizePolicy);
        m_FillSpaceWidget1->setMinimumSize(QSize(4, 2));
        m_FillSpaceWidget1->setMaximumSize(QSize(4, 25));

        m_MainLayoutTop->addWidget(m_FillSpaceWidget1);

        m_labelRoseNavigator = new QLabel(RoseStartupClass);
        m_labelRoseNavigator->setObjectName(QString::fromUtf8("m_labelRoseNavigator"));
        m_labelRoseNavigator->setMinimumSize(QSize(0, 25));
        m_labelRoseNavigator->setMaximumSize(QSize(16777215, 25));
        QFont font;
        font.setFamily(QString::fromUtf8("Arial"));
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        m_labelRoseNavigator->setFont(font);

        m_MainLayoutTop->addWidget(m_labelRoseNavigator);

        m_FillSpaceWidget2 = new QWidget(RoseStartupClass);
        m_FillSpaceWidget2->setObjectName(QString::fromUtf8("m_FillSpaceWidget2"));
        m_FillSpaceWidget2->setMinimumSize(QSize(0, 25));
        m_FillSpaceWidget2->setMaximumSize(QSize(16777215, 25));

        m_MainLayoutTop->addWidget(m_FillSpaceWidget2);


        m_MainVerticalLayout->addLayout(m_MainLayoutTop);

        m_MainSpacerTop = new QSpacerItem(20, 57, QSizePolicy::Minimum, QSizePolicy::Expanding);

        m_MainVerticalLayout->addItem(m_MainSpacerTop);

        m_MainLayoutMid = new QHBoxLayout();
        m_MainLayoutMid->setObjectName(QString::fromUtf8("m_MainLayoutMid"));
        m_MainSpacerLeft = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_MainLayoutMid->addItem(m_MainSpacerLeft);

        m_MainLayoutMidVertical = new QVBoxLayout();
        m_MainLayoutMidVertical->setObjectName(QString::fromUtf8("m_MainLayoutMidVertical"));
        m_RoseLogoWidget = new QWidget(RoseStartupClass);
        m_RoseLogoWidget->setObjectName(QString::fromUtf8("m_RoseLogoWidget"));
        m_RoseLogoWidget->setMinimumSize(QSize(500, 0));
        m_RoseLogoWidget->setMaximumSize(QSize(500, 16777215));
        m_RoseLogoWidgetLayout = new QHBoxLayout(m_RoseLogoWidget);
        m_RoseLogoWidgetLayout->setObjectName(QString::fromUtf8("m_RoseLogoWidgetLayout"));
        m_RoseLogoSpacerLeft = new QSpacerItem(60, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_RoseLogoWidgetLayout->addItem(m_RoseLogoSpacerLeft);

        m_labelFulllogo = new QLabel(m_RoseLogoWidget);
        m_labelFulllogo->setObjectName(QString::fromUtf8("m_labelFulllogo"));
        m_labelFulllogo->setMinimumSize(QSize(344, 100));
        m_labelFulllogo->setMaximumSize(QSize(344, 100));

        m_RoseLogoWidgetLayout->addWidget(m_labelFulllogo);

        m_RoseLogoSpacerRight = new QSpacerItem(60, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_RoseLogoWidgetLayout->addItem(m_RoseLogoSpacerRight);


        m_MainLayoutMidVertical->addWidget(m_RoseLogoWidget);

        m_controlsPanel = new QWidget(RoseStartupClass);
        m_controlsPanel->setObjectName(QString::fromUtf8("m_controlsPanel"));
        m_controlsPanel->setMinimumSize(QSize(500, 500));
        m_controlsPanel->setMaximumSize(QSize(500, 500));
        m_controlsPanel->setAutoFillBackground(true);
        m_controlsPanelLayout = new QHBoxLayout(m_controlsPanel);
        m_controlsPanelLayout->setObjectName(QString::fromUtf8("m_controlsPanelLayout"));
        m_CenterPanelSpacerLeft = new QSpacerItem(47, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_controlsPanelLayout->addItem(m_CenterPanelSpacerLeft);

        m_ControlsPanelWidget = new QWidget(m_controlsPanel);
        m_ControlsPanelWidget->setObjectName(QString::fromUtf8("m_ControlsPanelWidget"));
        m_ControlsPanelWidget->setMinimumSize(QSize(400, 0));
        m_ControlsPanelWidget->setMaximumSize(QSize(400, 16777215));
        m_ClientLabel = new QLabel(m_ControlsPanelWidget);
        m_ClientLabel->setObjectName(QString::fromUtf8("m_ClientLabel"));
        m_ClientLabel->setGeometry(QRect(10, 370, 351, 21));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Arial Black"));
        m_ClientLabel->setFont(font1);
        m_buttonCall = new QPushButton(m_ControlsPanelWidget);
        m_buttonCall->setObjectName(QString::fromUtf8("m_buttonCall"));
        m_buttonCall->setGeometry(QRect(30, 230, 93, 35));
        m_buttonCall->setMinimumSize(QSize(0, 0));
        m_HorizontalLine = new QFrame(m_ControlsPanelWidget);
        m_HorizontalLine->setObjectName(QString::fromUtf8("m_HorizontalLine"));
        m_HorizontalLine->setGeometry(QRect(10, 280, 381, 20));
        m_HorizontalLine->setFrameShape(QFrame::HLine);
        m_HorizontalLine->setFrameShadow(QFrame::Sunken);
        m_buttonCancel = new QPushButton(m_ControlsPanelWidget);
        m_buttonCancel->setObjectName(QString::fromUtf8("m_buttonCancel"));
        m_buttonCancel->setGeometry(QRect(260, 230, 93, 35));
        m_buttonCancel->setMinimumSize(QSize(0, 0));
        m_OperatorLabel = new QLabel(m_ControlsPanelWidget);
        m_OperatorLabel->setObjectName(QString::fromUtf8("m_OperatorLabel"));
        m_OperatorLabel->setGeometry(QRect(10, 300, 351, 21));
        m_OperatorLabel->setFont(font1);
        m_labelCustomerName = new QLabel(m_ControlsPanelWidget);
        m_labelCustomerName->setObjectName(QString::fromUtf8("m_labelCustomerName"));
        m_labelCustomerName->setGeometry(QRect(30, 120, 341, 101));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Arial Black"));
        font2.setPointSize(11);
        m_labelCustomerName->setFont(font2);
        m_labelStatusMessage = new QLabel(m_ControlsPanelWidget);
        m_labelStatusMessage->setObjectName(QString::fromUtf8("m_labelStatusMessage"));
        m_labelStatusMessage->setGeometry(QRect(10, 450, 381, 20));
        m_labelStatusMessage->setFont(font1);
        m_labelStatusMessage->setAlignment(Qt::AlignCenter);
        m_facesScrollTable = new QTableWidget(m_ControlsPanelWidget);
        m_facesScrollTable->setObjectName(QString::fromUtf8("m_facesScrollTable"));
        m_facesScrollTable->setGeometry(QRect(31, 30, 339, 90));
        m_facesScrollTable->setMinimumSize(QSize(0, 90));
        m_facesScrollTable->setMaximumSize(QSize(16777215, 90));
        m_facesScrollTable->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_facesScrollTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_facesScrollTable->setAutoScroll(true);
        m_facesScrollTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_facesScrollTable->setIconSize(QSize(150, 150));
        m_facesScrollTable->setShowGrid(false);
        m_facesScrollTable->setCornerButtonEnabled(false);
        m_buttonPrevious = new QPushButton(m_ControlsPanelWidget);
        m_buttonPrevious->setObjectName(QString::fromUtf8("m_buttonPrevious"));
        m_buttonPrevious->setGeometry(QRect(11, 30, 20, 90));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(m_buttonPrevious->sizePolicy().hasHeightForWidth());
        m_buttonPrevious->setSizePolicy(sizePolicy1);
        m_buttonPrevious->setMinimumSize(QSize(20, 0));
        m_buttonPrevious->setMaximumSize(QSize(20, 16777215));
        QIcon icon;
        icon.addFile(QString::fromUtf8("../Resources/previous.png"), QSize(), QIcon::Normal, QIcon::Off);
        m_buttonPrevious->setIcon(icon);
        m_buttonNext = new QPushButton(m_ControlsPanelWidget);
        m_buttonNext->setObjectName(QString::fromUtf8("m_buttonNext"));
        m_buttonNext->setGeometry(QRect(370, 30, 20, 90));
        sizePolicy1.setHeightForWidth(m_buttonNext->sizePolicy().hasHeightForWidth());
        m_buttonNext->setSizePolicy(sizePolicy1);
        m_buttonNext->setMinimumSize(QSize(20, 0));
        m_buttonNext->setMaximumSize(QSize(20, 16777215));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8("../Resources/next.png"), QSize(), QIcon::Normal, QIcon::Off);
        m_buttonNext->setIcon(icon1);
        m_buttonOperatorVolumeStepMin = new QPushButton(m_ControlsPanelWidget);
        m_buttonOperatorVolumeStepMin->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeStepMin"));
        m_buttonOperatorVolumeStepMin->setGeometry(QRect(11, 331, 30, 30));
        m_buttonOperatorVolumeStepMin->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeStepMin->setMaximumSize(QSize(30, 30));
        m_buttonOperatorVolumeStepPlus = new QPushButton(m_ControlsPanelWidget);
        m_buttonOperatorVolumeStepPlus->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeStepPlus"));
        m_buttonOperatorVolumeStepPlus->setGeometry(QRect(324, 331, 30, 30));
        m_buttonOperatorVolumeStepPlus->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeStepPlus->setMaximumSize(QSize(30, 30));
        m_buttonOperatorVolumeMute = new QPushButton(m_ControlsPanelWidget);
        m_buttonOperatorVolumeMute->setObjectName(QString::fromUtf8("m_buttonOperatorVolumeMute"));
        m_buttonOperatorVolumeMute->setGeometry(QRect(360, 331, 30, 30));
        m_buttonOperatorVolumeMute->setMinimumSize(QSize(30, 30));
        m_buttonOperatorVolumeMute->setMaximumSize(QSize(30, 30));
        m_buttonOperatorVolumeMute->setCheckable(true);
        m_buttonClientVolumeStepMin = new QPushButton(m_ControlsPanelWidget);
        m_buttonClientVolumeStepMin->setObjectName(QString::fromUtf8("m_buttonClientVolumeStepMin"));
        m_buttonClientVolumeStepMin->setGeometry(QRect(11, 401, 30, 30));
        m_buttonClientVolumeStepMin->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeStepMin->setMaximumSize(QSize(30, 30));
        m_buttonClientVolumeStepPlus = new QPushButton(m_ControlsPanelWidget);
        m_buttonClientVolumeStepPlus->setObjectName(QString::fromUtf8("m_buttonClientVolumeStepPlus"));
        m_buttonClientVolumeStepPlus->setGeometry(QRect(324, 401, 30, 30));
        m_buttonClientVolumeStepPlus->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeStepPlus->setMaximumSize(QSize(30, 30));
        m_buttonClientVolumeMute = new QPushButton(m_ControlsPanelWidget);
        m_buttonClientVolumeMute->setObjectName(QString::fromUtf8("m_buttonClientVolumeMute"));
        m_buttonClientVolumeMute->setGeometry(QRect(360, 401, 30, 30));
        m_buttonClientVolumeMute->setMinimumSize(QSize(30, 30));
        m_buttonClientVolumeMute->setMaximumSize(QSize(30, 30));
        m_buttonClientVolumeMute->setCheckable(true);

        m_controlsPanelLayout->addWidget(m_ControlsPanelWidget);

        m_CenterPanelSpacerRight = new QSpacerItem(47, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_controlsPanelLayout->addItem(m_CenterPanelSpacerRight);


        m_MainLayoutMidVertical->addWidget(m_controlsPanel);


        m_MainLayoutMid->addLayout(m_MainLayoutMidVertical);

        m_MainSpacerRight = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_MainLayoutMid->addItem(m_MainSpacerRight);


        m_MainVerticalLayout->addLayout(m_MainLayoutMid);

        m_MainLayoutBottom = new QHBoxLayout();
        m_MainLayoutBottom->setObjectName(QString::fromUtf8("m_MainLayoutBottom"));
        m_MainSpacerBottomLeft = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_MainLayoutBottom->addItem(m_MainSpacerBottomLeft);

        m_CloseButtonLayout = new QVBoxLayout();
        m_CloseButtonLayout->setObjectName(QString::fromUtf8("m_CloseButtonLayout"));
        m_MainSpacerBottomTop = new QSpacerItem(20, 56, QSizePolicy::Minimum, QSizePolicy::Expanding);

        m_CloseButtonLayout->addItem(m_MainSpacerBottomTop);

        m_ButtonClose = new QPushButton(RoseStartupClass);
        m_ButtonClose->setObjectName(QString::fromUtf8("m_ButtonClose"));

        m_CloseButtonLayout->addWidget(m_ButtonClose);

        m_MainSpacerBottomBottom = new QSpacerItem(20, 56, QSizePolicy::Minimum, QSizePolicy::Expanding);

        m_CloseButtonLayout->addItem(m_MainSpacerBottomBottom);


        m_MainLayoutBottom->addLayout(m_CloseButtonLayout);

        m_MainSpacerBottomRight = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        m_MainLayoutBottom->addItem(m_MainSpacerBottomRight);


        m_MainVerticalLayout->addLayout(m_MainLayoutBottom);


        retranslateUi(RoseStartupClass);

        QMetaObject::connectSlotsByName(RoseStartupClass);
    } // setupUi

    void retranslateUi(QWidget *RoseStartupClass)
    {
        RoseStartupClass->setWindowTitle(QApplication::translate("RoseStartupClass", "Form", 0, QApplication::UnicodeUTF8));
        m_labelRoseLogoSmallTopLeft->setText(QApplication::translate("RoseStartupClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_labelRoseNavigator->setText(QApplication::translate("RoseStartupClass", "ROSE Navigator", 0, QApplication::UnicodeUTF8));
        m_labelFulllogo->setText(QApplication::translate("RoseStartupClass", "TextLabel", 0, QApplication::UnicodeUTF8));
        m_ClientLabel->setText(QApplication::translate("RoseStartupClass", "CLIENT", 0, QApplication::UnicodeUTF8));
        m_buttonCall->setText(QApplication::translate("RoseStartupClass", "Call", 0, QApplication::UnicodeUTF8));
        m_buttonCancel->setText(QApplication::translate("RoseStartupClass", "Cancel", 0, QApplication::UnicodeUTF8));
        m_OperatorLabel->setText(QApplication::translate("RoseStartupClass", "OPERATOR", 0, QApplication::UnicodeUTF8));
        m_labelCustomerName->setText(QApplication::translate("RoseStartupClass", "Name + Address", 0, QApplication::UnicodeUTF8));
        m_labelStatusMessage->setText(QApplication::translate("RoseStartupClass", "status messages...", 0, QApplication::UnicodeUTF8));
        m_buttonPrevious->setText(QString());
        m_buttonNext->setText(QString());
        m_buttonOperatorVolumeStepMin->setText(QApplication::translate("RoseStartupClass", "---", 0, QApplication::UnicodeUTF8));
        m_buttonOperatorVolumeStepPlus->setText(QApplication::translate("RoseStartupClass", "++", 0, QApplication::UnicodeUTF8));
        m_buttonOperatorVolumeMute->setText(QApplication::translate("RoseStartupClass", "MU", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeStepMin->setText(QApplication::translate("RoseStartupClass", "---", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeStepPlus->setText(QApplication::translate("RoseStartupClass", "++", 0, QApplication::UnicodeUTF8));
        m_buttonClientVolumeMute->setText(QApplication::translate("RoseStartupClass", "MU", 0, QApplication::UnicodeUTF8));
        m_ButtonClose->setText(QApplication::translate("RoseStartupClass", "Close", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class RoseStartupClass: public Ui_RoseStartupClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROSESTARTUP_H
