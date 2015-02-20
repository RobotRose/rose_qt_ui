/********************************************************************************
** Form generated from reading UI file 'ConnectionLost.ui'
**
** Created: Fri May 2 09:59:14 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONNECTIONLOST_H
#define UI_CONNECTIONLOST_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConnectionLostClass
{
public:
    QWidget *m_TopLayer;
    QPushButton *m_ButtonHelp;
    QPushButton *m_ButtonRetry;
    QPushButton *m_ButtonPowerOff;
    QLabel *label;
    QLabel *label_2;

    void setupUi(QWidget *ConnectionLostClass)
    {
        if (ConnectionLostClass->objectName().isEmpty())
            ConnectionLostClass->setObjectName(QString::fromUtf8("ConnectionLostClass"));
        ConnectionLostClass->resize(700, 200);
        ConnectionLostClass->setAutoFillBackground(true);
        m_TopLayer = new QWidget(ConnectionLostClass);
        m_TopLayer->setObjectName(QString::fromUtf8("m_TopLayer"));
        m_TopLayer->setGeometry(QRect(5, 5, 690, 190));
        m_TopLayer->setAutoFillBackground(true);
        m_ButtonHelp = new QPushButton(m_TopLayer);
        m_ButtonHelp->setObjectName(QString::fromUtf8("m_ButtonHelp"));
        m_ButtonHelp->setGeometry(QRect(10, 150, 80, 26));
        m_ButtonRetry = new QPushButton(m_TopLayer);
        m_ButtonRetry->setObjectName(QString::fromUtf8("m_ButtonRetry"));
        m_ButtonRetry->setGeometry(QRect(500, 150, 80, 26));
        m_ButtonPowerOff = new QPushButton(m_TopLayer);
        m_ButtonPowerOff->setObjectName(QString::fromUtf8("m_ButtonPowerOff"));
        m_ButtonPowerOff->setGeometry(QRect(600, 150, 80, 26));
        label = new QLabel(m_TopLayer);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(18, 30, 501, 51));
        QFont font;
        font.setPointSize(26);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        label_2 = new QLabel(m_TopLayer);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 80, 471, 20));

        retranslateUi(ConnectionLostClass);

        QMetaObject::connectSlotsByName(ConnectionLostClass);
    } // setupUi

    void retranslateUi(QWidget *ConnectionLostClass)
    {
        ConnectionLostClass->setWindowTitle(QApplication::translate("ConnectionLostClass", "Form", 0, QApplication::UnicodeUTF8));
        m_ButtonHelp->setText(QApplication::translate("ConnectionLostClass", "Help", 0, QApplication::UnicodeUTF8));
        m_ButtonRetry->setText(QApplication::translate("ConnectionLostClass", "Retry", 0, QApplication::UnicodeUTF8));
        m_ButtonPowerOff->setText(QApplication::translate("ConnectionLostClass", "Exit", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ConnectionLostClass", "Connection Lost", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ConnectionLostClass", "One moment please, connection is recovered.........", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ConnectionLostClass: public Ui_ConnectionLostClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONNECTIONLOST_H
