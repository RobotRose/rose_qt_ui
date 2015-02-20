//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#include <QtCore/QEvent>
#include <iostream>
//------------------------------------------------------------------------------
//
//
#include "CLabel.h"

#include "Loggers/LogMaster.h"
#include "Loggers/CountingLogger.h"

//---------------------------------------------------------------------------------
//
//
CLabel::CLabel( QLabel *l) : text(l)
	,m_logger(Loggers::LogMaster::Instance()->Get<Loggers::CountingLogger>("Warning"))
{
    assert( text );
    m_mutex_label_cnt = 0;
}
//---------------------------------------------------------------------------------
//
//
void CLabel::ConsumeData(const std_msgs::String::ConstPtr& param)
{
    m_logger->Log();
    if ( m_mutex_label.try_lock() == false ) {
        m_mutex_label_cnt++;
    } else {
        strncpy( warning_text, param->data.c_str(), sizeof(warning_text) );        //todo copy this directly to QLabel
        QApplication::postEvent( this, new QEvent( CLABEL_EVENT ) );
        m_mutex_label.unlock();
    }
}
//---------------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------------
//
//
void CLabel::customEvent( QEvent *event)
{
    if ( m_mutex_label.try_lock() == false ) {
        m_mutex_label_cnt++;
    } else {
        text->setText(warning_text);
        m_mutex_label.unlock();
    }
}
