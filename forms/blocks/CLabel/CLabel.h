//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#ifndef CLABEL_H
#define CLABEL_H
//
#include <QtGui/QLabel>
#include <QtCore/QEvent>
#include <QtGui/QApplication>

#include "std_msgs/String.h"
#include <boost/thread/mutex.hpp>

#include "global.h"
#include "Loggers/CountingLogger.h"

//
  class CLabel : public QLabel
  {
//    Q_OBJECT

  public:

    CLabel(QLabel*);
//     ~CLabel();
    char warning_text[100];     //2011/03/31     nv ToDo Q&D temporary storage

    // links CLabel and LabelController
    void ConsumeData(const std_msgs::String::ConstPtr& param);

  protected:

  private:
    QLabel* text;
    virtual void customEvent( QEvent *event);
    unsigned int m_mutex_label_cnt;
    boost::mutex m_mutex_label;      //2011/04/04     nv Is a mutex really needed

    boost::shared_ptr<Loggers::CountingLogger> m_logger ;
  };


#endif // CLABEL_H

