/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/17
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#ifndef IMAGEDISPLAY_HPP
#define IMAGEDISPLAY_HPP


#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <ros/ros.h>
#include <QtCore/QTimer>
#include <QtGui/QPainter>
#include <QtGui/QApplication>
#include <QtGui/QColor>
#include <QtGui/QGraphicsObject>
#include <QtGui/QGraphicsScale>
#include <QtGui/QGraphicsScene>
#include <QtGui/QImage>
#include <QtGui/QWidget>
#include <Qt/qevent.h>
#include <QTime>
#include <vector>

#include "Loggers/CountingLogger.h"
#include "Loggers/LogMaster.h"
#include "Loggers/CountingLogger.h"

#include "rose_common/common.hpp"

#define SHOWIMAGE_FILEFORMAT_EVENT          ((QEvent::Type)(QEvent::User + 2))
#define SHOWIMAGE_RGB_EVENT                 ((QEvent::Type)(QEvent::User + 1))

using std::string;

class QImage ;
class QResizeEvent;
class QPaintEvent;
class QEvent;

class ImageDisplay : public QGraphicsObject
{
public:
    ImageDisplay();

    void ShowImageFromBuffer( const std::vector<unsigned char>& data, unsigned int sizex = 0, unsigned int sizey = 0, QRectF bounding_box = QRectF(0,0,0,0));

    void ShowImageFromQImage(const QImage& image);

    QRectF boundingRect() const;

    void paint( QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget );

    /**
     * Draw the given text in the given color in the bottom of the image
     * @param painter [description]
     * @param text    [description]
     * @param color   [description]
     */
    void subtitle( QPainter *painter, const QString text, const QColor color );

    /**
     * Set the text that is to be displayed as a subtitle along with the color
     * @param subtitle Subtitle text
     * @param color    Color in which to draw the text
     */
    void setSubtitle(string subtitle, QColor color, int secondsVisible=10);
    string getSubtitle();

protected:

    virtual bool event ( QEvent * pevent );
private:
    void CreateTestImage();

    static const int IMAGE_CHANNELS = 3 ;
    static const int IMAGE_WIDTH = 640;
    static const int IMAGE_HEIGHT = 480;

    boost::shared_ptr<QImage> m_image ;
    std::vector<unsigned char> m_data ;
    boost::mutex m_mutex ;
    boost::shared_ptr<Loggers::CountingLogger> m_logger ;

    int m_Width;
    int m_Height;
    QString m_subtitle;
    QColor m_subtitle_color;
    QTime m_show_subtitle_until;
};

#endif /* IMAGEDISPLAY_HPP */
