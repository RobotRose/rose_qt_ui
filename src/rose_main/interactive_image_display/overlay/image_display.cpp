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
#include "rose_main/interactive_image_display/overlay/image_display.hpp"

ImageDisplay::ImageDisplay()
    :QGraphicsObject()
    ,m_logger(Loggers::LogMaster::Instance()->Get<Loggers::CountingLogger>("ImageDisplay"))
	,m_Width( IMAGE_WIDTH )
	,m_Height( IMAGE_HEIGHT )
{
    m_subtitle = QString();
    m_data.reserve( IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS ) ;
    m_image = boost::shared_ptr<QImage>( new QImage( &m_data[0], IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888 ) );
    QPainter p;
    p.begin( m_image.get() );
    p.fillRect( QRect(0,0,IMAGE_WIDTH,IMAGE_HEIGHT), Qt::blue );
    p.end();
}
//---------------------------------------------------------------------------------
//
//
QRectF ImageDisplay::boundingRect() const
{
	return QRectF( 0, 0, m_Width, m_Height );
}
//---------------------------------------------------------------------------------
//
//
void ImageDisplay::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
    m_mutex.lock();

    painter->setTransform(this->transform());

    painter->setCompositionMode( QPainter::CompositionMode_Source);
    painter->drawImage( 0, 0, *m_image );

    //Only show the subtitle when  it has not timed out.
    if(QTime::currentTime() < m_show_subtitle_until)
    {
        subtitle(painter, m_subtitle, m_subtitle_color);
    }

    m_mutex.unlock();
}

void ImageDisplay::subtitle( QPainter *painter, const QString text, const QColor color)
{
    QPen pen;
    pen.setColor( color );
    painter->setPen( pen );

    QFont font = painter->font() ;

    /* twice the size than the current font size */
    font.setPointSize(32);
    painter->setFont(font);
    /* set the modified font to the painter */

    QRect rect = QRect(0.05*m_Width, 0.75*m_Height,
                       0.9*m_Width, 0.23*m_Height); //The heights dont add up to 1.0 to keep the text a little bit of the bottom

    //Draw some border around the text
    pen.setColor( Qt::black );
    painter->setPen( pen );
    int shadowSize = 2;
    painter->drawText(rect.translated(shadowSize,shadowSize), text, QTextOption(Qt::AlignCenter));
    painter->drawText(rect.translated(shadowSize,-shadowSize), text, QTextOption(Qt::AlignCenter));
    painter->drawText(rect.translated(-shadowSize,shadowSize), text, QTextOption(Qt::AlignCenter));
    painter->drawText(rect.translated(-shadowSize,-shadowSize), text, QTextOption(Qt::AlignCenter));

    //Draw the actual text
    pen.setColor( color );
    painter->setPen( pen );
    painter->drawText(rect, text, QTextOption(Qt::AlignCenter));
}

void ImageDisplay::setSubtitle(string subtitle, QColor color, int secondsVisible)
{
    if(subtitle.size() > 42)
    {
        ROS_WARN_NAMED(ROS_NAME, "Subtitle message '%s' may be too long. Max is 2 lines of ca 21 chars", subtitle.c_str());
    }

    m_subtitle = subtitle.c_str();
    m_subtitle_color = color;

    //A subtitle must now be shown forever but only until m_show_subtitle_until.
    //This is the current time + a variable amount of time.
    m_show_subtitle_until = QTime::currentTime().addSecs(secondsVisible);
    ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::setSubtitle will show subtitle until %s", m_show_subtitle_until.toString().toStdString().c_str());
}

string ImageDisplay::getSubtitle()
{
    return m_subtitle.toStdString();
}

//---------------------------------------------------------------------------------
//
//
void ImageDisplay::ShowImageFromBuffer( const std::vector<unsigned char>& data, unsigned int sizex , unsigned int sizey, QRectF bounding_box)
{
    // ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::ShowImageFromBuffer");
    // ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::lock");
    m_mutex.lock();
    // ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::locked");

    unsigned char data_copy[data.size()];
    int length = 0;
    for(const auto& elem : data)
    {
        data_copy[length++] = elem;
    }

    int image_width = bounding_box.width();
    int image_height = bounding_box.height();

    if( sizex != 0 && sizey != 0 )
    {
        // We receive a message of which the format is known, and there are the pre defined number of color channels available.
        if ( (int)length == (sizex * sizey * IMAGE_CHANNELS ) )
        {
            m_image = boost::shared_ptr<QImage>( new QImage (&data_copy[0], sizex, sizey, QImage::Format_RGB888) );
            QImage * img = m_image.get();            
            
            if( (int)length != (image_width * image_height * IMAGE_CHANNELS ) ) //scale
                *img = img->scaled(QSize(image_width, image_height));
         
            *img = img->rgbSwapped();
            QApplication::postEvent( this, new QEvent( SHOWIMAGE_RGB_EVENT ) );
        }
        else 
        {
            // TODO: Exception instead of couts.
            std::cout << "Aborting ShowImageFromBuffer since buffer is not of correct size !" << std::endl; 
        }
    }
    else
    {
        m_data = data;

        // The image data we receive contains a complete image including header from which the type, format
    	// and size can be retrieved.
        QPixmap pixmap;
        // LoadFromData reads original file header and interprets it.
        // It then knows the correct size of the image, the width and height.
        pixmap.loadFromData( &m_data[0], m_data.size() );
        m_Width = pixmap.width();
        m_Height = pixmap.height();
        m_image = boost::shared_ptr<QImage>( new QImage( m_Width, m_Height, QImage::Format_RGB888 ) );
        QPainter painter(m_image.get());
        painter.drawPixmap( 0, 0, pixmap);
        painter.end();

        QApplication::postEvent( this, new QEvent( SHOWIMAGE_FILEFORMAT_EVENT ) );
    }
    // ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::unlock");
    m_mutex.unlock();
    // ROS_DEBUG_NAMED(ROS_NAME, "ImageDisplay::ShowImageFromBuffer, done");
}
//---------------------------------------------------------------------------------
//
//
void ImageDisplay::ShowImageFromQImage(const QImage& image)
{
    m_mutex.lock();

    m_Width = image.width();
    m_Height = image.height();
    m_image = boost::shared_ptr<QImage>(new QImage(image));

    QApplication::postEvent( this, new QEvent( SHOWIMAGE_FILEFORMAT_EVENT ) );

    m_mutex.unlock();
}
//---------------------------------------------------------------------------------
//
//
bool ImageDisplay::event ( QEvent * pevent )
{
    if ( pevent->type() == SHOWIMAGE_RGB_EVENT )
    {        
        scene()->invalidate();
        m_logger->Log();
        update();
        return true;
    }
    if ( pevent->type() == SHOWIMAGE_FILEFORMAT_EVENT )
    {
    	// When a map is loaded, the size of the map will probably differ from the size
    	// of the initial image (created in constructor).
    	// When this image is resized (due to loading of the map), it seems that the scene isn't
    	// properly redrawn. For this reason we explicitly call invalidate on the
    	// scene so a full redraw will be forced.
    	scene()->invalidate();
    	//
        m_logger->Log();
        update();
        return true;
    }
    return QGraphicsObject::event(pevent);
}
//---------------------------------------------------------------------------------
//
//
void ImageDisplay::CreateTestImage()
{
    int w = IMAGE_WIDTH;
    int h = IMAGE_HEIGHT;
    //
    m_image = boost::shared_ptr<QImage>(new QImage(w, h, QImage::Format_RGB888 ));
    m_image->fill( Qt::red );
    QPen pen;
    pen.setColor( Qt::white );
    QPainter painter( m_image.get() );

    painter.setPen( pen );

    painter.drawLine( 0, 0, w, h );
    painter.drawLine( 0, h, w, 0 );

    painter.drawText( 10,20, QString("%1").arg(w) );
    painter.drawText( 10,50, QString("%1").arg(h) );
}
//---------------------------------------------------------------------------------
//
//
