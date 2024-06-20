#include "QtSvgUtil.h"
#include "MainWindow.h"

using namespace cnoid;

QPixmap QtSvgUtil::createPixmapFromSvgFile(const QString& filename, const QSize& size)
{
    if(!svgRenderer.load(filename)){
        return QPixmap(); // null pixmap
    }
    QPixmap pixmap(size);
    pixmap.fill(Qt::transparent);
    painter.begin(&pixmap);
    svgRenderer.render(&painter);
    painter.end();
    return pixmap;
}


QIcon QtSvgUtil::createIconFromSvgFile(const QString& filename)
{
    static QSize pixmapSize;
    if(!pixmapSize.isValid()){
        static MainWindow* mainWindow = MainWindow::instance();
        pixmapSize = 2.0 * mainWindow->devicePixelRatio() * mainWindow->iconSize();
    }
    static QtSvgUtil svgUtil;
    return QIcon(svgUtil.createPixmapFromSvgFile(filename, pixmapSize));
}
