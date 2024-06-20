#ifndef CNOID_BASE_QT_SVG_UTIL_H
#define CNOID_BASE_QT_SVG_UTIL_H

#include <QPixmap>
#include <QIcon>
#include <QSvgRenderer>
#include <QPainter>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT QtSvgUtil
{
public:
  QPixmap createPixmapFromSvgFile(const QString& filename, const QSize& size);
  static QIcon createIconFromSvgFile(const QString& filename);

private:
  QSvgRenderer svgRenderer;
  QPainter painter;
};

}

#endif

