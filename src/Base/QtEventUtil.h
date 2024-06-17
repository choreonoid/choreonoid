#ifndef CNOID_BASE_QT_EVENT_UTIL_H
#define CNOID_BASE_QT_EVENT_UTIL_H

#include <QMouseEvent>
#include <QDropEvent>

namespace cnoid {

inline QPoint getPosition(const QMouseEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return event->position().toPoint();
#else
    return event->pos();
#endif
}

inline QPoint getScenePosition(const QMouseEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return event->scenePosition().toPoint();
#else
    return event->windowPos().toPoint();
#endif
}

inline QPoint getGlobalPosition(const QMouseEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return event->globalPosition().toPoint();
#else
    return event->globalPos();
#endif
}


inline QPoint getPosition(const QDropEvent* event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return event->position().toPoint();
#else
    return event->pos();
#endif
}

}

#endif
