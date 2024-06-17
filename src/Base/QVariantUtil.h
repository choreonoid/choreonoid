#ifndef CNOID_BASE_QVARIANT_UTIL_H
#define CNOID_BASE_QVARIANT_UTIL_H

#include <QVariant>

namespace cnoid {

inline bool checkIfString(const QVariant& value)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return (value.typeId() == QMetaType::QString);
#else
    return (value.type() == QVariant::String);
#endif
}

inline bool checkIfDouble(const QVariant& value)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return (value.typeId() == QMetaType::Double);
#else
    return (value.type() == QVariant::Double);
#endif
}

}

#endif
