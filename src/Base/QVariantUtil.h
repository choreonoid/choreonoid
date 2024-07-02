#ifndef CNOID_BASE_QVARIANT_UTIL_H
#define CNOID_BASE_QVARIANT_UTIL_H

#include <QVariant>

namespace cnoid {

inline int variantType(const QVariant& value)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return value.typeId();
#else
    return value.type();
#endif
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
enum QVariantTypes
{
    QVariantBoolType = QMetaType::Bool,
    QVariantStringType = QMetaType::QString,
    QVariantIntType = QMetaType::Int,
    QVariantDoubleType = QMetaType::Double,
    QVariantStringListType = QMetaType::QStringList,
};
#else
enum QVariantTypes
{
    QVariantBoolType = QVariant::Bool,
    QVariantStringType = QVariant::String,
    QVariantIntType = QVariant::Int,
    QVariantDoubleType = QVariant::Double,
    QVariantStringListType = QVariant::StringList,
};
#endif

inline bool checkIfString(const QVariant& value)
{
    return variantType(value) == QVariantStringType;
}

inline bool checkIfDouble(const QVariant& value)
{
    return variantType(value) == QVariantDoubleType;
}

}

#endif
