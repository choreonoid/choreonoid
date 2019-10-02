#include "StringListComboBox.h"

using namespace cnoid;


StringListComboBox::StringListComboBox(QWidget* widget)
    : QComboBox(widget)
{

}


QStringList StringListComboBox::labels() const
{
    QStringList labelList;
    labelList << QString::number(currentIndex());
    for(int i=1; i < count(); ++i){
        labelList << itemText(i);
    }
    return labelList;
}


void StringListComboBox::setLabels(QStringList labels)
{
    if(labels.count() >= 2){
        for(int i=1; i < labels.count(); ++i){
            addItem(labels[i]);
        }
        setCurrentIndex(labels[0].toInt());
    }
}
