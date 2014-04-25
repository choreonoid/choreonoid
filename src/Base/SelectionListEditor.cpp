/**
   @author Shin'ichiro Nakaoka
*/

#include "SelectionListEditor.h"

using namespace cnoid;


SelectionListEditor::SelectionListEditor(QWidget* widget)
    : QComboBox(widget)
{

}


QStringList SelectionListEditor::labels() const
{
    QStringList labelList;
    labelList << QString::number(currentIndex());
    for(int i=1; i < count(); ++i){
        labelList << itemText(i);
    }
    return labelList;
}


void SelectionListEditor::setLabels(QStringList labels)
{
    if(labels.count() >= 2){
        for(int i=1; i < labels.count(); ++i){
            addItem(labels[i]);
        }
        setCurrentIndex(labels[0].toInt());
    }
}
