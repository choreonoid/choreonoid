/**
   @author Shin'ichiro Nakaoka
*/

#include "NameServerView.h"
#include <cnoid/CorbaUtil>
#include <cnoid/ViewManager>
#include <cnoid/TreeWidget>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class NameServerViewImpl
{
public:
    NameServerViewImpl(NameServerView* self);
    ~NameServerViewImpl();
    void updateObjectList();
    void appendBindingList(CosNaming::BindingList_var& bList);

    TreeWidget treeWidget;
    LineEdit hostAddressBox;
    SpinBox portNumberSpin;

    NamingContextHelper ncHelper;
};
}


void NameServerView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<NameServerView>(
        "NameServerView", N_("Nameserver"), ViewManager::SINGLE_OPTIONAL);
}


NameServerView::NameServerView()
{
    impl = new NameServerViewImpl(this);
}


NameServerViewImpl::NameServerViewImpl(NameServerView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();
    
    QHBoxLayout* hbox = new QHBoxLayout();
    hostAddressBox.setText("localhost");
    hostAddressBox.sigEditingFinished().connect
        (std::bind(&NameServerViewImpl::updateObjectList, this));
    hbox->addWidget(&hostAddressBox);

    portNumberSpin.setRange(0, 65535);
    portNumberSpin.setValue(2809);
    portNumberSpin.sigEditingFinished().connect
        (std::bind(&NameServerViewImpl::updateObjectList, this));
    hbox->addWidget(&portNumberSpin);

    PushButton* updateButton = new PushButton(_("Update"));
    updateButton->sigClicked().connect(std::bind(&NameServerViewImpl::updateObjectList, this));
    hbox->addWidget(updateButton);

    vbox->addLayout(hbox);

    treeWidget.setHeaderLabel(_("Object Name"));
    vbox->addWidget(&treeWidget);

    self->setLayout(vbox);
}


NameServerView::~NameServerView()
{
    delete impl;
}


NameServerViewImpl::~NameServerViewImpl()
{

}


void NameServerViewImpl::updateObjectList()
{
    treeWidget.clear();

    ncHelper.setLocation(hostAddressBox.string(), portNumberSpin.value());
    
    if(ncHelper.updateConnection()){

        NamingContextHelper::ObjectInfoList objects = ncHelper.getObjectList();

        for(size_t i=0; i < objects.size(); ++i){

            const NamingContextHelper::ObjectInfo& info = objects[i];
            //if(info.isAlive){
            if(true){

                QTreeWidgetItem* item = new QTreeWidgetItem();
                QString name = info.id_.c_str();
                if(!info.kind_.empty()){
                    name += QString("(%1)").arg(QString::fromStdString(info.kind_));
                }
                item->setText(0, name);
                treeWidget.addTopLevelItem(item);
            }
        }
    }
}
