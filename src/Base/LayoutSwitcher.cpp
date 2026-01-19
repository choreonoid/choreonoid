#include "LayoutSwitcher.h"
#include "ProjectManager.h"
#include "MenuManager.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include "Buttons.h"
#include "Dialog.h"
#include "LineEdit.h"
#include "QtEventUtil.h"
#include "LazyCaller.h"
#include <cnoid/ValueTree>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QMouseEvent>
#include <QMenuBar>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

LayoutSwitcher* instance_ = nullptr;

class LayoutButton : public ToolButton
{
public:
    function<void(QMouseEvent* event)> callbackOnContextMenuRrequest;
    
    LayoutButton(const string& caption);
    virtual void mousePressEvent(QMouseEvent* event) override;
};

class LayoutInfo : public Referenced
{
public:
    string name;
    MappingPtr layoutData;
    LayoutButton* button;

    ~LayoutInfo();
};

typedef ref_ptr<LayoutInfo> LayoutInfoPtr;

class NewLayoutDialog : public Dialog
{
public:
    LayoutSwitcher::Impl* impl;
    LineEdit nameEdit;

    NewLayoutDialog(LayoutSwitcher::Impl* impl);
    virtual void onAccepted() override;
};

class RenameLayoutDialog : public Dialog
{
public:
    LayoutSwitcher::Impl* impl;
    LayoutInfo* targetInfo;
    LineEdit nameEdit;

    RenameLayoutDialog(LayoutSwitcher::Impl* impl);
    void setTargetLayout(LayoutInfo* info);
    virtual void onAccepted() override;
};

}

namespace cnoid {

class LayoutSwitcher::Impl
{
public:
    LayoutSwitcher* self;
    vector<LayoutInfoPtr> layoutInfos;
    MenuManager menuManager;
    QHBoxLayout* buttonBox;
    NewLayoutDialog* dialog;
    RenameLayoutDialog* renameDialog;

    Impl(LayoutSwitcher* self);
    void onAddButtonClicked();
    void adjustSize();
    void storeCurrentLayoutAs(const std::string& name);
    void restoreLayout(LayoutInfo* info);
    void onLayoutContextMenuRequest(LayoutInfo* info, QMouseEvent* event);
    void renameLayout(LayoutInfo* info);
    void updateLayout(LayoutInfo* info);
    void removeLayout(LayoutInfo* info);
    bool store(Archive& archive);
    void restore(const Archive& archive);
};

}


LayoutButton::LayoutButton(const string& caption)
    : ToolButton(caption.c_str())
{
    setAutoRaise(true);
}


void LayoutButton::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::RightButton){
        callbackOnContextMenuRrequest(event);
    }
    ToolButton::mousePressEvent(event);
}


LayoutInfo::~LayoutInfo()
{
    delete button;
}


NewLayoutDialog::NewLayoutDialog(LayoutSwitcher::Impl* impl)
    : impl(impl)
{
    setWindowTitle(_("Register Layout"));
    auto vbox = new QVBoxLayout;
    setLayout(vbox);
    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name:")));
    nameEdit.setText(_("Layout"));
    hbox->addWidget(&nameEdit);
    vbox->addLayout(hbox);

    auto buttonBox = new QDialogButtonBox(this);
    auto okButton = new PushButton(_("&Register"));
    auto cancelButton = new PushButton(_("&Cancel"));
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox, &QDialogButtonBox::accepted, [this](){ this->accept(); });
    connect(buttonBox, &QDialogButtonBox::rejected, [this](){ this->reject(); });
    vbox->addWidget(buttonBox);
}


RenameLayoutDialog::RenameLayoutDialog(LayoutSwitcher::Impl* impl)
    : impl(impl),
      targetInfo(nullptr)
{
    setWindowTitle(_("Rename Layout"));
    auto vbox = new QVBoxLayout;
    setLayout(vbox);
    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name:")));
    hbox->addWidget(&nameEdit);
    vbox->addLayout(hbox);

    auto buttonBox = new QDialogButtonBox(this);
    auto okButton = new PushButton(_("&OK"));
    auto cancelButton = new PushButton(_("&Cancel"));
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox, &QDialogButtonBox::accepted, [this](){ this->accept(); });
    connect(buttonBox, &QDialogButtonBox::rejected, [this](){ this->reject(); });
    vbox->addWidget(buttonBox);
}


void RenameLayoutDialog::setTargetLayout(LayoutInfo* info)
{
    targetInfo = info;
    nameEdit.setText(info->name.c_str());
}


void RenameLayoutDialog::onAccepted()
{
    auto newName = nameEdit.text().toStdString();
    if(!newName.empty() && targetInfo){
        targetInfo->name = newName;
        targetInfo->button->setText(newName.c_str());
        impl->adjustSize();
    }
}


LayoutSwitcher::LayoutSwitcher()
{
    impl = new Impl(this);
    instance_ = this;
}


LayoutSwitcher::Impl::Impl(LayoutSwitcher* self)
    : self(self)
{
    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(0, 0, 0, 0);
    hbox->setSpacing(0);
    buttonBox = new QHBoxLayout;
    hbox->addLayout(buttonBox);
    auto addButton = new ToolButton("+");
    addButton->setAutoRaise(true);
    addButton->sigClicked().connect([&](){ onAddButtonClicked(); });
    hbox->addWidget(addButton);
    self->setLayout(hbox);

    dialog = nullptr;
    renameDialog = nullptr;
}


LayoutSwitcher::~LayoutSwitcher()
{
    delete impl;
}


void LayoutSwitcher::Impl::onAddButtonClicked()
{
    if(!dialog){
        dialog = new NewLayoutDialog(this);
    }
    dialog->show();
}


void NewLayoutDialog::onAccepted()
{
    auto name = nameEdit.text().toStdString();
    if(!name.empty()){
        impl->storeCurrentLayoutAs(name);
    }
}


void LayoutSwitcher::Impl::adjustSize()
{
    auto doAdjustSize = [this](){
        auto parent = self->parentWidget();
        while(parent){
            if(auto menuBar = dynamic_cast<QMenuBar*>(parent)){
                menuBar->adjustSize();
                break;
            }
            parent = parent->parentWidget();
        }
    };

#ifdef Q_OS_WIN
    callLater(doAdjustSize);
#else
    doAdjustSize();
#endif
}


void LayoutSwitcher::Impl::storeCurrentLayoutAs(const std::string& name)
{
    auto layoutData = ProjectManager::instance()->storeCurrentLayout();
    if(layoutData){
        auto info = new LayoutInfo;
        info->name = name;
        info->layoutData = layoutData;
        auto button = new LayoutButton(name);
        button->sigClicked().connect(
            [this, info](){ restoreLayout(info); });
        button->callbackOnContextMenuRrequest =
            [this, info](QMouseEvent* event){ onLayoutContextMenuRequest(info, event); };
        buttonBox->addWidget(button);
        info->button = button;

        adjustSize();

        layoutInfos.push_back(info);
    }
}


void LayoutSwitcher::Impl::restoreLayout(LayoutInfo* info)
{
    ProjectManager::instance()->restoreLayout(info->layoutData);
}


void LayoutSwitcher::Impl::onLayoutContextMenuRequest(LayoutInfo* info, QMouseEvent* event)
{
    menuManager.setNewPopupMenu(self);
    menuManager.addItem(_("Rename"))->sigTriggered().connect(
        [this, info](){ renameLayout(info); });
    menuManager.addItem(_("Update"))->sigTriggered().connect(
        [this, info](){ updateLayout(info); });
    menuManager.addItem(_("Remove"))->sigTriggered().connect(
        [this, info](){ removeLayout(info); });
    menuManager.popupMenu()->popup(getGlobalPosition(event));
}


void LayoutSwitcher::Impl::renameLayout(LayoutInfo* info)
{
    if(!renameDialog){
        renameDialog = new RenameLayoutDialog(this);
    }
    renameDialog->setTargetLayout(info);
    renameDialog->show();
}


void LayoutSwitcher::Impl::updateLayout(LayoutInfo* info)
{
    if(auto layoutData = ProjectManager::instance()->storeCurrentLayout()){
        info->layoutData = layoutData;
    }
}


void LayoutSwitcher::Impl::removeLayout(LayoutInfo* info)
{
    auto& infos = layoutInfos;
    infos.erase(std::remove(infos.begin(), infos.end(), info), infos.end());
    adjustSize();
}


void LayoutSwitcher::initializeClass(ExtensionManager* ext)
{
    ext->setProjectArchiver(
        "LayoutSwitcher",
        [](Archive& archive){
            if(instance_){
                return instance_->store(archive);
            }
            return false;
        },
        [](const Archive& archive){
            if(instance_){
                instance_->restore(archive);
            }
        });
}


bool LayoutSwitcher::Impl::store(Archive& archive)
{
    if(layoutInfos.empty()){
        return false;
    }

    ListingPtr layouts = new Listing;
    for(auto& info : layoutInfos){
        MappingPtr layoutEntry = new Mapping;
        layoutEntry->write("name", info->name, DOUBLE_QUOTED);
        if(info->layoutData){
            layoutEntry->insert(info->layoutData);
        }
        layouts->append(layoutEntry);
    }
    archive.insert("layouts", layouts);
    return true;
}


bool LayoutSwitcher::store(Archive& archive)
{
    return impl->store(archive);
}


void LayoutSwitcher::Impl::restore(const Archive& archive)
{
    // Clear existing layouts
    layoutInfos.clear();
    while(buttonBox->count() > 0){
        auto item = buttonBox->takeAt(0);
        delete item->widget();
        delete item;
    }

    auto layouts = archive.findListing("layouts");
    if(!layouts->isValid()){
        return;
    }

    for(int i = 0; i < layouts->size(); ++i){
        auto layoutEntry = layouts->at(i)->toMapping();
        if(!layoutEntry){
            continue;
        }

        string name;
        if(!layoutEntry->read("name", name)){
            continue;
        }

        auto info = new LayoutInfo;
        info->name = name;
        info->layoutData = layoutEntry;

        auto button = new LayoutButton(name);
        button->sigClicked().connect(
            [this, info](){ restoreLayout(info); });
        button->callbackOnContextMenuRrequest =
            [this, info](QMouseEvent* event){
                onLayoutContextMenuRequest(info, event);
            };
        buttonBox->addWidget(button);
        info->button = button;

        layoutInfos.push_back(info);
    }

    if(!layoutInfos.empty()){
        adjustSize();
    }
}


void LayoutSwitcher::restore(const Archive& archive)
{
    impl->restore(archive);
}
