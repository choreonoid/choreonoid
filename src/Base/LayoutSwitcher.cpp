#include "LayoutSwitcher.h"
#include "MainWindow.h"
#include "ProjectManager.h"
#include "MenuManager.h"
#include "Buttons.h"
#include "Dialog.h"
#include "LineEdit.h"
#include <cnoid/ValueTree>
#include <QMenuBar>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QMouseEvent>
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

}

namespace cnoid {

class LayoutSwitcher::Impl
{
public:
    vector<LayoutInfoPtr> layoutInfos;
    QWidget* baseWidget;
    MenuManager menuManager;
    QHBoxLayout* buttonBox;
    NewLayoutDialog* dialog;

    Impl();
    void onAddButtonClicked();
    void storeCurrentLayoutAs(const std::string& name);
    void restoreLayout(LayoutInfo* info);
    void onLayoutContextMenuRequest(LayoutInfo* info, QMouseEvent* event);
    void updateLayout(LayoutInfo* info);
    void removeLayout(LayoutInfo* info);
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


LayoutSwitcher* LayoutSwitcher::instance()
{
    return instance_;
}


NewLayoutDialog::NewLayoutDialog(LayoutSwitcher::Impl* impl)
    : impl(impl)
{
    setWindowTitle(_("Register Layout"));
    auto vbox = new QVBoxLayout;
    setLayout(vbox);
    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name:")));
    nameEdit.setText(_("My Layout"));
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


LayoutSwitcher::LayoutSwitcher()
{
    impl = new Impl;
    instance_ = this;
}


LayoutSwitcher::Impl::Impl()
{
    auto mainWindow = MainWindow::instance();
    auto menuBar = mainWindow->menuBar();

    baseWidget = new QWidget;
    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(0, 0, 0, 0);
    hbox->setSpacing(0);
    buttonBox = new QHBoxLayout;
    hbox->addLayout(buttonBox);
    auto addButton = new ToolButton("+");
    addButton->setAutoRaise(true);
    addButton->sigClicked().connect([&](){ onAddButtonClicked(); });
    hbox->addWidget(addButton);
    baseWidget->setLayout(hbox);

    menuBar->setCornerWidget(baseWidget, Qt::TopRightCorner);

    dialog = nullptr;
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
        MainWindow::instance()->menuBar()->adjustSize();
        layoutInfos.push_back(info);
    }
}


void LayoutSwitcher::Impl::restoreLayout(LayoutInfo* info)
{
    ProjectManager::instance()->restoreLayout(info->layoutData);
}


void LayoutSwitcher::Impl::onLayoutContextMenuRequest(LayoutInfo* info, QMouseEvent* event)
{
    menuManager.setNewPopupMenu(baseWidget);
    menuManager.addItem("Update")->sigTriggered().connect(
        [this, info](){ updateLayout(info); });
    menuManager.addItem("Remove")->sigTriggered().connect(
        [this, info](){ removeLayout(info); });
    menuManager.popupMenu()->popup(event->globalPos());
    //menuManager.popupMenu()->popup(button->mapToGlobal(QPoint(0,0)));
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
    MainWindow::instance()->menuBar()->adjustSize();
}
