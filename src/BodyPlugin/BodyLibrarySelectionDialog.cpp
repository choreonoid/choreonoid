#include "BodyLibrarySelectionDialog.h"
#include <cnoid/TreeWidget>
#include <cnoid/CheckBox>
#include <cnoid/PushButton>
#include <QLabel>
#include <QBoxLayout>
#include <QDialogButtonBox>
#include <QFileIconProvider>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class GroupItem : public QTreeWidgetItem
{
public:
    BodyLibraryView::ElementPtr element;
};

}

namespace cnoid {

class BodyLibrarySelectionDialog::Impl
{
public:
    BodyLibrarySelectionDialog* self;
    TreeWidget treeWidget;
    Connection selectionChangeConnection;
    CheckBox* checkBoxToStoreFile;
    PushButton* acceptButton;
    std::function<void(BodyLibraryView::ElementPtr group)> functionOnAccepted;

    Impl(BodyLibrarySelectionDialog* self);
    void updateToLatestBodyLibraryContents(BodyLibraryView::ElementPtr rootElement);
    void addGroups(BodyLibraryView::ElementPtr element, QTreeWidgetItem* parentItem);
    void updateAcceptButtonState();
    void onAcceptButtonClicked();    
};

}


BodyLibrarySelectionDialog::BodyLibrarySelectionDialog(QWidget* parent)
    : Dialog(parent)
{
    impl = new Impl(this);
}


BodyLibrarySelectionDialog::Impl::Impl(BodyLibrarySelectionDialog* self_)
    : self(self_)
{
    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    vbox->addWidget(new QLabel(_("Group selection")));

    treeWidget.setHeaderHidden(true);
    treeWidget.setColumnCount(1);
    treeWidget.setSelectionMode(QAbstractItemView::SingleSelection);
    treeWidget.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    treeWidget.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    treeWidget.setDragDropMode(QAbstractItemView::NoDragDrop);

    selectionChangeConnection =
        treeWidget.sigItemSelectionChanged().connect(
            [this]{ updateAcceptButtonState(); });
    
    vbox->addWidget(&treeWidget);

    checkBoxToStoreFile = new CheckBox(_("Store the body file in the library directory"));
    checkBoxToStoreFile->setVisible(false);
    vbox->addWidget(checkBoxToStoreFile);

    auto buttonBox = new QDialogButtonBox(self);
    acceptButton = new PushButton(_("&Accept"));
    acceptButton->setDefault(true);
    acceptButton->sigClicked().connect([this]{ onAcceptButtonClicked(); });
    buttonBox->addButton(acceptButton, QDialogButtonBox::AcceptRole);

    auto cancelButton = new PushButton(_("&Cancel"));
    cancelButton->sigClicked().connect([this]{ self->reject(); });
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    vbox->addWidget(buttonBox);
}


BodyLibrarySelectionDialog::~BodyLibrarySelectionDialog()
{
    delete impl;
}


void BodyLibrarySelectionDialog::setAcceptButtonLabel(const char* label)
{
    impl->acceptButton->setText(label);
}


CheckBox* BodyLibrarySelectionDialog::checkBoxToStoreFile()
{
    return impl->checkBoxToStoreFile;
}


void BodyLibrarySelectionDialog::updateToLatestBodyLibraryContents(BodyLibraryView::ElementPtr rootElement)
{
    impl->updateToLatestBodyLibraryContents(rootElement);
}


void BodyLibrarySelectionDialog::Impl::updateToLatestBodyLibraryContents(BodyLibraryView::ElementPtr rootElement)
{
    selectionChangeConnection.block();

    treeWidget.clear();
    auto parentItem = treeWidget.invisibleRootItem();
    addGroups(rootElement, parentItem);

    selectionChangeConnection.unblock();
    updateAcceptButtonState();
}


void BodyLibrarySelectionDialog::Impl::addGroups(BodyLibraryView::ElementPtr element, QTreeWidgetItem* parentItem)
{
    if(element->isGroup()){
        auto item = new GroupItem;
        item->element = element;
        if(element->isRoot()){
            item->setText(0, _("Top-level"));
        } else {
            item->setText(0, element->name().c_str());
        }
        static QFileIconProvider provider;
        item->setIcon(0, provider.icon(QFileIconProvider::Folder));
        parentItem->addChild(item);
        item->setExpanded(true);
        for(auto& child : element->elements()){
            addGroups(child, item);
        }
    }
}


void BodyLibrarySelectionDialog::setFunctionOnAccepted(std::function<void(BodyLibraryView::ElementPtr group)> func)
{
    impl->functionOnAccepted = func;
}


void BodyLibrarySelectionDialog::Impl::updateAcceptButtonState()
{
    acceptButton->setEnabled(!treeWidget.selectedItems().empty());
}


void BodyLibrarySelectionDialog::Impl::onAcceptButtonClicked()
{
    auto selected = treeWidget.selectedItems();
    if(selected.size() == 1){
        auto item = static_cast<GroupItem*>(selected[0]);
        if(functionOnAccepted){
            functionOnAccepted(item->element);
            self->accept();
        }
    }
}
