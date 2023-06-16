#include "MocapConversionToBodyMotionDialog.h"
#include "MocapConversionToBodyMotionPanel.h"
#include "MarkerMotionItem.h"
#include "SkeletonMotionItem.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/MessageOut>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/ItemList>
#include <cnoid/Archive>
#include <cnoid/Button>
#include <cnoid/Separator>
#include <fmt/format.h>
#include <QLabel>
#include <QDialogButtonBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

Plugin* plugin = nullptr;
MocapConversionToBodyMotionDialog* dialog = nullptr;

}


void MocapConversionToBodyMotionDialog::initialize(Plugin* plugin)
{
    if(!::plugin){
        ::plugin = plugin;

        plugin->setProjectArchiver(
            "MocapConversionToBodyMotionDialog",
            [](Archive& archive){
                if(auto instance = MocapConversionToBodyMotionDialog::instance()){
                    return instance->store(archive);
                }
                return true;
            },
            [](const Archive& archive){
                return MocapConversionToBodyMotionDialog::getOrCreateInstance()->restore(archive);
            });
    }
}


MocapConversionToBodyMotionDialog* MocapConversionToBodyMotionDialog::getOrCreateInstance()
{
    if(!dialog){
        dialog = plugin->manage(new MocapConversionToBodyMotionDialog);
    }
    return dialog;
}


MocapConversionToBodyMotionDialog* MocapConversionToBodyMotionDialog::instance()
{
    return dialog;
}


MocapConversionToBodyMotionDialog::MocapConversionToBodyMotionDialog()
{
    setWindowTitle(_("Convert Skeleton / Marker Motion to Body Motion"));

    currentPanel = nullptr;

    auto vbox = new QVBoxLayout;

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Converter")));
    
    converterCombo.sigAboutToShowPopup().connect(
        [this](){ updateConverterCombo(); });
    converterCombo.sigCurrentIndexChanged().connect(
        [this](int index){ onConverterComboIndexChanged(index); });
    hbox->addWidget(&converterCombo, 1);
    vbox->addLayout(hbox);

    panelLayout = new QVBoxLayout;
    vbox->addLayout(panelLayout);
    
    vbox->addStretch();
    setLayout(vbox);

    vbox->addWidget(new HSeparator);
    auto applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    auto buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([this](){ accept(); });
    vbox->addWidget(buttonBox);

    updateConverterCombo();
}


void MocapConversionToBodyMotionDialog::updateConverterCombo()
{
    int currentIndex = converterCombo.currentIndex();;
    int n = MocapConversionToBodyMotionPanel::numConversionPanels();

    if(n > 0){
        int m = converterCombo.count();
        if(n > m){
            converterCombo.blockSignals(true);
            for(int i=m; i < n; ++i){
                auto caption = MocapConversionToBodyMotionPanel::getConversionPanelCaption(i);
                converterCombo.addItem(caption.c_str());
            }
            converterCombo.blockSignals(false);
        }
        
        string name;
        if(currentIndex >= 0){
            name = MocapConversionToBodyMotionPanel::getConversionPanelName(currentIndex);
        }
        if(name != currentConverterName){
            for(int i=0; i < n; ++i){
                if(i != currentIndex){
                    auto name = MocapConversionToBodyMotionPanel::getConversionPanelName(i);
                    if(name == currentConverterName){
                        currentIndex = i;
                        converterCombo.setCurrentIndex(i);
                        break;
                    }
                }
            }
        }
        if(currentIndex < 0){
            currentIndex = 0;
            converterCombo.setCurrentIndex(0);
            onConverterComboIndexChanged(0);
        }
    }
}


void MocapConversionToBodyMotionDialog::onConverterComboIndexChanged(int index)
{
    MocapConversionToBodyMotionPanel* panel = nullptr;
    currentConverterName.clear();
    if(index >= 0){
        currentConverterName = MocapConversionToBodyMotionPanel::getConversionPanelName(index);
        panel = MocapConversionToBodyMotionPanel::getOrCreateConversionPanel(index);
    }
    if(panel != currentPanel){
        if(currentPanel){
            currentPanel->hide();
            panelLayout->removeWidget(currentPanel);
        }
        if(panel){
            panelLayout->addWidget(panel);
            panel->show();
        }
        currentPanel = panel;
    }
}


void MocapConversionToBodyMotionDialog::setCurrentConverter(const std::string& name)
{
    currentConverterName = name;
    updateConverterCombo();
}


void MocapConversionToBodyMotionDialog::onAccepted()
{
    if(!currentPanel){
        return;
    }
    
    bool hasTargets = false;

    auto rootItem = RootItem::instance();
    auto bodyItems = rootItem->selectedItems<BodyItem>();
    for(auto& bodyItem : bodyItems){
        auto selectedItems = rootItem->selectedItems();
        for(auto& item : selectedItems){
            if(auto markerMotionItem = dynamic_cast<MarkerMotionItem*>(item.get())){
                convertMotionItemToBodyMotionItem(markerMotionItem, bodyItem);
                hasTargets = true;
            }
            if(auto skeletonMotionItem = dynamic_cast<SkeletonMotionItem*>(item.get())){
                convertMotionItemToBodyMotionItem(skeletonMotionItem, bodyItem);
                hasTargets = true;
            }
        }
    }

    if(!hasTargets){
        showWarningDialog(
            _("Select the motion item(s) to be converted and the target body item(s)."));
    }
}


bool MocapConversionToBodyMotionDialog::store(Archive& archive)
{
    archive.write("converter", currentConverterName);

    int n = MocapConversionToBodyMotionPanel::numConversionPanels();
    for(int i=0; i < n; ++i){
        if(auto panel = MocapConversionToBodyMotionPanel::getConversionPanel(i)){
            auto name = MocapConversionToBodyMotionPanel::getConversionPanelName(i);
            panel->storeConfigurations(archive.createMapping(name));
        }
    }
    
    return true;
}


void MocapConversionToBodyMotionDialog::restore(const Archive& archive)
{
    int n = MocapConversionToBodyMotionPanel::numConversionPanels();
    for(int i=0; i < n; ++i){
        auto name = MocapConversionToBodyMotionPanel::getConversionPanelName(i);
        auto conf = archive.findMapping(name);
        if(conf->isValid()){
            auto panel = MocapConversionToBodyMotionPanel::getOrCreateConversionPanel(i);
            panel->restoreConfigurations(conf);
        }
    }
    
    string converterName;
    if(archive.read("converter", converterName)){
        setCurrentConverter(converterName);
    }
}


template <class MotionItem>
void MocapConversionToBodyMotionDialog::convertMotionItemToBodyMotionItem(MotionItem* motionItem, BodyItem* bodyItem)
{
    BodyMotionItemPtr bodyMotionItem = new BodyMotionItem;

    bool result = currentPanel->convert(*motionItem->motion(), bodyItem->body(), *bodyMotionItem->motion());

    auto mout = MessageOut::master();

    if(result){
        bodyMotionItem->setName(format("{0}-{1}", motionItem->name(), bodyItem->name()));
        bodyMotionItem->setTemporary(true);
        bodyItem->addChildItem(bodyMotionItem);
        bodyMotionItem->setSelected(true);
        mout->putln(
            format(_("Motion \"{0}\" has been converted to the motion of \"{1}\"."),
                   motionItem->name(), bodyItem->name()));
    } else {
        mout->putErrorln(
            format(_("Failed to convert motion \"{0}\" to the motion of \"{1}\"."),
                   motionItem->name(), bodyItem->name()));
    }
}
