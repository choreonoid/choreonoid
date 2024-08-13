#include "GeneralSceneFileLoadDialog.h"
#include <cnoid/SceneLoader>
#include <cnoid/MessageOut>
#include <cnoid/ValueTree>
#include <cnoid/ComboBox>
#include <cnoid/Format>
#include <QBoxLayout>
#include <QLabel>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

SceneLoader::LengthUnitType defaultLengthUnitHint = SceneLoader::Meter;

}

namespace cnoid {

class GeneralSceneFileLoadDialog::OptionSet::Panel : public QWidget
{
public:
    ComboBox unitCombo;
    ComboBox axisCombo;

    Panel(OptionSet* optionSet);
};

}


void GeneralSceneFileLoadDialog::setDefaultLengthUnitHint(SceneLoader::LengthUnitType hint)
{
    defaultLengthUnitHint = hint;
}


GeneralSceneFileLoadDialog::GeneralSceneFileLoadDialog()
{
    initialize();
}


GeneralSceneFileLoadDialog::GeneralSceneFileLoadDialog(QWidget* parent, Qt::WindowFlags f)
    : FileDialog(parent, f)
{
    initialize();
}


void GeneralSceneFileLoadDialog::initialize()
{
    setWindowTitle(_("Load a scene file"));
    fileTypeCaption_ = _("Scene file");
    setViewMode(QFileDialog::List);
    setFileMode(QFileDialog::ExistingFile);
    setLabelText(QFileDialog::Accept, _("Load"));
    setLabelText(QFileDialog::Reject, _("Cancel"));

    insertOptionPanel(optionSet.panel());
}    


void GeneralSceneFileLoadDialog::setFileTypeCaption(const std::string& caption)
{
    fileTypeCaption_ = caption;
}


int GeneralSceneFileLoadDialog::exec()
{
    loadedScene_.reset();
    setNameFilter(makeNameFilter(fileTypeCaption_, SceneLoader::availableFileExtensions()));
    auto mout = MessageOut::master();
    
    int result = FileDialog::exec();
    if(result == QDialog::Accepted){
        if(!sceneLoader){
            sceneLoader.reset(new SceneLoader);
            sceneLoader->setMessageSink(mout->cout());
        }
        string filename = selectedFiles().at(0).toStdString();

        sceneLoader->setLengthUnitHint(optionSet.lengthUnitHint());
        sceneLoader->setUpperAxisHint(optionSet.upperAxisHint());

        mout->put(formatR(_("Loading the file \"{0}\" ..."), filename));
        loadedScene_ = sceneLoader->load(filename);
        if(loadedScene_){
            mout->putln(_(" OK!"));
        } else {
            mout->putErrorln(_(" Failed."));
        }
    }

    return result;
}


void GeneralSceneFileLoadDialog::clearLoadedScene()
{
    loadedScene_.reset();
}


GeneralSceneFileLoadDialog::OptionSet::OptionSet()
{
    resetOptions();
    panel_ = nullptr;
}


GeneralSceneFileLoadDialog::OptionSet::~OptionSet()
{
    if(panel_){
        delete panel_;
        panel_ = nullptr;
    }
}


void GeneralSceneFileLoadDialog::OptionSet::resetOptions()
{
    lengthUnitHint_ = defaultLengthUnitHint;
    upperAxisHint_ = SceneLoader::Z_Upper;
}
    
    
void GeneralSceneFileLoadDialog::OptionSet::setLengthUnitHint(AbstractSceneLoader::LengthUnitType hint)
{
    if(panel_){
        auto& combo = panel_->unitCombo;
        combo.setCurrentIndex(combo.findData(hint));
    } else {
        lengthUnitHint_ = hint;
    }
}


void GeneralSceneFileLoadDialog::OptionSet::setUpperAxisHint(SceneLoader::UpperAxisType hint)
{
    if(panel_){
        auto& combo = panel_->axisCombo;
        combo.setCurrentIndex(combo.findData(hint));
    } else {
        upperAxisHint_ = hint;
    }
}


void GeneralSceneFileLoadDialog::OptionSet::storeOptions(Mapping* archive)
{
    if(lengthUnitHint_ != SceneLoader::Meter){
        archive->write(
            "length_unit_hint",
            (lengthUnitHint_ == SceneLoader::Millimeter) ? "millimeter" : "inch");
    }
    
    if(upperAxisHint_ != SceneLoader::Z_Upper){
        archive->write("upper_axis_hint", "Y");
    }
}

    
bool GeneralSceneFileLoadDialog::OptionSet::restoreOptions(const Mapping* archive)
{
    string symbol;

    lengthUnitHint_ = SceneLoader::Meter;
    if(archive->read({ "length_unit_hint", "meshLengthUnitHint" }, symbol)){
        if(symbol == "millimeter"){
            setLengthUnitHint(SceneLoader::Millimeter);
        } else if(symbol == "inch"){
            setLengthUnitHint(SceneLoader::Inch);
        }
    }

    upperAxisHint_ = SceneLoader::Z_Upper;
    if(archive->read({ "upper_axis_hint", "meshUpperAxisHint" }, symbol)){
        if(symbol == "Y"){
            setUpperAxisHint(SceneLoader::Y_Upper);
        }
    }

    return true;
}


QWidget* GeneralSceneFileLoadDialog::OptionSet::panel()
{
    if(!panel_){
        panel_ = new Panel(this);
    }
    return panel_;
}


GeneralSceneFileLoadDialog::OptionSet::Panel::Panel(OptionSet* optionSet)
{
    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(0, 0, 0, 0);
    setLayout(hbox);

    hbox->addWidget(new QLabel(_("[ import hints ]")));

    hbox->addWidget(new QLabel(_("Unit:")));
    unitCombo.addItem(_("Meter"), SceneLoader::Meter);
    unitCombo.addItem(_("Millimeter"), SceneLoader::Millimeter);
    unitCombo.addItem(_("Inch"), SceneLoader::Inch);
    unitCombo.setCurrentIndex(unitCombo.findData(optionSet->lengthUnitHint()));
    unitCombo.sigCurrentIndexChanged().connect(
        [this, optionSet](int index){
            optionSet->lengthUnitHint_ =
                static_cast<SceneLoader::LengthUnitType>(unitCombo.itemData(index).toInt());
        });
    hbox->addWidget(&unitCombo);
    
    hbox->addWidget(new QLabel(_("Upper axis:")));
    axisCombo.addItem("Z", SceneLoader::Z_Upper);
    axisCombo.addItem("Y", SceneLoader::Y_Upper);
    axisCombo.setCurrentIndex(axisCombo.findData(optionSet->upperAxisHint()));
    axisCombo.sigCurrentIndexChanged().connect(
        [this, optionSet](int index){
            optionSet->upperAxisHint_ =
                static_cast<SceneLoader::UpperAxisType>(unitCombo.itemData(index).toInt());
        });
    hbox->addWidget(&axisCombo);
}
