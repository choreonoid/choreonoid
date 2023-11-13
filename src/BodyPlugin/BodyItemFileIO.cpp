#include "BodyItemFileIO.h"
#include "BodyItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/GeneralSceneFileImporterBase>
#include <cnoid/BodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/StdSceneWriter>
#include <cnoid/ObjSceneWriter>
#include <cnoid/ItemManager>
#include <cnoid/SceneGraph>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QLabel>
#include <QSpinBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

/**
   \todo This class should be integrated with StdSceneFileExporter
*/
class SceneFileImporter : public GeneralSceneFileImporterBase
{
public:
    SceneFileImporter();
    virtual Item* createItem() override;
    virtual bool load(Item* item, const std::string& filename) override;
};

class SceneFileExporterBase : public BodyItemFileIoBase
{
public:
    QComboBox* shapeTypeCombo;
    QComboBox* coordinateCombo;

    SceneFileExporterBase(const char* caption, const char* format, const char* extension);
    virtual bool save(BodyItem* item, const std::string& filename) override;
    SgNode* strip(SgGroup* group, bool& out_stripped);
    virtual bool saveScene(const std::string& filename, SgGroup* scene) = 0;
    void addShapeTypeCombo(QBoxLayout* box);
    void addCoordinateCombo(QBoxLayout* box);
};

class StdSceneFileExporter : public SceneFileExporterBase
{
    unique_ptr<StdSceneWriter> sceneWriter;

public:
    StdSceneFileExporter();
    StdSceneWriter* ensureSceneWriter();
    virtual bool saveScene(const std::string& filename, SgGroup* scene) override;
    virtual void createOptionPanelForSaving() override;
    virtual void fetchOptionPanelForSaving() override;
};

class ObjFileExporter : public SceneFileExporterBase
{
    unique_ptr<ObjSceneWriter> sceneWriter;

public:
    ObjFileExporter();
    ObjSceneWriter* ensureSceneWriter();
    virtual bool saveScene(const std::string& filename, SgGroup* scene) override;
    virtual void createOptionPanelForSaving() override;
};


BodyItemBodyFileIO* bodyFileIO;
SceneFileImporter* meshFileIO;

}


void BodyItem::registerBodyItemFileIoSet(ItemManager* im)
{
    ::bodyFileIO = new BodyItemBodyFileIO;
    im->addFileIO<BodyItem>(::bodyFileIO);

    ::meshFileIO = new SceneFileImporter;
    im->addFileIO<BodyItem>(::meshFileIO);

    im->addFileIO<BodyItem>(new StdSceneFileExporter);
    im->addFileIO<BodyItem>(new ObjFileExporter);
}


BodyItemBodyFileIO* BodyItem::bodyFileIO()
{
    return ::bodyFileIO;
}


GeneralSceneFileImporterBase* BodyItem::meshFileIO()
{
    return ::meshFileIO;
}


BodyItemFileIoBase::BodyItemFileIoBase(const char* format, int api)
    : ItemFileIoBase<BodyItem>(format, api)
{
    optionPanel = nullptr;
    extModelFileModeCombo = nullptr;
    transformIntegrationCheck = nullptr;
}


BodyItemFileIoBase::~BodyItemFileIoBase()
{
    if(optionPanel){
        delete optionPanel;
    }
}


QWidget* BodyItemFileIoBase::getOptionPanelForSaving(BodyItem* /* item */)
{
    if(!optionPanel){
        optionPanel = new QWidget;
        optionVBox = new QVBoxLayout;
        optionVBox->setContentsMargins(0, 0, 0, 0);
        optionPanel->setLayout(optionVBox);
        
        createOptionPanelForSaving();
    }
    return optionPanel;
}


void BodyItemFileIoBase::addExtModelFileModeCombo(QBoxLayout* box)
{
    box->addWidget(new QLabel(_("Ext model file mode:")));
    extModelFileModeCombo = new QComboBox;
    extModelFileModeCombo->addItem(
        _("Link to the original model files"), StdSceneWriter::LinkToOriginalModelFiles);
    extModelFileModeCombo->addItem(
        _("Copy model files"), StdSceneWriter::CopyModelFiles);
    extModelFileModeCombo->addItem(
        _("Embed models"), StdSceneWriter::EmbedModels);
    extModelFileModeCombo->addItem(
        _("Replace with standard scene files"), StdSceneWriter::ReplaceWithStdSceneFiles);
    extModelFileModeCombo->addItem(
        _("Replace with OBJ model files"), StdSceneWriter::ReplaceWithObjModelFiles);
    box->addWidget(extModelFileModeCombo);
}


void BodyItemFileIoBase::addTransformIntegrationCheck(QBoxLayout* box)
{
    transformIntegrationCheck = new QCheckBox;
    transformIntegrationCheck->setText(_("Integrate transforms"));
    box->addWidget(transformIntegrationCheck);
}


BodyItemBodyFileIO::BodyItemBodyFileIO()
    : BodyItemFileIoBase("CHOREONOID-BODY", Load | Save | Options | OptionPanelForSaving)
{
    setCaption(_("Body"));
    setExtensionsForLoading({ "body", "yaml", "yml", "wrl" });
    setExtensionsForSaving({ "body" });
    addFormatAlias("OpenHRP-VRML-MODEL");
    setItemNameUpdateInSavingEnabled(false);

    bodyLoader_ = nullptr;
    bodyWriter_ = nullptr;
}


BodyItemBodyFileIO::~BodyItemBodyFileIO()
{
    if(bodyLoader_){
        delete bodyLoader_;
    }
    if(bodyWriter_){
        delete bodyWriter_;
    }
}


BodyLoader* BodyItemBodyFileIO::ensureBodyLoader()
{
    if(!bodyLoader_){
        bodyLoader_ = new BodyLoader;
        bodyLoader_->setMessageSink(os());
    }
    return bodyLoader_;
}


bool BodyItemBodyFileIO::load(BodyItem* item, const std::string& filename)
{
    BodyPtr newBody = new Body;
    if(!ensureBodyLoader()->load(newBody, filename)){
        return false;
    }
    item->setBody(newBody);
    
    auto itype = currentInvocationType();
    if(itype == Dialog || itype == DragAndDrop){
        item->setChecked(true);
    }
    
    return true;
}


StdBodyWriter* BodyItemBodyFileIO::ensureBodyWriter()
{
    if(!bodyWriter_){
        bodyWriter_ = new StdBodyWriter;
        bodyWriter_->setMessageSink(os());
        bodyWriter_->setOriginalShapeExtModelFileUriRewritingEnabled(true);
    }
    return bodyWriter_;
}


void BodyItemBodyFileIO::createOptionPanelForSaving()
{
    auto hbox = new QHBoxLayout;
    addExtModelFileModeCombo(hbox);
    addTransformIntegrationCheck(hbox);
    hbox->addStretch();
    optionVBox->addLayout(hbox);
}


void BodyItemBodyFileIO::fetchOptionPanelForSaving()
{
    ensureBodyWriter();
    int mode = extModelFileModeCombo->currentData().toInt();
    bodyWriter_->setExtModelFileMode(mode);
    bodyWriter_->setTransformIntegrationEnabled(transformIntegrationCheck->isChecked());
}


bool BodyItemBodyFileIO::save(BodyItem* item, const std::string& filename)
{
    ensureBodyWriter();

    filesystem::path itemFilePath(fromUTF8(item->filePath()));
    if(!itemFilePath.empty()){
        bodyWriter_->setOriginalBaseDirectory(toUTF8(itemFilePath.parent_path().generic_string()));
    } else {
        bodyWriter_->setOriginalBaseDirectory("");
    }
    
    if(bodyWriter_->writeBody(item->body(), filename)){
        if(auto overwriteAddon = item->findAddon<BodyOverwriteAddon>()){
            overwriteAddon->removeOverwriteItems(false);
        }
        return true;
    }
    return false;
}


namespace {

SceneFileImporter::SceneFileImporter()
{
    setCaption(_("Body"));
    setFileTypeCaption(_("Scene / Mesh"));
}


Item* SceneFileImporter::createItem()
{
    return new BodyItem;
}


bool SceneFileImporter::load(Item* item, const std::string& filename)
{
    SgNode* shape = loadScene(filename);
    if(!shape){
        return false;
    }
    
    auto bodyItem = static_cast<BodyItem*>(item);
    BodyPtr newBody = new Body;
    newBody->rootLink()->addShapeNode(shape);
    bodyItem->setBody(newBody);
    
    auto itype = currentInvocationType();
    if(itype == Dialog || itype == DragAndDrop){
        item->setChecked(true);
    }
    
    return true;
}


SceneFileExporterBase::SceneFileExporterBase(const char* caption, const char* format, const char* extension)
    : BodyItemFileIoBase(format, Save | Options | OptionPanelForSaving)
{
    setCaption(caption);
    setExtensionForSaving(extension);
    setInterfaceLevel(Conversion);
}


bool SceneFileExporterBase::save(BodyItem* item, const std::string& filename)
{
    bool saved = false;
    
    auto body = item->body();
    int numLinks = body->numLinks();
    vector<SgNode*> nodesToClearName;
    nodesToClearName.reserve(numLinks);
    const Isometry3 T0 = body->rootLink()->T();
    SgGroupPtr scene = new SgGroup;
    bool useGlobalCoordinate = (coordinateCombo->currentIndex() == 0);
    
    for(auto& link : body->links()){
        bool stripped;
        SgGroup* linkShapeGroup;
        if(shapeTypeCombo->currentIndex() == 0){
            linkShapeGroup = link->visualShape();
        } else {
            linkShapeGroup = link->collisionShape();
        }
        if(SgNode* shape = strip(linkShapeGroup, stripped)){
            if(useGlobalCoordinate || !link->T().isApprox(T0)){
                Isometry3 T;
                if(useGlobalCoordinate){
                    T = link->T();
                } else {
                    T = T0.inverse() * link->T();
                }
                auto transform = new SgPosTransform(T);
                if(stripped){
                    transform->addChild(shape);
                } else {
                    linkShapeGroup->copyChildrenTo(transform);
                }
                transform->setName(link->name());
                shape = transform;
            } else if(shape->name().empty()){
                shape->setName(link->name());
                nodesToClearName.push_back(shape);
            }
            scene->addChild(shape);
        }
    }
    if(!scene->empty()){
        saved = saveScene(filename, scene);
    }
    // Clear temporary names
    for(auto& node : nodesToClearName){
        node->setName("");
    }
    
    return saved;
}


SgNode* SceneFileExporterBase::strip(SgGroup* group, bool& out_stripped)
{
    int n = group->numChildren();
    if(n >= 2){
        out_stripped = false;
        return group;
    } else if(n == 1){
        out_stripped = true;
        return group->child(0);
    }
    return nullptr;
}


void SceneFileExporterBase::addShapeTypeCombo(QBoxLayout* box)
{
    box->addWidget(new QLabel(_("Shape type:")));
    shapeTypeCombo = new QComboBox;
    shapeTypeCombo->addItem(_("Visual"));
    shapeTypeCombo->addItem(_("Collision"));
    box->addWidget(shapeTypeCombo);
}


void SceneFileExporterBase::addCoordinateCombo(QBoxLayout* box)
{
    box->addWidget(new QLabel(_("Coordinate:")));
    coordinateCombo = new QComboBox;
    coordinateCombo->addItem(_("Global"));
    coordinateCombo->addItem(_("Local"));
    coordinateCombo->setCurrentIndex(1);
    box->addWidget(coordinateCombo);
}


StdSceneFileExporter::StdSceneFileExporter()
    : SceneFileExporterBase(_("Standard scene file"), "CHOREONOID-SCENE", "scen")
{

}


StdSceneWriter* StdSceneFileExporter::ensureSceneWriter()
{
    if(!sceneWriter){
        sceneWriter.reset(new StdSceneWriter);
        sceneWriter->setMessageSink(os());
        sceneWriter->setIndentWidth(1);
    }
    return sceneWriter.get();
}


bool StdSceneFileExporter::saveScene(const std::string& filename, SgGroup* scene)
{
    bool saved = false;
    vector<SgNode*> shapes;
    for(auto& node : *scene){
        shapes.push_back(node);
    }
    if(!shapes.empty()){
        saved = ensureSceneWriter()->writeScene(filename, shapes);
    }
    return saved;
}


void StdSceneFileExporter::createOptionPanelForSaving()
{
    auto vbox = new QVBoxLayout;
    auto hbox = new QHBoxLayout;
    addShapeTypeCombo(hbox);
    addCoordinateCombo(hbox);
    addTransformIntegrationCheck(hbox);
    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout;
    addExtModelFileModeCombo(hbox);
    hbox->addStretch();
    vbox->addLayout(hbox);
    optionVBox->addLayout(vbox);
}


void StdSceneFileExporter::fetchOptionPanelForSaving()
{
    ensureSceneWriter();
    int mode = extModelFileModeCombo->currentData().toInt();
    sceneWriter->setExtModelFileMode(mode);
    sceneWriter->setTransformIntegrationEnabled(transformIntegrationCheck->isChecked());
}


ObjFileExporter::ObjFileExporter()
    : SceneFileExporterBase(_("OBJ file"), "OBJ-FILE", "obj")
{

}


ObjSceneWriter* ObjFileExporter::ensureSceneWriter()
{
    if(!sceneWriter){
        sceneWriter.reset(new ObjSceneWriter);
    }
    return sceneWriter.get();
}


bool ObjFileExporter::saveScene(const std::string& filename, SgGroup* scene)
{
    return ensureSceneWriter()->writeScene(filename, scene);
}


void ObjFileExporter::createOptionPanelForSaving()
{
    auto hbox = new QHBoxLayout;
    addShapeTypeCombo(hbox);
    addCoordinateCombo(hbox);
    hbox->addStretch();
    optionVBox->addLayout(hbox);
}

}
