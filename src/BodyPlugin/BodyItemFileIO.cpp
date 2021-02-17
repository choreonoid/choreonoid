#include "BodyItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemFileIO>  
#include <cnoid/SceneItemFileIO>
#include <cnoid/BodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/StdSceneWriter>
#include <cnoid/ObjSceneWriter>
#include <cnoid/ItemManager>
#include <cnoid/SceneGraph>
#include <QBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

ItemFileIO* bodyFileIO;
ItemFileIO* meshFileIO;
ItemFileIO* stdSceneFileExport;


class BodyFileIO : public ItemFileIOBase<BodyItem>
{
    BodyLoader bodyLoader;
    unique_ptr<StdBodyWriter> bodyWriter;
    
public:
    BodyFileIO()
        : ItemFileIOBase<BodyItem>("CHOREONOID-BODY", Load | Save)
    {
        setCaption(_("Body"));
        setExtensions({ "body", "yaml", "yml", "wrl" });
        addFormatIdAlias("OpenHRP-VRML-MODEL");

        bodyLoader.setMessageSink(os());
    }

    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        BodyPtr newBody = new Body;
        if(!bodyLoader.load(newBody, filename)){
            return false;
        }
        item->setBody(newBody);

        if(item->name().empty()){
            item->setName(newBody->modelName());
        } else {
            newBody->setName(item->name());
        }

        auto itype = invocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        
        return true;
    }

    virtual bool save(BodyItem* item, const std::string& filename) override
    {
        if(!bodyWriter){
            bodyWriter.reset(new StdBodyWriter);
        }
        if(bodyWriter->writeBody(item->body(), filename)){
            item->getAddon<BodyOverwriteAddon>()->clearOverwriteItems();
            return true;
        }
        return false;
    }
};

class SceneFileIO : public SceneItemFileIO
{
public:
    SceneFileIO()
    {
        setCaption(_("Body"));
        setFileTypeCaption(_("Scene / Mesh"));
    }

    virtual Item* createItem() override
    {
        return new BodyItem;
    }

    virtual bool load(Item* item, const std::string& filename) override
    {
        SgNode* shape = loadScene(filename);
        if(!shape){
            return false;
        }

        auto bodyItem = static_cast<BodyItem*>(item);
        BodyPtr newBody = new Body;
        newBody->rootLink()->addShapeNode(shape);
        bodyItem->setBody(newBody);

        auto itype = invocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        
        return true;
    }
};

class SceneFileExportBase : public ItemFileIOBase<BodyItem>
{
public:
    QWidget* optionPanel;
    QVBoxLayout* optionVBox;
    QComboBox* shapeTypeCombo;

    SceneFileExportBase(const char* caption, const char* format, const char* extension)
        : ItemFileIOBase<BodyItem>(format, Save | Options | OptionPanelForSaving)
    {
        setCaption(caption);
        setExtensions({ extension });
        setInterfaceLevel(Conversion);

        optionPanel = nullptr;        
    }

    ~SceneFileExportBase()
    {
        if(optionPanel){
            delete optionPanel;
        }
    }

    virtual bool save(BodyItem* item, const std::string& filename) override
    {
        bool saved = false;

        auto body = item->body();
        int numLinks = body->numLinks();
        vector<SgNode*> nodesToClearName;
        nodesToClearName.reserve(numLinks);
        const Isometry3 T0 = body->rootLink()->T();
        SgGroupPtr scene = new SgGroup;
        for(auto& link : body->links()){
            bool stripped;
            SgGroup* linkShapeGroup;
            if(shapeTypeCombo->currentIndex() == 0){
                linkShapeGroup = link->visualShape();
            } else {
                linkShapeGroup = link->collisionShape();
            }
            if(SgNode* shape = strip(linkShapeGroup, stripped)){
                if(!link->T().isApprox(T0)){
                    auto transform = new SgPosTransform(T0.inverse() * link->T());
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

    SgNode* strip(SgGroup* group, bool& out_stripped)
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

    virtual bool saveScene(const std::string& filename, SgGroup* scene) = 0;

    virtual QWidget* getOptionPanelForSaving(BodyItem* /* item */) override
    {
        if(!optionPanel){
            optionPanel = new QWidget;
            optionVBox = new QVBoxLayout;
            optionVBox->setContentsMargins(0, 0, 0, 0);
            optionPanel->setLayout(optionVBox);
            
            createOptionPanel();
        }
        return optionPanel;
    }

    virtual void createOptionPanel() = 0;

    void addShapeTypeCombo(QBoxLayout* box)
    {
        box->addWidget(new QLabel(_("Shape type:")));
        shapeTypeCombo = new QComboBox;
        shapeTypeCombo->addItem(_("Visual"));
        shapeTypeCombo->addItem(_("Collision"));
        box->addWidget(shapeTypeCombo);
    }
};


class StdSceneFileExport : public SceneFileExportBase
{
    unique_ptr<StdSceneWriter> sceneWriter;
    QComboBox* extModelFileModeCombo;
    QCheckBox* transformIntegrationCheck;

public:
    StdSceneFileExport()
        : SceneFileExportBase(_("Standard scene file"), "STD-SCENE-FILE", "scen")
    {

    }

    StdSceneWriter* ensureSceneWriter()
    {
        if(!sceneWriter){
            sceneWriter.reset(new StdSceneWriter);
            sceneWriter->setMessageSink(os());
            sceneWriter->setIndentWidth(1);
        }
        return sceneWriter.get();
    }

    virtual bool saveScene(const std::string& filename, SgGroup* scene) override
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

    virtual void createOptionPanel() override
    {
        QHBoxLayout* hbox;
        hbox = new QHBoxLayout;

        addShapeTypeCombo(hbox);
        
        hbox->addWidget(new QLabel(_("Ext model file mode:")));
        extModelFileModeCombo = new QComboBox;
        extModelFileModeCombo->addItem(_("Embed models"));
        extModelFileModeCombo->addItem(_("Link to the original model files"));
        extModelFileModeCombo->addItem(_("Replace with standard scene files"));
        extModelFileModeCombo->addItem(_("Replace with OBJ model files"));
        hbox->addWidget(extModelFileModeCombo);

        transformIntegrationCheck = new QCheckBox;
        transformIntegrationCheck->setText(_("Integrate transforms"));
        hbox->addWidget(transformIntegrationCheck);
        hbox->addStretch();

        optionVBox->addLayout(hbox);
    }

    virtual void fetchOptionPanelForSaving() override
    {
        ensureSceneWriter();
        sceneWriter->setExtModelFileMode(extModelFileModeCombo->currentIndex());
        sceneWriter->setTransformIntegrationEnabled(transformIntegrationCheck->isChecked());
    }
};


class ObjFileExport : public SceneFileExportBase
{
    unique_ptr<ObjSceneWriter> sceneWriter;

public:
    ObjFileExport()
        : SceneFileExportBase(_("OBJ file"), "OBJ-FILE", "obj")
    {

    }

    ObjSceneWriter* ensureSceneWriter()
    {
        if(!sceneWriter){
            sceneWriter.reset(new ObjSceneWriter);
        }
        return sceneWriter.get();
    }

    virtual bool saveScene(const std::string& filename, SgGroup* scene) override
    {
        return ensureSceneWriter()->writeScene(filename, scene);
    }

    virtual void createOptionPanel() override
    {
        auto hbox = new QHBoxLayout;
        addShapeTypeCombo(hbox);
        hbox->addStretch();
        optionVBox->addLayout(hbox);
    }
};

}

namespace cnoid {

void registerBodyItemFileIoSet(ItemManager& im)
{
    ::bodyFileIO = new BodyFileIO;
    im.registerFileIO<BodyItem>(::bodyFileIO);

    ::meshFileIO = new SceneFileIO;
    im.registerFileIO<BodyItem>(::meshFileIO);

    im.registerFileIO<BodyItem>(new StdSceneFileExport);
    im.registerFileIO<BodyItem>(new ObjFileExport);
}

}


ItemFileIO* BodyItem::bodyFileIO()
{
    return ::bodyFileIO;
}


ItemFileIO* BodyItem::meshFileIO()
{
    return ::meshFileIO;
}
