#include "BodyItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemFileIO>  
#include <cnoid/BodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/StdSceneWriter>
#include <cnoid/SceneItemFileIO>
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
ItemFileIO* stdSceneFileOutput;


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
        bodyItem->body()->rootLink()->addShapeNode(shape);

        auto itype = invocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        
        return true;
    }
};

class StdSceneFileOutput : public ItemFileIOBase<BodyItem>
{
    unique_ptr<StdSceneWriter> sceneWriter;
    QWidget* optionPanel;
    QComboBox* shapeTypeCombo;
    QComboBox* meshOutputModeCombo;
    QCheckBox* transformIntegrationCheck;
    QSpinBox* vertexPrecisionSpin;

public:
    StdSceneFileOutput()
        : ItemFileIOBase<BodyItem>(
            "STD-SCENE-FILE",
            Save | Options | OptionPanelForSaving)
    {
        setCaption(_("Standard scene file"));
        setExtensions({ "scen" });
        setInterfaceLevel(Conversion);

        optionPanel = nullptr;        
    }

    ~StdSceneFileOutput()
    {
        if(optionPanel){
            delete optionPanel;
        }
    }

    StdSceneWriter* ensureSceneWriter()
    {
        if(!sceneWriter){
            sceneWriter.reset(new StdSceneWriter);
            sceneWriter->setIndentWidth(1);
        }
        return sceneWriter.get();
    }

    virtual bool save(BodyItem* item, const std::string& filename) override
    {
        bool saved = false;

        auto body = item->body();
        int numLinks = body->numLinks();
        vector<SgNode*> shapes;
        shapes.reserve(numLinks);
        vector<int> shapeIndicesToClearName;
        shapeIndicesToClearName.reserve(numLinks);
        const Isometry3 T0 = body->rootLink()->T();
        for(auto& link : body->links()){
            bool stripped;

            SgGroup* shapeGroup;
            if(shapeTypeCombo->currentIndex() == 0){
                shapeGroup = link->visualShape();
            } else {
                shapeGroup = link->collisionShape();
            }
            
            if(SgNode* shape = strip(shapeGroup, stripped)){
                if(!link->T().isApprox(T0)){
                    auto transform = new SgPosTransform(T0.inverse() * link->T());
                    if(stripped){
                        transform->addChild(shape);
                    } else {
                        shapeGroup->copyChildrenTo(transform);
                    }
                    transform->setName(link->name());
                    shape = transform;
                } else if(shape->name().empty()){
                    shape->setName(link->name());
                    shapeIndicesToClearName.push_back(shapes.size());
                }
                shapes.push_back(shape);
            }
        }
        if(!shapes.empty()){
            saved = ensureSceneWriter()->writeScene(filename, shapes);
        }
        // Clear temporary names
        for(auto& index : shapeIndicesToClearName){
            shapes[index]->setName("");
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

    virtual QWidget* getOptionPanelForSaving(BodyItem* /* item */) override
    {
        if(!optionPanel){
            createOptionPanel();
        }
        return optionPanel;
    }

    void createOptionPanel()
    {
        optionPanel = new QWidget;

        auto vbox = new QVBoxLayout;
        vbox->setContentsMargins(0, 0, 0, 0);
        optionPanel->setLayout(vbox);

        QHBoxLayout* hbox;
        hbox = new QHBoxLayout;

        hbox->addWidget(new QLabel(_("Shape type:")));
        shapeTypeCombo = new QComboBox;
        shapeTypeCombo->addItem(_("Visual"));
        shapeTypeCombo->addItem(_("Collision"));
        hbox->addWidget(shapeTypeCombo);
        
        hbox->addWidget(new QLabel(_("Mesh output mode:")));
        meshOutputModeCombo = new QComboBox;
        meshOutputModeCombo->addItem(_("Embedded"));
        meshOutputModeCombo->addItem(_("Original"));
        meshOutputModeCombo->addItem(_("Converted (STL)"));
        hbox->addWidget(meshOutputModeCombo);
        hbox->addStretch();
       
        vbox->addLayout(hbox);
        
        hbox = new QHBoxLayout;
        transformIntegrationCheck = new QCheckBox;
        transformIntegrationCheck->setText(_("Integrate transforms"));
        hbox->addWidget(transformIntegrationCheck);

        hbox->addWidget(new QLabel(_("Vertex precision:")));
        vertexPrecisionSpin = new QSpinBox;
        vertexPrecisionSpin->setRange(6, 16);
        vertexPrecisionSpin->setValue(ensureSceneWriter()->vertexPrecision());
        hbox->addWidget(vertexPrecisionSpin);
        hbox->addStretch();

        vbox->addLayout(hbox);
    }

    virtual void fetchOptionPanelForSaving() override
    {
        sceneWriter->setMeshOutputMode(meshOutputModeCombo->currentIndex());
        sceneWriter->setTransformIntegrationEnabled(transformIntegrationCheck->isChecked());
        sceneWriter->setVertexPrecision(vertexPrecisionSpin->value());
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

    ::stdSceneFileOutput = new StdSceneFileOutput;
    im.registerFileIO<BodyItem>(::stdSceneFileOutput);
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
