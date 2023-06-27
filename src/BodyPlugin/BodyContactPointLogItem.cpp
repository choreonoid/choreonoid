#include "BodyContactPointLogItem.h"
#include "BodyContactPointLoggerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>
#include <cnoid/YAMLWriter>
#include <cnoid/TimeSyncItemEngine>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class BodyContactPointLogFileIO : public ItemFileIO
{
public:
    BodyContactPointLogFileIO();
    virtual Item* createItem() override;
    virtual bool save(Item* item, const std::string& filename) override;
    void writeVector3(YAMLWriter& writer, const char* key, const Vector3& value);
};


class BodyContactPointLogEngine : public TimeSyncItemEngine
{
public:
    static BodyContactPointLogEngine* create(
        BodyContactPointLogItem* logItem, BodyContactPointLogEngine* engine0);
    
    weak_ref_ptr<BodyContactPointLoggerItem> loggerItemRef;
    BodyContactPointLogItem* logItem;

    BodyContactPointLogEngine(BodyContactPointLogItem* logItem, BodyContactPointLoggerItem* loggerItem);
    ~BodyContactPointLogEngine();
    
    virtual bool onTimeChanged(double time) override;
};

}


void BodyContactPointLogItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodyContactPointLogItem, ReferencedObjectSeqItem>(N_("BodyContactPointLogItem"))
        .addFileIO<BodyContactPointLogItem>(new BodyContactPointLogFileIO);
    
    TimeSyncItemEngineManager::instance()
        ->registerFactory<BodyContactPointLogItem, BodyContactPointLogEngine>(
            BodyContactPointLogEngine::create);
}


BodyContactPointLogItem::BodyContactPointLogItem()
{
    initialize();
}


BodyContactPointLogItem::BodyContactPointLogItem(const BodyContactPointLogItem& org, CloneMap* cloneMap)
    : ReferencedObjectSeqItem(org, cloneMap)
{
    initialize();
}


void BodyContactPointLogItem::initialize()
{
    seq()->setSeqContentName("BodyContactPointSeq");
}


Item* BodyContactPointLogItem::doCloneItem(CloneMap* cloneMap) const
{
    return new BodyContactPointLogItem(*this, cloneMap);
}


BodyContactPointLogFileIO::BodyContactPointLogFileIO()
    : ItemFileIO("BODY-CONTACT-POINT-LOG-YAML", Save)
{
    setCaption(_("Body Contact Point Log File"));
    setFileTypeCaption(_("Body Contact Point Log File (YAML)"));
    setExtension("yaml");
    setInterfaceLevel(Conversion);
}


Item* BodyContactPointLogFileIO::createItem()
{
    return new BodyContactPointLogItem;
}


bool BodyContactPointLogFileIO::save(Item* item, const std::string& filename)
{
    auto logItem = static_cast<BodyContactPointLogItem*>(item);

    Body* body = nullptr;
    if(auto bodyItem = logItem->findOwnerItem<BodyItem>()){
        body = bodyItem->body();
    }
    
    auto seq = logItem->seq();
    int numFrames = seq->numFrames();
    
    YAMLWriter writer(filename);

    writer.startMapping();
    
    writer.putKeyValue("type", "BodyContactPointSetSeq");
    writer.putKeyValue("content", "BodyContactPointSetSeq");
    writer.putKeyValue("format_version", "4");
    writer.putKeyValue("frame_rate", seq->frameRate());
    writer.putKeyValue("num_frames", numFrames);

    writer.putKey("frames");
    writer.startListing();

    for(int i=0; i < numFrames; ++i){
        auto frame = static_cast<BodyContactPointLogItem::LogFrame*>((*seq)[i].get());
        auto& bodyContactPoints = frame->bodyContactPoints();
        int numLinks = bodyContactPoints.size();
        bool hasContactPoints = false;
        for(int i=0; i < numLinks; ++i){
            auto& linkContactPoints = bodyContactPoints[i];
            if(!linkContactPoints.empty()){
                if(!hasContactPoints){
                    writer.startMapping();
                    hasContactPoints = true;
                }
                bool hasName = false;
                if(body){
                    if(auto link = body->link(i)){
                        auto& name = link->name();
                        if(!name.empty()){
                            writer.putKey(name, DOUBLE_QUOTED);
                            hasName = true;
                        }
                    }
                }
                if(!hasName){
                    writer.putKey(format("link{0}", i), DOUBLE_QUOTED);
                }
                writer.startListing();
                for(auto& point : linkContactPoints){
                    writer.startFlowStyleMapping();
                    writeVector3(writer, "position", point.position());
                    writeVector3(writer, "normal", point.normal());
                    writeVector3(writer, "force", point.force());
                    writeVector3(writer, "velocity", point.velocity());
                    writer.putKeyValue("depth", point.depth());
                    writer.endMapping();
                }
                writer.endListing();
            }
        }

        if(!hasContactPoints){
            // Put empty map item
            writer.startFlowStyleMapping();
        }
        writer.endMapping();
    }

    writer.endListing();
    writer.endMapping();
    
    return true;
}


void BodyContactPointLogFileIO::writeVector3(YAMLWriter& writer, const char* key, const Vector3& value)
{
    writer.putKey(key);
    writer.startFlowStyleListing();
    for(int i=0; i < 3; ++i){
        writer.putScalar(value[i]);
    }
    writer.endListing();
}


BodyContactPointLogEngine* BodyContactPointLogEngine::create
(BodyContactPointLogItem* logItem, BodyContactPointLogEngine* engine0)
{
    if(auto loggerItem = logItem->findOwnerItem<BodyContactPointLoggerItem>()){
        if(engine0 && engine0->loggerItemRef.lock() == loggerItem){
            return engine0;
        } else {
            return new BodyContactPointLogEngine(logItem, loggerItem);
        }
    }
    return nullptr;
}


BodyContactPointLogEngine::BodyContactPointLogEngine
(BodyContactPointLogItem* logItem, BodyContactPointLoggerItem* loggerItem)
    : TimeSyncItemEngine(logItem),
      loggerItemRef(loggerItem),
      logItem(logItem)
{

}


BodyContactPointLogEngine::~BodyContactPointLogEngine()
{
    if(auto loggerItem = loggerItemRef.lock()){
        loggerItem->setLogFrameToVisualize(nullptr);
    }
}


bool BodyContactPointLogEngine::onTimeChanged(double time)
{
    auto seq = logItem->seq();
    if(seq->empty()){
        return false;
    }
    auto loggerItem = loggerItemRef.lock();
    if(!loggerItem){
        return false;
    }
    bool isValid;
    int frame = seq->frameOfTime(time);
    if(frame < seq->numFrames()){
        isValid = true;
    } else {
        isValid = false;
        frame = seq->numFrames() - 1;
    }

    if(auto logFrame = dynamic_cast<BodyContactPointLogItem::LogFrame*>(seq->at(frame).get())){
        loggerItem->setLogFrameToVisualize(logFrame);
    }
    
    return isValid;
}
