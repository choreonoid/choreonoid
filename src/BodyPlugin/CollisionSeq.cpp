/**
   @file
   @author Shizuko Hattori
*/

#include "CollisionSeq.h"
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/CollisionSeqItem>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
static const string mdskey("CollisionPairLsit");
}

CollisionSeq::CollisionSeq(CollisionSeqItem* collisionSeqItem)
    : BaseSeqType("CollisionSeq")
{
    collisionSeqItem_ = collisionSeqItem;
    setSeqContentName(mdskey);
}


bool CollisionSeq::loadStandardYAMLformat(const std::string& filename)
{
    bool result = false;
    bool loaded = false;
    clearSeqMessage();
    YAMLReader reader;
    reader.expectRegularMultiListing();

    try {
        auto archive = reader.loadDocument(filename)->toMapping();
        if(archive->get<string>("type") != "CollisionSeq"){
            result = false;
        }else{
            result = readSeq(archive);
            if(result){
                loaded = true;
            } else {
                addSeqMessage(seqMessage());
            }
        }
    } catch(const ValueNode::Exception& ex){
        addSeqMessage(ex.message());
    }

    return (result && loaded);
}


bool CollisionSeq::saveAsStandardYAMLformat(const std::string& filename)
{
    YAMLWriter writer(filename);

    writer.setDoubleFormat("%.9g");

    writer.putComment("Collision data set format version 1.0 defined by cnoid-Robotics\n");

    if(numFrames() > 0){
        if(!writeSeq(writer)){
            addSeqMessage(seqMessage());
            return false;
        }
    }

    return true;
}


//! \todo Implement this function with GeneralSeqReader
bool CollisionSeq::doReadSeq(const Mapping* archive, std::ostream& os)
{
    os << _("The function to read CollisionSeq is not implemented.") << endl;
    return false;
    
    const Listing& values = *archive->findListing("frames");
    if(values.isValid()){
        const int nFrames = values.size();
        setDimension(nFrames, 1);
        readCollisionData(nFrames, values);
    }
    return true;
}


void CollisionSeq::readCollisionData(int nFrames, const Listing& values)
{
    /*
    WorldItem* worldItem = collisionSeqItem_->findOwnerItem<WorldItem>();
    if(!worldItem)
        return;
    */
    WorldItem* worldItem = 0;
    RootItem* rootItem = RootItem::instance();
    ItemList<WorldItem> worldItems;
    if(worldItems.extractChildItems(rootItem)){
        worldItem = worldItems.front();
    }
    if(!worldItem)
        return;

    for(int i=0; i < nFrames; ++i){
        const Mapping& frameNode = *values[i].toMapping();
        const Listing& linkPairs = *frameNode.findListing("LinkPairs");
        Frame f = frame(i);
        f[0] = std::make_shared<CollisionLinkPairList>();
        for(int j=0; j<linkPairs.size(); j++){
            CollisionLinkPairPtr destLinkPair = std::make_shared<CollisionLinkPair>();
            const Mapping& linkPair = *linkPairs[j].toMapping();
            string body0name = linkPair["body0"].toString();
            string body1name = linkPair["body1"].toString();
            string link0name = linkPair["link0"].toString();
            string link1name = linkPair["link1"].toString();
            BodyItem* body0Item = worldItem->findChildItem<BodyItem>(body0name);
            Body* body0=0;
            Body* body1=0;
            Link* link0=0;
            Link* link1=0;
            if(body0Item){
                body0 = body0Item->body();
                link0 = body0->link(link0name);
            }
            BodyItem* body1Item = worldItem->findChildItem<BodyItem>(body1name);
            if(body1Item){
                body1 = body1Item->body();
                link1 = body1->link(link1name);
            }
            destLinkPair->body[0] = body0;
            destLinkPair->link[0] = link0;
            destLinkPair->body[1] = body1;
            destLinkPair->link[1] = link1;
            const Listing& collisions = *linkPair.findListing("Collisions");
            for(int k=0; k<collisions.size(); k++){
                destLinkPair->collisions.push_back(Collision());
                Collision& destCol = destLinkPair->collisions.back();
                const Listing& collision = *collisions[k].toListing();
                destCol.point = Vector3(collision[0].toDouble(), collision[1].toDouble(), collision[2].toDouble());
                destCol.normal = Vector3(collision[3].toDouble(), collision[4].toDouble(), collision[5].toDouble());
                destCol.depth = collision[6].toDouble();
            }
            f[0]->push_back(destLinkPair);
        }
    }
}


void CollisionSeq::writeCollsionData(YAMLWriter& writer, std::shared_ptr<const CollisionLinkPairList> ptr)
{
    writer.startMapping();
    writer.putKey("LinkPairs");

    writer.startListing();
    for(auto it = ptr->begin(); it != ptr->end(); ++it){
        CollisionLinkPairPtr linkPair = *it;
        writer.startMapping();
        writer.putKeyValue("body0",linkPair->body[0]->name());
        writer.putKeyValue("link0",linkPair->link[0]->name());
        writer.putKeyValue("body1",linkPair->body[1]->name());
        writer.putKeyValue("link1",linkPair->link[1]->name());
        int numCollisions = linkPair->collisions.size();
        writer.putKey("Collisions");
        writer.startListing();
        for(int j=0; j<numCollisions; j++){
            Collision& collision = linkPair->collisions[j];
            writer.startFlowStyleListing();
            const Vector3& point = collision.point;
            writer.putScalar(point.x());
            writer.putScalar(point.y());
            writer.putScalar(point.z());
            const Vector3& normal = collision.normal;
            writer.putScalar(normal.x());
            writer.putScalar(normal.y());
            writer.putScalar(normal.z());
            writer.putScalar(collision.depth);
            writer.endListing();
        }
        writer.endListing();
        writer.endMapping();
        }
    writer.endListing();

    writer.endMapping();
}


bool CollisionSeq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    return BaseSeqType::doWriteSeq(
        writer,
        [&](){
            writer.putKeyValue("format", "PxPyPzNxNyNzD");

            if(additionalPartCallback) additionalPartCallback();
            
            writer.putKey("frames");
            writer.startListing();
            const int n = numFrames();
            for(int i=0; i < n; ++i){
                Frame f = frame(i);
                writeCollsionData(writer, f[0]);
            }
            writer.endListing();
        });
}
