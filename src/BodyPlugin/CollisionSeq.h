#ifndef CNOID_BODY_COLLISION_SEQ_H
#define CNOID_BODY_COLLISION_SEQ_H

#include <cnoid/CollisionLinkPairList>
#include <cnoid/MultiSeq>
#include <cnoid/YAMLWriter>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class YAMLWriter;
class CollisionSeqItem;

class CNOID_EXPORT CollisionSeq : public MultiSeq<std::shared_ptr<CollisionLinkPairList>>
{
    typedef MultiSeq<std::shared_ptr<CollisionLinkPairList>> BaseSeqType;

public:
    CollisionSeqItem* collisionSeqItem_;
    CollisionSeq(CollisionSeqItem* collisionSeqItem);

    using BaseSeqType::operator=;

    bool loadStandardYAMLformat(const std::string& filename, std::ostream& os = nullout());
    bool saveAsStandardYAMLformat(const std::string& filename);
    void writeCollsionData(YAMLWriter& writer, std::shared_ptr<const CollisionLinkPairList> ptr);
    void readCollisionData(int nFrames, const Listing& values);

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;
};

}

#endif
