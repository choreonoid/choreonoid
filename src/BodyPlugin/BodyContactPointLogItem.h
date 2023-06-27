#ifndef CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOG_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOG_ITEM_H

#include <cnoid/ReferencedObjectSeqItem>
#include <cnoid/Link>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyContactPointLogItem : public ReferencedObjectSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    class LogFrame : public Referenced
    {
    public:
        std::vector<std::vector<Link::ContactPoint>>& bodyContactPoints() {
            return bodyContactPoints_;
        }
        const std::vector<std::vector<Link::ContactPoint>>& bodyContactPoints() const {
            return bodyContactPoints_;
        }

        std::vector<Link::ContactPoint>& linkContactPoints(int linkIndex) {
            return bodyContactPoints_[linkIndex];
        }
        const std::vector<Link::ContactPoint>& linkContactPoints(int linkIndex) const {
            return bodyContactPoints_[linkIndex];
        }

    private:
        std::vector<std::vector<Link::ContactPoint>> bodyContactPoints_;
    };

    typedef ref_ptr<LogFrame> LogFramePtr;

    BodyContactPointLogItem();

    LogFrame* logFrame(int frameIndex) { return static_cast<LogFrame*>((*seq())[frameIndex].get()); }
    const LogFrame* logFrame(int frameIndex) const { return static_cast<LogFrame*>((*seq())[frameIndex].get()); }
    
protected:
    BodyContactPointLogItem(const BodyContactPointLogItem& org, CloneMap* cloneMap);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    void initialize();
};

typedef ref_ptr<BodyContactPointLogItem> BodyContactPointLogItemPtr;

}

#endif
