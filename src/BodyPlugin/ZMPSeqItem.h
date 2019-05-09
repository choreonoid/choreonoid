/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_ZMPSEQ_ITEM_H
#define CNOID_BODY_PLUGIN_ZMPSEQ_ITEM_H

#include <cnoid/Vector3SeqItem>
#include <cnoid/ZMPSeq>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
    
class CNOID_EXPORT ZMPSeqItem : public Vector3SeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    ZMPSeqItem();
    ZMPSeqItem(std::shared_ptr<ZMPSeq> seq);
    ZMPSeqItem(const ZMPSeqItem& org);

    std::shared_ptr<ZMPSeq> zmpseq() { return zmpseq_; }
    std::shared_ptr<const ZMPSeq> zmpseq() const { return zmpseq_; }

    bool makeRootRelative(bool on);

protected:
    virtual ~ZMPSeqItem();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    std::shared_ptr<ZMPSeq> zmpseq_;
};

typedef ref_ptr<ZMPSeqItem> ZMPSeqItemPtr;

}

#endif
