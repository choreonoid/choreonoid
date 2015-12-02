/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_ZMPSEQ_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_ZMPSEQ_ITEM_H_INCLUDED

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
    ZMPSeqItem(ZMPSeqPtr seq);
    ZMPSeqItem(const ZMPSeqItem& org);

    const ZMPSeqPtr& zmpseq() { return zmpseq_; }

    bool makeRootRelative(bool on);

protected:
    virtual ~ZMPSeqItem();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    ZMPSeqPtr zmpseq_;
};

typedef ref_ptr<ZMPSeqItem> ZMPSeqItemPtr;
}

#endif
