/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ABSTRACT_SEQ_ITEM_H
#define CNOID_BASE_ABSTRACT_SEQ_ITEM_H

#include "Item.h"
#include <cnoid/AbstractSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractSeqItem : public Item
{
public:
    AbstractSeqItem();
    AbstractSeqItem(const AbstractSeqItem& org);
    virtual ~AbstractSeqItem();

    virtual std::shared_ptr<AbstractSeq> abstractSeq() = 0;

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
};

typedef ref_ptr<AbstractSeqItem> AbstractSeqItemPtr;


class CNOID_EXPORT AbstractMultiSeqItem : public AbstractSeqItem
{
public:
    AbstractMultiSeqItem();
    AbstractMultiSeqItem(const AbstractMultiSeqItem& org);
    virtual ~AbstractMultiSeqItem();

    virtual std::shared_ptr<AbstractSeq> abstractSeq() override;
    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() = 0;

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
};

typedef ref_ptr<AbstractMultiSeqItem> AbstractMultiSeqItemPtr;

}

#endif
