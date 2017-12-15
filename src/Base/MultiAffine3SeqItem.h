/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_AFFINE3_SEQ_ITEM_H
#define CNOID_BASE_MULTI_AFFINE3_SEQ_ITEM_H

#include "MultiSeqItem.h"
#include <cnoid/MultiAffine3Seq>

namespace cnoid {

typedef MultiSeqItem<MultiAffine3Seq> MultiAffine3SeqItem;
typedef MultiAffine3SeqItem::Ptr MultiAffine3SeqItemPtr;

template<> void MultiSeqItem<MultiAffine3Seq>::initializeClass(ExtensionManager* ext);

}

#endif
