/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_SE3_SEQ_ITEM_H_INCLUDED
#define CNOID_BASE_MULTI_SE3_SEQ_ITEM_H_INCLUDED

#include "MultiSeqItem.h"
#include <cnoid/MultiSE3Seq>

namespace cnoid {
typedef MultiSeqItem<MultiSE3Seq> MultiSE3SeqItem;
typedef MultiSE3SeqItem::Ptr MultiSE3SeqItemPtr;

template<> void MultiSeqItem<MultiSE3Seq>::initializeClass(ExtensionManager* ext);
}

#endif
