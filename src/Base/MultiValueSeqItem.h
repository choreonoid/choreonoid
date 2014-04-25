/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_MULTI_VALUE_SEQ_ITEM_H_INCLUDED
#define CNOID_GUIBASE_MULTI_VALUE_SEQ_ITEM_H_INCLUDED

#include "MultiSeqItem.h"
#include <cnoid/MultiValueSeq>

namespace cnoid {

typedef MultiSeqItem<MultiValueSeq> MultiValueSeqItem;
typedef MultiValueSeqItem::Ptr MultiValueSeqItemPtr;

template<> void MultiSeqItem<MultiValueSeq>::initializeClass(ExtensionManager* ext);
}

#endif
