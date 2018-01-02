/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_SE3_MATRIX_SEQ_ITEM_H
#define CNOID_BASE_MULTI_SE3_MATRIX_SEQ_ITEM_H

#include "MultiSeqItem.h"
#include <cnoid/MultiSE3MatrixSeq>

namespace cnoid {

typedef MultiSeqItem<MultiSE3MatrixSeq> MultiSE3MatrixSeqItem;
typedef MultiSE3MatrixSeqItem::Ptr MultiSE3MatrixSeqItemPtr;

template<> void MultiSeqItem<MultiSE3MatrixSeq>::initializeClass(ExtensionManager* ext);

}

#endif
