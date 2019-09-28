/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3MatrixSeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

template<> void MultiSeqItem<MultiSE3MatrixSeq>::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiSE3MatrixSeqItem>(N_("MultiSE3MatrixSeqItem"));
    
    ext->itemManager().addCreationPanel<MultiSE3MatrixSeqItem>(
        new MultiSeqItemCreationPanel(_("Number of SE3 values in a frame")));
}

#ifdef _WIN32
template class MultiSeqItem<MultiSE3MatrixSeq>;
#endif
