/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3SeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

template<> void MultiSeqItem<MultiSE3Seq>::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiSE3SeqItem>(N_("MultiSE3SeqItem"));
    
    ext->itemManager().addCreationPanel<MultiSE3SeqItem>(
        new MultiSeqItemCreationPanel(_("Number of SE3 values in a frame")));
}

#ifdef _WIN32
template class MultiSeqItem<MultiSE3Seq>;
#endif

