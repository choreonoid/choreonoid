/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiAffine3SeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

template<> void MultiSeqItem<MultiAffine3Seq>::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiAffine3SeqItem>(N_("MultiAffine3SeqItem"));
    
    ext->itemManager().addCreationPanel<MultiAffine3SeqItem>(
        new MultiSeqItemCreationPanel(_("Number of Affine3 values in a frame")));
}

#ifdef WIN32
template class MultiSeqItem<MultiAffine3Seq>;
#endif

