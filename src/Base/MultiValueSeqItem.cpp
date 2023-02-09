#include "MultiValueSeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include <ostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

bool loadPlainSeqFormat(MultiValueSeqItem* item, const string& filename, ostream& os)
{
    if(item->seq()->loadPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->seqMessage();
        return false;
    }
}


bool saveAsPlainSeqFormat(MultiValueSeqItem* item, const string& filename, ostream& os)
{
    if(item->seq()->saveAsPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->seqMessage();
        return false;
    }
}

}


void MultiValueSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiValueSeqItem, AbstractMultiSeqItem>(N_("MultiValueSeqItem"));

    ext->itemManager().addCreationPanel<MultiValueSeqItem>(
        new MultiSeqItemCreationPanel(_("Number of values in a frame")));
    
    ext->itemManager().addLoaderAndSaver<MultiValueSeqItem>(
        _("Plain Format of a Multi Value Sequence"), "PLAIN-MULTI-VALUE-SEQ", "*",
        [](MultiValueSeqItem* item, const string& filename, ostream& os, Item* /* parentItem */){
            return loadPlainSeqFormat(item, filename, os);
        },
        [](MultiValueSeqItem* item, const string& filename, ostream& os, Item* /* parentItem */){
            return saveAsPlainSeqFormat(item, filename, os);
        },
        ItemManager::PRIORITY_CONVERSION);
}


MultiValueSeqItem::MultiValueSeqItem()
    : seq_(std::make_shared<MultiValueSeq>())
{

}


MultiValueSeqItem::MultiValueSeqItem(std::shared_ptr<MultiValueSeq> seq)
    : seq_(seq)
{

}


MultiValueSeqItem::MultiValueSeqItem(const MultiValueSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiValueSeq>(*org.seq_))
{

}


MultiValueSeqItem::MultiValueSeqItem(const MultiValueSeqItem& org, std::shared_ptr<MultiValueSeq> seq)
    : AbstractMultiSeqItem(org),
      seq_(seq)
{

}


Item* MultiValueSeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MultiValueSeqItem(*this);
}


std::shared_ptr<AbstractMultiSeq> MultiValueSeqItem::abstractMultiSeq()
{
    return seq_;
}


void MultiValueSeqItem::resetSeq(std::shared_ptr<MultiValueSeq> seq)
{
    seq_ = seq;
}

