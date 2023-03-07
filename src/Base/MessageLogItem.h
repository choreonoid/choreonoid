#ifndef CNOID_BASE_MESSAGE_LOG_ITEM_H
#define CNOID_BASE_MESSAGE_LOG_ITEM_H

#include "AbstractTextItem.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MessageLogItem : public AbstractTextItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MessageLogItem();
    virtual ~MessageLogItem();
    virtual const std::string& textFilename() const override;
    enum FileMode { APPEND = 0, OVERWRITE, N_FILE_MODES };

protected:
    MessageLogItem(const MessageLogItem& org);
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MessageLogItem> MessageLogItemPtr;

}

#endif
