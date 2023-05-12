#ifndef CNOID_BODY_PLUGIN_BODY_LIBRARY_VIEW_H
#define CNOID_BODY_PLUGIN_BODY_LIBRARY_VIEW_H

#include <cnoid/View>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Mapping;
class MessageOut;

class CNOID_EXPORT BodyLibraryView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    static BodyLibraryView* instance();

    BodyLibraryView();
    virtual ~BodyLibraryView();

    bool setLibraryDirectory(const std::string& directory, bool ensureDirectoryAndIndexFile = false);
    const std::string& libraryDirectory() const;
    void addReferenceDirectory(const std::string& directory);

    bool saveIndexFileIfUpdated();

    class Element;
    typedef std::shared_ptr<Element> ElementPtr;
    
    class Element
    {
    public:
        bool isGroup() const;
        bool isRoot() const;
        bool isBody() const;
        std::string name() const;
        std::vector<ElementPtr> elements() const;
    private:
        Element(intptr_t handle);
        Element() = delete;
        static ElementPtr create(intptr_t handle);
        intptr_t handle;
        friend class BodyLibraryView;
    };

    ElementPtr getRootElement();
    bool hasLibraryElements() const;

    bool createTopGroupWithDialog();
    bool createGroupWithDialog(ElementPtr parentGroup);
    bool registerBodyWithDialog(BodyItem* bodyItem, Mapping* extraInfo, ElementPtr group, bool doStoreFiles);
    bool setBodyThumbnailWithDialog(ElementPtr element);
    bool renameLibraryItemWithDialog(ElementPtr group);

    bool importLibrary(const std::string& filename, MessageOut* mout = nullptr);
    bool exportLibrary(const std::string& filename, MessageOut* mout = nullptr);

    virtual BodyItem* loadBodyItem(const std::string& name, const std::string& filename, Mapping* extraInfo);

    class Impl;

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool eventFilter(QObject* obj, QEvent* event) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
