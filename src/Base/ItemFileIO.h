#ifndef CNOID_BASE_ITEM_FILE_IO_H
#define CNOID_BASE_ITEM_FILE_IO_H

#include "ItemList.h"
#include <cnoid/Referenced>
#include <string>
#include <vector>
#include "exportdecl.h"

class QWidget;

namespace cnoid {

class Item;
class ItemManager;
class ItemFileIOExtenderBase;
class ItemFileDialog;
class Mapping;

class CNOID_EXPORT ItemFileIO : public Referenced
{
public:
    enum API {
        Load = 1 << 0,
        Options = 1 << 1,
        OptionPanelForLoading = 1 << 2,
        Save = 1 << 3,
        OptionPanelForSaving = 1 << 4,
    };
    enum InterfaceLevel { Standard, Conversion, Internal };
    enum InvocationType { Direct, Dialog, DragAndDrop };
    class Impl;

protected:
    ItemFileIO(const std::string& formatId, int api);
    ItemFileIO(const ItemFileIO& org);
    ItemFileIO();
    void copyFrom(const ItemFileIO& org);
    
public:
    ~ItemFileIO();

    bool isFormat(const std::string& id) const;
    int api() const;
    void setApi(int apiSet);
    bool hasApi(int api) const;
    void setCaption(const std::string& caption);
    const std::string& caption() const;
    void setFileTypeCaption(const std::string& caption);
    const std::string& fileTypeCaption() const;
    void addFormatIdAlias(const std::string& formatId);

    void setExtension(const std::string& extension);
    void setExtensions(const std::vector<std::string>& extensions);

    // deprecated. This is internally used for specifing SceneItem's extensions dynamically.
    // The dynamic extension specification should be achieved by a signal to update the
    // extensions and usual the registerExtensions function.
    void setExtensionFunction(std::function<std::string()> func);

    std::vector<std::string> extensions() const;
    
    void setInterfaceLevel(InterfaceLevel level);
    int interfaceLevel() const;
    
    virtual Item* createItem() = 0;

    // Set the invocation type before calling the loadItem or saveItem function
    // if the invocation type is not "Direct".
    void setInvocationType(int type);

    // Load API
    Item* loadItem(
        const std::string& filename,
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr,
        const Mapping* options = nullptr);

    bool loadItem(
        Item* item, const std::string& filename,
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr,
        const Mapping* options = nullptr);

    virtual bool load(Item* item, const std::string& filename);
    
    // Save API
    bool saveItem(Item* item, const std::string& filename, const Mapping* options = nullptr);
    virtual bool save(Item* item, const std::string& filename);
    
    // Options API
    virtual void resetOptions();
    virtual void storeOptions(Mapping* options);
    virtual bool restoreOptions(const Mapping* options);
    
    // OptionPanelForLoading API
    virtual QWidget* getOptionPanelForLoading();
    virtual void fetchOptionPanelForLoading();

    // OptionPanelForPostLoading API (pending)
    //virtual QWidget* getOptionPanelForPostLoading(Item* item);
    //virtual void fetchOptionPanelForPostLoading(Item* item);
    //virtual bool doPostProcessForLoading(Item* item);

    // OptionPanelForSaving API
    virtual QWidget* getOptionPanelForSaving(Item* item);
    virtual void fetchOptionPanelForSaving();

    Item* parentItem();
    int invocationType() const;

    bool isRegisteredForSingletonItem() const;
    Item* findSingletonItemInstance() const;

    void setItemClassInfo(Referenced* info);
    const Referenced* itemClassInfo() const;

    static std::vector<std::string> separateExtensions(const std::string& multiExtString);

protected:
    std::ostream& os();
    void putWarning(const std::string& message);
    void putError(const std::string& message);

    /**
       When the file is loaded as a composite item tree and the item
       that actually loads the file is different from the top item,
       the following function must be called with the corresponding item.
    */
    void setActuallyLoadedItem(Item* item);
    
private:
    Impl* impl;
    friend class ItemManager;
    friend class ItemFileIOExtenderBase;
    friend class ItemFileDialog;
};

typedef ref_ptr<ItemFileIO> ItemFileIOPtr;

template<class ItemType>
class ItemFileIOBase : public ItemFileIO
{
public:
    ItemFileIOBase(const std::string& formatId, int api)
        : ItemFileIO(formatId, api) {
    }
    virtual Item* createItem() override {
        return new ItemType;
    }
    virtual bool load(Item* item, const std::string& filename) final {
        if(auto derived = dynamic_cast<ItemType*>(item)){
            return load(derived, filename);
        }
        return false;
    }
    virtual bool load(ItemType* /* item */, const std::string& /* filename */) {
        return false;
    }
    virtual bool save(Item* item, const std::string& filename) final {
        if(auto derived = dynamic_cast<ItemType*>(item)){
            return save(derived, filename);
        }
        return false;
    }
    virtual bool save(ItemType* /* item */, const std::string& /* filename */) {
        return false;
    }
    virtual QWidget* getOptionPanelForSaving(Item* item) final {
        return getOptionPanelForSaving(static_cast<ItemType*>(item));
    }
    virtual QWidget* getOptionPanelForSaving(ItemType* item) {
        return nullptr;
    }
};


class CNOID_EXPORT ItemFileIOExtenderBase : public ItemFileIO
{
    ItemFileIOPtr baseFileIO;

protected:
    ItemFileIOExtenderBase(const std::type_info& type, const std::string& formatId);
    
public:
    bool isAvailable() const;
    virtual bool load(Item* item, const std::string& filename) final;
    virtual void resetOptions() override;
    virtual void storeOptions(Mapping* options) override;
    virtual bool restoreOptions(const Mapping* options) override;
    virtual QWidget* getOptionPanelForLoading() override;
    virtual void fetchOptionPanelForLoading() override;
    virtual bool save(Item* item, const std::string& filename) final;
    virtual QWidget* getOptionPanelForSaving(Item* item) final;
    virtual void fetchOptionPanelForSaving() override;
};


template<class ItemType, class BaseItemType = ItemType>
class ItemFileIOExtender : public ItemFileIOExtenderBase
{
public:
    ItemFileIOExtender(const std::string& formatId = "")
        : ItemFileIOExtenderBase(typeid(BaseItemType), formatId)
    { }

    ItemType* loadItem(
        const std::string& filename,
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr,
        const Mapping* options = nullptr) {
        return static_cast<ItemType*>(
            ItemFileIO::loadItem(filename, parentItem, doAddition, nextItem, options));
    }
    
protected:
    virtual Item* createItem() override {
        return new ItemType;
    }
    virtual bool load(ItemType* item, const std::string& filename) {
        return ItemFileIOExtenderBase::load(item, filename);
    }
    virtual bool save(ItemType* item, const std::string& filename) {
        return ItemFileIOExtenderBase::save(item, filename);
    }
    virtual QWidget* getOptionPanelForSaving(ItemType* item) {
        return ItemFileIOExtenderBase::getOptionPanelForSaving(item);
    }
};

}

#endif
