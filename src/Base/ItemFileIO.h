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
class ItemManagerImpl;
class ItemFileIOExtenderBase;
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

    int api() const;
    void setApi(int api);
    void setCaption(const std::string& name);
    void addFormatIdAlias(const std::string& formatId);
    void setExtension(const std::string& extension);
    void setExtensions(const std::vector<std::string>& extensions);
    // deprecated. This is internally used for specifing SceneItem's extensions dynamically.
    // The dynamic extension specification should be achieved by a signal to update the
    // extensions and usual the registerExtensions function.
    void setExtensionFunction(std::function<std::string()> func);
    void setInterfaceLevel(InterfaceLevel level);

    Item* loadItem(
        const std::string& filename,
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr,
        const Mapping* options = nullptr);
    bool loadItem(
        Item* item, const std::string& filename,
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr,
        const Mapping* options = nullptr);
    ItemList<Item> loadItemsWithDialog(
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr);

    // Pending
    //bool saveItem(Item* item, const std::string& filename);

protected:
    virtual Item* createItem() = 0;
    // Load API
    virtual bool load(Item* item, const std::string& filename);
    // Options API
    virtual void resetOptions();
    virtual void storeOptions(Mapping* archive);
    virtual bool restoreOptions(const Mapping* archive);
    // OptionPanelForLoading API
    virtual QWidget* getOptionPanelForLoading();
    virtual void fetchOptionPanelForLoading();

    // OptionPanelForPostLoading API (pending)
    //virtual QWidget* getOptionPanelForPostLoading(Item* item);
    //virtual void fetchOptionPanelForPostLoading(Item* item);
    //virtual bool doPostProcessForLoading(Item* item);

    // Save API
    virtual bool save(Item* item, const std::string& filename);
    // OptionPanelForSaving API
    virtual QWidget* getOptionPanelForSaving(Item* item);
    virtual void fetchSaveOptionPanel();

    Item* parentItem();
    InvocationType invocationType() const;

    std::ostream& os();
    void putWarning(const std::string& message);
    void putError(const std::string& message);
    
private:
    Impl* impl;

    friend class ItemManager;
    friend class ItemManagerImpl;
    friend class ItemFileIOExtenderBase;
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
    virtual void storeOptions(Mapping* archive) override;
    virtual bool restoreOptions(const Mapping* archive) override;
    virtual QWidget* getOptionPanelForLoading() override;
    virtual void fetchOptionPanelForLoading() override;
    virtual bool save(Item* item, const std::string& filename) final;
    virtual QWidget* getOptionPanelForSaving(Item* item) final;
    virtual void fetchSaveOptionPanel() override;
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
    ItemList<ItemType> loadItemsWithDialog(
        Item* parentItem = nullptr, bool doAddition = true, Item* nextItem = nullptr) {
        return ItemFileIO::loadItemsWithDialog(parentItem, doAddition, nextItem);
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
