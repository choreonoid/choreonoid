#ifndef CNOID_BASE_ITEM_FILE_IO_H
#define CNOID_BASE_ITEM_FILE_IO_H

#include <cnoid/Referenced>
#include <string>
#include <vector>
#include "exportdecl.h"

class QWidget;

namespace cnoid {

class Item;
class ItemManager;
class ItemManagerImpl;
class Mapping;

class CNOID_EXPORT ItemFileIOBase : public Referenced
{
public:
    enum API {
        Load = 1 << 0,
        Options = 1 << 1,
        OptionPanelForLoading = 1 << 2,
        Save = 1 << 3,
        OptionPanelForSaving = 1 << 4,
    };
    enum InterfaceLevel { Standard, Conversion, Compatibility };
    enum InvocationType { Direct, Dialog, DragAndDrop };

    ItemFileIOBase(const std::string& formatId, int api);
    ~ItemFileIOBase();

    // Load API
    virtual bool load(Item* item, const std::string& filename) = 0;
    // Options API
    virtual void resetOptions();
    virtual void storeOptions(Mapping* archive);
    virtual bool restoreOptions(const Mapping* archive);
    // OptionPanelForLoading API
    virtual QWidget* optionPanelForLoading();
    virtual void fetchOptionPanelForLoading();

    // OptionPanelForPostLoading API (pending)
    //virtual QWidget* optionPanelForPostLoading(Item* item);
    //virtual void fetchOptionPanelForPostLoading(Item* item);
    //virtual bool doPostProcessForLoading(Item* item);

    // Save API
    virtual bool save(Item* item, const std::string& filename) = 0;
    // OptionPanelForSaving API
    virtual QWidget* optionPanelForSaving(Item* item);
    virtual void fetchSaveOptionPanel();

protected:
    // Available in the constructor
    void setCaption(const std::string& name);
    void addFormatIdAlias(const std::string& formatId);
    void registerExtension(const std::string& extension);
    void registerExtensions(const std::vector<std::string>& extensions);
    // deprecated. This is internally used for specifing SceneItem's extensions dynamically.
    // The dynamic extension specification should be achieved by a signal to update the
    // extensions and usual the registerExtensions function.
    void setExtensionFunction(std::function<std::string()> func);
    void setInterfaceLevel(InterfaceLevel level);

    // Not available in the constructor
    std::ostream& os();
    void putWarning(const std::string& message);
    void putError(const std::string& message);

    Item* parentItem();
    InvocationType invocationType() const;

private:
    class Impl;
    Impl* impl;

    friend class ItemManager;
    friend class ItemManagerImpl;
};

typedef ref_ptr<ItemFileIOBase> ItemFileIOBasePtr;

template<class ItemType>
class ItemFileIO : public ItemFileIOBase
{
public:
    ItemFileIO(const std::string& formatId, int api)
        : ItemFileIOBase(formatId, api) {
    }
    virtual bool load(Item* item, const std::string& filename) final {
        return load(static_cast<ItemType*>(item), filename);
    }
    virtual bool load(ItemType* /* item */, const std::string& /* filename */) {
        return false;
    }
    virtual bool save(Item* item, const std::string& filename) final {
        return save(static_cast<ItemType*>(item), filename);
    }
    virtual bool save(ItemType* /* item */, const std::string& /* filename */) {
        return false;
    }
    virtual QWidget* optionPanelForSaving(Item* item) final {
        return optionPanelForSaving(static_cast<ItemType*>(item));
    }
    virtual QWidget* optionPanelForSaving(ItemType* item) {
        return nullptr;
    }
};

}

#endif
