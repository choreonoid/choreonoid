/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_ITEMMANAGERIMPL_H_INCLUDED
#define CNOID_ITEMMANAGERIMPL_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <QDialog>
#include <QLayout>
#include <QSignalMapper>
#include <set>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class ItemManagerImpl
{
public:
    ItemManagerImpl(const string& moduleName, MenuManager& menuManager);
    ~ItemManagerImpl();

    typedef list<ItemManager::CreationPanelFilterBasePtr> CreationPanelFilterList;
    typedef set< pair<string, ItemManager::CreationPanelFilterBasePtr> > CreationPanelFilterSet;

    class CreationPanelBase : public QDialog
    {
    public:
        CreationPanelBase(const QString& title, ItemPtr protoItem);
        void addPanel(ItemCreationPanel* panel);
        ItemPtr createItem(ItemPtr parentItem);
        CreationPanelFilterList preFilters;
        CreationPanelFilterList postFilters;
    private:
        QVBoxLayout* panelLayout;
        ItemPtr protoItem;
    };

    struct ClassInfo
    {
        ClassInfo() { factory = 0; creationPanelBase = 0; }
        ~ClassInfo() { delete factory; delete creationPanelBase; }
        string moduleName;
        string className;
        ItemManager::FactoryBase* factory;
        CreationPanelBase* creationPanelBase;
    };
    typedef boost::shared_ptr<ClassInfo> ClassInfoPtr;

    class Loader : public QObject
    {
    public:
        string typeId;
        string formatId;
        string caption;
        int priority;
        ItemManager::FileFunctionBasePtr loadingFunction;
        ItemManager::FactoryBase* factory;
        vector<string> extensions;
    };
    typedef boost::shared_ptr<Loader> LoaderPtr;
        
    struct Saver
    {
        string typeId;
        string formatId;
        string caption;
        int priority;
        vector<string> extensions;
        ItemManager::FileFunctionBasePtr savingFunction;
    };
    typedef boost::shared_ptr<Saver> SaverPtr;

    typedef list<SaverPtr> SaverList;

    string moduleName;
    string textDomain;
    MenuManager& menuManager;
    set<string> registeredTypeIds;
    set<ItemCreationPanel*> registeredCreationPanels;
    CreationPanelFilterSet registeredCreationPanelFilters;
    set<LoaderPtr> registeredLoaders;
    set<SaverPtr> registeredSavers;

    QSignalMapper* mapperForNewItemActivated;
    QSignalMapper* mapperForLoadSpecificTypeItemActivated;

    void detachManagedTypeItems(Item* parentItem);
        
    void registerClass(ItemManager::FactoryBase* factory, const string& typeId, const string& className);
    static bool getClassIdentifier(ItemPtr item, string& out_moduleName, string& out_className);

    static ItemPtr create(const string& moduleName, const string& className);

    void addCreationPanel(const std::string& typeId, ItemCreationPanel* panel);
    void addCreationPanelFilter(
        const std::string& typeId, ItemManager::CreationPanelFilterBasePtr filter, bool afterInitializionByPanels);
    CreationPanelBase* getOrCreateCreationPanelBase(const string& typeId);

    void addLoader
    (const std::string& typeId, const std::string& caption, const std::string& formatId,
     const std::string& extensions, const ItemManager::FileFunctionBasePtr function, int priority);

    static bool load(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId);
    static bool load(LoaderPtr loader, Item* item, const std::string& filename, Item* parentItem);

    void addSaver
    (const string& typeId, const string& caption, const string& formatId, const string& extensions,
     ItemManager::FileFunctionBasePtr function, int priority);

    static bool save(Item* item, bool useDialogToGetFilename, bool doExport, std::string filename, const std::string& formatId);
    static SaverPtr getSaverAndFilenameFromSaveDialog(
        SaverList& savers, bool doExport,
        const string& itemLabel, const string& formatId, string& io_filename);
    static SaverPtr determineSaver(SaverList& savers, const string& filename, const string& formatId);
    static bool overwrite(Item* item, bool forceOverwrite, const std::string& formatId);

    static void onNewItemActivated(CreationPanelBase* base);
    void onLoadItemActivated();
    static void onLoadSpecificTypeItemActivated(LoaderPtr loader);
    void onReloadSelectedItemsActivated();
    void onSaveSelectedItemsActivated();
    void onSaveSelectedItemsAsActivated();
    void onSaveAllItemsActivated();
    void onExportSelectedItemsActivated();
};
}

#endif
