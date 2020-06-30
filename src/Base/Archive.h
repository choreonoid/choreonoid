/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ARCHIVE_H
#define CNOID_BASE_ARCHIVE_H

#include <cnoid/ValueTree>
#include <string>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class View;
class ViewManager;
class ProjectManager;
class FilePathVariableProcessor;
class ArchiveSharedData;

class CNOID_EXPORT Archive : public Mapping
{
public:
    Archive();
    Archive(int line, int column);
    virtual ~Archive();

    void initSharedInfo();
    void initSharedInfo(const std::string& projectFile);
    void inheritSharedInfoFrom(Archive& archive);

    void addProcessOnSubTreeRestored(const std::function<void()>& func) const;
    void addPostProcess(const std::function<void()>& func, int priority = 0) const;

    Archive* findSubArchive(const std::string& name);
    const Archive* findSubArchive(const std::string& name) const;
    bool forSubArchive(const std::string& name, std::function<bool(const Archive& archive)> func) const;
    Archive* openSubArchive(const std::string& name);
    Archive* subArchive(Mapping* node);

    ValueNodePtr getItemId(Item* item) const;
    Item* findItem(ValueNodePtr id) const;
    
    int getViewId(View* view) const;
    View* findView(int id) const;

    void clearIds();
        
    template<class ItemType> inline ItemType* findItem(ValueNodePtr id) const {
        return dynamic_cast<ItemType*>(findItem(id));
    }

    void writeItemId(const std::string& key, Item* item);

    template<class ItemType> inline ItemType* findItem(const std::string& key) const {
        ValueNode* id = find(key);
        return id->isValid() ? findItem<ItemType>(id) : 0;
    }

    std::string expandPathVariables(const std::string& path) const;
        
    /**
       This method expands path variables and adds the path to the project file
       if the path is relative one
    */
    std::string resolveRelocatablePath(const std::string& relocatable) const;
        
    bool readRelocatablePath(const std::string& key, std::string& out_value) const;
    std::string readItemFilePath() const;
    bool loadFileTo(Item* item) const;
    bool loadFileTo(const std::string& filepath, Item* item) const;
    //! \deprecated
    bool loadItemFile(Item* item, const std::string& fileNameKey, const std::string& fileFormatKey = std::string()) const;
    
    std::string getRelocatablePath(const std::string& path) const;
    bool writeRelocatablePath(const std::string& key, const std::string& path);
    bool writeFileInformation(Item* item);

    Item* currentParentItem() const;

    std::string projectDirectory() const;

    FilePathVariableProcessor* filePathVariableProcessor() const;

private:
    ref_ptr<ArchiveSharedData> shared;

    Item* findItem(int id) const;
    void setCurrentParentItem(Item* parentItem);
    static Archive* invalidArchive();
    void registerItemId(Item* item, int id);
    void registerViewId(View* view, int id);

    // called from ItemTreeArchiver
    void setPointerToProcessesOnSubTreeRestored(std::vector<std::function<void()>>* pfunc);
    void callPostProcesses();

    friend class ItemTreeArchiver;
    friend class ViewManager;
    friend class ProjectManager;
};

typedef ref_ptr<Archive> ArchivePtr;

}

#endif
