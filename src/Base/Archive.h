/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ARCHIVE_H_INCLUDED
#define CNOID_BASE_ARCHIVE_H_INCLUDED

#include <cnoid/ValueTree>
#include <cnoid/Referenced>
#include <boost/function.hpp>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;
typedef ref_ptr<Item> ItemPtr;

class View;
class ViewManager;

class Archive;
typedef boost::intrusive_ptr<Archive> ArchivePtr;

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

    void addPostProcess(const boost::function<void()>& func) const;

    Archive* findSubArchive(const std::string& name);

    const Archive* findSubArchive(const std::string& name) const {
        return const_cast<Archive*>(this)->findSubArchive(name);
    }
            
    Archive* openSubArchive(const std::string& name);

    //int getItemId(Item* item) const;
    int getItemId(ItemPtr item) const;
    Item* findItem(int id) const;

    int getViewId(View* view) const;
    View* findView(int id) const;

    void clearIds();
        
    template<class ItemType> inline ItemType* findItem(int id) const {
        return dynamic_cast<ItemType*>(findItem(id));
    }

    void writeItemId(const std::string& key, ItemPtr item);

    template<class ItemType> inline ItemType* findItem(const std::string& key) const {
        int id;
        return read(key, id) ? findItem<ItemType>(id) : 0;
    }

    std::string expandPathVariables(const std::string& path) const;
        
    /**
       This method expands path variables and adds the path to the project file
       if the path is relative one
    */
    std::string resolveRelocatablePath(const std::string& relocatable) const;
        
    bool readRelocatablePath(const std::string& key, std::string& out_value) const;

    std::string getRelocatablePath(const std::string& path) const;
    void writeRelocatablePath(const std::string& key, const std::string& path);

    Item* currentParentItem() const;

private:

    ref_ptr<ArchiveSharedData> shared;

    void setCurrentParentItem(Item* parentItem);

    static ArchivePtr invalidArchive();
    void registerItemId(Item* item, int id);
    void registerViewId(View* view, int id);

    // called from ItemTreeArchiver
    void callPostProcesses();

    friend class ItemTreeArchiver;
    friend class ViewManager;
    friend class ProjectManagerImpl;
};
}

#endif
