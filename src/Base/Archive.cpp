/**
   @author Shin'ichiro Nakaoka
*/

#include "Archive.h"
#include "Item.h"
#include "MessageView.h"
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

typedef map<Item*, int> ItemToIdMap;
typedef map<int, Item*> IdToItemMap;
typedef map<View*, int> ViewToIdMap;
typedef map<int, View*> IdToViewMap;

typedef list<std::function<void()>> PostProcessList;

}

namespace cnoid {
    
class ArchiveSharedData : public Referenced
{
public:
    FilePathVariableProcessorPtr pathVariableProcessor;
    
    IdToItemMap idToItemMap;
    ItemToIdMap itemToIdMap;

    IdToViewMap idToViewMap;
    ViewToIdMap viewToIdMap;
        
    Item* currentParentItem;

    std::vector<std::function<void()>>* pointerToProcessesOnSubTreeRestored;
    PostProcessList postProcesses;

    ArchiveSharedData(){
        currentParentItem = nullptr;
        pointerToProcessesOnSubTreeRestored = nullptr;
    }
};

}


Archive* Archive::invalidArchive()
{
    static ArchivePtr invalidArchive_ = new Archive();
    invalidArchive_->typeBits = ValueNode::INVALID_NODE;
    return invalidArchive_;
}


Archive::Archive()
{

}


Archive::Archive(int line, int column)
    : Mapping(line, column)
{
    
}


Archive::~Archive()
{

}


void Archive::initSharedInfo()
{
    shared = new ArchiveSharedData;
    shared->pathVariableProcessor = FilePathVariableProcessor::systemInstance();
}    


void Archive::initSharedInfo(const std::string& projectFile)
{
    initSharedInfo();

    auto projectDir = toUTF8(
        filesystem::absolute(fromUTF8(projectFile)).parent_path().generic_string());
    shared->pathVariableProcessor->setBaseDirectory(projectDir);
    shared->pathVariableProcessor->setProjectDirectory(projectDir);
}


void Archive::inheritSharedInfoFrom(Archive& archive)
{
    shared = archive.shared;
}


void Archive::addProcessOnSubTreeRestored(const std::function<void()>& func) const
{
    if(shared->pointerToProcessesOnSubTreeRestored){
        shared->pointerToProcessesOnSubTreeRestored->push_back(func);
    }
}


void Archive::setPointerToProcessesOnSubTreeRestored(std::vector<std::function<void()>>* pfunc)
{
    shared->pointerToProcessesOnSubTreeRestored = pfunc;
}


void Archive::addPostProcess(const std::function<void()>& func, int priority) const
{
    if(shared){
        if(priority <= 0){
            shared->postProcesses.push_back(func);
        } else {
            shared->postProcesses.push_back(
                [this, func, priority](){ addPostProcess(func, priority - 1); });
        }
    }
}


void Archive::callPostProcesses()
{
    if(shared){
        PostProcessList::iterator p = shared->postProcesses.begin();
        while(p != shared->postProcesses.end()){
            (*p)(); // call a post-process function
            ++p;
        }
    }
}


Archive* Archive::findSubArchive(const std::string& name)
{
    Mapping* mapping = findMapping(name);
    if(mapping->isValid()){
        Archive* archive = dynamic_cast<Archive*>(mapping);
        if(archive){
            archive->inheritSharedInfoFrom(*this);
            return archive;
        }
    }
    return invalidArchive();
}


const Archive* Archive::findSubArchive(const std::string& name) const
{
    return const_cast<Archive*>(this)->findSubArchive(name);
}


bool Archive::forSubArchive(const std::string& name, std::function<bool(const Archive& archive)> func) const
{
    const Archive* subArchive = findSubArchive(name);
    if(subArchive->isValid()){
        return func(*subArchive);
    }
    return false;
}


Archive* Archive::openSubArchive(const std::string& name)
{
    Mapping* mapping = findMapping(name);
    Archive* archive = nullptr;
    if(mapping->isValid()){
        archive = dynamic_cast<Archive*>(mapping);
    }
    if(!archive){
        archive = new Archive();
        archive->inheritSharedInfoFrom(*this);
        if(mapping->isValid()){
            Mapping::const_iterator p = mapping->begin();
            while(p != mapping->end()){
                archive->insert(p->first, p->second);
                ++p;
            }
        }
        insert(name, archive);
    }
    return archive;
}


Archive* Archive::subArchive(Mapping* node)
{
    Archive* archive = dynamic_cast<Archive*>(node);
    if(archive){
        archive->inheritSharedInfoFrom(*this);
        return archive;
    }
    return invalidArchive();
}


std::string Archive::expandPathVariables(const std::string& path) const
{
    auto expanded = shared->pathVariableProcessor->expand(path, false);
    if(expanded.empty()){
        MessageView::instance()->putln(
            shared->pathVariableProcessor->errorMessage(), MessageView::Warning);
    }
    return expanded;
}


std::string Archive::resolveRelocatablePath(const std::string& relocatable) const
{
    auto expanded = shared->pathVariableProcessor->expand(relocatable, true);
    if(expanded.empty()){
        expanded = relocatable; // Follow the past specification
        MessageView::instance()->putln(
            shared->pathVariableProcessor->errorMessage(), MessageView::Warning);
    }
    return expanded;
}


bool Archive::readRelocatablePath(const std::string& key, std::string& out_value) const
{
    string relocatable;
    if(read(key, relocatable)){
        out_value = resolveRelocatablePath(relocatable);
        return true;
    }
    return false;
}


std::string Archive::readItemFilePath() const
{
    string filepath;
    if(!readRelocatablePath("file", filepath)){
        // for the backward compatibility
        readRelocatablePath("filename", filepath);
    }
    return filepath;
}


bool Archive::loadFileTo(Item* item) const
{
    string file, format;
    if(readRelocatablePath("file", file)){
        read("format", format);
        return item->load(file, currentParentItem(), format, this);
    }
    // for the backward compatibility
    else if(readRelocatablePath("filename", file)){
        read("format", format);
        return item->load(file, currentParentItem(), format, this);
    }
    return false;
}


bool Archive::loadFileTo(const std::string& filepath, Item* item) const
{
    string format;
    read("format", format);
    return item->load(filepath, currentParentItem(), format, this);
}


bool Archive::loadItemFile(Item* item, const std::string& fileNameKey, const std::string& fileFormatKey) const
{
    string filename, format;
    if(readRelocatablePath(fileNameKey, filename)){
        if(!fileFormatKey.empty()){
            read(fileFormatKey, format);
        }
        return item->load(filename, currentParentItem(), format, this);
    }
    return false;
}


std::string Archive::getRelocatablePath(const std::string& orgPathString) const
{
    return shared->pathVariableProcessor->parameterize(orgPathString);
}


bool Archive::writeRelocatablePath(const std::string& key, const std::string& path)
{
    if(!path.empty()){
        write(key, getRelocatablePath(path), DOUBLE_QUOTED);
        return true;
    }
    return false;
}


bool Archive::writeFileInformation(Item* item)
{
    if(writeRelocatablePath("file", item->filePath())){
        auto& format = item->fileFormat();
        write("format", format);
        if(auto fileOptions = item->fileOptions()){
            insert(fileOptions);
        }
        return true;
    }
    return false;
}


void Archive::clearIds()
{
    if(shared){
        shared->idToItemMap.clear();
        shared->itemToIdMap.clear();
        shared->idToViewMap.clear();
        shared->viewToIdMap.clear();
    }
}
        

void Archive::registerItemId(Item* item, int id)
{
    if(shared){
        shared->idToItemMap[id] = item;
        shared->itemToIdMap[item] = id;
    }
}


ValueNodePtr Archive::getItemId(Item* item) const
{
    if(shared){
        int i = 0;
        Item* mainItem = item;
        while(mainItem->isSubItem()){
            ++i;
            mainItem = mainItem->parentItem();
        }
        ItemToIdMap::const_iterator p = shared->itemToIdMap.find(mainItem);
        if(p != shared->itemToIdMap.end()){
            const int id = p->second;
            if(i == 0){
                return new ScalarNode(id);
            } else {
                ListingPtr idPath = new Listing(i + 1);
                idPath->setFlowStyle(true);
                while(item->isSubItem()){
                    idPath->write(i--, item->name(), DOUBLE_QUOTED);
                    item = item->parentItem();
                }
                idPath->write(0, id);
                return idPath;
            }
        }
    }
    return nullptr;
}


void Archive::writeItemId(const std::string& key, Item* item)
{
    if(item){
        ValueNodePtr id = getItemId(item);
        if(id){
            insert(key, id);
        }
    }
}


Item* Archive::findItem(int id) const
{
    if(shared){
        IdToItemMap::iterator p = shared->idToItemMap.find(id);
        if(p != shared->idToItemMap.end()){
            return p->second;
        }
    }
    return nullptr;
}


Item* Archive::findItem(const ValueNodePtr idNode) const
{
    Item* item = nullptr;
    if(idNode && shared){
        if(idNode->isScalar()){
            item = findItem(idNode->toInt());
        } else if(idNode->isListing()){
            const Listing& idPath = *idNode->toListing();
            const int n = idPath.size();
            if(n >= 2){
                item = findItem(idPath.front()->toInt());
                if(item){
                    for(int i=1; i < n; ++i){
                        item = item->findChildItem(
                            idPath[i].toString(), [](Item* item){ return item->isSubItem(); });
                        if(!item){
                            item = nullptr;
                            break;
                        }
                    }
                }
            }
        }
    }
    return item;
}


void Archive::registerViewId(View* view, int id)
{
    if(shared){
        shared->idToViewMap[id] = view;
        shared->viewToIdMap[view] = id;
    }
}


/**
   @return -1 if item does not belong to the archive
*/
int Archive::getViewId(View* view) const
{
    if(shared && view){
        ViewToIdMap::const_iterator p = shared->viewToIdMap.find(view);
        if(p != shared->viewToIdMap.end()){
            return p->second;
        }
    }
    return -1;
}


View* Archive::findView(int id) const
{
    if(shared){
        IdToViewMap::iterator p = shared->idToViewMap.find(id);
        if(p != shared->idToViewMap.end()){
            return p->second;
        }
    }
    return nullptr;
}


Item* Archive::currentParentItem() const
{
    if(shared){
        return shared->currentParentItem;
    }
    return nullptr;
}


void Archive::setCurrentParentItem(Item* parentItem)
{
    if(shared){
        shared->currentParentItem = parentItem;
    }
}


std::string Archive::projectDirectory() const
{
    if(shared){
        return shared->pathVariableProcessor->projectDirectory();
    }
    return string();
}


FilePathVariableProcessor* Archive::filePathVariableProcessor() const
{
    if(shared){
        return shared->pathVariableProcessor;
    }
    return nullptr;
}

    


