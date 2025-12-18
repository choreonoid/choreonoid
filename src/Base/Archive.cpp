#include "Archive.h"
#include "Item.h"
#include "ProjectBackupManager.h"
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <cnoid/UTF8>
#include <filesystem>
#include <map>
#include <list>
#include <deque>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef map<Item*, int> ItemToIdMap;
typedef map<int, Item*> IdToItemMap;
typedef map<View*, int> ViewToIdMap;
typedef map<int, View*> IdToViewMap;

struct FunctionInfo {
    function<void()> func;
    int priority;
    FunctionInfo(function<void()> func, int priority)
        : func(func), priority(priority) { }
};

deque<function<void()>> finalProcesses;

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

    Item* currentItem;
    Item* currentParentItem;

    typedef list<function<void()>> ProcessList;
    map<Item*, ProcessList> processesOnSubTreeRestored;
    vector<FunctionInfo> postProcesses;
    vector<FunctionInfo> nextPostProcesses;
    bool isDoingPostProcesses;
    bool isSavingProjectAsBackup;

    MessageOut* mout;

    ArchiveSharedData(){
        currentItem = nullptr;
        currentParentItem = nullptr;
        isDoingPostProcesses = false;
    }
};

}


Archive* Archive::invalidArchive()
{
    static ArchivePtr invalidArchive_ = new Archive;
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


void Archive::initSharedInfo
(const std::string& projectFile, bool isSubProject, MessageOut* mout, bool isSavingProjectAsBackup)
{
    shared = new ArchiveSharedData;

    auto currentFpvp = FilePathVariableProcessor::currentInstance();

    if(!isSubProject){
        shared->pathVariableProcessor = currentFpvp;
    } else {
        shared->pathVariableProcessor = new FilePathVariableProcessor(*currentFpvp);
    }

    auto projectDir = toUTF8(filesystem::absolute(fromUTF8(projectFile)).parent_path().generic_string());
    shared->pathVariableProcessor->setBaseDirectory(projectDir);
    shared->pathVariableProcessor->setProjectDirectory(projectDir);

    shared->mout = mout;
    shared->isSavingProjectAsBackup = isSavingProjectAsBackup;
}


void Archive::inheritSharedInfoFrom(Archive& archive)
{
    shared = archive.shared;
}


void Archive::addProcessOnSubTreeRestored(const std::function<void()>& func) const
{
    addProcessOnSubTreeRestored(shared->currentItem, func);
}


void Archive::addProcessOnSubTreeRestored(Item* item, const std::function<void()>& func) const
{
    shared->processesOnSubTreeRestored[item].push_back(func);
}


void Archive::callProcessesOnSubTreeRestored(Item* item)
{
    auto& processes = shared->processesOnSubTreeRestored[item];
    while(!processes.empty()){
        auto& process = processes.front();
        process();
        processes.pop_front();
    }
}


void Archive::addPostProcess(const std::function<void()>& func, int priority) const
{
    if(shared){
        if(!shared->isDoingPostProcesses){
            shared->postProcesses.emplace_back(func, priority);
        } else {
            shared->nextPostProcesses.emplace_back(func, priority);
        }
    }
}


void Archive::callPostProcesses()
{
    if(shared){
        shared->isDoingPostProcesses = true;
        auto& processes = shared->postProcesses;
        while(!processes.empty()){
            std::sort(processes.begin(), processes.end(),
                      [](const FunctionInfo& info1, const FunctionInfo& info2){
                          return info1.priority < info2.priority;
                      });
            for(auto& process : processes){
                process.func();
            }
            processes.clear();
            if(!shared->nextPostProcesses.empty()){
                processes.swap(shared->nextPostProcesses);
            }
        }
        shared->isDoingPostProcesses = false;
    }
}


void Archive::addFinalProcess(const std::function<void()>& func) const
{
    finalProcesses.push_back(func);
}


void Archive::callFinalProcesses()
{
    while(!finalProcesses.empty()){
        finalProcesses.front()();
        finalProcesses.pop_front();
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
        archive = new Archive;
        archive->inheritSharedInfoFrom(*this);
        if(mapping->isValid()){
            auto p = mapping->begin();
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


const Archive* Archive::subArchive(Mapping* node) const
{
    return const_cast<Archive*>(this)->subArchive(node);
}


std::string Archive::resolveRelocatablePath(const std::string& relocatable, bool doAbsolutize) const
{
    auto expanded = shared->pathVariableProcessor->expand(relocatable, doAbsolutize);
    if(expanded.empty()){
        mout()->putErrorln(shared->pathVariableProcessor->errorMessage());
    }
    return expanded;
}


bool Archive::readRelocatablePath(const std::string& key, std::string& out_value) const
{
    string relocatable;
    if(read(key, relocatable)){
        out_value = resolveRelocatablePath(relocatable);
        if(!out_value.empty()){
            return true;
        }
    }
    return false;
}


std::string Archive::readItemFilePath() const
{
    string filepath;
    if(read({ "file", "filename" }, filepath)){
        filepath = resolveRelocatablePath(filepath);
    }
    return filepath;
}


bool Archive::loadFileTo(Item* item, bool& out_hasFileInformation) const
{
    bool loaded = false;
    auto mout_ = mout();
    out_hasFileInformation = false;

    string file;
    string fileFormat;
    string backupFile;
    string backupFileFormat;

    if(read({ "file", "filename" }, file)){
        out_hasFileInformation = true;
        file = resolveRelocatablePath(file);
    }
    read("format", fileFormat);
    read("backup_file", backupFile);
    read("backup_file_format", backupFileFormat);

    if(backupFile.empty()){
        if(!file.empty()){
            loaded = item->load(file, currentParentItem(), fileFormat, this, mout_);
        }
    } else  {
        backupFile = resolveRelocatablePath(backupFile);
        if(!backupFile.empty()){
            loaded = item->load(backupFile, currentParentItem(), backupFileFormat, this, mout_);
            if(loaded){
                item->updateFileInformation(file, fileFormat, item->fileOptions(), false);
                if(!file.empty()){
                    filesystem::path filePath(fromUTF8(file));
                    filesystem::path backupPath(fromUTF8(backupFile));
                    std::error_code ec;
                    if(!filesystem::equivalent(filePath, backupPath, ec)){
                        item->setConsistentWithFile(false);
                    }
                    mout_->putln(
                        formatR(_("Note that item \"{0}\" will be saved to the original file \"{1}\"."),
                                item->displayName(), file));
                }
            }
        }
    }
    
    return loaded;
}


bool Archive::loadFileTo(Item* item) const
{
    bool hasFileInformation;
    return loadFileTo(item, hasFileInformation);
}


bool Archive::loadFileTo(Item* item, const std::string& filepath) const
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
        const auto& format = item->fileFormat();
        if(!format.empty()){
            write("format", format);
        }
        if(auto fileOptions = item->fileOptions()){
            insert(fileOptions);
        }
        return true;
    }
    return false;
}


bool Archive::saveItemToFile(Item* item)
{
    if(!shared){
        return false;
    }
    
    bool saved = false;
    
    if(!shared->isSavingProjectAsBackup){
        if(item->overwriteOrSaveWithDialog()){
            writeFileInformation(item);
            saved = true;
        }
    } else {
        auto backupManager = ProjectBackupManager::instance();
        filesystem::path backupFilePath;
        filesystem::path hardLinkFilePath;
        string fileFormat;
        
        if(backupManager->getItemBackupFileInformation(item, backupFilePath, hardLinkFilePath, fileFormat)){

            string backupFilename = toUTF8(backupFilePath.string());

            if(!hardLinkFilePath.empty()){
                std::error_code ec;
                filesystem::create_hard_link(hardLinkFilePath, backupFilePath, ec);
                if(ec){
                    mout()->putErrorln(toUTF8(ec.message()));
                } else {
                    saved = true;
                }
            } else {
                string orgFilename = item->filePath();
                string orgFileFormat = item->fileFormat();
                auto orgModificationTime = item->fileModificationTime();
                bool orgProjectConsistency = item->isConsistentWithProjectArchive();
                bool orgFileConsistency = item->isConsistentWithFile();

                item->updateFileInformation(backupFilename, item->fileFormat(), item->fileOptions(), false);
                item->setConsistentWithFile(false);
                saved = item->overwrite(false, std::string(), orgModificationTime, mout());

                fileFormat = item->fileFormat();
                backupManager->setItemBackupFileFormat(item, fileFormat);
                
                item->updateFileInformation(orgFilename, orgFileFormat, item->fileOptions(), false);
                item->setConsistentWithFile(orgFileConsistency);
                item->setConsistentWithProjectArchive(orgProjectConsistency);
            }

            if(saved){
                writeFileInformation(item);
                if(!writeRelocatablePath("backup_file", backupFilename)){
                    saved = false;
                } else if(!fileFormat.empty()){
                    write("backup_file_format", fileFormat);
                }
            }
        }
    }
    
    return saved;
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
        

void Archive::registerItemId(const Item* item, int id)
{
    if(shared){
        shared->idToItemMap[id] = const_cast<Item*>(item);
        shared->itemToIdMap[const_cast<Item*>(item)] = id;
    }
}


ValueNodePtr Archive::getItemIdNode(const Item* item) const
{
    if(shared){
        int i = 0;
        Item* mainItem = const_cast<Item*>(item);
        while(mainItem->isSubItem()){
            ++i;
            mainItem = mainItem->parentItem();
        }
        auto p = shared->itemToIdMap.find(mainItem);
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
        if(auto id = getItemIdNode(item)){
            insert(key, id);
        }
    }
}


Item* Archive::findItem(int id) const
{
    if(shared){
        auto p = shared->idToItemMap.find(id);
        if(p != shared->idToItemMap.end()){
            return p->second;
        }
    }
    return nullptr;
}


Item* Archive::findItem(const ValueNode* idNode) const
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
                            break;
                        }
                    }
                }
            }
        }
    }
    return item;
}


void Archive::registerViewId(const View* view, int id)
{
    if(shared){
        shared->idToViewMap[id] = const_cast<View*>(view);
        shared->viewToIdMap[const_cast<View*>(view)] = id;
    }
}


/**
   @return -1 if item does not belong to the archive
*/
int Archive::getViewId(const View* view) const
{
    if(shared && view){
        auto p = shared->viewToIdMap.find(const_cast<View*>(view));
        if(p != shared->viewToIdMap.end()){
            return p->second;
        }
    }
    return -1;
}


View* Archive::findView(int id) const
{
    if(shared){
        auto p = shared->idToViewMap.find(id);
        if(p != shared->idToViewMap.end()){
            return p->second;
        }
    }
    return nullptr;
}


void Archive::setCurrentItem(Item* item)
{
    if(shared){
        shared->currentItem = item;
    }
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


bool Archive::isSavingProjectAsBackup() const
{
    return shared ? shared->isSavingProjectAsBackup : false;
}


MessageOut* Archive::mout() const
{
    if(shared){
        return shared->mout;
    }
    return MessageOut::nullout();
}
