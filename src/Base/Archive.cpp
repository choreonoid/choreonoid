/**
   @author Shin'ichiro Nakaoka
*/

#include "Archive.h"
#include "Item.h"
#include "AppConfig.h"
#include "MessageView.h"
#include <cnoid/ExecutablePath>
#include <cnoid/Referenced>
#include <cnoid/FileUtil>
#include <map>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <QRegExp>
#include <iostream>
#include <cstdlib>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

QRegExp regexVar("^\\$\\{(\\w+)\\}");

typedef map<Item*, int> ItemToIdMap;
typedef map<int, Item*> IdToItemMap;
typedef map<View*, int> ViewToIdMap;
typedef map<int, View*> IdToViewMap;

typedef list< boost::function<void()> > PostProcessList;

}

namespace cnoid {
    
class ArchiveSharedData : public Referenced
{
public:
    Mapping* directoryVariableMap;
    filesystem::path projectDirPath;
    filesystem::path topDirPath;
    filesystem::path shareDirPath;
    filesystem::path homeDirPath;
    QString topDirString;
    QString shareDirString;
    QString homeDirString;

    IdToItemMap idToItemMap;
    ItemToIdMap itemToIdMap;

    IdToViewMap idToViewMap;
    ViewToIdMap viewToIdMap;
        
    Item* currentParentItem;

    PostProcessList postProcesses;
};


/**
   \todo Introduce a tree structure to improve the efficiency of searching matched directories
*/
bool findSubDirectoryOfDirectoryVariable
(ArchiveSharedData* shared, const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath)
{
    out_relativePath.clear();
    int maxMatchSize = 0;
    filesystem::path relativePath;
    Mapping::const_iterator p;
    for(p = shared->directoryVariableMap->begin(); p != shared->directoryVariableMap->end(); ++p){
        Listing* paths = p->second->toListing();
        if(paths){
            for(int i=0; i < paths->size(); ++i){
                filesystem::path dirPath(paths->at(i)->toString());
                int n = findSubDirectory(dirPath, path, relativePath);
                if(n > maxMatchSize){
                    maxMatchSize = n;
                    out_relativePath = relativePath;
                    out_varName = fromUTF8(p->first);
                }
            }
        }
    }
    return (maxMatchSize > 0);
}


void replaceDirectoryVariable(ArchiveSharedData* shared, QString& io_pathString, const QString& varname, int pos, int len)
{
    Listing* paths = shared->directoryVariableMap->findListing(varname.toStdString());
    if(paths){
        for(int i=0; i < paths->size(); ++i){
            string vpath;
            QString replaced(io_pathString);
            replaced.replace(pos, len, paths->at(i)->toString().c_str());
            filesystem::file_status fstatus = filesystem::status(filesystem::path(replaced.toStdString()));
            if(filesystem::is_directory(fstatus) || filesystem::exists(fstatus)) {
                io_pathString = replaced;
                return;
            }
        }
    }
    MessageView::mainInstance()->putln(
        MessageView::WARNING,
        QString(_("${%1} of \"%2\" cannot be expanded !")).arg(varname).arg(io_pathString));
}

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


void Archive::initSharedInfo(bool useHomeRelativeDirectories)
{
    shared = new ArchiveSharedData;

    shared->topDirPath = executableTopDirectory();
    shared->shareDirPath = shareDirectory();

    shared->topDirString = executableTopDirectory().c_str();
    shared->shareDirString = shareDirectory().c_str();

    char* home = getenv("HOME");
    if(home){
        if(useHomeRelativeDirectories){
            shared->homeDirPath = filesystem::path(home);
        }
        shared->homeDirString = home;
    }
    
    shared->currentParentItem = 0;
}    
    

void Archive::initSharedInfo(const std::string& projectFile, bool useHomeRelativeDirectories)
{
    initSharedInfo(useHomeRelativeDirectories);
    
    shared->directoryVariableMap = AppConfig::archive()->openMapping("pathVariables");

    shared->projectDirPath = getAbsolutePath(filesystem::path(projectFile)).parent_path();
}


void Archive::inheritSharedInfoFrom(Archive& archive)
{
    shared = archive.shared;
}


void Archive::addPostProcess(const boost::function<void()>& func, int priority) const
{
    if(shared){
        if(priority <= 0){
            shared->postProcesses.push_back(func);
        } else {
            shared->postProcesses.push_back(
                boost::bind(&Archive::addPostProcess, this, func, priority - 1));
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


bool Archive::forSubArchive(const std::string& name, boost::function<bool(const Archive& archive)> func) const
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
    Archive* archive = 0;
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


std::string Archive::expandPathVariables(const std::string& path) const
{
    QString qpath(path.c_str());

    // expand variables in the path
    int pos = regexVar.indexIn(qpath);
    if(pos != -1){
        int len = regexVar.matchedLength();
        if(regexVar.captureCount() > 0){
            QString varname = regexVar.cap(1);
            if(varname == "SHARE"){
                qpath.replace(pos, len, shared->shareDirString);
            } else if(varname == "PROGRAM_TOP"){
                qpath.replace(pos, len, shared->topDirString);
            } else if(varname == "HOME"){
                qpath.replace(pos, len, shared->homeDirString);
            } else {
                replaceDirectoryVariable(shared, qpath, varname, pos, len);
            }
        }
    }
            
    return qpath.toStdString();
}


std::string Archive::resolveRelocatablePath(const std::string& relocatable) const
{
    filesystem::path path(expandPathVariables(relocatable));

    if(checkAbsolute(path)){
        return getNativePathString(path);
    } else {
        filesystem::path fullPath = shared->projectDirPath / path;
        if(!path.empty() && (*path.begin() == "..")){
            filesystem::path compact;
            makePathCompact(fullPath, compact);
            return getNativePathString(compact);
        } else {
            return getNativePathString(fullPath);
        }
    }
    return relocatable;
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


/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
std::string Archive::getRelocatablePath(const std::string& orgPathString) const
{
    filesystem::path orgPath(orgPathString);
    filesystem::path relativePath;
    string varName;

    // In the case where the path is originally relative one
    if(!orgPath.is_complete()){
        return getGenericPathString(orgPath);

    } else if(findSubDirectory(shared->projectDirPath, orgPath, relativePath)){
        return getGenericPathString(relativePath);
    
    } else if(findSubDirectoryOfDirectoryVariable(shared, orgPath, varName, relativePath)){
        return string("${") + varName + ("}/") + getGenericPathString(relativePath);

    } else if(findSubDirectory(shared->shareDirPath, orgPath, relativePath)){
        return string("${SHARE}/") + getGenericPathString(relativePath);

    } else if(findSubDirectory(shared->topDirPath, orgPath, relativePath)){
        return string("${PROGRAM_TOP}/") + getGenericPathString(relativePath);

    } else if(findSubDirectory(shared->homeDirPath, orgPath, relativePath)){
        return string("${HOME}/") + getGenericPathString(relativePath);

    } else if(findRelativePath(shared->projectDirPath, orgPath, relativePath)){
        return getGenericPathString(relativePath);
    }

    return getGenericPathString(orgPath);
}


void Archive::writeRelocatablePath(const std::string& key, const std::string& path)
{
    write(key, getRelocatablePath(path), DOUBLE_QUOTED);
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
    return 0;
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
    return 0;
}


Item* Archive::findItem(const ValueNodePtr idNode) const
{
    Item* item = 0;
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
                        item = item->findSubItem(idPath[i].toString());
                        if(!item){
                            item = 0;
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
    return 0;
}


Item* Archive::currentParentItem() const
{
    if(shared){
        return shared->currentParentItem;
    }
    return 0;
}


void Archive::setCurrentParentItem(Item* parentItem)
{
    if(shared){
        shared->currentParentItem = parentItem;
    }
}
