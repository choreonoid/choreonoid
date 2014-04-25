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
#include <vector>
#include <boost/signals.hpp>
#include <boost/algorithm/string.hpp>
#include <QRegExp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {
QRegExp regexVar("^\\$\\{(\\w+)\\}");

typedef map<ItemPtr, int> ItemToIdMap;
typedef map<View*, int> ViewToIdMap;
}


namespace cnoid {
    
class ArchiveSharedData : public Referenced
{
public:
    Mapping* directoryVariableMap;
    filesystem::path projectDirPath;
    filesystem::path topDirPath;
    filesystem::path shareDirPath;
    QString topDirString;
    QString shareDirString;

    vector<Item*> idToItems;
    ItemToIdMap itemToIds;

    vector<View*> idToViews;
    ViewToIdMap viewToIds;
        
    Item* currentParentItem;

    boost::signal<void()> postProcesses;
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
    MessageView::mainInstance()->putln(QString(_("Warning: ${%1} of \"%2\" cannot be expanded !")).arg(varname).arg(io_pathString));
}
}


ArchivePtr Archive::invalidArchive()
{
    static ArchivePtr invalidArchive_ = new Archive();
    invalidArchive_->type_ = ValueNode::INVALID_NODE;
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

    shared->topDirPath = executableTopDirectory();
    shared->shareDirPath = shareDirectory();

    shared->topDirString = executableTopDirectory().c_str();
    shared->shareDirString = shareDirectory().c_str();
    
    shared->currentParentItem = 0;
}    
    

void Archive::initSharedInfo(const std::string& projectFile)
{
    initSharedInfo();
    
    shared->directoryVariableMap = AppConfig::archive()->openMapping("pathVariables");

    shared->projectDirPath = getAbsolutePath(filesystem::path(projectFile)).parent_path();
}


void Archive::inheritSharedInfoFrom(Archive& archive)
{
    shared = archive.shared;
}


// This cannot be signal because the shared instance may not exist
void Archive::addPostProcess(const boost::function<void()>& func) const
{
    if(shared){
        shared->postProcesses.connect(func);
    }
}


void Archive::callPostProcesses()
{
    if(shared){
        shared->postProcesses();
    }
}


Archive* Archive::findSubArchive(const std::string& name)
{
    Mapping* mapping = findMapping(name);
    if(mapping->isValid()){
        Archive* archive = dynamic_cast<Archive*>(mapping);
        if(archive){
            return archive;
        }
    }

    return invalidArchive().get();
}


Archive* Archive::openSubArchive(const std::string& name)
{
    Archive* archive = findSubArchive(name);
    if(!archive){
        archive = new Archive();
        archive->inheritSharedInfoFrom(*this);
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
            } else {
                replaceDirectoryVariable(shared.get(), qpath, varname, pos, len);
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
    
    } else if(findSubDirectoryOfDirectoryVariable(shared.get(), orgPath, varName, relativePath)){
        return string("${") + varName + ("}/") + getGenericPathString(relativePath);

    } else if(findSubDirectory(shared->shareDirPath, orgPath, relativePath)){
        return string("${SHARE}/") + getGenericPathString(relativePath);

    } else if(findSubDirectory(shared->topDirPath, orgPath, relativePath)){
        return string("${PROGRAM_TOP}/") + getGenericPathString(relativePath);

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
        shared->idToItems.clear();
        shared->itemToIds.clear();
        shared->idToViews.clear();
        shared->viewToIds.clear();
    }
}
        

void Archive::registerItemId(Item* item, int id)
{
    if(shared){
        if(id >= (signed)shared->idToItems.size()){
            shared->idToItems.resize(id + 1);
        }
        shared->idToItems[id] = item;
        shared->itemToIds[item] = id;
    }
}


/**
   @return -1 if item does not belong to the archive
*/
//int Archive::getItemId(Item* item) const
int Archive::getItemId(ItemPtr item) const
{
    if(shared && item){
        ItemToIdMap::const_iterator p = shared->itemToIds.find(item);
        if(p != shared->itemToIds.end()){
            return p->second;
        }
    }
    return -1;
}


void Archive::writeItemId(const std::string& key, ItemPtr item)
{
    if(item){
        int id = getItemId(item);
        if(id >= 0){
            write(key, id);
        }
    }
}


Item* Archive::findItem(int id) const
{
    if(shared){
        if(id >= 0 && id < (signed)shared->idToItems.size()){
            return shared->idToItems[id];
        }
    }
    return 0;
}


void Archive::registerViewId(View* view, int id)
{
    if(shared){
        if(id >= (signed)shared->idToViews.size()){
            shared->idToViews.resize(id + 1);
        }
        shared->idToViews[id] = view;
        shared->viewToIds[view] = id;
    }
}


/**
   @return -1 if item does not belong to the archive
*/
int Archive::getViewId(View* view) const
{
    if(shared && view){
        ViewToIdMap::const_iterator p = shared->viewToIds.find(view);
        if(p != shared->viewToIds.end()){
            return p->second;
        }
    }
    return -1;
}


View* Archive::findView(int id) const
{
    if(shared){
        if(id >= 0 && id < (signed)shared->idToViews.size()){
            return shared->idToViews[id];
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
