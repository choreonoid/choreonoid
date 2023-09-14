#include "ProjectPacker.h"
#include "ProjectManager.h"
#include "Item.h"
#include "RootItem.h"
#include "MessageView.h"
#include <cnoid/ValueTree>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/Config>
#include <fmt/format.h>
#include <zip.h>
#include <map>
#include <deque>
#include <algorithm>
#include <regex>
#include <cstdio>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace fs = stdx::filesystem;

namespace cnoid {

class ProjectPacker::Impl
{
public:
    ProjectPacker* self;
    Item* topItemForPacking;

    MessageOut* mout;

    struct PathNode
    {
        fs::path fullPath;
        std::map<string, PathNode> childNodes;
        bool isShared;
        bool isNecessary;

        PathNode() : isShared(false), isNecessary(false) { }
    };

    PathNode rootNode;

    vector<string> itemDependentFiles;
    vector<fs::path> allPaths; // unified format paths
    fs::path topDirPath; // unified format path
    fs::path packingDirPath;
    string unpackedProjectFile;

    vector<fs::path> refDirPaths;
    string unpackingDir;

    Impl(ProjectPacker* self);
    fs::path getUnifiedFormatPath(const std::string& pathString, bool& out_isAbsolute);
    bool packProjectToZipFile(const std::string& filename, const std::string& projectName);
    bool packProjectToDirectory(const std::string& packingDirectory, const std::string& projectName);
    void checkFileDependency(Item* item);
    bool checkIfPathInReferenceDirectory(const fs::path& path);
    void updatePackingItmes(Item* item);
    std::string getRelocatedFilePath(const std::string& pathString);
    bool createProjectZipFile(const string& zipFilename);
    bool addDirectoryToZip(zip_t * zip, fs::path dirPath, const fs::path& zipTopDirPath);
    bool unpackProject(const std::string& projectPackFile);
    bool extractFiles(
        zip_t* zip, const string& zipFilename, const fs::path&  zipFilePath, const fs::path& topDirPath);
    bool loadUnpackedProject(const std::string& projectFile);    
};

}


ProjectPacker::ProjectPacker()
{
    impl = new Impl(this);
}


ProjectPacker::Impl::Impl(ProjectPacker* self)
    : self(self)
{
    topItemForPacking = nullptr;
    
    mout = MessageOut::master();
    self->mout_ = mout;
}


ProjectPacker::~ProjectPacker()
{
    delete impl;
}


void ProjectPacker::setTopItemForPacking(Item* item)
{
    impl->topItemForPacking = item;
}


void ProjectPacker::addReferenceDirectory(std::string directory)
{
    bool isAbsolute;
    auto ufPath = impl->getUnifiedFormatPath(directory, isAbsolute);
    if(isAbsolute){
        impl->refDirPaths.push_back(ufPath);
    }
}


void ProjectPacker::clearReferenceDirectories()
{
    impl->refDirPaths.clear();
}


void ProjectPacker::setUnpackingDirectory(const std::string& directory)
{
    impl->unpackingDir = directory;
}


const std::string& ProjectPacker::unpackingDirectory() const
{
    return impl->unpackingDir;
}


fs::path ProjectPacker::Impl::getUnifiedFormatPath(const std::string& pathString, bool& out_isAbsolute)
{
    fs::path ufPath;

#ifndef _WIN32
    ufPath = fromUTF8(pathString);
    out_isAbsolute = ufPath.is_absolute();
#else
    // Replace a drive letter
    static regex re("^([A-za-z]):");
    string replaced = regex_replace(pathString, re, "\\$1");
    out_isAbsolute = (replaced != pathString);
    ufPath = fromUTF8(replaced);
#endif

    return ufPath.make_preferred().lexically_normal();
}


bool ProjectPacker::packProjectToZipFile(const std::string& filename)
{
    auto projectName = toUTF8(fs::path(fromUTF8(filename)).stem().string());
    return impl->packProjectToZipFile(filename, projectName);
}


bool ProjectPacker::packProjectToZipFile(const std::string& filename, const std::string& projectName)
{
    return impl->packProjectToZipFile(filename, projectName);
}


bool ProjectPacker::Impl::packProjectToZipFile(const std::string& filename, const std::string& projectName)
{
    auto directory = filename + ".tmp";
    bool packed = packProjectToDirectory(directory, projectName);
    if(packed){
        packed = createProjectZipFile(filename);
        stdx::error_code ec;        
        fs::remove_all(fromUTF8(directory), ec);
    }
    return packed;
}


bool ProjectPacker::packProjectToDirectory(const std::string& packingDirectory)
{
    fs::path dirPath(fromUTF8(packingDirectory));
    string projectName = toUTF8(dirPath.filename().string());
    return impl->packProjectToDirectory(packingDirectory, projectName);
}


bool ProjectPacker::packProjectToDirectory(const std::string& packingDirectory, const std::string& projectName)
{
    return impl->packProjectToDirectory(packingDirectory, projectName);
}


bool ProjectPacker::Impl::packProjectToDirectory(const std::string& packingDirectory, const std::string& projectName)
{
    Item* topItem = topItemForPacking ? topItemForPacking : RootItem::instance();

    rootNode.childNodes.clear();
    rootNode.isShared = false;
    rootNode.isNecessary = false;
    allPaths.clear();
    
    checkFileDependency(topItem);

    if(allPaths.size() >= 2){
        for(size_t i = 0; i < allPaths.size() - 1; ++i){
            for(size_t j = i + 1; j < allPaths.size(); ++j){
                auto& pi = allPaths[i];
                auto& pj = allPaths[j];
                auto mismatched =
                    std::mismatch(
                        pi.begin(), pi.end(), pj.begin(), pj.end()
#ifdef _WIN32
                        // Case insentive comparison is necessary for Windows
                        , [](const fs::path& p1, const fs::path& p2){
                            auto s1 = p1.string();
                            auto s2 = p2.string();
                            if(s1.size() == s2.size()){
                                std::transform(s1.begin(), s1.end(), s1.begin(), ::tolower);
                                std::transform(s2.begin(), s2.end(), s2.begin(), ::tolower);
                                return s1 == s2;
                            }
                            return false;
                        }
#endif
                        );
                
                // Set a flag for the upper element of the first mismatch element
                auto pi_mismatched = mismatched.first;
                if(pi_mismatched != pi.end() && pi_mismatched != pi.begin()){
                    PathNode* node = &rootNode;
                    auto it = pi.begin();
                    do {
                        auto key = it->string();
#ifdef _WIN32
                        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
#endif
                        node = &node->childNodes[key];
                        ++it;
                        if(it == pi_mismatched){
                            node->isShared = true;
                            node->isNecessary = true;
                            break;
                        }
                    } while(it != pi.end());
                }
            }
        }
    }

    stdx::error_code ec;
    
    try {
        // Find the top directory of the packed project using breadth-first search
        topDirPath.clear();
        deque<PathNode*> nodeQueue;
        nodeQueue.push_back(&rootNode);
        while(!nodeQueue.empty()){
            auto node = nodeQueue.front();
            nodeQueue.pop_front();
            if(node->isNecessary){
                auto& path = node->fullPath;
                if(node->isShared || !path.has_filename() || !fs::is_regular_file(path)){
                    topDirPath = path;
                } else {
                    topDirPath = path.remove_filename();
                }
                break;
            }
            for(auto& kv : node->childNodes){
                auto childNode = &kv.second;
                nodeQueue.push_back(childNode);
            }
        }

        // Copy the files
        packingDirPath = fromUTF8(packingDirectory);
        if(!packingDirPath.is_absolute()){
            packingDirPath = fs::absolute(packingDirPath);
        }

        if(fs::exists(packingDirPath)){
            fs::remove_all(packingDirPath, ec);
            if(ec){
                mout->putErrorln(
                    format(_("Directory \"{0}\" for packing a project already exists and cannot be removed: {1}."),
                           toUTF8(packingDirPath.string()), toUTF8(ec.message())));
                return false;
            }
        }
    }
    catch(const fs::filesystem_error& error) {
        mout->putErrorln(
            format(_("File system error in packing a project: {0}"), toUTF8(error.what())));
        return false;
    }

    fs::create_directories(packingDirPath, ec);
    if(ec){
        mout->putErrorln(
            format(_("Directory \"{0}\" for packing a project cannot be created: {1}."),
                   toUTF8(packingDirPath.string()), toUTF8(ec.message())));
        return false;
    }

    for(auto& path : allPaths){
        auto relDirPath = getRelativePath(path.parent_path(), topDirPath);
        if(!relDirPath){
            continue; // Ignore an invalid path
        }
        auto destDirPath = (packingDirPath / *relDirPath).lexically_normal();

#ifdef _WIN32
        // Restore the original drive symbol in Windows
        static regex re("^\\\\([A-Za-z])");
        path = regex_replace(path.string(), re, "$1:");
#endif
        if(fs::is_directory(path)){
            destDirPath /= path.filename();
        }
        fs::create_directories(destDirPath, ec);
        if(ec){
            mout->putErrorln(
                format(_("Directory \"{0}\" for \"{1}\" cannot be created in \"{2}\": {3}."),
                       toUTF8(relDirPath->string()),
                       toUTF8(path.filename().string()),
                       toUTF8(packingDirPath.string()),
                       toUTF8(ec.message())));
            return false;
        }
#if __cplusplus > 201402L
        fs::copy(
            path, destDirPath,
            fs::copy_options::overwrite_existing | fs::copy_options::recursive, ec);
#else
        if(!fs::copy_directory_recursively(path, destDirPath, ec)){
            if(!ec){
                return false;
            }
        }
#endif
        if(ec){
            mout->putErrorln(
                format(_("File \"{0}\" cannot be copied into the directory \"{1}\" for packing: {2}."),
                       toUTF8(path.string()),
                       toUTF8(destDirPath.generic_string()),
                       toUTF8(ec.message())));
            return false;
        }
    }

    // Update the project
    updatePackingItmes(topItem);

    // Save project file
    auto projectFilePath = packingDirPath / (fromUTF8(projectName) + ".cnoid");
    auto projectFile = toUTF8(projectFilePath.generic_string());
    bool saved = ProjectManager::instance()->saveProject(projectFile);

    if(!saved){
        mout->putErrorln(
            format(_("The project for packing cannot be saved as project file \"{0}\"."),
                   projectFile));
    }

    return true;
}


void ProjectPacker::Impl::checkFileDependency(Item* item)
{
    itemDependentFiles.clear();

    self->getItemDependentFiles(item, itemDependentFiles);

    for(auto file : itemDependentFiles){
        bool isAbsolute;
        fs::path ufPath = getUnifiedFormatPath(file, isAbsolute);
        if(!isAbsolute){
            mout->putErrorln(
                format(_("A relative path \"{0}\" is given to the project packer as a file path to which {1} depends"
                         " but it must be an absolute path."),
                       file, item->displayName()));
            continue;
        }
        if(!checkIfPathInReferenceDirectory(ufPath)){
            PathNode* node = &rootNode;
            for(auto& element : ufPath){
                auto key = element.string();
#ifdef _WIN32
                std::transform(key.begin(), key.end(), key.begin(), ::tolower);
#endif
                auto nextNode = &node->childNodes[key];
                nextNode->fullPath = node->fullPath / element;
                node = nextNode;
            }
            // last element is always necessary
            node->isNecessary = true;
            allPaths.emplace_back(std::move(ufPath));
        }
    }

    for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        checkFileDependency(childItem);
    }
}


bool ProjectPacker::Impl::checkIfPathInReferenceDirectory(const fs::path& path)
{
    for(auto& refDirPath : refDirPaths){
        if(checkIfSubFilePath(path, refDirPath)){
            return true;
        }
    }
    return false;
}


void ProjectPacker::Impl::updatePackingItmes(Item* item)
{
    auto packingItem = self->getPackingItem(item);
    if(!packingItem){
        item->removeFromParentItem();
    } else {
        if(packingItem != item){
            packingItem->replace(item);
        }
        ItemPtr childItem = packingItem->childItem();
        while(childItem){
            ItemPtr nextItem = childItem->nextItem();
            updatePackingItmes(childItem);
            childItem = nextItem;
        }
    }
}


std::string ProjectPacker::getRelocatedFilePath(const std::string& path)
{
    return impl->getRelocatedFilePath(path);
}


std::string ProjectPacker::Impl::getRelocatedFilePath(const std::string& pathString)
{
    bool isAbsolute;
    fs::path ufPath = getUnifiedFormatPath(pathString, isAbsolute);
    if(isAbsolute){
        if(checkIfPathInReferenceDirectory(ufPath)){
            return pathString;
        } else {
            if(auto relPath = getRelativePath(ufPath, topDirPath)){
                return toUTF8((packingDirPath / *relPath).generic_string());
            }
        }
    }
    return string();
}


void ProjectPacker::getItemDependentFiles(Item* item, std::vector<std::string>& out_files)
{
    if(!item->filePath().empty()){
        out_files.push_back(item->filePath());
    }
}


Item* ProjectPacker::getPackingItem(Item* item)
{
    Item* packingItem = nullptr;

    const auto& filePath = item->filePath();
    if(filePath.empty()){
        packingItem = item;
    } else {
        auto relocatedPath = getRelocatedFilePath(filePath);
        if(relocatedPath.empty()){
            mout_->putErrorln(
                format(_("The file path \"{0}\" for {1} cannot be relocated to be a file path in the project pack."),
                       filePath, item->displayName()));
        } else {
            if(relocatedPath != filePath){
                Mapping* options = nullptr;
                if(auto orgOptions = item->fileOptions()){
                    options = orgOptions->clone()->toMapping();
                }
                item->updateFileInformation(relocatedPath, item->fileFormat(), options);
            }
            packingItem = item;
        }
    }
    
    return packingItem;
}


bool ProjectPacker::Impl::createProjectZipFile(const string& zipFilename)
{
    fs::path zipFilePath(fromUTF8(zipFilename));

    stdx::error_code ec;
    if(fs::exists(zipFilePath)){
        fs::remove(zipFilePath, ec);
        if(ec){
            mout->putErrorln(
                format(_("The project pack file \"{0}\" already exists and cannot be removed: {1}."),
                       zipFilename, toUTF8(ec.message())));
            return false;
        }
    }
        
    int errorp;
    zip_t* zip = zip_open(toUTF8(zipFilePath.make_preferred().string()).c_str(), ZIP_CREATE, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        mout->putErrorln(
            format(_("Failed to create the project pack file \"{0}\": {1}"),
                   zipFilename, zip_error_strerror(&error)));
        return false;
    }

    fs::path zipTopDirPath(zipFilePath.stem());
    bool zipped = addDirectoryToZip(zip, packingDirPath, zipTopDirPath);
    zip_close(zip);

    if(!zipped){
        if(fs::exists(zipFilePath)){
            fs::remove(zipFilePath, ec);
        }
        mout->putErrorln(
            format(_("Failed to create the project pack file \"{0}\"."), zipFilename));
    }

    return zipped;
}


bool ProjectPacker::Impl::addDirectoryToZip(zip_t* zip, fs::path dirPath, const fs::path& zipTopDirPath)
{
    auto relDirPath = getRelativePath(dirPath, packingDirPath);
    if(!relDirPath){
        return false;
    }
    fs::path localDirPath = (zipTopDirPath / *relDirPath).lexically_normal();
    string localDirStr = toUTF8(localDirPath.generic_string());
    int index = zip_dir_add(zip, localDirStr.c_str(), ZIP_FL_ENC_UTF_8);
    if(index < 0){
        mout->putErrorln(
            format(_("Failed to add directory \"{0}\" to the project pack: {1}"),
                   localDirStr, zip_strerror(zip)));
        return false;
    }

    for(const fs::directory_entry& entry : fs::directory_iterator(dirPath)){
        auto entryPath = entry.path();
        if(fs::is_directory(entryPath)){
            if(!addDirectoryToZip(zip, entryPath, zipTopDirPath)){
                return false;
            }
        } else {
            auto localPath = zipTopDirPath / *getRelativePath(entryPath, packingDirPath);
            auto localPathStr = toUTF8(localPath.generic_string());
            auto sourcePath = toUTF8(entryPath.make_preferred().string());
            zip_source_t* source = zip_source_file(zip, sourcePath.c_str(), 0, 0);
            if(!source){
                mout->putErrorln(
                    format(_("Failed to add file \"{0}\" to the project pack: {1}"),
                           localPathStr, zip_strerror(zip)));
                return false;
            }
            int index = zip_file_add(zip, localPathStr.c_str(), source, ZIP_FL_ENC_UTF_8);
            if(index < 0){
                zip_source_free(source);
                mout->putErrorln(
                    format(_("Failed to add file \"{0}\" to the project pack: {1}"),
                           localPathStr, zip_strerror(zip)));
                return false;
            }
            // The deflate compression is applied by default. The following code is not necessary.
            /*
            if(zip_set_file_compression(zip, index, ZIP_CM_DEFLATE, 0) < 0){
                zip_source_free(source);
                mout->putErrorln(
                    format(_("Failed to compress file \"{0}\" in the project pack: {1}"),
                           localPathStr, zip_strerror(zip)));
                return false;
            }
            */
        }
    }

    return true;
}


bool ProjectPacker::unpackProject(const std::string& projectPackFile)
{
    return impl->unpackProject(projectPackFile);
}


bool ProjectPacker::Impl::unpackProject(const std::string& projectPackFile)
{
    unpackedProjectFile.clear();
    
    fs::path projectPackFilePath(fromUTF8(projectPackFile));

    int errorp;
    zip_t* zip = zip_open(projectPackFilePath.make_preferred().string().c_str(), ZIP_RDONLY, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        mout->putErrorln(
            format(_("Failed to open the project pack file \"{0}\": {1}"),
                   projectPackFile, zip_error_strerror(&error)));
        return false;
    }

    fs::path topDirPath;
    if(!unpackingDir.empty()){
        topDirPath = fromUTF8(unpackingDir);
    } else {
        topDirPath = projectPackFilePath.parent_path();
    }

    bool unpacked = extractFiles(zip, projectPackFile, projectPackFilePath, topDirPath);
    zip_close(zip);

    return unpacked;
}


bool ProjectPacker::Impl::extractFiles
(zip_t* zip, const string& zipFilename, const fs::path&  zipFilePath, const fs::path& topDirPath)
{
    fs::path projectFile(zipFilePath.stem());
    projectFile += ".cnoid";
    vector<unsigned char> buf(1024 * 1024);
    stdx::error_code ec;
    
    int numEntries = zip_get_num_entries(zip, 0);
    
    for(int i = 0; i < numEntries; ++i){
        zip_stat_t stat;
        if(zip_stat_index(zip, i, 0, &stat) < 0){
            mout->putErrorln(
                format(_("Entry {0} in \"{1}\" cannot be extracted."), i, zipFilename));
            return false;
        } else {
            string name(stat.name);
            auto entryPath = topDirPath / fromUTF8(name);
            if(name[name.size() - 1] == '/'){
                fs::create_directories(entryPath, ec);
                if(ec){
                    mout->putErrorln(
                        format(_("Directory \"{0}\" in the project pack \"{1}\" cannot be created: {2}."),
                               name, zipFilename, toUTF8(ec.message())));
                    return false;
                }
            } else {
                bool failed = false;
                zip_file_t* zf = zip_fopen_index(zip, i, 0);
                if(!zf){
                    failed = true;
                } else {
                    FILE* file = fopen(entryPath.make_preferred().string().c_str(), "wb");
                    if(!file){
                        failed = true;
                    } else {
                        long long sum = 0;
                        while(sum < stat.size){
                            int len = zip_fread(zf, buf.data(), buf.size());
                            if(len < 0){
                                failed = true;
                            } else {
                                if(fwrite(buf.data(), sizeof(unsigned char), len, file) < len){
                                    failed = true;
                                    break;
                                }
                                sum += len;
                            }
                        }
                        fclose(file);

                        if(unpackedProjectFile.empty()){
                            fs::path filePath(fromUTF8(name));
                            if(*getRelativePath(filePath, *filePath.begin()) == projectFile){
                                unpackedProjectFile = entryPath.generic_string();
                            }
                        }
                    }
                }
                if(failed){
                    mout->putErrorln(
                        format(_("File \"{0}\" in the project pack \"{1}\" cannot be extracted."),
                               name, zipFilename));
                    return false;
                }
                    
                zip_fclose(zf);
            }
        }
    }
    return true;
}


bool ProjectPacker::loadPackedProject(const std::string& projectPackFile)
{
    if(impl->unpackProject(projectPackFile)){
        if(impl->unpackedProjectFile.empty()){
            mout_->putErrorln(
                format(_("The project pack file \"{0}\" does not include a project file."), projectPackFile));
            return false;
        }
        return impl->loadUnpackedProject(impl->unpackedProjectFile);
    }
    return false;
}


bool ProjectPacker::loadUnpackedProject(const std::string& projectFile)
{
    return impl->loadUnpackedProject(projectFile);
}


//! \todo Set the path variables for the project pack
bool ProjectPacker::Impl::loadUnpackedProject(const std::string& projectFile)
{
    auto items = ProjectManager::instance()->loadProject(projectFile);
    return !items.empty();
}
