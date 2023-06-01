#ifndef CNOID_BASE_PROJECT_PACKER_H
#define CNOID_BASE_PROJECT_PACKER_H

#include <cnoid/MessageOut>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT ProjectPacker
{
public:
    ProjectPacker();
    virtual ~ProjectPacker();

    void setTopItemForPacking(Item* item);
    void addReferenceDirectory(std::string directory);
    void clearReferenceDirectories();
    void setUnpackingDirectory(const std::string& directory);
    const std::string& unpackingDirectory() const;
    bool packProjectToZipFile(const std::string& filename);
    bool packProjectToZipFile(const std::string& filename, const std::string& projectName);
    bool packProjectToDirectory(const std::string& packingDirectory);
    bool packProjectToDirectory(const std::string& packingDirectory, const std::string& projectName);
    bool loadPackedProject(const std::string& projectPackFile);
    bool unpackProject(const std::string& projectPackFile);
    bool loadUnpackedProject(const std::string& projectFile);

protected:
    MessageOut* mout() { return mout_; }
    std::string getRelocatedFilePath(const std::string& path);

    virtual void getItemDependentFiles(Item* item, std::vector<std::string>& out_files);
    virtual Item* getPackingItem(Item* item);

private:
    MessageOut* mout_;
    
    class Impl;
    Impl* impl;
};

}

#endif
