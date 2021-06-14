/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldLogFileItem.h"
#include "SimulatorItem.h"
#include "SubSimulatorItem.h"
#include "ControllerItem.h"
#include "BodyItemFileIO.h"
#include <cnoid/MainWindow>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/ProjectManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/SceneItem>
#include <cnoid/SceneItemFileIO>
#include <cnoid/FolderItem>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/PutPropertyFunction>
#include <cnoid/FileDialog>
#include <cnoid/Archive>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <QDateTime>
#include <QMessageBox>
#include <fstream>
#include <stack>
#include <map>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace {

static const int frameHeaderSize =
      sizeof(int)   // offset to the prev frame
    + sizeof(float) // time
    + sizeof(int)   // data size
    ;

enum DataTypeID {
    BODY_STATE,
    LINK_POSITIONS,
    JOINT_POSITIONS,
    DEVICE_STATES
};

struct NotEnoughDataException { };

class ReadBuf
{
public:
    vector<char> data;
    ifstream& ifs;
    int pos;

    ReadBuf(ifstream& ifs)
        : ifs(ifs) {
        pos = 0;
    }

    bool checkSize(int size){
        int left = data.size() - pos;
        if(left < size){
            int len = size - left;
            data.resize(data.size() + len);
            ifs.read(&data[pos], len);
            if(!ifs.fail()){
                return true;
            } else {
                ifs.clear();
                return false;
            }
        }
        return true;
    }

    void ensureSize(int size){
        if(!checkSize(size)){
            throw NotEnoughDataException();
        }
    }

    void seekToNextBlock(){
        int size = readSeekOffset();
        seek(pos + size);
    }

    int readNextBlockPos(){
        int size = readSeekOffset();
        return pos + size;
    }

    char* buf() {
        return &data.front();
    }

    void clear(){
        data.clear();
        pos = 0;
    }

    int size() const {
        return data.size();
    }

    char* current() {
        return &data[pos];
    }

    char* end() {
        return &data.front() + data.size();
    }

    bool isEnd() {
        return (pos >= static_cast<int>(data.size()));
    }

    void seek(int pos = 0) { this->pos = pos; }

    char readID(){
        ensureSize(1);
        return data[pos++];
    }

    bool readBool(){
        ensureSize(1);
        return data[pos++];
    }

    char readOctet(){
        ensureSize(1);
        return data[pos++];
    }

    short readShort(){
        ensureSize(2);
        unsigned char low = data[pos++];
        unsigned char high = data[pos++];
        short value = low + (high << 8);
        return value;
    }

    int readInt(){
        ensureSize(4);
        unsigned char d0 = data[pos++];
        unsigned char d1 = data[pos++];
        unsigned char d2 = data[pos++];
        unsigned char d3 = data[pos++];
        int value = d0 + (d1 << 8) + (d2 << 16) + (d3 << 24);
        return value;
    }

    int readSeekOffset(){
        return readInt();
    }

    float readFloat(){
        ensureSize(sizeof(float));
        float value;
        char* p = (char*)&value;
        const int n = sizeof(float);
        for(int i=0; i < n; ++i){
            p[i] = data[pos++];
        }
        return value;
    }

    SE3 readSE3(){
        SE3 position;
        Vector3& p = position.translation();
        p.x() = readFloat();
        p.y() = readFloat();
        p.z() = readFloat();
        Quaternion& q = position.rotation();
        q.w() = readFloat();
        q.x() = readFloat();
        q.y() = readFloat();
        q.z() = readFloat();
        return position;
    }

    std::string readString(){
        ensureSize(2);
        const int size = (unsigned int)readShort();
        ensureSize(size);
        std::string str;
        str.reserve(size);
        for(int i=0; i < size; ++i){
            str.append(1, data[pos++]);
        }
        return str;
    }
};


class WriteBuf
{
public:
    vector<char> data;
    ofstream& ofs;
    size_t seekOffset;

    WriteBuf(ofstream& ofs)
        : ofs(ofs) {
        seekOffset = 0;
    }
    
    char* buf() {
        return &data.front();
    }

    size_t pos() {
        return data.size();
    }

    size_t seekPos() {
        return seekOffset + data.size();
    }

    void clear(){
        data.clear();
        seekOffset = ofs.tellp();
    }

    int size() const {
        return data.size();
    }

    void flush(){
        ofs.write(&data.front(), data.size());
        ofs.flush();
        clear();
    }
        
    void writeID(DataTypeID id){
        writeOctet((char)id);
    }

    void writeBool(bool value){
        data.push_back(value);
    }

    void writeOctet(char value){
        data.push_back(value);
    }

    void writeShort(short value){
        data.push_back(value & 0xff);
        data.push_back(value >> 8);
    }

    void writeInt(int value){
        data.push_back(value & 0xff);
        data.push_back((value >> 8) & 0xff);
        data.push_back((value >> 16) & 0xff);
        data.push_back((value >> 24) & 0xff);
    }

    void writeInt(int pos, int value){
        data[pos++] = value & 0xff;
        data[pos++] = (value >> 8) & 0xff;
        data[pos++] = (value >> 16) & 0xff;
        data[pos++] = (value >> 24) & 0xff;
    }

    void writeSeekPos(int pos){
        writeInt(pos);
    }

    void writeSeekOffset(int offset){
        writeInt(offset);
    }

    void writeSeekOffset(int pos, int offset){
        writeInt(pos, offset);
    }
    
    void writeFloat(float value){
        char* p = (char*)&value;
        const int n = sizeof(float);
        for(int i=0; i < n; ++i){
            data.push_back(p[i]);
        }
    }

    void writeSE3(const SE3& position){
        const Vector3& p = position.translation();
        writeFloat(p.x());
        writeFloat(p.y());
        writeFloat(p.z());
        const Quaternion& q = position.rotation();
        writeFloat(q.w());
        writeFloat(q.x());
        writeFloat(q.y());
        writeFloat(q.z());
    }

    void writeString(const std::string& str){
        const int size = str.size();
        data.reserve(data.size() + size + 1);
        writeShort((unsigned char)size);
        for(int i=0; i < size; ++i){
            writeOctet(str[i]);
        }
    }
};


class DeviceInfo {
public:
    size_t lastStateSeekPos;
    vector<double> lastState;
    bool isConsistent;
    DeviceInfo() {
        lastStateSeekPos = 0;
        isConsistent = false;
    }
};


class BodyInfo : public Referenced
{
public:
    BodyItem* bodyItem;
    Body* body;
    vector<DeviceInfo> deviceInfos;
    
    BodyInfo(BodyItem* bodyItem){
        this->bodyItem = bodyItem;
        if(bodyItem){
            body = bodyItem->body();
            deviceInfos.resize(body->numDevices());
        } else {
            body = nullptr;
        }
    }
    
    DeviceInfo& deviceInfo(int deviceIndex){
        if(deviceIndex >= static_cast<int>(deviceInfos.size())){
            deviceInfos.resize(deviceIndex + 1);
        }
        return deviceInfos[deviceIndex];
    }
};
typedef ref_ptr<BodyInfo> BodyInfoPtr;


ItemList<BodyItem>::iterator findItemOfName(ItemList<BodyItem>& items, const std::string& name)
{
    for(ItemList<BodyItem>::iterator p = items.begin(); p != items.end(); ++p){
        if((*p)->name() == name){
            return p;
        }
    }
    return items.end();
}


class WorldLogFileEngine : public TimeSyncItemEngine
{
public:
    WorldLogFileItemPtr logItem;
    WorldLogFileEngine(WorldLogFileItem* item)
        : TimeSyncItemEngine(item)
    {
        logItem = item;
    }
    virtual bool onTimeChanged(double time) {
        return logItem->recallStateAtTime(time);
    }
};

typedef map<ItemPtr, ItemPtr> ItemToItemMap;

struct ArchiveInfo
{
    ItemToItemMap orgItemToArchiveItemMap;
    filesystem::path archiveDirPath;
    BodyItemBodyFileIO* bodyFileIO;
    SceneItemStdSceneFileExporter* stdSceneFileExporter;
    map<string, int> baseNameCounterMap;
};

}

namespace cnoid {

class WorldLogFileItem::Impl
{
public:
    WorldLogFileItem* self;
    QDateTime recordingStartTime;
    bool isTimeStampSuffixEnabled;
    vector<string> bodyNames;
    
    ofstream ofs;
    WriteBuf writeBuf;
    int lastOutputFramePos;
    double recordingFrameRate;
    stack<int> sizeHeaderStack;

    // for device state recording and playback
    struct DeviceStateCache : public Referenced {
        DeviceStatePtr state;
        int seekPos;
    };
    typedef ref_ptr<DeviceStateCache> DeviceStateCachePtr;
    
    vector<DeviceStateCachePtr> deviceStateCacheArrays[2];
    vector<DeviceStateCachePtr>* pLastDeviceStateCacheArray;
    vector<DeviceStateCachePtr>* pCurrentDeviceStateCacheArray;
    int deviceIndex;
    int numDeviceStateCaches;
    int currentDeviceStateCacheArrayIndex;
    vector<double> doubleWriteBuf;

    ifstream ifs;
    ReadBuf readBuf;
    ReadBuf readBuf2;
    int currentReadFramePos;
    int currentReadFrameDataSize;
    int prevReadFrameOffset;
    double currentReadFrameTime;
    bool isCurrentFrameDataLoaded;
    bool isOverRange;
        
    vector<BodyInfoPtr> bodyInfos;
    ScopedConnection worldSubTreeChangedConnection;
    bool isBodyInfoUpdateNeeded;
    
    Impl(WorldLogFileItem* self);
    Impl(WorldLogFileItem* self, Impl& org);
    ~Impl();
    bool setLogFile(const std::string& name, bool isLoading = false);
    string getActualFilename();
    void updateBodyInfos();
    void onWorldSubTreeChanged();
    bool readTopHeader();
    bool readFrameHeader(int pos);
    bool seek(double time);
    bool recallStateAtTime(double time);
    bool loadCurrentFrameData();
    void readBodyStatees();
    void readBodyState(BodyInfo* bodyInfo, double time);
    int readLinkPositions(Body* body);
    int readJointPositions(Body* body);
    void readDeviceStates(BodyInfo* bodyInfo, double time);
    void readDeviceState(DeviceInfo& devInfo, Device* device, ReadBuf& buf, int size);
    void readLastDeviceState(DeviceInfo& devInfo, Device* device);
    void clearOutput();
    void reserveSizeHeader();
    void fixSizeHeader();
    void endHeaderOutput();
    void beginFrameOutput(double time);
    void outputDeviceState(DeviceState* state);
    void exchangeDeviceStateCacheArrays();
    void openDialogToSelectDirectoryToSavePlaybackArchive();
    void saveProjectAsPlaybackArchive(const string& filename);
    int createArchiveItemMap(Item* item, ArchiveInfo& info);
    ItemPtr createArchiveModelItem(Item* modelItem, ArchiveInfo& info, bool isBodyItem);
    int replaceWithArchiveItems(Item* item, ItemToItemMap& orgItemToArchiveItemMap);
};

}


void WorldLogFileItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<WorldLogFileItem>(N_("WorldLogFileItem"));
    im.addCreationPanel<WorldLogFileItem>();
    im.addLoader<WorldLogFileItem>(
        _("World Log"), "CNOID-WORLD-LOG", "log",
        [](WorldLogFileItem* item, const std::string& filename, std::ostream&, Item*){
            return item->impl->setLogFile(filename, true);
        });

    ItemTreeView::instance()->customizeContextMenu<WorldLogFileItem>(
        [](WorldLogFileItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.setPath("/");
            menuManager.addItem(_("Save project as log playback archive"))->sigTriggered().connect(
                [item](){ item->impl->openDialogToSelectDirectoryToSavePlaybackArchive(); });
            menuManager.setPath("/");
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });

    TimeSyncItemEngineManager::instance()
        ->registerFactory<WorldLogFileItem, WorldLogFileEngine>(
            [](WorldLogFileItem* item, WorldLogFileEngine* engine0){
                return engine0 ? engine0 : new WorldLogFileEngine(item);
            });
}


WorldLogFileItem::WorldLogFileItem()
{
    impl = new Impl(this);
    setName("WorldLogFile");
}


WorldLogFileItem::Impl::Impl(WorldLogFileItem* self)
    : self(self),
      writeBuf(ofs),
      readBuf(ifs),
      readBuf2(ifs)
{
    isTimeStampSuffixEnabled = false;
    recordingFrameRate = 0.0;
    isBodyInfoUpdateNeeded = true;
}


WorldLogFileItem::WorldLogFileItem(const WorldLogFileItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


WorldLogFileItem::Impl::Impl(WorldLogFileItem* self, Impl& org)
    : self(self),
      writeBuf(ofs),
      readBuf(ifs),
      readBuf2(ifs)
{
    isTimeStampSuffixEnabled = org.isTimeStampSuffixEnabled;
    recordingFrameRate = org.recordingFrameRate;
    isBodyInfoUpdateNeeded = true;
}


WorldLogFileItem::~WorldLogFileItem()
{
    delete impl;
}


WorldLogFileItem::Impl::~Impl()
{

}


Item* WorldLogFileItem::doDuplicate() const
{
    return new WorldLogFileItem(*this);
}


void WorldLogFileItem::notifyUpdate()
{
    impl->isBodyInfoUpdateNeeded = true;
    Item::notifyUpdate();
}


const std::string& WorldLogFileItem::logFile() const
{
    return filePath();
}


bool WorldLogFileItem::setLogFile(const std::string& filename)
{
    return impl->setLogFile(filename);
}


bool WorldLogFileItem::Impl::setLogFile(const std::string& filename, bool isLoading)
{
    self->updateFileInformation(filename, "CNOID-WORLD-LOG");
    bool loaded = readTopHeader();
    return isLoading ? loaded : true;
}


void WorldLogFileItem::setTimeStampSuffixEnabled(bool on)
{
    impl->isTimeStampSuffixEnabled = on;
}


bool WorldLogFileItem::isTimeStampSuffixEnabled() const
{
    return impl->isTimeStampSuffixEnabled;
}


string WorldLogFileItem::Impl::getActualFilename()
{
    if(isTimeStampSuffixEnabled && recordingStartTime.isValid()){
        filesystem::path filepath(fromUTF8(self->filePath()));
        string suffix = recordingStartTime.toString("-yyyy-MM-dd-hh-mm-ss").toStdString();
        string fname = filepath.stem().string() + suffix;
        fname += filepath.extension().string();
        return toUTF8((filepath.parent_path() / fname).generic_string());
    } else {
        return self->filePath();
    }
}


void WorldLogFileItem::setRecordingFrameRate(double rate)
{
    impl->recordingFrameRate = rate;
}


double WorldLogFileItem::recordingFrameRate() const
{
    return impl->recordingFrameRate;
}


void WorldLogFileItem::Impl::updateBodyInfos()
{
    bodyInfos.clear();
    
    if(!bodyNames.empty()){
        if(auto worldItem = self->findOwnerItem<WorldItem>()){
            auto items = worldItem->descendantItems<BodyItem>();
            if(!items.empty()){
                for(size_t i=0; i < bodyNames.size(); ++i){
                    ItemList<BodyItem>::iterator p = findItemOfName(items, bodyNames[i]);
                    if(p != items.end()){
                        bodyInfos.push_back(new BodyInfo(*p));
                        items.erase(p);
                    } else {
                        bodyInfos.push_back(0);
                    }
                }
            }
        }
    }

    isBodyInfoUpdateNeeded = false;
}


void WorldLogFileItem::onTreePathChanged()
{
    WorldItem* worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        impl->worldSubTreeChangedConnection.disconnect();
    } else {
        impl->worldSubTreeChangedConnection.reset(
            worldItem->sigSubTreeChanged().connect(
                [&](){ impl->onWorldSubTreeChanged(); }));
    }
    
    impl->isBodyInfoUpdateNeeded = true;
}


void WorldLogFileItem::Impl::onWorldSubTreeChanged()
{
    isBodyInfoUpdateNeeded = true;
}


bool WorldLogFileItem::Impl::readTopHeader()
{
    bool result = false;
    
    bodyNames.clear();

    currentReadFramePos = 0;
    currentReadFrameDataSize = 0;
    prevReadFrameOffset = 0;
    currentReadFrameTime = -1.0;
    
    if(ifs.is_open()){
        ifs.close();
    }
    string fname = fromUTF8(getActualFilename());
    if(filesystem::exists(fname)){
        ifs.open(fname.c_str(), ios::in | ios::binary);
        if(ifs.is_open()){
            readBuf.clear();
            try {
                int headerSize = readBuf.readSeekOffset();
                if(readBuf.checkSize(headerSize)){
                    while(!readBuf.isEnd()){
                        bodyNames.push_back(readBuf.readString());
                    }
                    currentReadFramePos = readBuf.pos;
                    result = readFrameHeader(readBuf.pos);
                }
            } catch(NotEnoughDataException& ex){
                bodyNames.clear();
            }
        }
    }

    isBodyInfoUpdateNeeded = true;

    return result;
}


bool WorldLogFileItem::Impl::readFrameHeader(int pos)
{
    isCurrentFrameDataLoaded = false;
    
    if(!ifs.is_open()){
        return false;
    }

    ifs.seekg(pos);

    if(ifs.eof()){
        ifs.seekg(currentReadFramePos);
        return false;
    }

    readBuf.clear();
    if(!readBuf.checkSize(frameHeaderSize)){
        ifs.seekg(currentReadFramePos);
        return false;
    }
    
    currentReadFramePos = pos;
    prevReadFrameOffset = readBuf.readSeekOffset();
    currentReadFrameTime = readBuf.readFloat();
    currentReadFrameDataSize = readBuf.readSeekOffset();

    return true;
}
        
        
bool WorldLogFileItem::Impl::seek(double time)
{
    isOverRange = false;

    if(!readFrameHeader(currentReadFramePos)){
        readTopHeader();
    }
    
    if(currentReadFrameTime == time){
        return true;
    }

    if(currentReadFrameTime < time){
        while(true){
            int pos = currentReadFramePos;
            if(!readFrameHeader(currentReadFramePos + frameHeaderSize + currentReadFrameDataSize)){
                isOverRange = true;
                return (currentReadFrameTime >= 0.0);
            }
            if(currentReadFrameTime == time){
                return true;
            } else if(currentReadFrameTime > time){
                return readFrameHeader(pos);
            }
        }
    }

    // currentReadFrameTime > time
    while(true){
        if(prevReadFrameOffset <= 0){
            isOverRange = true;
            return (currentReadFrameTime >= 0.0);
        }
        if(!readFrameHeader(currentReadFramePos - prevReadFrameOffset)){
            return false;
        }
        if(currentReadFrameTime <= time){
            return true;
        }
    }
}


bool WorldLogFileItem::Impl::loadCurrentFrameData()
{
    ifs.seekg(currentReadFramePos + frameHeaderSize);
    readBuf.clear();
    isCurrentFrameDataLoaded = readBuf.checkSize(currentReadFrameDataSize);
    return isCurrentFrameDataLoaded;
}


/**
   @return True if the time is within the data range and the frame is correctly recalled.
   False if the time is outside the data range or the frame cannot be recalled.
*/
bool WorldLogFileItem::recallStateAtTime(double time)
{
    return impl->recallStateAtTime(time);
}


bool WorldLogFileItem::Impl::recallStateAtTime(double time)
{
    if(!seek(time)){
        return false;
    }

    if(!isCurrentFrameDataLoaded){
        if(!loadCurrentFrameData()){
            return false;
        }
    }
    readBuf.seek(0);

    if(isBodyInfoUpdateNeeded){
        updateBodyInfos();
    }
    
    int bodyIndex = 0;
    while(!readBuf.isEnd()){
        int dataTypeID = readBuf.readID();
        switch(dataTypeID){
        case BODY_STATE:
        {
            BodyInfo* bodyInfo = nullptr;
            if(bodyIndex < static_cast<int>(bodyInfos.size())){
                bodyInfo = bodyInfos[bodyIndex];
            }
            if(bodyInfo){
                readBodyState(bodyInfo, time);
            } else {
                readBuf.seekToNextBlock();
            }
            ++bodyIndex;
            break;
        }

        default:
            readBuf.seekToNextBlock();
        }
    }

    return !isOverRange;
}


void WorldLogFileItem::Impl::readBodyState(BodyInfo* bodyInfo, double time)
{
    int endPos = readBuf.readNextBlockPos();
    bool updated = false;
    bool doForwardKinematics = true;
    int numLinks;
    
    while(readBuf.pos < endPos){
        int dataType = readBuf.readID();
        switch(dataType){
        case LINK_POSITIONS:
            numLinks = readLinkPositions(bodyInfo->body);
            if(numLinks > 0){
                updated = true;
                if(numLinks > 1){
                    doForwardKinematics = false;
                }
            }
            break;
        case JOINT_POSITIONS:
            if(readJointPositions(bodyInfo->body)){
                updated = true;
            }
            break;
        case DEVICE_STATES:
            if(updated){
                bodyInfo->bodyItem->notifyKinematicStateChange(doForwardKinematics);
                updated = false;
            }
            readDeviceStates(bodyInfo, time);
            break;
        default:
            readBuf.seekToNextBlock();
            break;
        }
    }
    if(updated){
        bodyInfo->bodyItem->notifyKinematicStateChange(doForwardKinematics);
    }
}


int WorldLogFileItem::Impl::readLinkPositions(Body* body)
{
    int endPos = readBuf.readNextBlockPos();
    int size = readBuf.readShort();
    int n = std::min(size, body->numLinks());
    for(int i=0; i < n; ++i){
        SE3 position = readBuf.readSE3();
        Link* link = body->link(i);
        link->p() = position.translation();
        link->R() = position.rotation().toRotationMatrix();
    }
    readBuf.seek(endPos);
    return n;
}


int WorldLogFileItem::Impl::readJointPositions(Body* body)
{
    int endPos = readBuf.readNextBlockPos();
    int size = readBuf.readShort();
    int n = std::min(size, body->numAllJoints());
    for(int i=0; i < n; ++i){
        body->joint(i)->q() = readBuf.readFloat();
    }
    readBuf.seek(endPos);
    return n;
}


void WorldLogFileItem::Impl::readDeviceStates(BodyInfo* bodyInfo, double time)
{
    const int endPos = readBuf.readNextBlockPos();
    Body* body = bodyInfo->body;
    const int numDevices = body->numDevices();
    int deviceIndex = 0;
    while(readBuf.pos < endPos && deviceIndex < numDevices){
        DeviceInfo& devInfo = bodyInfo->deviceInfo(deviceIndex);
        Device* device = bodyInfo->body->device(deviceIndex);
        const int header = readBuf.readShort();
        if(header < 0){
            readLastDeviceState(devInfo, device);
        } else {
            const int size = header;
            int nextPos = readBuf.pos + sizeof(float) * size;
            readDeviceState(devInfo, device, readBuf, size);
            readBuf.seek(nextPos);
        }
        device->notifyTimeChange(time);
        ++deviceIndex;
    }
    readBuf.seek(endPos);
}


void WorldLogFileItem::Impl::readDeviceState(DeviceInfo& devInfo, Device* device, ReadBuf& buf, int size)
{
    const int stateSize = device->stateSize();
    if(stateSize <= size){
        vector<double>& state = devInfo.lastState;
        state.resize(stateSize);
        for(int i=0; i < stateSize; ++i){
            state[i] = buf.readFloat();
        }
        device->readState(&state.front());
        device->notifyStateChange();
        devInfo.isConsistent = true;
    }
}


void WorldLogFileItem::Impl::readLastDeviceState(DeviceInfo& devInfo, Device* device)
{
    size_t pos = readBuf.readSeekOffset();
    if(pos == devInfo.lastStateSeekPos){
        if(!devInfo.isConsistent){
            device->readState(&devInfo.lastState.front());
            device->notifyStateChange();
            devInfo.isConsistent = true;
        }
    } else {
        ifs.seekg(pos);
        devInfo.lastStateSeekPos = pos;
        readBuf2.clear();
        int size = readBuf2.readShort();
        if(size > 0){
            readDeviceState(devInfo, device, readBuf2, size);
        }
    }
}


void WorldLogFileItem::invalidateLastStateConsistency()
{
    vector<BodyInfoPtr>& bodyInfos = impl->bodyInfos;
    for(size_t i=0; i < bodyInfos.size(); ++i){
        vector<DeviceInfo>& devInfos = bodyInfos[i]->deviceInfos;
        for(size_t j=0; j < devInfos.size(); ++j){
            devInfos[j].isConsistent = false;
        }
    }
}


void WorldLogFileItem::clearOutput()
{
    impl->clearOutput();
}


void WorldLogFileItem::Impl::clearOutput()
{
    bodyNames.clear();

    if(ifs.is_open()){
        ifs.close();
    }
    if(ofs.is_open()){
        ofs.close();
    }
    recordingStartTime = QDateTime::currentDateTime();
    
    ofs.open(fromUTF8(getActualFilename()).c_str(), ios::out | ios::binary | ios::trunc);
    writeBuf.clear();
    lastOutputFramePos = 0;

    currentDeviceStateCacheArrayIndex = 0;
    exchangeDeviceStateCacheArrays();
}


void WorldLogFileItem::Impl::reserveSizeHeader()
{
    sizeHeaderStack.push(writeBuf.size());
    writeBuf.writeSeekOffset(0);
}


void WorldLogFileItem::Impl::fixSizeHeader()
{
    if(!sizeHeaderStack.empty()){
        writeBuf.writeSeekOffset(sizeHeaderStack.top(), writeBuf.size() - (sizeHeaderStack.top() + sizeof(int)));
        sizeHeaderStack.pop();
    }
}


void WorldLogFileItem::beginHeaderOutput()
{
    impl->writeBuf.clear();
    impl->reserveSizeHeader();
}


int WorldLogFileItem::outputBodyHeader(const std::string& name)
{
    int index = impl->bodyNames.size();
    impl->bodyNames.push_back(name);
    impl->writeBuf.writeString(name);
    return index;
}


void WorldLogFileItem::endHeaderOutput()
{
    impl->endHeaderOutput();
}


void WorldLogFileItem::Impl::endHeaderOutput()
{
    fixSizeHeader();
    writeBuf.flush();
}


int WorldLogFileItem::numBodies() const
{
    return impl->bodyNames.size();
}


const std::string& WorldLogFileItem::bodyName(int bodyIndex) const
{
    return impl->bodyNames[bodyIndex];
}


void WorldLogFileItem::beginFrameOutput(double time)
{
    impl->beginFrameOutput(time);
}


void WorldLogFileItem::Impl::beginFrameOutput(double time)
{
    size_t pos = writeBuf.seekPos();
    
    if(lastOutputFramePos){
        writeBuf.writeSeekOffset(pos - lastOutputFramePos);
    } else {
        writeBuf.writeSeekOffset(0);
    }
    lastOutputFramePos = pos;
    
    deviceIndex = 0;
    writeBuf.writeFloat(time);
    reserveSizeHeader(); // area for the frame data size
}


void WorldLogFileItem::beginBodyStateOutput()
{
    impl->writeBuf.writeID(BODY_STATE);
    impl->reserveSizeHeader();
}


void WorldLogFileItem::outputLinkPositions(SE3* positions, int size)
{
    impl->writeBuf.writeID(LINK_POSITIONS);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(size);
    for(int i=0; i < size; ++i){
        impl->writeBuf.writeSE3(positions[i]);
    }
    impl->fixSizeHeader();
}


void WorldLogFileItem::outputJointPositions(double* values, int size)
{
    impl->writeBuf.writeID(JOINT_POSITIONS);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(size);
    for(int i=0; i < size; ++i){
        impl->writeBuf.writeFloat(values[i]);
    }
    impl->fixSizeHeader();
}


void WorldLogFileItem::beginDeviceStateOutput()
{
    impl->writeBuf.writeID(DEVICE_STATES);
    impl->reserveSizeHeader();
}


void WorldLogFileItem::outputDeviceState(DeviceState* state)
{
    impl->outputDeviceState(state);
}


void WorldLogFileItem::Impl::outputDeviceState(DeviceState* state)
{
    DeviceStateCache* cache = nullptr;
        
    if(deviceIndex >= numDeviceStateCaches){
        cache = new DeviceStateCache;
    } else {
        cache = (*pLastDeviceStateCacheArray)[deviceIndex];
        if(state == cache->state){
            writeBuf.writeShort(-1);
            writeBuf.writeSeekOffset(cache->seekPos);
            goto endOutputDeviceState;
        }
    }
    cache->state = state;
    cache->seekPos = writeBuf.seekPos();
    if(!state){
        writeBuf.writeShort(0);
    } else {
        int size = state->stateSize();
        writeBuf.writeShort(size);
        doubleWriteBuf.resize(size);
        state->writeState(&doubleWriteBuf.front());
        for(int i=0; i < size; ++i){
            writeBuf.writeFloat(doubleWriteBuf[i]);
        }
    }
endOutputDeviceState:

    pCurrentDeviceStateCacheArray->push_back(cache);
    ++deviceIndex;
}


void WorldLogFileItem::endDeviceStateOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endBodyStateOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endFrameOutput()
{
    impl->fixSizeHeader();
    impl->writeBuf.flush();
    impl->exchangeDeviceStateCacheArrays();
}


void WorldLogFileItem::Impl::exchangeDeviceStateCacheArrays()
{
    int i = 1 - currentDeviceStateCacheArrayIndex;
    pCurrentDeviceStateCacheArray = &deviceStateCacheArrays[i];
    pLastDeviceStateCacheArray = &deviceStateCacheArrays[1-i];
    numDeviceStateCaches = pLastDeviceStateCacheArray->size();
    currentDeviceStateCacheArrayIndex = i;
}
    

void WorldLogFileItem::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty logFileProperty(filePath(), { string(_("World Log File (*.log)")) });
    logFileProperty.setExistingFileMode(false);
    putProperty(_("Log file"), logFileProperty,
                [&](const string& file){ return impl->setLogFile(file); });
    putProperty(_("Actual log file"), FilePathProperty(impl->getActualFilename()));
    putProperty(_("Time-stamp suffix"), impl->isTimeStampSuffixEnabled,
                changeProperty(impl->isTimeStampSuffixEnabled));
    putProperty(_("Recording frame rate"), impl->recordingFrameRate,
                changeProperty(impl->recordingFrameRate));
}


bool WorldLogFileItem::store(Archive& archive)
{
    archive.writeFileInformation(this);
    archive.write("timeStampSuffix", impl->isTimeStampSuffixEnabled);
    archive.write("recordingFrameRate", impl->recordingFrameRate);
    return true;
}


bool WorldLogFileItem::restore(const Archive& archive)
{
    archive.read("timeStampSuffix", impl->isTimeStampSuffixEnabled);
    archive.read("recordingFrameRate", impl->recordingFrameRate);

    std::string filename;
    if(archive.read({ "file", "filename" }, filename)){
        impl->setLogFile(archive.resolveRelocatablePath(filename));
    }
    return true;
}


void WorldLogFileItem::Impl::openDialogToSelectDirectoryToSavePlaybackArchive()
{
    FileDialog dialog;
    dialog.setWindowTitle(_("Save project as log playback archive"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.updatePresetDirectories();

    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    auto logfile = toUTF8(filesystem::path(fromUTF8(getActualFilename())).stem().generic_string());
    if(!logfile.empty()){
        dialog.selectFile(QString("%1-log.cnoid").arg(logfile.c_str()));
    }
        
    if(dialog.exec()){
        auto filenames = dialog.selectedFiles();
        string filename(filenames.at(0).toStdString());
        if(!filename.empty()){
            QString message(_("The current project will be replaced with a new project for the log playback. "
                              "Do you want to continue?"));
            auto button = QMessageBox::warning(
                MainWindow::instance(), _("Project replacement"), message, QMessageBox::Ok | QMessageBox::Cancel);
            if(button == QMessageBox::Ok){
                saveProjectAsPlaybackArchive(filename);
            }
        }
    }
}


void WorldLogFileItem::Impl::saveProjectAsPlaybackArchive(const string& projectFile)
{
    auto worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem){
        showWarningDialog(format(_("The world item of {0} is not found."), self->displayName()));
        return;
    }

    ArchiveInfo info;

    info.bodyFileIO = dynamic_cast<BodyItemBodyFileIO*>(BodyItem::bodyFileIO());
    if(!info.bodyFileIO){
        return;
    }
    info.stdSceneFileExporter =
        dynamic_cast<SceneItemStdSceneFileExporter*>(SceneItem::stdSceneFileExporter());
    if(!info.stdSceneFileExporter){
        return;
    }

    stdx::error_code ec;

    auto logFilePath = filesystem::absolute(fromUTF8(getActualFilename()), ec);
    if(!filesystem::exists(logFilePath ,ec)){
        showWarningDialog(format(_("Log file of {0} does not exist."), self->displayName()));
        return;
    }

    auto mv = MessageView::instance();
    mv->putln(_("Creating the project for the log playback ..."));
    mv->flush();
    
    auto projectFilePath = filesystem::absolute(fromUTF8(projectFile), ec);
    if(ec){
        return;
    }
    if(!projectFilePath.has_extension()){
        projectFilePath += ".cnoid";
    }
        
    info.archiveDirPath = projectFilePath.parent_path() / projectFilePath.stem();
    filesystem::create_directories(info.archiveDirPath, ec);
    if(ec){
        showWarningDialog(format(_("Archive directory \"{0}\" cannot be created: {1}"),
                                 info.archiveDirPath.string(), ec.message()));
        return;
    }

    info.bodyFileIO->bodyWriter()->setExtModelFileMode(StdBodyWriter::ReplaceWithObjModelFiles);
    
    if(createArchiveItemMap(worldItem, info) > 0){

        filesystem::copy_file(
            logFilePath, info.archiveDirPath / logFilePath.filename(),
#if __cplusplus > 201402L            
            filesystem::copy_options::overwrite_existing,
#else
            filesystem::copy_option::overwrite_if_exists,
#endif
            ec);
        if(ec){
            showWarningDialog(
                format(_("Log file \"{0}\" cannot be copied to \"{1}\": {2}"),
                       toUTF8(logFilePath.filename().string()),
                       toUTF8(info.archiveDirPath.string()),
                       ec.message()));
            return;
        }
        setLogFile(toUTF8((info.archiveDirPath / logFilePath.filename()).generic_string()));
        isTimeStampSuffixEnabled = false;

        auto rootItem = RootItem::instance();
        info.orgItemToArchiveItemMap[rootItem] = rootItem;
        info.orgItemToArchiveItemMap[worldItem] = worldItem;
        info.orgItemToArchiveItemMap[self] = self;

        if(replaceWithArchiveItems(rootItem, info.orgItemToArchiveItemMap)){
            TimeBar::instance()->setTime(0.0);
            self->setSelected(true);
            ProjectManager::instance()->saveProject(toUTF8(projectFilePath.string()));
        }
    }
}


int WorldLogFileItem::Impl::createArchiveItemMap(Item* item, ArchiveInfo& info)
{
    int numModelItems = 0;
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        ItemPtr archiveChildItem;
        bool isBodyItem = false;
        ItemPtr modelItem = dynamic_cast<BodyItem*>(childItem);
        if(modelItem){
            isBodyItem = true;
        } else {
            modelItem = dynamic_cast<SceneItem*>(childItem);
        }
        if(modelItem){
            auto archiveModelItem = createArchiveModelItem(modelItem, info, isBodyItem);
            if(!archiveModelItem){
                return -1;
            }
            ++numModelItems;
            archiveChildItem = archiveModelItem;
        }
        int numSubTreeModelItems = createArchiveItemMap(childItem, info);
        if(numSubTreeModelItems < 0){ // error
            return -1;
        }
        numModelItems += numSubTreeModelItems;
        if(archiveChildItem){
            info.orgItemToArchiveItemMap[childItem] = archiveChildItem;
        }
    }
    return numModelItems;
}


ItemPtr WorldLogFileItem::Impl::createArchiveModelItem(Item* modelItem, ArchiveInfo& info, bool isBodyItem)
{
    ItemPtr archiveModelItem;
    
    // Replace space and symbol characters in the filename string
    string baseName = regex_replace(modelItem->name(), regex("\\s"), "_");
    baseName = regex_replace(baseName, regex("[\\W]"), "_");

    // Check and avoid the duplication of the model directory names
    auto emplaced = info.baseNameCounterMap.emplace(baseName, 1);
    if(baseName.empty() || !emplaced.second){
        auto& counter = emplaced.first->second;
        if(!baseName.empty()){
            ++counter;
        }
        string baseNameWithId;
        while(true){
            baseNameWithId = format("{0}-{1}", baseName, counter++);
            if(info.baseNameCounterMap.find(baseNameWithId) == info.baseNameCounterMap.end()){
                baseName = baseNameWithId;
                break;
            }
        }
    }

    string nativeBaseName = fromUTF8(baseName);
    filesystem::path modelDirPath = info.archiveDirPath / nativeBaseName;
    stdx::error_code ec;

    filesystem::create_directories(modelDirPath, ec);
    if(ec){
        MessageView::instance()->putln(
            format(_("Directory \"{0}\" cannot be created: {1}."),
                   toUTF8(modelDirPath.filename().string()), ec.message()),
            MessageView::Error);
    } else {
        archiveModelItem = modelItem->duplicate();
        auto filePath = modelDirPath / nativeBaseName;
        bool saved = false;
        if(isBodyItem){
            filePath += ".body"; // Is this necessary?
            saved = info.bodyFileIO->saveItem(archiveModelItem, toUTF8(filePath.string()));
        } else {
            filePath += ".scen"; // Is this necessary?
            saved = info.stdSceneFileExporter->saveItem(archiveModelItem, toUTF8(filePath.string()));
        }
        if(!saved){
            archiveModelItem.reset();
        }
    }
    return archiveModelItem;
}


int WorldLogFileItem::Impl::replaceWithArchiveItems(Item* item, ItemToItemMap& orgItemToArchiveItemMap)
{
    int numValidItems = 0;
    bool keptOrReplaced = false;
    ItemPtr archiveItem;
    
    auto p = orgItemToArchiveItemMap.find(item);

    if(p != orgItemToArchiveItemMap.end()){
        archiveItem = p->second;
        if(archiveItem != item){
            if(archiveItem->replace(item)){
                item = archiveItem;
            }
        }
    }
    
    // Child items must be preserved in advance because each item may be replaced 
    vector<ItemPtr> childItems;
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        childItems.push_back(childItem);
    }
    for(auto& childItem : childItems){
        numValidItems += replaceWithArchiveItems(childItem, orgItemToArchiveItemMap);
    }

    if(!archiveItem){
        if(!item->isTemporal() &&
           !item->isSubItem() &&
           item->filePath().empty() &&
           !dynamic_cast<SimulatorItem*>(item) &&
           !dynamic_cast<SubSimulatorItem*>(item) &&
           !dynamic_cast<ControllerItem*>(item))
        {
            archiveItem = item;
        }
    }
    if(archiveItem){
        ++numValidItems;
    }

    if(numValidItems == 0){
        item->removeFromParentItem();

    } else if(!archiveItem){
        archiveItem = new FolderItem;
        archiveItem->setName(item->displayName());
        archiveItem->replace(item);
    }

    item->setSelected(false);

    return numValidItems;
}
