/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldLogFileItem.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/FileUtil>
#include <cnoid/Archive>
#include <QDateTime>
#include <boost/bind.hpp>
#include <fstream>
#include <stack>

#include <iostream>

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

static const int frameHeaderSize =
      sizeof(int)   // offset to the prev frame
    + sizeof(float) // time
    + sizeof(int)   // data size
    ;

enum DataTypeID {
    BODY_STATUS,
    LINK_POSITIONS,
    JOINT_POSITIONS,
    DEVICE_STATUSES
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
        return (pos >= data.size());
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
        Quat& q = position.rotation();
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

    char* buf() {
        return &data.front();
    }

    void clear(){
        data.clear();
    }

    int size() const {
        return data.size();
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
        const Quat& q = position.rotation();
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
    WorldLogFileEngine(WorldLogFileItem* item){
        logItem = item;
    }
    virtual bool onTimeChanged(double time) {
        return logItem->recallStatusAtTime(time);
    }
};


TimeSyncItemEngine* createWorldLogFileEngine(Item* sourceItem)
{
    if(WorldLogFileItem* logItem = dynamic_cast<WorldLogFileItem*>(sourceItem)){
        return new WorldLogFileEngine(logItem);
    }
    return 0;
}


bool loadWorldLogFile(WorldLogFileItem* item, const std::string& filename, std::ostream& os)
{
    return item->setLogFileName(filename);
}

}

namespace cnoid {

class WorldLogFileItemImpl
{
public:
    WorldLogFileItem* self;
    string filename;
    QDateTime recordingStartTime;
    bool isTimeStampSuffixEnabled;
    vector<string> bodyNames;
    ItemList<BodyItem> bodyItems;
    
    ofstream ofs;
    WriteBuf writeBuf;
    int lastOutputFramePos;
    vector<double> doubleReadBuf;
    vector<double> doubleWriteBuf;
    stack<int> sizeHeaderStack;
    double recordingFrameRate;

    ifstream ifs;
    ReadBuf readBuf;
    int currentReadFramePos;
    int currentReadFrameDataSize;
    int prevReadFrameOffset;
    double currentReadFrameTime;
    bool isCurrentFrameDataLoaded;
    bool isOverRange;
        
    WorldLogFileItemImpl(WorldLogFileItem* self);
    WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org);
    ~WorldLogFileItemImpl();
    bool setLogFileName(const std::string& name);
    string getActualFilename();
    void updateBodyItems();
    bool readTopHeader();
    bool readFrameHeader(int pos);
    bool seek(double time);
    bool recallStatusAtTime(double time);
    bool loadCurrentFrameData();
    void readBodyStatuses();
    void readBodyStatus(BodyItem* bodyItem);
    int readLinkPositions(Body* body);
    int readJointPositions(Body* body);
    void readDeviceStatuses(Body* body);
    void clearOutput();
    void reserveSizeHeader();
    void fixSizeHeader();
    void flushWriteBuf();
    void endHeaderOutput();
    void beginFrameOutput(double time);
};

}


void WorldLogFileItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<WorldLogFileItem>(N_("WorldLogFileItem"));
    im.addCreationPanel<WorldLogFileItem>();
    im.addLoader<WorldLogFileItem>(
        _("World Log"), "CNOID-WORLD-LOG", "log", boost::bind(loadWorldLogFile, _1, _2, _3));

    ext->timeSyncItemEngineManger().addEngineFactory(createWorldLogFileEngine);    
}


WorldLogFileItem::WorldLogFileItem()
{
    impl = new WorldLogFileItemImpl(this);
}


WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self)
    : self(self),
      readBuf(ifs)
{
    isTimeStampSuffixEnabled = false;
    recordingFrameRate = 0.0;
}


WorldLogFileItem::WorldLogFileItem(const WorldLogFileItem& org)
    : Item(org)
{
    impl = new WorldLogFileItemImpl(this, *org.impl);
}


WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org)
    : self(self),
      readBuf(ifs)
{
    filename = org.filename;
    isTimeStampSuffixEnabled = org.isTimeStampSuffixEnabled;
    recordingFrameRate = org.recordingFrameRate;
}


WorldLogFileItem::~WorldLogFileItem()
{
    delete impl;
}


WorldLogFileItemImpl::~WorldLogFileItemImpl()
{

}


ItemPtr WorldLogFileItem::doDuplicate() const
{
    return new WorldLogFileItem(*this);
}


void WorldLogFileItem::notifyUpdate()
{
    impl->updateBodyItems();
    Item::notifyUpdate();
}


const std::string& WorldLogFileItem::logFileName() const
{
    return impl->filename;
}


bool WorldLogFileItem::setLogFileName(const std::string& filename)
{
    return impl->setLogFileName(filename);
}


bool WorldLogFileItemImpl::setLogFileName(const std::string& name)
{
    if(name != filename){
        filename = name;
        readTopHeader();
    }
    return true;
}


string WorldLogFileItemImpl::getActualFilename()
{
    if(isTimeStampSuffixEnabled && recordingStartTime.isValid()){
        filesystem::path filepath(filename);
        string suffix = recordingStartTime.toString("-yyyy-MM-dd-hh-mm-ss").toStdString();
        string fname = getBasename(filepath) + suffix;
        string ext = getExtension(filepath);
        if(!ext.empty()){
            fname = fname + "." + ext;
        }
        return getPathString(filepath.parent_path() / filesystem::path(fname));
    } else {
        return filename;
    }
}


double WorldLogFileItem::recordingFrameRate() const
{
    return impl->recordingFrameRate;
}


void WorldLogFileItemImpl::updateBodyItems()
{
    bodyItems.clear();
    if(!bodyNames.empty()){
        WorldItem* worldItem = self->findOwnerItem<WorldItem>();
        if(worldItem){
            ItemList<BodyItem> items;
            if(items.extractChildItems(worldItem)){
                for(size_t i=0; i < bodyNames.size(); ++i){
                    ItemList<BodyItem>::iterator p = findItemOfName(items, bodyNames[i]);
                    if(p != items.end()){
                        bodyItems.push_back(*p);
                        items.erase(p);
                    } else {
                        bodyItems.push_back(0);
                    }
                }
            }
        }
    }
}


void WorldLogFileItem::onPositionChanged()
{
    impl->updateBodyItems();
}


bool WorldLogFileItemImpl::readTopHeader()
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
    string fname = getActualFilename();
    if(filesystem::exists(fname)){
        ifs.open(fname.c_str(), ios::in | ios::binary);
        if(ifs.is_open()){
            readBuf.clear();
            try {
                int headerSize = readBuf.readSeekOffset();
                if(readBuf.checkSize(headerSize)){
                    while(!readBuf.isEnd()){
                        bodyNames.push_back(readBuf.readString());
                        cout << bodyNames.back() << endl;
                    }
                    currentReadFramePos = readBuf.pos;
                    result = readFrameHeader(readBuf.pos);
                }
            } catch(NotEnoughDataException& ex){
                bodyNames.clear();
            }
        }
    }

    updateBodyItems();

    return result;
}


bool WorldLogFileItemImpl::readFrameHeader(int pos)
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
        
        
bool WorldLogFileItemImpl::seek(double time)
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


bool WorldLogFileItemImpl::loadCurrentFrameData()
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
bool WorldLogFileItem::recallStatusAtTime(double time)
{
    return impl->recallStatusAtTime(time);
}


bool WorldLogFileItemImpl::recallStatusAtTime(double time)
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

    int bodyIndex = 0;
    while(!readBuf.isEnd()){
        int dataTypeID = readBuf.readID();
        switch(dataTypeID){
        case BODY_STATUS:
        {
            BodyItem* bodyItem = 0;
            if(bodyIndex < bodyItems.size()){
                bodyItem = bodyItems[bodyIndex];
            }
            if(bodyItem){
                readBodyStatus(bodyItem);
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


void WorldLogFileItemImpl::readBodyStatus(BodyItem* bodyItem)
{
    int endPos = readBuf.readNextBlockPos();
    Body* body = bodyItem->body();
    bool updated = false;
    bool doForwardKinematics = true;
    int numLinks;
    
    while(readBuf.pos < endPos){
        int dataType = readBuf.readID();
        switch(dataType){
        case LINK_POSITIONS:
            numLinks = readLinkPositions(body);
            if(numLinks > 0){
                updated = true;
                if(numLinks > 1){
                    doForwardKinematics = false;
                }
            }
            break;
        case JOINT_POSITIONS:
            if(readJointPositions(body)){
                updated = true;
            }
            break;
        case DEVICE_STATUSES:
            if(updated){
                bodyItem->notifyKinematicStateChange(doForwardKinematics);
                updated = false;
            }
            readDeviceStatuses(body);
            break;
        default:
            readBuf.seekToNextBlock();
            break;
        }
    }
    if(updated){
        bodyItem->notifyKinematicStateChange(doForwardKinematics);
    }
}


int WorldLogFileItemImpl::readLinkPositions(Body* body)
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


int WorldLogFileItemImpl::readJointPositions(Body* body)
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


void WorldLogFileItemImpl::readDeviceStatuses(Body* body)
{
    const int endPos = readBuf.readNextBlockPos();
    DeviceList<> devices = body->devices();
    const int numDevices = devices.size();
    int deviceIndex = 0;
    while(readBuf.pos < endPos && deviceIndex < numDevices){
        const int numFloatElements = readBuf.readOctet();
        const int nextPos = readBuf.pos + sizeof(float) * numFloatElements;
        Device* device = devices[deviceIndex];
        const int stateSize = device->stateSize();
        if(stateSize <= numFloatElements){
            doubleReadBuf.resize(stateSize);
            for(int i=0; i < stateSize; ++i){
                doubleReadBuf[i] = readBuf.readFloat();
            }
            device->readState(&doubleReadBuf.front());
            device->notifyStateChange();
        }
        readBuf.seek(nextPos);
        ++deviceIndex;
    }
    readBuf.seek(endPos);
}


void WorldLogFileItem::clearOutput()
{
    impl->clearOutput();
}


void WorldLogFileItemImpl::clearOutput()
{
    bodyNames.clear();

    if(ifs.is_open()){
        ifs.close();
    }
    if(ofs.is_open()){
        ofs.close();
    }
    recordingStartTime = QDateTime::currentDateTime();
    
    ofs.open(getActualFilename().c_str(), ios::out | ios::binary | ios::trunc);
    writeBuf.clear();
    lastOutputFramePos = 0;
}


void WorldLogFileItemImpl::reserveSizeHeader()
{
    sizeHeaderStack.push(writeBuf.size());
    writeBuf.writeSeekOffset(0);
}


void WorldLogFileItemImpl::fixSizeHeader()
{
    if(!sizeHeaderStack.empty()){
        writeBuf.writeSeekOffset(sizeHeaderStack.top(), writeBuf.size() - (sizeHeaderStack.top() + sizeof(int)));
        sizeHeaderStack.pop();
    }
}


void WorldLogFileItemImpl::flushWriteBuf()
{
    ofs.write(writeBuf.buf(), writeBuf.size());
    ofs.flush();
    writeBuf.clear();
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


void WorldLogFileItemImpl::endHeaderOutput()
{
    fixSizeHeader();
    flushWriteBuf();
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


void WorldLogFileItemImpl::beginFrameOutput(double time)
{
    int pos = ofs.tellp();
    if(lastOutputFramePos){
        writeBuf.writeSeekOffset(pos - lastOutputFramePos);
    } else {
        writeBuf.writeSeekOffset(0);
    }
    lastOutputFramePos = pos;
    writeBuf.writeFloat(time);
    reserveSizeHeader(); // area for the frame data size
}


void WorldLogFileItem::beginBodyStatusOutput()
{
    impl->writeBuf.writeID(BODY_STATUS);
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


void WorldLogFileItem::beginDeviceStatusOutput()
{
    impl->writeBuf.writeID(DEVICE_STATUSES);
    impl->reserveSizeHeader();
}


void WorldLogFileItem::outputDeviceStatus(DeviceState* status)
{
    WriteBuf& writeBuf = impl->writeBuf;

    if(!status){
        writeBuf.writeOctet(0);
    } else {
        int size = status->stateSize();
        writeBuf.writeOctet(size);
        impl->doubleWriteBuf.resize(size);
        status->writeState(&impl->doubleWriteBuf.front());
        for(size_t i=0; i < size; ++i){
            writeBuf.writeFloat(impl->doubleWriteBuf[i]);
        }
    }
}


void WorldLogFileItem::endDeviceStatusOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endBodyStatusOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endFrameOutput()
{
    impl->fixSizeHeader();
    impl->flushWriteBuf();
}


void WorldLogFileItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Log file name"), impl->filename,
                boost::bind(&WorldLogFileItemImpl::setLogFileName, impl, _1));
    putProperty(_("Actual log file"), impl->getActualFilename());
    putProperty(_("Time-stamp suffix"), impl->isTimeStampSuffixEnabled,
                changeProperty(impl->isTimeStampSuffixEnabled));
    putProperty(_("Recording frame rate"), impl->recordingFrameRate,
                changeProperty(impl->recordingFrameRate));
}


bool WorldLogFileItem::store(Archive& archive)
{
    archive.write("filename", impl->filename);
    archive.write("timeStampSuffix", impl->isTimeStampSuffixEnabled);
    archive.write("recordingFrameRate", impl->recordingFrameRate);
    return true;
}


bool WorldLogFileItem::restore(const Archive& archive)
{
    string filename;
    archive.read("timeStampSuffix", impl->isTimeStampSuffixEnabled);
    archive.read("recordingFrameRate", impl->recordingFrameRate);
    if(archive.read("filename", filename)){
        impl->setLogFileName(archive.expandPathVariables(filename));
    }
    return true;
}
