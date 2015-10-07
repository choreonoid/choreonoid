/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldLogFileItem.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/FileUtil>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <fstream>
#include <stack>

#include <iostream>

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

enum HeaderID {
    BODY_STATUS,
    LINK_POSITIONS,
    JOINT_VALUES
};

struct NotEnoughDataException { };

class ReadBuf
{
public:
    vector<char> data;
    fstream& file;
    int pos;

    ReadBuf(fstream& file)
        : file(file) {
        pos = 0;
    }

    bool checkSize(int size){
        int left = data.size() - pos;
        if(left < size){
            int len = size - left;
            data.resize(data.size() + len);
            file.read(&data[pos], len);
            return (file.gcount() == len);
        }
        return true;
    }

    void ensureSize(int size){
        if(!checkSize(size)){
            throw NotEnoughDataException();
        }
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
        return (pos == data.size());
    }

    void seek(int pos = 0) { this->pos = pos; }

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

    void writeID(HeaderID id){
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

}


namespace cnoid {

class WorldLogFileItemImpl
{
public:
    WorldLogFileItem* self;
    string filename;
    vector<string> bodyNames;
    ItemList<BodyItem> bodyItems;
    fstream file;
    ReadBuf readBuf;
    WriteBuf writeBuf;

    stack<int> sizeHeaderStack;
        
    WorldLogFileItemImpl(WorldLogFileItem* self);
    WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org);
    ~WorldLogFileItemImpl();
    bool setLogFileName(const std::string& name);
    void readHeaders();
    void updateBodyItems();
    void clear();
    void reserveSizeHeader();
    void fixSizeHeader();
    void flushWriteBuf();
    void endHeaderOutput();
};

}


void WorldLogFileItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<WorldLogFileItem>(N_("WorldLogFileItem"));
    im.addCreationPanel<WorldLogFileItem>();
}


WorldLogFileItem::WorldLogFileItem()
{
    impl = new WorldLogFileItemImpl(this);

}

WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self)
    : self(self),
      readBuf(file)
{

}


WorldLogFileItem::WorldLogFileItem(const WorldLogFileItem& org)
    : Item(org)
{
    impl = new WorldLogFileItemImpl(this, *org.impl);
}


WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org)
    : self(self),
      readBuf(file)
{
    filename = org.filename;
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
    filename = name;
    readHeaders();
    return true;
}


void WorldLogFileItemImpl::readHeaders()
{
    bodyNames.clear();
    
    if(!file.is_open()){
        if(filesystem::exists(filename)){
            file.open(filename.c_str(), ios::in | ios::out | ios::binary);
        }
    }
    if(file.is_open()){
        file.seekg(0);
        readBuf.clear();
        try {
            int headerSize = readBuf.readInt();
            if(readBuf.checkSize(headerSize)){
                while(!readBuf.isEnd()){
                    bodyNames.push_back(readBuf.readString());
                    //cout << bodyNames.back() << endl;
                }
            }
        } catch(NotEnoughDataException& ex){
            bodyNames.clear();
        }
    }

    updateBodyItems();
}


void WorldLogFileItemImpl::updateBodyItems()
{
    bodyItems.clear();
    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        if(bodyItems.extractChildItems(worldItem)){

        }
    }
}


void WorldLogFileItem::onPositionChanged()
{
    impl->updateBodyItems();
}


void WorldLogFileItem::clear()
{
    impl->clear();
}


void WorldLogFileItemImpl::clear()
{
    bodyNames.clear();
    if(file.is_open()){
        file.close();
    }
    file.open(filename.c_str(), ios::in | ios::out | ios::binary | ios::trunc);
    writeBuf.clear();
}


void WorldLogFileItemImpl::reserveSizeHeader()
{
    sizeHeaderStack.push(writeBuf.size());
    writeBuf.writeInt(0);
}


void WorldLogFileItemImpl::fixSizeHeader()
{
    if(!sizeHeaderStack.empty()){
        writeBuf.writeInt(sizeHeaderStack.top(), writeBuf.size() - (sizeHeaderStack.top() + sizeof(int)));
        sizeHeaderStack.pop();
    }
}


void WorldLogFileItemImpl::flushWriteBuf()
{
    file.write(writeBuf.buf(), writeBuf.size());
    file.flush();
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
    impl->reserveSizeHeader();
    impl->writeBuf.writeFloat(time);
}


void WorldLogFileItem::beginBodyStatusOutput()
{
    impl->writeBuf.writeID(BODY_STATUS);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(numBodies());
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


void WorldLogFileItem::outputJointValues(double* values, int size)
{
    impl->writeBuf.writeID(JOINT_VALUES);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(size);
    for(int i=0; i < size; ++i){
        impl->writeBuf.writeFloat(values[i]);
    }
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


void WorldLogFileItem::notifyUpdate()
{
    Item::notifyUpdate();
}


bool WorldLogFileItem::recallStatusAtTime(double time)
{
    return false;
}


void WorldLogFileItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Log file"), impl->filename,
                boost::bind(&WorldLogFileItemImpl::setLogFileName, impl, _1));
}


bool WorldLogFileItem::store(Archive& archive)
{
    archive.writeRelocatablePath("filename", impl->filename);
    return true;
}


bool WorldLogFileItem::restore(const Archive& archive)
{
    string filename;
    if(archive.read("filename", filename)){
        impl->setLogFileName(archive.expandPathVariables(filename));
    }
    return true;
}
