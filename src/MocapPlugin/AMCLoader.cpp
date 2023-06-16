#include "AMCLoader.h"
#include "Skeleton.h"
#include "Bone.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

// ASF symbol ids
enum {
    NO_SYMBOL = 0,

    S_VERSION,
    S_NAME,
    S_UNITS,
    S_DOCUMENTATION,
    S_ROOT,
    S_BONEDATA,
    S_HIERARCHY,

    S_MASS,
    S_LENGTH,
    S_ANGLE,

    S_ORDER,
    S_AXIS,
    S_POSITION,
    S_ORIENTATION,

    S_BEGIN,
    S_END,

    S_ID,
    S_DIRECTION,
    S_DOF,
    S_LIMITS,
    S_BODYMASS,
    S_COFMASS
};

struct TSymbol {
    int id;
    const char* symbol;
};

TSymbol symbols[] = {
    { S_VERSION, "version" },
    { S_NAME, "name" },
    { S_UNITS, "units" },
    { S_DOCUMENTATION, "documentation" },
    { S_ROOT, "root" },
    { S_BONEDATA, "bonedata" },
    { S_HIERARCHY, "hierarchy" },
    { S_MASS, "mass" },
    { S_LENGTH, "length" },
    { S_ANGLE, "angle" },
    { S_ORDER, "order" },
    { S_AXIS, "axis" },
    { S_POSITION, "position" },
    { S_ORIENTATION, "orientation" },
    { S_BEGIN, "begin" },
    { S_END, "end" },
    { S_ID, "id" },
    { S_DIRECTION, "direction" },
    { S_AXIS, "axis" },
    { S_DOF, "dof" },
    { S_LIMITS, "limits" },
    { S_BODYMASS, "bodymass" },
    { S_COFMASS, "cofmass" },
    { 0, "" }
};

}


AMCLoader::AMCLoader()
{
    for(int i=0; symbols[i].id != 0; i++){
        scanner.registerSymbol(symbols[i].id, symbols[i].symbol);
    }
}


void AMCLoader::enableCoordinateFlipping(bool on)
{
    if(on){
        axisMap = CoordAxisMap(CoordAxisMap::Z, CoordAxisMap::X, CoordAxisMap::Y);
    } else {
        axisMap = CoordAxisMap();
    }
}
    

SkeletonPtr AMCLoader::loadASF(const std::string& filename, std::ostream& os)
{
    skeleton = nullptr;

    version.clear();
    name_.clear();
    documentation.clear();
    massUnit = 0.0;
    toRadian = 1.0;
    toMeter = 2.54 * 0.01;
    boneInfoMap.clear();
    
    try {
        scanner.loadFile(filename);

        skeleton = new Skeleton;
        rootBone = new Bone;
        rootBone->setName("root");
        boneInfoMap[rootBone->name()] = new BoneInfo(rootBone);
        
        if(!readASF(os)){
            skeleton = nullptr;
        } else {
            skeleton->setRootBone(rootBone);
            updateChannels();
        }
    } catch(const EasyScanner::Exception& ex){
        os << ex.getFullMessage() << endl;
        skeleton = nullptr;
    }

    if(!skeleton){
        rootBone = nullptr;
    }

    return skeleton;
}


bool AMCLoader::readASF(std::ostream& os)
{
    bool hierarchyProcessed = false;
    bool unitsProcessed = false;
    bool rootProcessed = false;
    
    while(true){
        scanner.skipBlankLines();
        
        if(!scanner.readChar(':')){
            break;
        }

        if(!scanner.readSymbol()){
            return false;
        }

        switch(scanner.symbolValue){

        case S_VERSION:
            if(scanner.readLine()){
                version = scanner.stringValue;
            }
            break;

        case S_NAME:
            if(scanner.readLine()){
                skeleton->setName(scanner.stringValue);
            }
            break;

        case S_DOCUMENTATION:
            scanner.readLF();

            while(true){
                if(scanner.peekChar() == ':'){
                    break;
                }
                if(!scanner.readLine()){
                    break;
                }
                documentation += scanner.stringValue;
                documentation += "\n";
            }
            break;

        case S_UNITS:
            readUnits();
            unitsProcessed = true;
            break;

        case S_ROOT:
            readRoot();
            rootProcessed = true;
            break;

        case S_BONEDATA:
            if(hierarchyProcessed){
                return false;
            }
            readBones();
            break;

        case S_HIERARCHY:
            readHierarchy();
            hierarchyProcessed = true;
            break;

        default:
            scanner.throwException("Unknown section");
            break;
        }
    }

    bool completed = (hierarchyProcessed && rootProcessed && unitsProcessed);
    if(!completed){
        if(!unitsProcessed){
            os << "Error :units is not found.\n";
        }
        if(!rootProcessed){
            os << "Error :root is not found.\n";
        }
        if(!hierarchyProcessed){
            os << "Error :hierarchy is not found.\n";
        }
        os.flush();
    }

    return completed;
}
    
            
void AMCLoader::readUnits()
{
    scanner.readLFex();
    
    while(true){
        scanner.skipBlankLines();

        if(scanner.peekChar() == ':'){
            break;
        }
        if(!scanner.readSymbol()){
            break;
        }
        switch(scanner.symbolValue){

        case S_MASS:
            massUnit = scanner.readDoubleEx();
            break;

        case S_LENGTH:
            lengthUnit = scanner.readDoubleEx();
            toMeter = 2.54 * 0.01 / lengthUnit;
            break;

        case S_ANGLE:
            scanner.readWordEx();
            if(scanner.stringValue == "deg"){
                toRadian = M_PI / 180.0;
            } else if(scanner.stringValue == "rad"){
                toRadian = 1.0;
            } else {
                scanner.throwException(
                    format(_("Invalid angle unit {0}. It should be either \"deg\" or \"rad\"."), scanner.stringValue));
            }
            break;

        default:
            scanner.throwException("Error in :units");
            break;
        }
        scanner.readLFex();
    }
}


void AMCLoader::readRoot()
{
    vector<Bone::Axis> axes;
    bool hasOrientation = false;
    Vector3 orientation;

    scanner.readLFex();
    
    while(true){
        scanner.skipBlankLines();

        if(scanner.peekChar() == ':'){
            break;
        }
        if(!scanner.readSymbol()){
            break;
        }
        switch(scanner.symbolValue){

        case S_ORDER:
            readRootDofOrder();
            scanner.readLFex();
            break;
            
        case S_POSITION:
        {
            Vector3 offset;
            for(int i=0; i < 3; ++i){
                offset[axisMap.axisForSource(i)] = scanner.readDoubleEx();
            }
            rootBone->setOffsetTranslation(toMeter * offset);
            scanner.readLFex();
            break;
        }

        case S_AXIS:
            readRotationAxes(axes);
            scanner.readLFex();
            break;
            
        case S_ORIENTATION:
            for(int i=0; i < 3; ++i){
                orientation[axisMap.axisForSource(i)] = toRadian * scanner.readDoubleEx();
            }
            hasOrientation = true;
            scanner.readLFex();
            break;

        default:
            scanner.throwException("error in :root");
            break;
        }
    }

    if(hasOrientation){
        if(axes.empty()){
            scanner.throwException("\"axis\" must be correctly specified in :root");
        }
        Isometry3 T = Bone::calcAxisSetTransform(axes, orientation.data(), true);
        rootBone->setRotationOrientation(T.linear());
    }
}


void AMCLoader::readRootDofOrder()
{
    while(scanner.readWord()){
        string& axisLabel = scanner.stringValue;
        Bone::Axis axis;
        if(axisLabel == "TX"){
            axis = axisMap.axisForSource(Bone::TX, Bone::TX);
        } else if(axisLabel == "TY"){
            axis = axisMap.axisForSource(Bone::TY, Bone::TX);
        } else if(axisLabel == "TZ"){
            axis = axisMap.axisForSource(Bone::TZ, Bone::TX);
        } else if(axisLabel == "RX"){
            axis = axisMap.axisForSource(Bone::RX, Bone::RX);
        } else if(axisLabel == "RY"){
            axis = axisMap.axisForSource(Bone::RY, Bone::RX);
        } else if(axisLabel == "RZ"){
            axis = axisMap.axisForSource(Bone::RZ, Bone::RX);
        } else {
            scanner.throwException("Undefined symbol in transform order");
        }
        rootBone->appendAxis(axis);
    }
    rootBone->setEulerAngleMode(true);
}


void AMCLoader::readRotationAxes(std::vector<Bone::Axis>& out_axes)
{
    for(int i=0; i < 3; ++i){
        int axisLabel = scanner.readCharEx();
        Bone::Axis axis;
        if(axisLabel == 'X'){
            axis = Bone::RX;
        } else if(axisLabel == 'Y'){
            axis = Bone::RY;
        } else if(axisLabel == 'Z'){
            axis = Bone::RZ;
        } else {
            scanner.throwException("Undefined character in rotation order");
        }
        out_axes.push_back(axisMap.axisForSource(axis, Bone::RX));
    }
}


void AMCLoader::readBoneDofOrder(Bone* bone)
{
    while(scanner.readWord()){
        string& axisLabel = scanner.stringValue;
        Bone::Axis axis;
        if(axisLabel == "rx"){
            axis = Bone::RX;
        } else if(axisLabel == "ry"){
            axis = Bone::RY;
        } else if(axisLabel == "rz"){
            axis = Bone::RZ;
        } else {
            scanner.throwException("Undefined symbol in transform order");
        }
        bone->appendAxis(axisMap.axisForSource(axis, Bone::RX));
    }
    bone->setEulerAngleMode(true);
}


void AMCLoader::readBones()
{
    scanner.readLFex();

    while(true){
        scanner.skipBlankLines();

        if(scanner.readString("begin")){
            scanner.readLFex();
            readBone();
        } else {
            break;
        }
    }
}


void AMCLoader::readBone()
{
    BonePtr bone = new Bone;
    BoneInfoPtr boneInfo = new BoneInfo(bone);
    
    while(true){
        scanner.skipBlankLines();

        if(!scanner.readSymbol()){
            scanner.throwException("Invalid symbol in bone:");
        }
        
        if(scanner.symbolValue == S_END){
            scanner.readLFex();
            break;
        }

        bool LFread = false;
        
        switch(scanner.symbolValue){

        case S_ID:
            scanner.readIntEx();
            break;

        case S_NAME:
            bone->setName(scanner.readWordEx());
            if(boneInfoMap.find(bone->name()) != boneInfoMap.end()){
                scanner.throwException(format(_("Bone name \"{0}\" is overlapping."), bone->name()));
            }
            boneInfoMap[bone->name()] = boneInfo;
            break;

        case S_AXIS:
            {
                Vector3 angles;
                for(int i=0; i < 3; ++i){
                    angles[i] = toRadian * scanner.readDoubleEx();
                }
                vector<Bone::Axis> axes;
                readRotationAxes(axes);
                Isometry3 T = Bone::calcAxisSetTransform(axes, angles.data(), true);
                bone->setRotationOrientation(T.linear());
            }
            break;

        case S_DIRECTION:
            for(int i=0; i < 3; ++i){
                boneInfo->direction[axisMap.axisForSource(i)] = scanner.readDoubleEx();
            }
            boneInfo->direction.normalize();
            break;

        case S_LENGTH:
            boneInfo->length = toMeter * scanner.readDoubleEx();
            break;

        case S_DOF:
            readBoneDofOrder(bone);
            break;

        case S_LIMITS:
            while(true){
                if(!scanner.readChar('(')){
                    break;
                }
                double lower = readLimitValue();
                double upper = readLimitValue();
                scanner.readCharEx(')');
                scanner.readLFex();
                LFread = true;
            }
            break;
                        

        case S_BODYMASS:
            scanner.readDoubleEx();
            break;

        case S_COFMASS:
            scanner.throwException("cofmass is not supported.");
            break;

        default:
            scanner.throwException("Invalid keyword in :units");
            break;
        }

        if(!LFread){
            scanner.readLFex();
        }
    }

    if(bone->name().empty()){
        scanner.throwException("Bone's name is not found.");
    }
}


double AMCLoader::readLimitValue()
{
    double sign;
    if(scanner.readChar('-')){
        sign = -1.0;
    } else {
        sign = 1.0;
    }

    if(scanner.readString("inf")){
        return sign * numeric_limits<double>::max();
    } else {
        return sign * toRadian * scanner.readDoubleEx();
    }
}


void AMCLoader::readHierarchy()
{
    scanner.readLFex();

    scanner.checkStringEx("begin");
    scanner.readLFex();
    scanner.skipBlankLines();

    //scanner.checkStringEx("root");
    //readChildBones(rootBone);

    while(true){
        if(scanner.readString("end")){
            scanner.readLFex();
            break;
        }
        
        const string& parentName = scanner.readWordEx();
        
        auto p = boneInfoMap.find(parentName);
        if(p == boneInfoMap.end()){
            scanner.throwException("Undefined bone in hierarchy section");
        }
        readChildBones(p->second);
    }

    addEndNodeBones(rootBone);
}


void AMCLoader::readChildBones(BoneInfo* parentBoneInfo)
{
    while(!scanner.checkLF()){
        auto p = boneInfoMap.find(scanner.readWordEx());
        if(p == boneInfoMap.end()){
            scanner.throwException("Undefined bone in hierarchy section.");
        }
        auto& boneInfo = p->second;
        Bone* childBone = boneInfo->bone;
        childBone->setOffsetTranslation(parentBoneInfo->length * parentBoneInfo->direction);
        parentBoneInfo->bone->appendChild(childBone);
    }
    scanner.skipBlankLines();
}


void AMCLoader::addEndNodeBones(Bone* bone)
{
    if(bone->child()){
        for(Bone* child = bone->child(); child; child = child->sibling()){
            addEndNodeBones(child);
        }
    } else {
        auto p = boneInfoMap.find(bone->name());
        if(p != boneInfoMap.end()){
            Bone* endBone = new Bone;
            endBone->setName(bone->name() + "__end");
            auto& boneInfo = p->second;
            endBone->setOffsetTranslation(boneInfo->length * boneInfo->direction);
            bone->appendChild(endBone);
        }
    }
}


void AMCLoader::updateChannels()
{
    int channel = 0;
    for(int i=0; i < skeleton->numBones(); ++i){
        Bone* bone = skeleton->bone(i);
        auto p = boneInfoMap.find(bone->name());
        if(p != boneInfoMap.end()){
            auto& info = p->second;
            info->channel = channel;
            channel += bone->numAxes();
        }
    }
}        


SkeletonMotionPtr AMCLoader::loadAMC(const std::string& filename, std::ostream& os)
{
    skeletonMotion.reset();

    if(!skeleton){
        os << "AMC file cannot be loaded because there is no valid ASF (skeleton) file information." << endl;

    } else {
        try {
            scanner.loadFile(filename);
            skeletonMotion.reset(new SkeletonMotion(new Skeleton(*skeleton)));
            skeletonMotion->setAxisMap(axisMap);
            checkAmcHeader();
            readAmcFrames();

            // AMC file does not have the information on the frame rate, and the default frame rate value
            // is set by the following code. A user should specify the correct value manually on GUI.
            skeletonMotion->setFrameRate(100.0);

        } catch(const EasyScanner::Exception& ex){
            os << ex.getFullMessage() << endl;
            skeletonMotion.reset();
        }
    }
    
    return skeletonMotion;
}


void AMCLoader::checkAmcHeader()
{
    scanner.skipBlankLines();
    scanner.checkStringEx(":FULLY-SPECIFIED");
    scanner.readLFex();

    scanner.skipBlankLines();
    if(scanner.readString(":DEGREES")){
        toRadian = M_PI / 180.0;
    } else if(scanner.readString(":RADIAN")){
        toRadian = 1.0;
    } else {
        scanner.throwException("Angle unit is not correctly specified.");
    }
    scanner.readLFex();
    scanner.skipBlankLines();
}


void AMCLoader::readAmcFrames()
{
    skeletonMotion->setDimension(1, skeleton->numAxes(), true);

    int frame = scanner.readIntEx() - 1;
    scanner.readLFex();
    
    while(true){

        if(scanner.readInt()){
            frame = scanner.intValue - 1;
            scanner.readLFex();
            scanner.skipBlankLines();

            if(frame >= skeletonMotion->numFrames()){
                skeletonMotion->setNumFrames(frame + 1, true);
            }
        }
        
        if(!scanner.readWord()){
            break;
        }
        readBoneJointPosition(scanner.stringValue, frame);
    }

    scanner.skipBlankLines();
    scanner.readLFEOFex();
}


void AMCLoader::readBoneJointPosition(const std::string& boneName, int frame)
{
    auto p = boneInfoMap.find(boneName);
    if(p == boneInfoMap.end()){
        scanner.throwException(format("Bone \"{0}\" is not defined.", boneName));
    }

    SkeletonMotion::Frame f = skeletonMotion->frame(frame);
    const auto& info = p->second;
    const Bone* bone = info->bone;
    for(int i=0; i < bone->numAxes(); ++i){
        if(Bone::checkAxisType(bone->axis(i)) == Bone::TranslationAxis){
            f[i + info->channel] = toMeter * scanner.readDoubleEx();
        } else { // rotation
            f[i + info->channel] = toRadian * scanner.readDoubleEx();
        }
    }
    
    scanner.readLFex();
    scanner.skipBlankLines();
}
