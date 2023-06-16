#include "SkeletonMotion.h"
#include "Bone.h"
#include <cnoid/EasyScanner>
#include <cnoid/EigenUtil>
#include <fmt/format.h>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const double BVH_DEFAULT_TO_METER = 0.01; // convert centimeter to meter
const double BVH_METER_TO_DEFAULT = 100.0;

}

SkeletonMotion::SkeletonMotion()
{
    skeleton_ = new Skeleton;
    currentFrame_ = -1;
    T_offset_.setIdentity();
}


/**
   \note skeleton should not be shared with other SkeletonMotions
*/
SkeletonMotion::SkeletonMotion(SkeletonPtr skeleton)
    : skeleton_(skeleton)
{
    currentFrame_ = -1;
    T_offset_.setIdentity();
}


SkeletonMotion::SkeletonMotion(const SkeletonMotion& org)
    : MultiValueSeq(org)
{
    if(org.skeleton_){
        skeleton_ = new Skeleton(*org.skeleton_);
    }
    currentFrame_ = -1;
    T_offset_ = org.T_offset_;
}


SkeletonMotion::~SkeletonMotion()
{

}


void SkeletonMotion::resetSkeleton(SkeletonPtr skeleton)
{
    skeleton_ = skeleton;
}


void SkeletonMotion::setPositionOffset(const Isometry3& T_offset)
{
    T_offset_ = T_offset;
    currentFrame_ = -1;
}


void SkeletonMotion::setTranslation(const Vector3& translation)
{
    T_offset_.translation() = translation;
    currentFrame_ = -1;
}


void SkeletonMotion::setYawRotation(double angle)
{
    T_offset_.linear() = AngleAxis(angle, Vector3::UnitZ()).toRotationMatrix();
    currentFrame_ = -1;
}


double SkeletonMotion::yawRotation() const
{
    return rpyFromRot(T_offset_.linear()).z();
}


bool SkeletonMotion::setFrameToSkeleton(int frameIndex)
{
    if(frameIndex < 0){
        frameIndex = 0;
    } else if(frameIndex >= numFrames()){
        frameIndex = numFrames() - 1;
    }

    Frame elements = frame(frameIndex);
    int elementIndex = 0;
    auto scale = skeleton_->scale();
    bool hasScale = scale != 1.0;
    for(auto& bone : skeleton_->bones()){
        for(int i=0; i < bone->numAxes(); ++i){
            bone->q(i) = elements[elementIndex++];
            if(hasScale && Bone::checkAxisType(bone->axis(i)) == Bone::TranslationAxis){
                bone->q(i) *= scale;
            }
        }
    }

    skeleton_->calcForwardKinematics(T_offset_);

    currentFrame_ = frameIndex;

    return true;
}


bool SkeletonMotion::loadBVH(const std::string& filename, double scale)
{
    currentFrame_ = -1; // reset state
    
    try {

        EasyScanner scanner(filename);

        messages_.clear();

        axisMap = CoordAxisMap(CoordAxisMap::Z, CoordAxisMap::X, CoordAxisMap::Y);

        // skeleton part
        scanner.checkStringEx("HIERARCHY", _("There is no HIERARCHY part."));
        scanner.readLFex(_("line break is missing"));
        skeleton_ = new Skeleton;
        scanner.checkStringEx("ROOT", _("There is no ROOT node"));
        skeleton_->setRootBone(readBvhJoint(scanner, false));

        // motion part
        scanner.checkStringEx("MOTION", _("There is no MOTION part."));
        scanner.readLFex(_("line break is missing"));
        readBvhMotion(scanner);

    } catch(EasyScanner::Exception& ex){
        messages_ += ex.getFullMessage() + "\n";
        return false;
    }

    return true;
}


Bone* SkeletonMotion::readBvhJoint(EasyScanner& scanner, bool isEnd)
{
    Bone* bone = new Bone;

    try {
        scanner.skipSpace();
        string name;
        if(scanner.readLine()){
            // removing last space characters
            int i = scanner.stringValue.size();
            while(i > 0){
                if(scanner.stringValue[i-1] != ' ' &&
                   scanner.stringValue[i-1] != '\t'){
                    break;
                }
            }
            name = scanner.stringValue.substr(0, i);
        }
        if(name.empty()){
            scanner.throwException(_("Node name is missing"));
        }
        bone->setName(name);
    
        scanner.readCharEx('{', _("'{' is missing."));
        scanner.readLFex(_("line break is missing"));

        bool offsetRead = false;
        bool channelsRead = isEnd;
        
        while(scanner.readString()){

            if(scanner.stringValue == "}"){
                break;

            } else if(scanner.stringValue == "OFFSET"){
                Vector3 offset;
                for(int i=0; i < 3; ++i){
                    offset[i] = scanner.readDoubleEx(_("OFFSET values are not correctly specified"));
                }
                bone->setOffsetTranslation(axisMap.getVectorForSource(offset * BVH_DEFAULT_TO_METER));
                scanner.readLFex(_("line break is missing"));
                offsetRead = true;
                
            } else if(scanner.stringValue == "CHANNELS"){
                if(isEnd){
                    scanner.throwException(_("End node cannot have CHANNELS"));
                }
                readBvhChannels(scanner, bone);
                channelsRead = true;

            } else if(scanner.stringValue == "JOINT"){
                if(isEnd){
                    scanner.throwException(_("End node cannot have JOINT"));
                }
                bone->appendChild(readBvhJoint(scanner, false));

            } else if(scanner.stringValue == "End"){
                if(isEnd){
                    scanner.throwException(_("End node cannot have End"));
                }
                bone->appendChild(readBvhJoint(scanner, true));

            } else {
                scanner.throwException(_("Unknown field"));
            }
        }
        
        if(!offsetRead){
            scanner.throwException(_("OFFSET is missing"));
        }
        if(!channelsRead){
            scanner.throwException(_("CHANNELS is missing"));
        }
        scanner.readLFEOFex(_("line break is missing"));

    } catch(EasyScanner::Exception& ex){
        delete bone;
        throw ex;
    }

    return bone;
}


int SkeletonMotion::readBvhChannels(EasyScanner& scanner, Bone* bone)
{
    int n = scanner.readIntEx(_("The number of channels is not correctly specified"));

    for(int i=0; i < n; ++i){
        string axisLabel = scanner.readStringEx(_("Illeagal format"));
        Bone::Axis axis;
        if(axisLabel == "Xposition"){
            axis = axisMap.axisForSource(Bone::TX, Bone::TX);
        } else if(axisLabel == "Yposition"){
            axis = axisMap.axisForSource(Bone::TY, Bone::TX);
        } else if(axisLabel == "Zposition"){
            axis = axisMap.axisForSource(Bone::TZ, Bone::TX);
        } else if(axisLabel == "Xrotation"){
            axis = axisMap.axisForSource(Bone::RX, Bone::RX);
        } else if(axisLabel == "Yrotation"){
            axis = axisMap.axisForSource(Bone::RY, Bone::RX);
        } else if(axisLabel == "Zrotation"){
            axis = axisMap.axisForSource(Bone::RZ, Bone::RX);
        } else {
            scanner.throwException(_("Illegal element type"));
        }
        bone->appendAxis(axis);
    }
    
    scanner.readLFex(_("line break is missing"));

    return n;
}


void SkeletonMotion::readBvhMotion(EasyScanner& scanner)
{
    int specifiedNumFrames = 0;
    bool framesRead = false;
    bool frameTimeRead = false;
    
    while(!framesRead || !frameTimeRead){
        if(scanner.readString("Frames:")){
            specifiedNumFrames = scanner.readIntEx(_("The number of frames is not correctly specified"));
            scanner.readLFex(_("line break is missing"));
            framesRead = true;
        } else if(scanner.readString("Frame Time:")){
            this->setTimeStep(scanner.readDoubleEx(_("Frame time is not correctly specified")));
            scanner.readLFex(_("line break is missing"));
            frameTimeRead = true;
        } else {
            scanner.throwException(_("Unknown field"));
        }
    }

    int numAxes = skeleton_->numAxes();
    this->resize(specifiedNumFrames, numAxes);

    int frameIndex = 0;

    while(!scanner.isEOF()){

        if(frameIndex >= this->numFrames()){
            this->setNumFrames(frameIndex + 1);
        }

        Frame c = frame(frameIndex++);
        for(int i=0; i < numAxes; ++i){
            if(scanner.readDouble()){
                auto& axis = skeleton_->axis(i);
                if(Bone::checkAxisType(axis.axis) == Bone::TranslationAxis){
                    double p = BVH_DEFAULT_TO_METER * scanner.doubleValue;
                    // Store only the difference from the bone's offset as a translation value
                    c[i] = p - axis.bone->offsetTranslation()[axis.axis];
                } else { // rotation
                    c[i] = radian(scanner.doubleValue);
                }
            } else {
                scanner.throwException(format(_("{0}-th motion element is illegal"), i));
            }
        }
        scanner.readLFEOFex(_("line break is missing"));

        if(frameIndex >= specifiedNumFrames){
            scanner.skipBlankLines();
        }
    }

    if(frameIndex !=  specifiedNumFrames){
        messages_ += _("The actual number of frames is different from that specified at 'Frames:' field.\n");
    }
}


void SkeletonMotion::convertToStandardForm()
{
    vector<Bone::Axis> stdRootAxes;
    for(int i=0; i < 3; ++i){
        stdRootAxes.push_back(axisMap.axisForSource(static_cast<Bone::Axis>(i)));
    }
    for(int i=0; i < 3; ++i){
        auto axis = static_cast<Bone::Axis>(i + Bone::RX);
        stdRootAxes.push_back(axisMap.axisForSource(axis, Bone::RX));
    }

    auto rootBone = skeleton_->rootBone();
    
    bool hasJointTranslationDisplacement = false;
    for(auto& axis : skeleton_->axes()){
        if(axis.bone != rootBone){
            if(Bone::checkAxisType(axis.axis) == Bone::TranslationAxis){
                hasJointTranslationDisplacement = true;
                break;
            }
        }
    }
    vector<Bone::Axis> stdJointAxes;
    if(hasJointTranslationDisplacement){
        stdJointAxes = stdRootAxes;
    } else {
        for(int i=0; i < 3; ++i){
            auto axis = static_cast<Bone::Axis>(i + Bone::RX);
            stdJointAxes.push_back(axisMap.axisForSource(axis, Bone::RX));
        }
    }

    SkeletonMotionPtr orgMotion;

    bool needConversion = false;

    for(int i=0; i < skeleton_->numBones(); ++i){
        Bone* bone = skeleton_->bone(i);
        if(!bone->child()){
            continue;
        }
        vector<Bone::Axis>* stdAxes;
        if(bone == rootBone){
            stdAxes = &stdRootAxes;
        } else {
            stdAxes = &stdJointAxes;
        }
        if(bone->axes() != *stdAxes || !bone->rotationOrientation().isApprox(Matrix3::Identity()) || bone->isEulerAngleMode()){
            if(!orgMotion){
                orgMotion.reset(new SkeletonMotion(*this));
            }
            bone->setAxes(*stdAxes);
            bone->setRotationOrientation(Matrix3::Identity());
            bone->setEulerAngleMode(false);
            needConversion = true;
        }
    }
    
    if(!needConversion){
        return;
    }

    skeleton_->updateBones();

    SkeletonPtr orgSkeleton = orgMotion->skeleton();

    setNumParts(skeleton_->numAxes());

    vector<int> eulerIndices;

    for(int i=0; i < numFrames(); ++i){

        orgMotion->setFrameToSkeleton(i);
        Frame f = frame(i);
        
        for(int j=0; j < skeleton_->numBones(); ++j){
            
            Bone* bone = skeleton_->bone(j);
            Isometry3 T_org = orgSkeleton->bone(j)->calcJointTransform();

            // Calculate the converted rotation angle set
            Vector3 eulerAngles = Vector3::Zero();
            eulerIndices.clear();
            for(auto& axis : bone->axes()){
                if(Bone::checkAxisType(axis) == Bone::RotationAxis){
                    eulerIndices.push_back(axis - Bone::RX);
                }
            }
            if(eulerIndices.size() == 3){
                eulerAngles = T_org.linear().eulerAngles(
                    eulerIndices[0], eulerIndices[1], eulerIndices[2]);
            }
            int eulerAngleIndex = 0;
            
            int elementIndex = bone->skeletonAxisIndex();
            for(auto& axis : bone->axes()){
                if(Bone::checkAxisType(axis) == Bone::TranslationAxis){
                    f[elementIndex++] = T_org.translation()[axis];
                } else { // rotation
                    f[elementIndex++] = eulerAngles[eulerAngleIndex++];
                }
            }
        }
    }
}


bool SkeletonMotion::saveBVH(const std::string& filename)
{
    ofstream ofs(filename.c_str());

    convertToStandardForm();

    auto orgAxisMap = axisMap;
    axisMap = CoordAxisMap(CoordAxisMap::Z, CoordAxisMap::X, CoordAxisMap::Y);
    
    ofs << "HIERARCHY\n";

    string indent;
    writeBvhBone(skeleton_->rootBone(), ofs, 0, indent);

    ofs << "MOTION\n";
    ofs << "Frames: " << numFrames() << "\n";
    ofs << "Frame Time: " << getTimeStep() << "\n";

    const int numAxes = numParts();
    if(numAxes > 0){
        for(int i=0; i < numFrames(); ++i){
            Frame f = frame(i);
            int j=0;
            while(true){
                auto& axis = skeleton_->axis(j);
                if(Bone::checkAxisType(axis.axis) == Bone::TranslationAxis){
                    double p = f[j] + axis.bone->offsetTranslation()[axis.axis];
                    ofs << (BVH_METER_TO_DEFAULT * p);
                } else {
                    ofs << degree(f[j]);
                }
                if(++j == numAxes){
                    break;
                }
                ofs << " ";
            }
            ofs << "\n";
        }
    }

    axisMap = orgAxisMap;

    return true;
}


void SkeletonMotion::writeBvhBone(Bone* bone, std::ostream& os, int depth, std::string& indent)
{
    os << indent;
    if(!bone->parent()){
        os << "ROOT " << bone->name() << "\n";
    } else if(bone->child()){
        os <<"JOINT " << bone->name() << "\n";
    } else {
        os << "End Site\n";
    }
    
    os << indent << "{\n";

    ++depth;
    indent.resize(depth * 4, ' ');
    
    const Vector3 o = axisMap.getSourceVector(BVH_METER_TO_DEFAULT * bone->offsetTranslation());
    os << indent << "OFFSET " << o.x() << " " << o.y() << " " << o.z() << "\n";
    
    if(bone->numAxes() > 0){
        os << indent << "CHANNELS " << bone->numAxes();
        for(auto& axis : bone->axes()){
            int sourceMotionAxis;
            if(Bone::checkAxisType(axis) == Bone::TranslationAxis){
                sourceMotionAxis = axisMap.sourceAxis(axis, Bone::TX);
            } else {
                sourceMotionAxis = axisMap.sourceAxis(axis, Bone::RX);
            }
            switch(sourceMotionAxis){
            case Bone::TX: os << " Xposition"; break;
            case Bone::TY: os << " Yposition"; break;
            case Bone::TZ: os << " Zposition"; break;
            case Bone::RX: os << " Xrotation"; break;
            case Bone::RY: os << " Yrotation"; break;
            case Bone::RZ: os << " Zrotation"; break;
            }
        }
        os << "\n";
    }
    
    for(Bone* child = bone->child(); child; child = child->sibling()){
        writeBvhBone(child, os, depth, indent);
    }

    --depth;
    indent.resize(depth * 4, ' ');
    
    os << indent << "}\n";
}
