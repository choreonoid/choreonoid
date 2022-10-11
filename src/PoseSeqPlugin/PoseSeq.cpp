#include "PoseSeq.h"
#include "BodyKeyPose.h"
#include "PronunSymbol.h"
#include "LipSyncTranslator.h"
#include <cnoid/CloneMap>
#include <cnoid/Body>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fstream>

using namespace std;
using namespace cnoid;


PoseSeq::PoseSeq()
{
    
}


PoseSeq::PoseSeq(const PoseSeq& org, CloneMap* cloneMap)
{
    iterator current = begin();
    for(auto it = org.begin(); it != org.end(); ++it){
        current = copyElement(current, it, 0.0, cloneMap);
    }
}


PoseSeq::~PoseSeq()
{

}


PoseSeq::iterator PoseSeq::copyElement(iterator seekpos, const_iterator org, double offset, CloneMap* cloneMap)
{
    iterator pos;

    if(auto orgPose = org->pose()){
        auto copiedPose = CloneMap::getClone(orgPose, cloneMap);
        pos = insert(seekpos, org->time() + offset, copiedPose);
        pos->setMaxTransitionTime(org->maxTransitionTime());
    } else {
        pos = seekpos;
    }

    return seekpos;
}


Referenced* PoseSeq::doClone(CloneMap* cloneMap) const
{
    return new PoseSeq(*this, cloneMap);
}


void PoseSeq::setName(const std::string& name)
{
    name_ = name;
}


bool PoseSeq::load(const std::string& filename, const Body* body)
{
    errorMessage_.clear();

    /// \todo emit signals
    poses.clear();
    
    YAMLReader parser;
    if(parser.load(filename)){
        const Mapping& archive = *parser.document()->toMapping();
        restore(archive, body);
        setName(archive["name"]);
        return true;
    }

    return false;
}


bool PoseSeq::restore(const Mapping& archive, const Body* body)
{
    setTargetBodyName(archive.get("targetBody", body->name()));
                                   
    const Listing& refs = *archive.findListing("refs");

    if(!refs.isValid()){
        return false;
    }
        
    PoseSeq::iterator current = begin();
    
    for(int i=0; i < refs.size(); ++i){
        const Mapping& ref = *refs[i].toMapping();
        bool isInserted = false;
        double time = ref["time"].toDouble();
        const ValueNode& referred = ref["refer"];
        const Mapping& mReferred = *referred.toMapping();
        const string& type = mReferred["type"];
        AbstractPosePtr pose;
        if(type == "Pose"){
            pose = new BodyKeyPose;
        } else if(type == "PronunSymbol"){
            pose = new PronunSymbol;
        }
        if(pose && pose->restore(mReferred, body)){
            current = insert(current, time, pose);
            isInserted = true;
        }

        if(isInserted){
            current->setMaxTransitionTime(ref.get("maxTransitionTime", 0.0));
        }
    }

    return true;
}


bool PoseSeq::save(const std::string& filename, const Body* body)
{
    YAMLWriter writer(filename);
    writer.setKeyOrderPreservationMode(true);
    MappingPtr archive = new Mapping;
    archive->setFloatingNumberFormat("%.9g");
    store(*archive, body);
    writer.putComment("Body pose sequence format version 1.0 defined by cnoid-Robotics\n");
    writer.putNode(archive);
    return true;
}


void PoseSeq::store(Mapping& archive, const Body* body) const
{
    archive.write("type", "PoseSeq");
    archive.write("name", name(), DOUBLE_QUOTED);
    archive.write("targetBody", body->name(), DOUBLE_QUOTED);

    Listing& refsNode = *archive.createListing("refs");
    
    for(auto& pose : poses){
        MappingPtr refNode = refsNode.newMapping();
        refNode->write("time", pose.time());
        if(pose.maxTransitionTime() > 0.0){
            refNode->write("maxTransitionTime", pose.maxTransitionTime());
        }
        MappingPtr childNode = refNode->createMapping("refer");
        pose.pose()->store(*childNode, body);
    }
}


bool PoseSeq::exportTalkPluginFile(const std::string& filename)
{
    ofstream ofs(filename.c_str());
    double standardTransitionTiem = 0.135;

    double prevTime = 0.0;
    string prevSymbol;

    static const bool ENABLE_QUICK_MODE = true;
    
    if(ENABLE_QUICK_MODE){
        
        if(!poses.empty()){
            bool isInitial = true;
            for(auto& pose : poses){
                PronunSymbol* symbol = pose.get<PronunSymbol>();
                if(symbol && !symbol->symbol().empty()){
                    const double time = pose.time();
                    if(isInitial){
                        isInitial = false;
                    } else {
                        double durationTime = time - prevTime;
                        if(durationTime <= 0.6){
                            ofs << prevSymbol << " " << durationTime << "\n";
                        } else {
                            ofs << prevSymbol << " " << 0.6 << "\n";
                            ofs << "n" << " " << (durationTime - 0.6) << "\n";
                        }
                    }
                    prevTime = time;
                    prevSymbol = symbol->symbol();
                }
            }
            
            ofs << prevSymbol << " " << standardTransitionTiem << "\n";
        }
    } else {
        for(auto& pose : poses){
            PronunSymbolPtr symbol = pose.get<PronunSymbol>();
            if(symbol && !symbol->symbol().empty()){
                const double time = pose.time();
                double transitionTime = time - prevTime;
                ofs << symbol->symbol() << " " << transitionTime << "\n";
                prevTime = time;
            }
        }
    }

    ofs.close();

    return true;
}



bool PoseSeq::exportSeqFileForFaceController(const std::string& filename)
{
    LipSyncTranslator translator;
    translator.translatePoseSeq(*this);
    return translator.exportSeqFileForFaceController(filename);
}


PoseSeq::iterator PoseSeq::changeTime(iterator it, double newTime)
{
    iterator newpos;
    iterator insertpos = seek(it, newTime, true);
    iterator nextpos = it;
    nextpos++;
    
    if(insertpos == it || insertpos == nextpos){
        beginPoseModification(it);
        it->time_ = newTime;
        endPoseModification(it);
        newpos = it;
    } else {
        sigPoseAboutToBeRemoved_(it, true);
        SequentialPose newPose(it->pose(), newTime);
        newPose.setMaxTransitionTime(it->maxTransitionTime());
        poses.erase(it);
        newpos = poses.insert(insertpos, newPose);
        sigPoseInserted_(newpos, true);
    }

    return newpos;
}


PoseSeq::iterator PoseSeq::seek(PoseSeq::iterator current, double time, bool seekPosToInsert)
{
    if(current == poses.end()){
        if(current == poses.begin()){
            return current;
        }
        current--;
    }

    double ct = current->time();

    if(ct == time){
        if(seekPosToInsert){
            current++;
        }
    } else if(ct > time){
        while(current != poses.begin()){
            current--;
            ct = current->time();
            if(ct == time){
                if(seekPosToInsert){
                    current++;
                }
                break;
            } else if(ct < time){
                current++;
                break;
            }
        }
    } else {
        while(current != poses.end() && (current->time() < time)){
            current++;
        }
    }

    return current;
}
            

PoseSeq::iterator PoseSeq::insert(iterator current, double time, AbstractPose* pose, bool doAdjustMaxTransitionTime)
{
    SequentialPose sequentialPose(pose, time);
    return insert(current, time, sequentialPose, doAdjustMaxTransitionTime);
}


PoseSeq::iterator PoseSeq::insert(iterator current, double time, SequentialPose& pose, bool doAdjustMaxTransitionTime)
{
    iterator it = seek(current, time);

    if(doAdjustMaxTransitionTime && it != poses.begin()){
        if(auto bkPose = pose.get<BodyKeyPose>()){
            bool matched = false;
            double prevTime = -std::numeric_limits<double>::max();
            int nj = bkPose->numJoints();
            auto precedentIter = it;
            --precedentIter;
            do {
                if(auto precedent = precedentIter->get<BodyKeyPose>()){
                    auto ikLinkIter = bkPose->ikLinkBegin();
                    while(ikLinkIter != bkPose->ikLinkEnd()){
                        int linkIndex = ikLinkIter->first;
                        if(precedent->ikLinkInfo(linkIndex)){
                            matched = true;
                            break;
                        }
                        ++ikLinkIter;
                    }
                    if(!matched){
                        for(int i=0; i < nj; ++i){
                            if(bkPose->isJointValid(i) && precedent->isJointValid(i)){
                                matched = true;
                                break;
                            }
                        }
                    }
                    if(matched){
                        prevTime = precedentIter->time();
                        break;
                    }
                }
                --precedentIter;
            } while(precedentIter != poses.begin());
            
            if(matched){
                double tt = time - prevTime;
                if(tt >= 0.0 && tt <= 10.0){
                    pose.setMaxTransitionTime(tt);
                }
            }
        }
    }

    it = poses.insert(it, pose);
    sigPoseInserted_(it, false);
    return it;
}


PoseSeq::iterator PoseSeq::erase(iterator it)
{
    sigPoseAboutToBeRemoved_(it, false);
    return poses.erase(it);
}


void PoseSeq::getDomain(double& out_lower, double& out_upper)
{
    if(poses.empty()){
        out_lower = 0.0;
        out_upper = 0.0;
    } else {
        out_lower = poses.front().time();
        out_upper = poses.back().time();
    }
}
