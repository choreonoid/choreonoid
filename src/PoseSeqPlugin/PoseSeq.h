#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_H

#include "AbstractPose.h"
#include "SequentialPose.h"
#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include <string>
#include <list>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Mapping;

class CNOID_EXPORT PoseSeq : public ClonableReferenced
{
public:
    typedef std::list<SequentialPose>::iterator iterator;
    typedef std::list<SequentialPose>::const_iterator const_iterator;

    PoseSeq();
    PoseSeq(const PoseSeq& org, CloneMap* cloneMap = nullptr);
    ~PoseSeq();

    void setName(const std::string& name);
    const std::string& name() const { return name_; }

    void setTargetBodyName(const std::string& name) { targetBodyName_ = name; }
    const std::string& targetBodyName() { return targetBodyName_; }

    bool load(const std::string& filename, const Body* body);
    bool save(const std::string& filename, const Body* body);

    bool exportTalkPluginFile(const std::string& filename);
    bool exportSeqFileForFaceController(const std::string& filename);

    const std::string& errorMessage() { return errorMessage_; }

    bool restore(const Mapping& archive, const Body* body);
    void store(Mapping& archive, const Body* body) const;

    iterator changeTime(iterator it, double time);

    bool empty() const {
        return poses.empty();
    }

    std::list<SequentialPose>::size_type size() const {
        return poses.size();
    }

    iterator begin(){
        return poses.begin();
    }

    const_iterator begin() const {
        return poses.begin();
    }

    iterator end(){
        return poses.end();
    }

    const_iterator end() const {
        return poses.end();
    }

    SequentialPose& front() {
        return poses.front();
    }

    SequentialPose& back() {
        return poses.back();
    }
            
    iterator seek(iterator current, double time, bool seekPosToInsert = false);
    iterator insert(iterator current, double time, AbstractPose* pose, bool doAdjustMaxTransitionTime = false);
    iterator erase(iterator it);

    // TODO: Implement the folloing function for efficient batch deletion.
    // void erase(const std::vector<iterator>& poses);

    iterator copyElement(iterator seekpos, const_iterator org, double offset = 0.0, CloneMap* cloneMap = nullptr);

    inline double beginningTime() { return poses.empty() ? 0.0 : poses.front().time(); }
    inline double endingTime() { return poses.empty() ? 0.0 : poses.back().time(); }

    void getDomain(double& out_lower, double& out_upper);

    SignalProxy<void(iterator, bool isMoving)> sigPoseInserted() {
        return sigPoseInserted_;
    }
    SignalProxy<void(iterator, bool isMoving)> sigPoseAboutToBeRemoved(){
        return sigPoseAboutToBeRemoved_;
    }
    SignalProxy<void(iterator)> sigPoseAboutToBeModified(){
        return sigPoseAboutToBeModified_;
    }
    SignalProxy<void(iterator)> sigPoseModified(){
        return sigPoseModified_;
    }

    void beginPoseModification(iterator it){
        sigPoseAboutToBeModified_(it);
    }

    void endPoseModification(iterator it){
        sigPoseModified_(it);
    }

    /// \todo Implement and use the followings. For example for loading a file.
    void blockSignals();
    void unblockSignals();

protected:
    virtual Referenced* doClone(CloneMap*) const override;
    
private:
    typedef std::list<SequentialPose> PoseList;
    PoseList poses;
    std::string name_;
    Signal<void(iterator, bool isMoving)> sigPoseInserted_;
    Signal<void(iterator, bool isMoving)> sigPoseAboutToBeRemoved_;
    Signal<void(iterator)> sigPoseAboutToBeModified_;
    Signal<void(iterator)> sigPoseModified_;
    std::string targetBodyName_;
    std::string errorMessage_;

    iterator insert(iterator current, double time, SequentialPose& pose, bool doAdjustMaxTransitionTime);
    
    friend class SequentialPose;
};

typedef ref_ptr<PoseSeq> PoseSeqPtr;

}

#endif
