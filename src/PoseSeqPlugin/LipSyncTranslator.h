/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_CHOREOGRAPHY_LIP_SYNC_TRANSLATOR_H_INCLUDED
#define CNOID_CHOREOGRAPHY_LIP_SYNC_TRANSLATOR_H_INCLUDED

#include <list>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class PoseSeq;

class CNOID_EXPORT LipSyncTranslator
{
public:

    enum LipShapeId {
        LS_A, LS_I, LS_U, LS_E, LS_O, LS_N,
        LS_a, LS_i, LS_u, LS_e, LS_o,
        NUM_LIP_SHAPES
    };

    struct Phoneme {
        double time;
        int shapeId;
    };

    typedef std::list<Phoneme>::iterator iterator;
    typedef std::list<Phoneme>::const_iterator const_iterator;

    LipSyncTranslator();

    void translatePoseSeq(PoseSeq& poseSeq);

    bool appendSyllable(double time, const std::string& syllable);

    bool exportSeqFileForFaceController(const std::string& filename);

    void enableMaxTransitionTime(bool on){
        isMaxTransitionTimeEnabled_ = on;
    }

    bool isMaxTransitionTimeEnabled() const {
        return isMaxTransitionTimeEnabled_;
    }

    void setMaxTransitionTime(double ttime) {
        maxTransitionTime_ = ttime;
    }

    double maxTransitionTime() const {
        return maxTransitionTime_;
    }
            
    void clear();

    inline bool empty() const {
        return seq.empty();
    }

    inline std::list<Phoneme>::size_type size() const {
        return seq.size();
    }

    inline iterator begin(){
        return seq.begin();
    }

    inline const_iterator begin() const {
        return seq.begin();
    }

    inline iterator end(){
        return seq.end();
    }

    inline const_iterator end() const {
        return seq.end();
    }

private:

    typedef std::list<Phoneme> PhonemeList;
    PhonemeList seq;

    bool isMaxTransitionTimeEnabled_;
    double maxTransitionTime_;
};
}

#endif
