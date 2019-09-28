/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "LipSyncTranslator.h"
#include "PronunSymbol.h"
#include "PoseSeq.h"
#include <fstream>

using namespace std;
using namespace cnoid;


LipSyncTranslator::LipSyncTranslator()
{
    isMaxTransitionTimeEnabled_ = false;
    maxTransitionTime_ = 0.2;
}


void LipSyncTranslator::clear()
{
    seq.clear();
}


void LipSyncTranslator::translatePoseSeq(PoseSeq& poseSeq)
{
    clear();

    for(PoseSeq::iterator poseIter = poseSeq.begin(); poseIter != poseSeq.end(); ++poseIter){
        PronunSymbolPtr symbol = poseIter->get<PronunSymbol>();
        if(symbol && !symbol->name().empty()){
            appendSyllable(poseIter->time(), symbol->name());
        }
    }
}


bool LipSyncTranslator::appendSyllable(double time, const std::string& syllable)
{
    if(syllable.empty()){
        return false;
    }

    int vowel = -1;

    switch(tolower(syllable[syllable.size()-1])){

    case 'a': vowel = LS_A; break;
    case 'i': vowel = LS_I; break;
    case 'u': vowel = LS_U; break;
    case 'e': vowel = LS_E; break;
    case 'o': vowel = LS_O; break;
    case 'n': vowel = LS_N; break;
    case ',': vowel = LS_N; break;
    case '.': vowel = LS_N; break;

    default: 
        break;
    }

    if(vowel < 0){
        return false;
    }

    Phoneme phoneme0;
    phoneme0.shapeId = -1;
    Phoneme phoneme1;
    phoneme1.shapeId = -1;

    if(vowel != LS_N && syllable.size() >= 2){
        int consonantChar = tolower(syllable[0]);
        if(consonantChar == 'm' || consonantChar == 'b' || consonantChar == 'p'){
            phoneme0.shapeId = LS_N;
        } else if(!seq.empty()){
            int prevVowel = seq.back().shapeId;
            if(vowel == prevVowel){
                phoneme0.shapeId = vowel + LS_a;
            }
        }
    }

    if(phoneme0.shapeId < 0){
        phoneme0.shapeId = vowel;
    } else {
        phoneme1.shapeId = vowel;
    }
    
    while(!seq.empty()){
        double prevTime = seq.back().time;
        double ttime = time - prevTime;
        if(ttime <= 0.0){
            seq.pop_back();
            continue;
        }
        if(isMaxTransitionTimeEnabled_ && (ttime > maxTransitionTime_)){
            seq.push_back(seq.back());
            seq.back().time = time - maxTransitionTime_;
        }
        break;
    }

    phoneme0.time = time;
    seq.push_back(phoneme0);

    if(phoneme1.shapeId >= 0){
        phoneme1.time = time + 0.05;
        seq.push_back(phoneme1);
    }

    return true;
}


bool LipSyncTranslator::exportSeqFileForFaceController(const std::string& filename)
{
    ofstream ofs(filename.c_str());

    for(iterator p = seq.begin(); p != seq.end(); ++p){
        ofs << p->time << ", ";
        switch (p->shapeId) {
        case LS_A: ofs << "a";  break;
        case LS_a: ofs << "a0"; break;
        case LS_I: ofs << "i";  break;
        case LS_i: ofs << "i0"; break;
        case LS_U: ofs << "u";  break;
        case LS_u: ofs << "u0"; break;
        case LS_E: ofs << "e";  break;
        case LS_e: ofs << "e0"; break;
        case LS_O: ofs << "o";  break;
        case LS_o: ofs << "o0"; break;
        case LS_N: ofs << "n";  break;
        default: break;
        }
        ofs << "\n";
    }

    ofs.close();

    return true;
}
