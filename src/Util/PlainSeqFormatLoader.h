/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PLAIN_SEQ_FILE_LOADER_H_INCLUDED
#define CNOID_UTIL_PLAIN_SEQ_FILE_LOADER_H_INCLUDED

#include <list>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PlainSeqFileLoader
{
public:
        
    typedef std::list< std::vector<double> > Seq;
    typedef Seq::iterator iterator;
        
    bool load(const std::string& filename);
        
    inline iterator begin() { return seq.begin(); }
    inline iterator end() { return seq.end(); }
        
    inline int numParts() { return numParts_; }
    inline int numFrames() { return numFrames_; }
    inline double timeStep() { return timeStep_; }

    const std::string& errorMessage();
        
private:
        
    Seq seq;
    int numParts_;
    int numFrames_;
    double timeStep_;
    std::string errorMessage_;
};
}

#endif
