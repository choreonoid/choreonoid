/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PLAIN_SEQ_FILE_LOADER_H
#define CNOID_UTIL_PLAIN_SEQ_FILE_LOADER_H

#include "NullOut.h"
#include <list>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PlainSeqFileLoader
{
public:
    typedef std::list<std::vector<double>> Seq;
    typedef Seq::iterator iterator;

    bool load(const std::string& filename, std::ostream& os = nullout());
        
    inline iterator begin() { return seq.begin(); }
    inline iterator end() { return seq.end(); }
        
    inline int numParts() { return numParts_; }
    inline int numFrames() { return numFrames_; }
    inline double timeStep() { return timeStep_; }

private:
    Seq seq;
    int numParts_;
    int numFrames_;
    double timeStep_;
};

}

#endif
