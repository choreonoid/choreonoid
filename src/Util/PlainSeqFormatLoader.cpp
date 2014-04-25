/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PlainSeqFormatLoader.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


bool PlainSeqFileLoader::load(const std::string& filename)
{
    ifstream is(filename.c_str());
  
    string errorMessageBase("\"");
    errorMessageBase += filename + "\"";
  
    if(!is){
        errorMessage_ = errorMessageBase + " cannot be opened.";
        return false;
    }
  
    size_t nColumns = 0;
    size_t nLines = 0;
  
    typedef char_separator<char> Separator;
    typedef tokenizer<Separator> Tokenizer;
    Separator sep(" \t\r\n");
  
    seq.clear();
  
    string line;
    while(getline(is, line)){
        nLines++;
        seq.push_back(vector<double>(nColumns));
        vector<double>& v = seq.back();
        v.clear();
        Tokenizer tokens(line, sep);
        for(Tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it){
            v.push_back(lexical_cast<double>(*it));
        }
    
        if(nColumns == 0){
            nColumns = v.size();
        } else if(v.size() != nColumns){
            errorMessage_ = errorMessageBase + " contains different size columns";
            return false;
        }
    }

    if(nColumns < 2 || nLines < 1){
        errorMessage_ = errorMessageBase + ": Empty sequence.";
        return false;
    }
  
    numParts_ = nColumns - 1;
    numFrames_ = nLines;

    if(numFrames_ >= 2){
        iterator it = seq.begin();
        double time0 = (*it)[0];
        double time1 = (*(++it))[0];
        timeStep_ = time1 - time0;
        if(timeStep_ <= 0.0){
            errorMessage_ = errorMessageBase + ": Time values are not arranged.";
            return false;
        }
    } else {
        timeStep_ = 0.01;
    }
  
    return true;
}


const std::string& PlainSeqFileLoader::errorMessage()
{
    return errorMessage_;
}
