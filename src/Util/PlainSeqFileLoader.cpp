#include "PlainSeqFileLoader.h"
#include "UTF8.h"
#include "Format.h"
#include <cnoid/Tokenizer>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;


bool PlainSeqFileLoader::load(const std::string& filename, std::ostream& os)
{
    ifstream is(fromUTF8(filename).c_str());
  
    if(!is){
        os << formatR(_("\"{}\" cannot be opened."), filename) << endl;
        return false;
    }
  
    size_t nColumns = 0;
    size_t nLines = 0;
    seq.clear();
    string line;
    Tokenizer<CharSeparator<char>> tokens(CharSeparator<char>(" \t\r\n"));
    
    while(getline(is, line)){
        nLines++;
        seq.push_back(vector<double>(nColumns));
        vector<double>& v = seq.back();
        v.clear();

        tokens.assign(line);
        for(auto& token : tokens){
            v.push_back(std::stod(token));
        }

        if(nColumns == 0){
            nColumns = v.size();
        } else if(v.size() != nColumns){
            os << formatR(_("\"{}\" contains different size columns."), filename) << endl;
            return false;
        }
    }

    if(nColumns < 2 || nLines < 1){
        os << formatR(_("\"{}\": Empty sequence."), filename) << endl;
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
            os << formatR(_("\"{}\": Time values are not arranged."), filename) << endl;
            return false;
        }
    } else {
        timeStep_ = 0.01;
    }
  
    return true;
}
