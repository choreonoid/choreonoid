#include "NullOut.h"
#include <streambuf>
#include <ostream>

namespace {

class null_streambuf : public std::basic_streambuf<char>
{
public:
};

}


namespace cnoid {

std::ostream& nullout()
{
    static null_streambuf sbuf;
    static std::ostream os(&sbuf);
    return os;
}

}
