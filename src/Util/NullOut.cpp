/**
   @author Shin'ichiro Nakaoka
*/

#include "NullOut.h"
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>

using namespace std;
using namespace boost;

namespace {

class NullSink : public iostreams::sink
{
public:
    streamsize write(const char*, streamsize n) { return n; }
};

}


namespace cnoid {

//! \todo check if this is thread safe ?
std::ostream& nullout()
{
    static NullSink nullSink;
    static iostreams::stream_buffer<NullSink> sbuf(nullSink);
    static ostream os(&sbuf);
        
    return os;
}
}
