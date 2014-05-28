/**
  @author Shin'ichiro Nakaoka
*/

#include "../MediaUtil.h"
#include <cnoid/LazyCaller>
#include <boost/python.hpp>
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;

namespace {

    void pyPlayAudioFileMain(const std::string& filename, bool& out_result)
    {
        out_result = cnoid::playAudioFile(filename);
    }
    
    bool pyPlayAudioFile(const std::string& filename)
    {
        bool result;
        callSynchronously(boost::bind(pyPlayAudioFileMain, filename, boost::ref(result)));
        return result;
    }
}

BOOST_PYTHON_MODULE(Media)
{
    python::def("playAudioFile", pyPlayAudioFile);
}
