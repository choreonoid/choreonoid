/**
  @author Shin'ichiro Nakaoka
*/

#include "../MediaUtil.h"
#include <cnoid/LazyCaller>
#include <boost/python.hpp>

using namespace cnoid;
namespace python = boost::python;

namespace {

void pyPlayAudioFileMain(const std::string& filename, double volumeRatio, bool& out_result)
{
#ifndef WIN32
    out_result = cnoid::playAudioFile(filename, volumeRatio);
#endif
}
    
bool pyPlayAudioFile1(const std::string& filename)
{
    bool result;
    callSynchronously([&](){ pyPlayAudioFileMain(filename, -1.0, result); });
    return result;
}

bool pyPlayAudioFile2(const std::string& filename, double volumeRatio)
{
    bool result;
    callSynchronously([&](){ pyPlayAudioFileMain(filename, volumeRatio, result); });
    return result;
}


BOOST_PYTHON_MODULE(MediaPlugin)
{
    python::def("playAudioFile", pyPlayAudioFile1);
    python::def("playAudioFile", pyPlayAudioFile2);
}

}
