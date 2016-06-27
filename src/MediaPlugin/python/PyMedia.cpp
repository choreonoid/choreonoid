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

void pyPlayAudioFileMain(const std::string& filename, double volumeRatio, bool& out_result)
{
#ifndef WIN32
    out_result = cnoid::playAudioFile(filename, volumeRatio);
#endif
}
    
bool pyPlayAudioFile1(const std::string& filename)
{
    bool result;
    callSynchronously(boost::bind(pyPlayAudioFileMain, filename, -1.0, boost::ref(result)));
    return result;
}

bool pyPlayAudioFile2(const std::string& filename, double volumeRatio)
{
    bool result;
    callSynchronously(boost::bind(pyPlayAudioFileMain, filename, volumeRatio, boost::ref(result)));
    return result;
}


BOOST_PYTHON_MODULE(Media)
{
    python::def("playAudioFile", pyPlayAudioFile1);
    python::def("playAudioFile", pyPlayAudioFile2);
}

}
