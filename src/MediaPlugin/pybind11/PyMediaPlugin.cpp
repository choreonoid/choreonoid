/**
  @author Shin'ichiro Nakaoka
*/

#include "../MediaUtil.h"
#include <cnoid/LazyCaller>
#include <pybind11/pybind11.h>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

bool pyPlayAudioFile(const string& filename, double volumeRatio)
{
    bool result = false;
#ifndef _WIN32
    callSynchronously([&](){ result = cnoid::playAudioFile(filename, volumeRatio); });
#endif
    return result;
}
    
PYBIND11_MODULE(MediaPlugin, m)
{
    m.doc() = "Choreonoid MediaPlugin module";
    m.def("playAudioFile", pyPlayAudioFile);
    m.def("playAudioFile", [](const string& filename){ pyPlayAudioFile(filename, -1.0); });
}

}
