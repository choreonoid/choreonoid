/**
  @author Shin'ichiro Nakaoka
*/

#include "../MediaUtil.h"
#include <cnoid/LazyCaller>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace cnoid;

namespace {

void pyPlayAudioFileMain(const std::string& filename, double volumeRatio, bool& out_result)
{
#ifndef WIN32
    out_result = cnoid::playAudioFile(filename, volumeRatio);
#endif
}
    
PYBIND11_PLUGIN(Media)
{
    py::module m("Media", "Media Python Module");

    m.def("playAudioFile", [](const std::string& filename, double volumeRatio) {
        bool result;
        callSynchronously([&](){ pyPlayAudioFileMain(filename,volumeRatio, result); });
        return result;
    }, py::arg("filename"), py::arg("volumeRation")=-1.0 );

    return m.ptr();
}

}
