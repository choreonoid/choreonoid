#include "../MediaUtil.h"
#include <cnoid/LazyCaller>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace {

bool pyPlayAudioFile(const string& filename, double volumeRatio)
{
    bool result = false;
#ifndef _WIN32
    callSynchronously([&](){ result = cnoid::playAudioFile(filename, volumeRatio); });
#endif
    return result;
}

NB_MODULE(MediaPlugin, m)
{
    m.doc() = "Choreonoid MediaPlugin module";
    m.def("playAudioFile", pyPlayAudioFile, nb::arg("filename"), nb::arg("volumeRatio") = -1.0);
}

}
