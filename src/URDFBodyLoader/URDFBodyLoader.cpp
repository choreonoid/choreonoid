#include "URDFBodyLoader.h"

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <cnoid/BodyLoader>
#include <cnoid/NullOut>
#include <cnoid/SceneLoader>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <pugixml.hpp>

using namespace std;
using namespace cnoid;

namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {
struct Registration
{
    Registration()
    {
        BodyLoader::registerLoader({"urdf", "xacro"},
                                   []() -> AbstractBodyLoaderPtr {
                                       return make_shared<URDFBodyLoader>();
                                   });
    }
} registration;

}  // namespace

namespace cnoid {
class URDFBodyLoader::Impl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }

    Impl();
    bool load(Body* body, const string& filename);
};

}  // namespace cnoid

URDFBodyLoader::URDFBodyLoader()
{
    impl = new Impl;
}


URDFBodyLoader::Impl::Impl()
{
    os_ = &nullout();
}


URDFBodyLoader::~URDFBodyLoader()
{
    delete impl;
}


void URDFBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


bool URDFBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool URDFBodyLoader::Impl::load(Body* body, const string& filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    os() << "Load result: " << result.description() << endl;
    return true;
}
