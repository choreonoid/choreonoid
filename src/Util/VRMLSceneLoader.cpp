/*!
  @author Shin'ichiro Nakaoka
*/

#include "VRMLSceneLoader.h"
#include "SceneLoader.h"
#include "VRMLParser.h"
#include "VRMLToSGConverter.h"
#include "EasyScanner.h"
#include "NullOut.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "wrl",
            []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<VRMLSceneLoader>(); });
    }
} registration;

}

namespace cnoid {

class VRMLSceneLoaderImpl
{
public:
    VRMLParser parser;
    VRMLToSGConverter converter;
    ostream* os_;

    VRMLSceneLoaderImpl();
    ostream& os() { return *os_; }
    SgNode* load(const std::string& filename);
};

}
    

VRMLSceneLoader::VRMLSceneLoader()
{
    impl = new VRMLSceneLoaderImpl;
}


VRMLSceneLoaderImpl::VRMLSceneLoaderImpl()
{
    os_ = &nullout();
}


VRMLSceneLoader::~VRMLSceneLoader()
{
    delete impl;
}


void VRMLSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->parser.setMessageSink(os);
    impl->converter.setMessageSink(os);
}
    

SgNode* VRMLSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* VRMLSceneLoaderImpl::load(const std::string& filename)
{
    converter.clearConvertedNodeMap();

    SgGroupPtr top = new SgGroup;

    try {
        parser.load(filename);
        
        while(VRMLNodePtr vrml = parser.readNode()){
            SgNodePtr node = converter.convert(vrml);
            if(node){
                top->addChild(node);
            }
        }
        parser.checkEOF();

    } catch(EasyScanner::Exception& ex){
        os() << ex.getFullMessage() << endl;
        return 0;
    }

    if(top->empty()){
        os() << format(_("VRML file \"{}\" does not have any valid entity."), filename) << endl;
        return 0;
    }

    if(top->numChildren() == 1){
        return top->child(0);
    }

    return top.retn();
}
