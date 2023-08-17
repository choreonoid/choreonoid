#include <cnoid/URDFBodyWriter>
#include <cnoid/BodyLoader>
#include <cnoid/Body>

#include <boost/program_options.hpp>

#include <iostream>

using namespace cnoid;
using namespace boost::program_options;

int main(int argc, char **argv)
{

    options_description desc("$ body2urdf [options] inputfile.body");
    desc.add_options()
    ("robot-name,r", value<std::string>(), "robot name")
    ("output,O", value<std::string>(), "output file")
    ("mesh-file-prefix", value<std::string>()->default_value("/tmp/"), "prefix of mesh file (e.g. directory)")
    ("mesh-url-prefix", value<std::string>()->default_value("file:///tmp/"), "prefix of url refered in output URDF file")
    ("no-offset", "surpress output about link position ")
    ("no-geometry", "suppress output about geometry-info")
    ("no-xacro", "do not use xacro")
    ("export-devices", "export devices as link")
    ("verbose,v", "verbose message")
    ("help,h", "help")
    ;

    variables_map vm;
    auto const presult = parse_command_line(argc, argv, desc);
    store(presult, vm);
    notify(vm);

    if (vm.count("help") > 0) {
        std::cout << desc << std::endl;
        return 0;
    }

    std::string fname;
    for (auto const& str : collect_unrecognized(presult.options, include_positional)) {
        fname = str;
    }

    if(fname.empty()) {
        std::cerr << "Error! / please input filename" << std::endl;
        std::cerr << "$ body2urdf [options] inputfile.body" << std::endl;
        return -1;
    }
    std::string outputf;
    std::string rname;
    bool no_offset = false;
    bool no_geom   = false;
    bool no_xacro  = false;
    bool verbose   = false;
    bool export_devices = false;
    if (vm.count("output") > 0) {
        outputf = vm["option"].as<std::string>();
    }
    if (vm.count("robot-name") > 0) {
        rname = vm["robot-name"].as<std::string>();
    }
    if (vm.count("no-offset") > 0) {
        no_offset = true;
    }
    if (vm.count("no-geometry") > 0) {
        no_geom = true;
    }
    if (vm.count("no-xacro") > 0) {
        no_xacro = true;
    }
    if (vm.count("verbose") > 0) {
        verbose = true;
    }
    if (vm.count("export-devices") > 0) {
        export_devices = true;
    }
    if (verbose) {
        std::cerr << "fname: " << fname << std::endl;
        std::cerr << "robot name:     " << rname << std::endl;
        std::cerr << "no-offset:      "   << no_offset << std::endl;
        std::cerr << "no-geometry:    " << no_geom << std::endl;
        std::cerr << "no-xacro:       "    << no_xacro << std::endl;
        std::cerr << "export-devices: "    << export_devices << std::endl;
        std::cerr << "output: " << outputf << std::endl;
    }
    BodyLoader bl;
    if (verbose) {
        bl.setMessageSink(std::cerr);
        bl.setVerbose(true);
    }

    Body *bd = bl.load(fname);
    if (!bd) {
        std::cerr << "body open error! : " << fname << std::endl;
        return -1;
    }

    bd->updateLinkTree();
    bd->initializePosition();
    bd->calcForwardKinematics();

    URDFBodyWriter ubw;
    if (verbose) {
        ubw.setMessageSink(std::cerr);
        ubw.setVerbose(true);
    }
    ubw.setUseXacro(!no_xacro);
    ubw.setAddGeometry(!no_geom);
    ubw.setAddOffset(!no_offset);
    ubw.setExportDevices(export_devices);

    std::string fp_ = vm["mesh-file-prefix"].as<std::string>();
    ubw.setMeshFilePrefix(fp_);
    std::string up_ = vm["mesh-url-prefix"].as<std::string>();
    ubw.setMeshURLPrefix(up_);
    if(!rname.empty()){
        ubw.setRobotName(rname);
    }

    if (outputf.empty()) {
        std::string out;
        ubw.writeBodyAsString(bd, out);
        std::cout << out << std::endl;
    } else {
        bool res = ubw.writeBody(bd, outputf);
        if(!res) {
            std::cerr << "failed to output : " << outputf << std::endl;
            return -1;
        }
    }

    return 0;
}
