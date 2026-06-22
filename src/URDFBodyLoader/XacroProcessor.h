#ifndef CNOID_URDF_BODY_LOADER_XACRO_PROCESSOR_H
#define CNOID_URDF_BODY_LOADER_XACRO_PROCESSOR_H

#include <iosfwd>
#include <string>
#include <vector>

namespace cnoid {

// Returns local ROS package search directories inferred from the input xacro/URDF path.
std::vector<std::string> findLocalPackageSearchDirectories(const std::string& filename);

// Expands a xacro file into URDF XML and reports xacro diagnostics through the given stream.
bool expandXacro(
    const std::string& filename,
    const std::string& executableDir,
    const std::vector<std::string>& localPackageSearchDirectories,
    std::ostream& os,
    std::string& out_urdfContent,
    std::string& out_errorOutput);

}

#endif
