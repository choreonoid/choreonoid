# PhysX 5.6.1 bundled build for Choreonoid
#
# Provides the PhysX static libraries used by the Choreonoid PhysXPlugin.
# Only the modules actually linked by the plugin are built:
#     PhysXFoundation_static_64, PhysXCommon_static_64, PhysX_static_64,
#     PhysXExtensions_static_64, PhysXCooking_static_64, PhysXPvdSDK_static_64
# along with the internal object libraries they depend on:
#     LowLevel, LowLevelAABB, LowLevelDynamics, SceneQuery,
#     SimulationController, PhysXTask
#
# GPU, CharacterKinematic, Vehicle, and ImmediateMode are excluded.
# OmniPVD is disabled at compile time (PX_SUPPORT_OMNI_PVD=0), so the
# pvdruntime library is not needed.
#
# See CHOREONOID_README.md for the extraction procedure.

cmake_minimum_required(VERSION 3.16)

# Variables expected by the upstream cmake fragments.
set(PHYSX_ROOT_DIR     "${CMAKE_CURRENT_SOURCE_DIR}")
set(PHYSX_SOURCE_DIR   "${PHYSX_ROOT_DIR}/source")
set(PROJECT_CMAKE_FILES_DIR "source/compiler/cmake")

# Pick the upstream platform folder. Linux and Windows are supported.
if(WIN32)
    set(TARGET_BUILD_PLATFORM "windows")
elseif(UNIX AND NOT APPLE)
    set(TARGET_BUILD_PLATFORM "linux")
else()
    message(FATAL_ERROR "Bundled PhysX supports Linux and Windows only. "
        "Set ENABLE_PHYSX=OFF to skip the bundled build, or set ENABLE_PHYSX=OFF "
        "and BUILD_PHYSX_PLUGIN=ON with PHYSX_DIR pointing to a pre-built SDK.")
endif()

# Upstream expects a static build.
set(PX_GENERATE_STATIC_LIBRARIES ON)
set(PUBLIC_RELEASE 1)
set(PX_GENERATE_GPU_PROJECTS OFF)
set(PX_GENERATE_GPU_PROJECTS_ONLY OFF)
set(PX_EXPORT_LOWLEVEL_PDB OFF)

# All targets are built with -fPIC so they can be linked into CnoidPhysXPlugin
# (a shared library).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The upstream NvidiaBuildOptions.cmake insists on PX_OUTPUT_LIB_DIR. Bypass it
# entirely; we only need the *_POSTFIX machinery to make the output archive
# names end with "_static_64", matching the name used by PhysXPlugin.
set(CMAKE_DEBUG_POSTFIX   "_64")
set(CMAKE_RELEASE_POSTFIX "_64")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_64")
set(CMAKE_MINSIZEREL_POSTFIX "_64")

# Compiler flags / preprocessor definitions.
#
# These mirror upstream source/compiler/cmake/{linux,windows}/CMakeLists.txt
# trimmed to the release-only, CPU-only, non-OmniPVD build. The flags are
# applied per-target so they do not leak into the rest of Choreonoid.
#
# Note: strict-aliasing is intentionally *not* part of the Linux common flags.
# Upstream sets -fno-strict-aliasing on GCC but -fstrict-aliasing on Clang,
# so the choice is made per compiler below.
set(PHYSX_LINUX_COMMON_CXX_FLAGS
    -std=c++14
    -D_GLIBCXX_USE_CXX11_ABI=1
    -fno-rtti
    -fno-exceptions
    -ffunction-sections
    -fdata-sections
    -fvisibility=hidden
)

set(PHYSX_GCC_WARNING_FLAGS
    -Wno-address
    -Wno-aligned-new
    -Wno-array-bounds
    -Wno-class-memaccess
    -Wno-conversion-null
    # -Wno-format is intentionally omitted: it conflicts with Debian's
    # mandatory -Wformat -Werror=format-security packaging policy, and the
    # PhysX 5.6.1 sources we build do not actually emit -Wformat warnings
    # (we do not pass -Werror, so any future warning would be non-fatal).
    -Wno-format-overflow
    -Wno-invalid-offsetof
    -Wno-misleading-indentation
    -Wno-mismatched-new-delete
    -Wno-nonnull
    -Wno-nonnull-compare
    -Wno-pragmas
    -Wno-restrict
    -Wno-stringop-overflow
    -Wno-stringop-overread
    -Wno-subobject-linkage
    -Wno-template-id-cdtor
    -Wno-uninitialized
    -Wno-unused-but-set-variable
    -Wno-unused-function
    -Wno-unused-result
    -Wno-unknown-pragmas
    -Wno-use-after-free
)

# Upstream drives Clang with -Weverything and then suppresses the groups
# PhysX does not care about. We replicate that list so builds stay warning-
# clean in both bundled and non-bundled modes.
set(PHYSX_CLANG_WARNING_FLAGS
    -ferror-limit=0
    -Wall
    -Wextra
    -Weverything
    -Wno-unused-but-set-variable
    -Wno-switch-default
    -Wno-cast-qual
    -Wno-invalid-offsetof
    -Wno-unsafe-buffer-usage
    -Wno-alloca
    -Wno-atomic-implicit-seq-cst
    -Wno-c++98-compat-pedantic
    -Wno-c++98-compat
    -Wno-cast-align
    -Wno-conversion
    -Wno-covered-switch-default
    -Wno-deprecated
    -Wno-documentation-deprecated-sync
    -Wno-documentation-unknown-command
    -Wno-exit-time-destructors
    -Wno-extra-semi-stmt
    -Wno-float-equal
    -Wno-format-nonliteral
    -Wno-global-constructors
    -Wno-implicit-fallthrough
    -Wno-inconsistent-missing-destructor-override
    -Wno-inconsistent-missing-override
    -Wno-missing-noreturn
    -Wno-missing-prototypes
    -Wno-missing-variable-declarations
    -Wno-newline-eof
    -Wno-non-virtual-dtor
    -Wno-old-style-cast
    -Wno-padded
    -Wno-reserved-id-macro
    -Wno-suggest-destructor-override
    -Wno-suggest-override
    -Wno-switch-enum
    -Wno-undef
    -Wno-undefined-reinterpret-cast
    -Wno-unknown-warning-option
    -Wno-unreachable-code
    -Wno-unused-function
    -Wno-unused-macros
    -Wno-unused-member-function
    -Wno-unused-private-field
    -Wno-used-but-marked-unused
    -Wno-weak-template-vtables
    -Wno-weak-vtables
    -Wno-zero-as-null-pointer-constant
    -Wno-reserved-identifier
    -Wno-undefined-func-template
)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # GCC path - see upstream linux/CMakeLists.txt L129
    set(PHYSX_COMPILER_SPECIFIC_FLAGS -fno-strict-aliasing)
    set(PHYSX_WARNING_FLAGS ${PHYSX_GCC_WARNING_FLAGS})
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # Clang path - see upstream linux/CMakeLists.txt L122-127
    set(PHYSX_COMPILER_SPECIFIC_FLAGS -fstrict-aliasing -Wstrict-aliasing=2)
    if(NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0.0")
        list(APPEND PHYSX_COMPILER_SPECIFIC_FLAGS -ffp-exception-behavior=maytrap)
    else()
        list(APPEND PHYSX_COMPILER_SPECIFIC_FLAGS -Wno-shadow)
    endif()
    set(PHYSX_WARNING_FLAGS ${PHYSX_CLANG_WARNING_FLAGS})
else()
    set(PHYSX_COMPILER_SPECIFIC_FLAGS "")
    set(PHYSX_WARNING_FLAGS "")
endif()

# MSVC flags - see upstream windows/CMakeLists.txt L30-74.
# We deliberately do NOT include the upstream windows/CMakeLists.txt because
# it mutates CMAKE_CXX_FLAGS / CMAKE_SHARED_LINKER_FLAGS globally, which would
# leak into the rest of Choreonoid. Instead we apply the PhysX-specific flags
# via target_compile_options() below, matching the Linux approach.
#
# Intentionally omitted from upstream:
#   /WX   - Choreonoid builds third-party code without -Werror;
#   /d2Zi+ - internal debug-info tweak not needed for our build.
#   /std:c++14 - handled via set_target_properties(CXX_STANDARD 14) below,
#                which reliably wins over Choreonoid's /std:c++20.
set(PHYSX_MSVC_COMMON_CXX_FLAGS
    /MP
    /GF
    /GS-
    /Gd
    /fp:fast
    /Oy
    /W4
    /GR-
)

set(PHYSX_MSVC_WARNING_FLAGS
    /wd4514
    /wd4820
    /wd4127
    /wd4710
    /wd4711
    /wd4577
    /wd4996
)

# Force OmniPVD off: the runtime library (pvdruntime) is not bundled.
# PUBLIC_RELEASE=1 also disables the internal/device directory, which is not
# part of the public source drop.
set(PHYSX_LINUX_COMPILE_DEFS
    "PX_PUBLIC_RELEASE=1"
    "DISABLE_CUDA_PHYSX"
)
set(PHYSX_LINUX_DEBUG_COMPILE_DEFS
    "PX_DEBUG=1"
    "PX_CHECKED=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_LINUX_CHECKED_COMPILE_DEFS
    "NDEBUG"
    "PX_CHECKED=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_LINUX_PROFILE_COMPILE_DEFS
    "NDEBUG"
    "PX_PROFILE=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_LINUX_RELEASE_COMPILE_DEFS
    "NDEBUG"
    "PX_SUPPORT_PVD=0"
    "PX_SUPPORT_OMNI_PVD=0"
)

# Windows counterparts - see upstream windows/CMakeLists.txt L124-135.
# The upstream file caches these as CACHE INTERNAL; we keep them wrapper-scoped
# and also set PHYSX_LIBTYPE_DEFS, which windows/*.cmake reads from but which
# upstream only populates when windows/CMakeLists.txt is included.
set(PHYSX_WINDOWS_COMPILE_DEFS
    "WIN32"
    "_CRT_SECURE_NO_DEPRECATE"
    "_CRT_NONSTDC_NO_DEPRECATE"
    "_WINSOCK_DEPRECATED_NO_WARNINGS"
    "PX_PUBLIC_RELEASE=1"
    "DISABLE_CUDA_PHYSX"
)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    list(APPEND PHYSX_WINDOWS_COMPILE_DEFS "WIN64")
endif()
set(PHYSX_WINDOWS_DEBUG_COMPILE_DEFS
    "PX_DEBUG=1"
    "PX_CHECKED=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_WINDOWS_CHECKED_COMPILE_DEFS
    "PX_CHECKED=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_WINDOWS_PROFILE_COMPILE_DEFS
    "PX_PROFILE=1"
    "PX_NVTX=0"
    "PX_SUPPORT_PVD=1"
    "PX_SUPPORT_OMNI_PVD=0"
)
set(PHYSX_WINDOWS_RELEASE_COMPILE_DEFS
    "PX_SUPPORT_PVD=0"
    "PX_SUPPORT_OMNI_PVD=0"
)
if(WIN32)
    set(PHYSX_LIBTYPE_DEFS "PX_PHYSX_STATIC_LIB" CACHE INTERNAL "PhysX lib type defs")
endif()

# Per-module configuration variables referenced by the upstream *.cmake files.
# Most of these are written as if the user had invoked the upstream build with
# PX_GENERATE_STATIC_LIBRARIES=ON, PUBLIC_RELEASE=1, and GPU projects disabled.
set(PHYSXFOUNDATION_LIBTYPE STATIC)
set(PHYSXTASK_LIBTYPE       OBJECT)
set(PHYSXPVDSDK_LIBTYPE     STATIC)
set(LOWLEVEL_LIBTYPE        OBJECT)
set(LOWLEVELAABB_LIBTYPE    OBJECT)
set(LOWLEVELDYNAMICS_LIBTYPE OBJECT)
set(SCENEQUERY_LIBTYPE      OBJECT)
set(SIMULATIONCONTROLLER_LIBTYPE OBJECT)
set(PHYSXCOMMON_LIBTYPE     STATIC)
set(PHYSXCOOKING_LIBTYPE    STATIC)
set(PHYSX_LIBTYPE           STATIC)
set(PHYSXEXTENSIONS_LIBTYPE STATIC)

# Per-module preprocessor definitions. Config-specific ones use generator
# expressions so they follow CMAKE_BUILD_TYPE. The PhysX internal build-type
# names (Debug/Checked/Profile/Release) are mapped from CMake's standard
# configs (Debug/Release/RelWithDebInfo/MinSizeRel):
#   CMake Debug            -> PhysX Debug
#   CMake RelWithDebInfo   -> PhysX Profile
#   CMake Release          -> PhysX Release
#   CMake MinSizeRel       -> PhysX Release
#   (empty)                -> PhysX Release
if(WIN32)
    set(_PHYSX_PLATFORM_COMPILE_DEFS       ${PHYSX_WINDOWS_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_DEBUG_DEFS         ${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_PROFILE_DEFS       ${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_RELEASE_DEFS       ${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS})
else()
    set(_PHYSX_PLATFORM_COMPILE_DEFS       ${PHYSX_LINUX_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_DEBUG_DEFS         ${PHYSX_LINUX_DEBUG_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_PROFILE_DEFS       ${PHYSX_LINUX_PROFILE_COMPILE_DEFS})
    set(_PHYSX_PLATFORM_RELEASE_DEFS       ${PHYSX_LINUX_RELEASE_COMPILE_DEFS})
endif()

set(_PHYSX_CONFIG_DEFS
    "$<$<CONFIG:Debug>:${_PHYSX_PLATFORM_DEBUG_DEFS}>"
    "$<$<CONFIG:RelWithDebInfo>:${_PHYSX_PLATFORM_PROFILE_DEFS}>"
    "$<$<CONFIG:MinSizeRel>:${_PHYSX_PLATFORM_RELEASE_DEFS}>"
    "$<$<CONFIG:Release>:${_PHYSX_PLATFORM_RELEASE_DEFS}>"
    "$<$<CONFIG:>:${_PHYSX_PLATFORM_RELEASE_DEFS}>"
)

set(PHYSXFOUNDATION_COMPILE_DEFS   ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PHYSXTASK_COMPILE_DEFS         ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PHYSXPVDSDK_COMPILE_DEFS       ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(LOWLEVEL_COMPILE_DEFS          ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(LOWLEVELAABB_COMPILE_DEFS      ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(LOWLEVELDYNAMICS_COMPILE_DEFS  ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(SCENEQUERY_COMPILE_DEFS        ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(SIMULATIONCONTROLLER_COMPILE_DEFS ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PXCOMMON_COMPILE_DEFS          ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PHYSXCOOKING_COMPILE_DEFS      ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PHYSX_COMPILE_DEFS             ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})
set(PHYSXEXTENSIONS_COMPILE_DEFS   ${_PHYSX_PLATFORM_COMPILE_DEFS} PX_PHYSX_STATIC_LIB ${_PHYSX_CONFIG_DEFS})

# Platform-specific include directories used by the upstream fragments.
# Only PhysXFoundation has platform-specific headers (foundation/linux or
# foundation/windows); the other modules keep these empty.
if(WIN32)
    set(PHYSXFOUNDATION_PLATFORM_INCLUDES "${PHYSX_ROOT_DIR}/include/foundation/windows")
else()
    set(PHYSXFOUNDATION_PLATFORM_INCLUDES "${PHYSX_ROOT_DIR}/include/foundation/linux")
endif()
set(PHYSXTASK_PLATFORM_INCLUDES "")
set(PHYSXPVDSDK_PLATFORM_INCLUDES "")
set(LOWLEVEL_PLATFORM_INCLUDES "")
set(LOWLEVELAABB_PLATFORM_INCLUDES "")
set(LOWLEVELDYNAMICS_PLATFORM_INCLUDES "")
set(SCENEQUERY_PLATFORM_INCLUDES "")
set(SIMULATIONCONTROLLER_PLATFORM_INCLUDES "")
set(PXCOMMON_PLATFORM_INCLUDES "")
set(PHYSXCOOKING_PLATFORM_INCLUDES "")
set(PHYSX_PLATFORM_INCLUDES "")
set(PHYSXEXTENSIONS_PLATFORM_INCLUDES "")

# Platform-specific linked libraries. Linux needs libdl for dlopen(); Windows
# auto-links kernel32/user32/ws2_32 via the /DEFAULTLIB pragmas emitted by the
# Windows system headers, so nothing extra is required here.
set(PHYSXFOUNDATION_PLATFORM_LINKED_LIBS "")
set(PHYSXPVDSDK_PLATFORM_LINKED_LIBS "")
set(PXCOMMON_PLATFORM_LINKED_LIBS "")
set(PHYSXCOOKING_PLATFORM_LINKED_LIBS "")
set(PHYSX_PRIVATE_PLATFORM_LINKED_LIBS "")
if(WIN32)
    set(PHYSX_PLATFORM_LINKED_LIBS "")
else()
    set(PHYSX_PLATFORM_LINKED_LIBS dl)
endif()
set(PHYSXEXTENSIONS_PRIVATE_PLATFORM_LINKED_LIBS "")

# On Linux the upstream linux/*.cmake fragments are NOT included (the Linux
# wrapper inlines everything), so the variables below must be pre-seeded to
# empty values that the upstream PhysXFoundation.cmake etc. will reference.
#
# On Windows we DO include the upstream windows/*.cmake fragments, and those
# are responsible for populating PHYSXFOUNDATION_PLATFORM_FILES,
# PHYSX_PLATFORM_SRC_FILES, etc. Seeding them empty here would wipe out the
# fragments' work, so on Windows we skip this block.
if(NOT WIN32)
    set(PHYSXFOUNDATION_PLATFORM_FILES "")
    set(PHYSXFOUNDATION_RESOURCE_FILE "")
    set(PHYSXPVDSDK_PLATFORM_FILES "")
    set(PXCOMMON_PLATFORM_SRC_FILES "")
    set(PHYSXCOOKING_PLATFORM_SRC_FILES "")
    set(PHYSX_PLATFORM_SRC_FILES "")
    set(PHYSXEXTENSIONS_PLATFORM_SRC_FILES "")
endif()

# Linker-flag variables referenced by both linux/ and windows/ fragments.
# These remain empty across platforms for our release-style CPU-only build.
set(PHYSX_PLATFORM_LINK_FLAGS "")
set(PHYSX_PLATFORM_LINK_FLAGS_DEBUG "")
set(PHYSX_PLATFORM_LINK_FLAGS_CHECKED "")
set(PHYSX_PLATFORM_LINK_FLAGS_PROFILE "")
set(PHYSX_PLATFORM_LINK_FLAGS_RELEASE "")
set(PXCOMMON_PLATFORM_LINK_FLAGS "")
set(PXCOMMON_PLATFORM_LINK_FLAGS_DEBUG "")
set(PXCOMMON_PLATFORM_LINK_FLAGS_CHECKED "")
set(PXCOMMON_PLATFORM_LINK_FLAGS_PROFILE "")
set(PXCOMMON_PLATFORM_LINK_FLAGS_RELEASE "")
set(PHYSXCOOKING_PLATFORM_LINK_FLAGS "")
set(PHYSXCOOKING_PLATFORM_LINK_FLAGS_DEBUG "")
set(PHYSXCOOKING_PLATFORM_LINK_FLAGS_CHECKED "")
set(PHYSXCOOKING_PLATFORM_LINK_FLAGS_PROFILE "")
set(PHYSXCOOKING_PLATFORM_LINK_FLAGS_RELEASE "")
set(PHYSXEXTENSIONS_PLATFORM_LINK_FLAGS "")
set(PHYSXEXTENSIONS_PLATFORM_LINK_FLAGS_DEBUG "")
set(PHYSXEXTENSIONS_PLATFORM_LINK_FLAGS_CHECKED "")
set(PHYSXEXTENSIONS_PLATFORM_LINK_FLAGS_PROFILE "")
set(PHYSXEXTENSIONS_PLATFORM_LINK_FLAGS_RELEASE "")
set(LOWLEVEL_PLATFORM_LINK_FLAGS "")
set(LOWLEVELAABB_PLATFORM_LINK_FLAGS "")
set(LOWLEVELDYNAMICS_PLATFORM_LINK_FLAGS "")
set(SCENEQUERY_PLATFORM_LINK_FLAGS "")
set(SIMULATIONCONTROLLER_PLATFORM_LINK_FLAGS "")

# Pull in the upstream module definitions in dependency order. The upstream
# files themselves call add_library() and target_link_libraries(), which is
# exactly what we want.
set(_CMAKE_FRAG_DIR "${PHYSX_SOURCE_DIR}/compiler/cmake")
include("${_CMAKE_FRAG_DIR}/PhysXFoundation.cmake")
include("${_CMAKE_FRAG_DIR}/PhysXTask.cmake")
include("${_CMAKE_FRAG_DIR}/LowLevel.cmake")
include("${_CMAKE_FRAG_DIR}/LowLevelAABB.cmake")
include("${_CMAKE_FRAG_DIR}/LowLevelDynamics.cmake")
include("${_CMAKE_FRAG_DIR}/SceneQuery.cmake")
include("${_CMAKE_FRAG_DIR}/SimulationController.cmake")
include("${_CMAKE_FRAG_DIR}/PhysXPvdSDK.cmake")
include("${_CMAKE_FRAG_DIR}/PhysXCommon.cmake")
include("${_CMAKE_FRAG_DIR}/PhysXCooking.cmake")
include("${_CMAKE_FRAG_DIR}/PhysX.cmake")
include("${_CMAKE_FRAG_DIR}/PhysXExtensions.cmake")

# Apply Choreonoid-scoped compile options and warning suppressions to every
# PhysX target. Doing this here keeps the upstream fragments untouched.
set(_PHYSX_TARGETS
    PhysXFoundation
    PhysXTask
    LowLevel
    LowLevelAABB
    LowLevelDynamics
    SceneQuery
    SimulationController
    PhysXPvdSDK
    PhysXCommon
    PhysXCooking
    PhysX
    PhysXExtensions
)
if(WIN32)
    set(_PHYSX_TARGET_FLAGS
        ${PHYSX_MSVC_COMMON_CXX_FLAGS}
        ${PHYSX_MSVC_WARNING_FLAGS}
    )
else()
    set(_PHYSX_TARGET_FLAGS
        ${PHYSX_LINUX_COMMON_CXX_FLAGS}
        ${PHYSX_COMPILER_SPECIFIC_FLAGS}
        ${PHYSX_WARNING_FLAGS}
    )
endif()

foreach(_t ${_PHYSX_TARGETS})
    target_compile_options(${_t} PRIVATE ${_PHYSX_TARGET_FLAGS})
    # Pin each PhysX target to C++14. PhysX's PX_NOCOPY macro and some of its
    # enum class forward-declarations do not compile as C++20, so we cannot
    # let Choreonoid's global /std:c++20 (or -std=c++20) apply here. Using a
    # target property (instead of appending /std:c++14 to the flag list) makes
    # CMake emit the flag after the global one, so the PhysX setting wins
    # deterministically across generators.
    set_target_properties(${_t} PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    # Treat the PhysX public headers as system headers so that downstream
    # code (PhysXPlugin, and anything else that links these targets) does
    # not inherit PhysX's own warning noise (for example
    # -Winconsistent-missing-override on Clang).
    get_target_property(_ifaces ${_t} INTERFACE_INCLUDE_DIRECTORIES)
    if(_ifaces)
        set_target_properties(${_t} PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_ifaces}"
        )
    endif()
endforeach()

# Rename the STATIC archives so they land as libPhysX*_static_64.a on disk.
# PhysXPlugin expects those filenames when linking against a pre-built tree.
# Because PhysXTask/LowLevel/SceneQuery/SimulationController/LowLevelAABB/
# LowLevelDynamics are OBJECT libraries, this only applies to the STATICs.
foreach(_t PhysXFoundation PhysXPvdSDK PhysXCommon PhysXCooking PhysX PhysXExtensions)
    set_target_properties(${_t} PROPERTIES
        OUTPUT_NAME ${_t}_static
    )
endforeach()

# Provide alias targets with the full "_static_64" name so downstream code
# (PhysXPlugin in particular) can link using the same names in both the
# bundled and the external-build paths.
foreach(_t PhysXFoundation PhysXPvdSDK PhysXCommon PhysXCooking PhysX PhysXExtensions)
    if(NOT TARGET ${_t}_static_64)
        add_library(${_t}_static_64 ALIAS ${_t})
    endif()
endforeach()
