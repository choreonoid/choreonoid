# PhysX 5.6.1 bundled for Choreonoid

This directory contains a subset of NVIDIA-Omniverse/PhysX intended for
linking into Choreonoid's PhysXPlugin. It is built by Choreonoid's CMake
when `ENABLE_PHYSX=ON` (the default on all platforms). Supported hosts:

- Linux with GCC 8+ or Clang 10+
- Windows with Visual Studio 2019 or 2022 (x64)

## Extraction base

- Upstream: https://github.com/NVIDIA-Omniverse/PhysX
- Commit: `1097800e83d908b997cbf3957bab788d5a7f1623` (2025-09-16,
  "Update PhysX to 51c1f783").
- Equivalent tag: the `physx/` subtree of `107.3-omni-and-physx-5.6.1`
  (`09ff24f3`, 2025-10-21) matches this commit exactly, so `1097800e`
  is the effective "final form" of PhysX SDK 5.6.1. The NVIDIA
  PhysX SDK standalone tag `107.3-physx-5.6.1` (`5ca9f472`, 2025-07-22)
  predates a round of solver / articulation improvements that are
  included here.

Since NVIDIA's focus has shifted to the Omniverse-oriented `ovphysx`
wrapper, the PhysX SDK tag cadence has become irregular. When updating,
pick a commit that touches `physx/` meaningfully rather than a tag name.

## What is included

- `include/` - upstream `physx/include/`, with the
  `include/characterkinematic/` and `include/vehicle2/` subdirectories
  removed (see "What is excluded" below). The rest is kept verbatim,
  including a few GPU-related public headers (`include/gpu/`,
  `include/cudamanager/`, `include/PxDirectGPUAPI.h`,
  `include/PxParticleGpu.h`, `include/extensions/PxCudaHelpersExt.h`)
  that cannot be removed because they are `#include`d unconditionally
  by public PhysX headers (for example `PxPBDParticleSystem.h` pulls in
  `cudamanager/PxCudaTypes.h`). At compile time their bodies are still
  disabled by `PX_SUPPORT_GPU_PHYSX=0`.
- `include/PxPhysicsAPI.h` is a modified copy of the upstream file:
  the `#include` lines that pulled in `characterkinematic/*` and
  `vehicle2/*` have been removed so that those subdirectories can be
  dropped.
- `source/foundation/`, `source/common/`, `source/geomutils/`,
  `source/physx/`, `source/physxextensions/`, `source/physxcooking/`,
  `source/pvd/`, `source/task/`, `source/lowlevel/`, `source/lowlevelaabb/`,
  `source/lowleveldynamics/`, `source/scenequery/`,
  `source/simulationcontroller/`, `source/physxmetadata/`,
  `source/filebuf/`, `source/immediatemode/`.
- `source/physxgpu/include/PxPhysXGpu.h` only - the only GPU-related
  header that is `#include`d outside `PX_SUPPORT_GPU_PHYSX` guards
  from non-GPU compilation units (notably `NpImmediateMode.cpp`).
  `source/physxgpu/src/` is not needed and is omitted.
- `source/compiler/cmake/` - upstream CMake fragments. These are
  included verbatim from `CMakeLists.txt` below. (Note: the fragments
  for `PhysXCharacterKinematic` and `PhysXVehicle` are not kept, since
  the corresponding source trees are excluded; the `modules/`
  subdirectory is kept as a reference for future version bumps even
  though Choreonoid does not `include()` any of those helper scripts.)
- `source/compiler/windows/resource/` - upstream Windows resource
  scripts (`PhysXFoundation.rc`, `PhysX.rc`, `PhysXCommon.rc`,
  `PhysXCooking.rc`, `resource.h`). The upstream `windows/*.cmake`
  fragments splice these into `*_PLATFORM_FILES` unconditionally, so
  they must be present even for the static build. `PhysXGpu.rc`,
  `PhysXCharacterKinematic.rc` and `PhysXVehicle*.rc` are not kept
  because the matching modules are removed.
- `LICENSE.md`, `README.md`, `CHANGELOG.md`, `version.txt` - upstream
  redistribution requirements.

## What is excluded

- GPU runtime implementations: `source/gpuarticulation/`,
  `source/gpubroadphase/`, `source/gpunarrowphase/`,
  `source/gpusimulationcontroller/`, `source/gpusolver/`,
  `source/gpucommon/`, `source/cudamanager/`, `source/physxgpu/src/`,
  `source/physx/src/opensource/`. All GPU code paths are disabled via
  `DISABLE_CUDA_PHYSX` / `PX_SUPPORT_GPU_PHYSX=0`.
- `source/physxcharacterkinematic/`, `source/physxvehicle/`,
  `include/characterkinematic/`, `include/vehicle2/` - unused by
  PhysXPlugin. `PxPhysicsAPI.h` has been edited to not `#include`
  these directories.
- `source/physxextensions/src/ExtDeformableSkinning.cpp`,
  `ExtParticleClothCooker.cpp`, `ExtParticleExt.cpp` - upstream only
  compiles these when GPU projects are generated.
- `snippets/`, `samples/`, `documentation/`, `tools/`, `buildtools/` -
  examples and build infrastructure; not needed at link time.
- `pvdruntime/` - OmniPVD runtime. Disabled by setting
  `PX_SUPPORT_OMNI_PVD=0` across all configurations.
- `dependencies.xml` - upstream packman manifest; Choreonoid bundles
  what it needs directly.

The removed files are still present in the corresponding upstream
commit and can be re-introduced by re-copying from `physx/` if a
future PhysX update starts referencing them from non-GPU code paths.

## CMake layout

Choreonoid's `thirdparty/physx-5.6.1/CMakeLists.txt` is a thin wrapper
that:

1. Sets `TARGET_BUILD_PLATFORM` to `linux` or `windows` based on the
   host OS, then sets the variables the upstream
   `source/compiler/cmake/*.cmake` fragments expect (`PHYSX_ROOT_DIR`,
   `PROJECT_CMAKE_FILES_DIR`, `PX_GENERATE_STATIC_LIBRARIES`,
   `PHYSXFOUNDATION_LIBTYPE` and friends, the platform-specific
   `PHYSX_LINUX_*` / `PHYSX_WINDOWS_*` `*_COMPILE_DEFS` lists, and
   `PHYSXFOUNDATION_PLATFORM_INCLUDES` pointing at the matching
   `include/foundation/{linux,windows}`).
2. `include()`s the upstream `.cmake` files in dependency order:
   `PhysXFoundation`, `PhysXTask`, `LowLevel`, `LowLevelAABB`,
   `LowLevelDynamics`, `SceneQuery`, `SimulationController`,
   `PhysXPvdSDK`, `PhysXCommon`, `PhysXCooking`, `PhysX`,
   `PhysXExtensions`. Each common fragment in turn `include()`s its
   platform-specific counterpart (`linux/` or `windows/`) and then
   calls `add_library()`. Upstream `windows/CMakeLists.txt` is
   **not** included, because it rewrites `CMAKE_CXX_FLAGS` and
   `CMAKE_SHARED_LINKER_FLAGS` globally; instead the wrapper supplies
   the `PHYSX_WINDOWS_*_COMPILE_DEFS` variables that `windows/*.cmake`
   reads, plus `PHYSX_LIBTYPE_DEFS=PX_PHYSX_STATIC_LIB`.
3. Adds per-target `target_compile_options()` adapted from upstream:
   - On GCC / Clang: `-fno-rtti`, `-fno-exceptions`,
     `-ffunction-sections`, `-fdata-sections`, `-fvisibility=hidden`,
     plus the full warning suppression lists. GCC uses
     `-fno-strict-aliasing`; Clang uses
     `-fstrict-aliasing -Wstrict-aliasing=2 -ffp-exception-behavior=maytrap`
     plus `-Weverything` with the upstream `-Wno-*` set.
   - On MSVC: `/MP /GF /GS- /Gd /fp:fast /Oy /W4 /GR-` plus
     `/wd4514 /wd4820 /wd4127 /wd4710 /wd4711 /wd4577 /wd4996`.
   C++14 is pinned via `set_target_properties(CXX_STANDARD 14)` on each
   PhysX target, which reliably overrides Choreonoid's global
   `/std:c++20` or `-std=c++20`. Flags are scoped so they do not leak
   into the rest of Choreonoid. Public PhysX headers are marked as
   SYSTEM include directories on the exported targets, so downstream
   code (including PhysXPlugin) does not inherit PhysX's own warning
   noise.
4. Renames the archive output names to `PhysX*_static` and, combined
   with `CMAKE_RELEASE_POSTFIX=_64`, produces the
   `lib{PhysX*,}_static_64.{a,lib}` names PhysXPlugin expects on Linux
   and Windows respectively.
5. Adds alias targets (`PhysXFoundation_static_64`, etc.) so downstream
   code links using the same identifier in both the bundled path and
   the `ENABLE_PHYSX=OFF` external path.

OBJECT libraries (`LowLevel`, `LowLevelAABB`, `LowLevelDynamics`,
`PhysXTask`, `SceneQuery`, `SimulationController`) are merged into the
`PhysX` static archive via the upstream `{linux,windows}/PhysX.cmake`
`$<TARGET_OBJECTS:...>` list, matching upstream's release layout.

## Updating PhysX

1. Pick a new upstream commit (use `git log ... -- physx/` to confirm
   the `physx/` subtree actually changed).
2. Diff the `source/` directory list against this README and adjust
   the copy list if new top-level directories have appeared or
   disappeared.
3. Re-copy every included directory verbatim from the upstream tree.
4. Inspect `source/compiler/cmake/*.cmake` (upstream),
   `source/compiler/cmake/linux/*.cmake` and
   `source/compiler/cmake/windows/*.cmake` for:
   - new entries in the `SET(...)` source lists (especially
     platform-specific `*_PLATFORM_SOURCE` additions under `windows/`),
   - new `TARGET_INCLUDE_DIRECTORIES` paths (especially GPU or
     metadata ones),
   - new `*_COMPILE_DEFS` additions,
   - new `*_RESOURCE_FILE` entries under `windows/*.cmake` - if the
     upstream adds a new `.rc` reference, copy the file into
     `source/compiler/windows/resource/` (the upstream fragments
     splice `*_RESOURCE_FILE` into `*_PLATFORM_FILES` unconditionally,
     so missing `.rc` files cause configure errors),
   - new special `SET_SOURCE_FILES_PROPERTIES` flags.
   These are the files Choreonoid's wrapper pulls in via `include()`;
   the wrapper variables may need small additions when new PhysX
   modules reference previously unseen platform variables.
5. Re-apply the deletions listed under "What is excluded" above
   (GPU source trees, `source/physx/src/opensource/`,
   `ExtDeformableSkinning.cpp` and friends, unused CMake fragments,
   `include/characterkinematic/`, `include/vehicle2/`). Also re-apply
   the local edit to `include/PxPhysicsAPI.h` that removes the
   `characterkinematic/*` and `vehicle2/*` `#include` lines; without
   this edit the newly re-copied upstream header will try to pull in
   the deleted directories.
6. After a successful build, compare compiled object files against
   `.cpp` files on disk to find any newly arriving sources that are
   present but unused:
   ```sh
   find build-physx-bundled/thirdparty/physx-5.6.1/CMakeFiles \
        -name '*.cpp.o' | awk -F'CMakeFiles/[^/]*/' '{print $2}' \
     | sed 's|\.o$||' | sort -u > /tmp/compiled.txt
   find thirdparty/physx-5.6.1/source -name '*.cpp' \
     | sed 's|.*thirdparty/physx-5.6.1/||' | sort -u > /tmp/existing.txt
   comm -23 /tmp/existing.txt /tmp/compiled.txt
   ```
   The output lists `.cpp` files that are bundled but not compiled.
   Linux builds expect only the Windows `.cpp` files to show up
   (`source/foundation/windows/Fd*.cpp`, Windows-only delay-load hooks,
   etc.), and Windows builds expect only the Linux `.cpp` files.
7. Re-copy any newly added headers that are `#include`d outside
   `PX_SUPPORT_GPU_PHYSX` guards (past trouble spots:
   `source/physxgpu/include/PxPhysXGpu.h`,
   `include/cudamanager/PxCudaTypes.h`,
   `include/extensions/PxCudaHelpersExt.h`).
8. Check that `PxConfig.h` in `include/` is still a no-op (upstream
   normally regenerates it via `CONFIGURE_FILE`; the bundled copy
   is intentionally empty).
9. Re-build with `ENABLE_PHYSX=ON` (default) on both Linux and Windows
   and exercise `sample/PhysX/PxCrawlerJoystick.cnoid` and
   `sample/PhysX/PxTank.cnoid`.
10. On Linux/GCC, also re-build with `ENABLE_LTO=ON` and watch the
    link of `CnoidPhysXPlugin` for new link-time warnings (`-Wodr`,
    `-Wstringop-overread`, etc.). If new symbols trigger `-Wodr` or a
    new warning category appears, update the `target_link_options()`
    call in `src/PhysXPlugin/CMakeLists.txt` and the matching entry in
    "Known upstream quirks neutralised here" below.

## Known upstream quirks neutralised here

- Upstream `snippets/` build is broken in the default public release
  preset on Linux. Not an issue here - snippets are excluded.
- Upstream `NpImmediateMode.cpp` `#include`s `PxPhysXGpu.h`
  unconditionally. That header is bundled (inside
  `source/physxgpu/include/`); the interface bodies are
  `#if PX_SUPPORT_GPU_PHYSX`-guarded so no implementation is needed.
- Several public headers in `include/` pull in GPU-related headers
  unconditionally (for example `PxPBDParticleSystem.h` includes
  `cudamanager/PxCudaTypes.h`, and `PxDeformableVolumeExt` refers to
  `extensions/PxCudaHelpersExt.h`). These public headers cannot be
  removed even in a CPU-only build, so the whole `include/` tree is
  kept as-is. Only their runtime implementations (`source/cudamanager/`,
  `source/gpucommon/`, the GPU source modules) are stripped.
- `OmniPvdPxSampler.cpp` and `OmniPvdPxExtensionsSampler.cpp` require
  `-fpermissive` under GCC in upstream builds. With
  `PX_SUPPORT_OMNI_PVD=0`, the bodies are `#if`-guarded away and the
  flag is not needed.
- LTO (`ENABLE_LTO=ON`) re-runs whole-program analysis at link time,
  which re-emits warnings inside inlined PhysX code. The per-source
  `PHYSX_GCC_WARNING_FLAGS` `-Wno-*` set in this directory's
  `CMakeLists.txt` only applies at compile time, so the link step
  surfaces them again. Two upstream-originated warnings show up under
  GCC LTO:
  - `-Wodr`: `Scene`, `NpScene`, `SolverDt` and `SortedTriangle` are
    declared with the same name in multiple translation units. The
    layouts are effectively identical but LTO flags them as ODR
    violations.
  - `-Wstringop-overread`: LTO inlining makes some PhysX `memcpy`
    sites look like they read past the source object (notably the
    `PxMat33` access in
    `source/lowleveldynamics/src/DyFeatherstoneForwardDynamic.cpp`).
    The paths are not reached at runtime.
  Both are suppressed at the link step via `target_link_options(...)`
  on the `CnoidPhysXPlugin` target in `src/PhysXPlugin/CMakeLists.txt`,
  gated on `ENABLE_LTO AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"`.
  When updating PhysX, re-run an LTO build and update both the symbol
  list above and the `target_link_options()` call if new symbols start
  triggering `-Wodr` or if a different warning category appears. The
  upstream code is intentionally not patched.

## Windows-specific notes

- Tested with Visual Studio 2022 (v143 / `cl.exe` 19.41), x64 only.
  Earlier toolsets should work with the same flag set.
- Upstream `source/compiler/cmake/windows/CMakeLists.txt` is **not**
  included, because it overwrites `CMAKE_CXX_FLAGS` /
  `CMAKE_SHARED_LINKER_FLAGS` globally. The wrapper supplies the
  `PHYSX_WINDOWS_*_COMPILE_DEFS` variables the Windows fragments read
  and sets `PHYSX_LIBTYPE_DEFS=PX_PHYSX_STATIC_LIB` directly.
- `source/compiler/windows/resource/` contains only four `.rc` files
  plus `resource.h`. `PhysXCharacterKinematic.rc`, `PhysXVehicle*.rc`
  and `PhysXGpu.rc` are intentionally omitted.
- C++14 is enforced per target via `CXX_STANDARD 14`, so
  Choreonoid's global `/std:c++20` does not reach PhysX source files.
- `-fno-exceptions` is Linux-only. On MSVC the default exception
  behaviour (`/EHsc`, inherited from Choreonoid) is used; PhysX is
  happy with it.
