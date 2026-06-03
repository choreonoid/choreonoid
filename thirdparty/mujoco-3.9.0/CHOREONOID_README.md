# MuJoCo 3.9.0 bundled for Choreonoid

This directory contains the official **prebuilt** MuJoCo dynamic library plus
its public C headers, bundled so that Choreonoid's MuJoCoPlugin can be built
without installing MuJoCo separately. It is enabled by Choreonoid's CMake when
`ENABLE_MUJOCO=ON` (the default). Supported hosts:

- Linux x86_64 (glibc 2.27+, i.e. any distribution from ~2018 onward)
- Windows x86_64 (Visual C++ 2015-2022 runtime; the same runtime Choreonoid
  itself requires)

Unlike the bundled PhysX (which is built from source), MuJoCo is shipped as a
prebuilt shared library. MuJoCo's public API is pure C, and the library links
its C++ dependencies (libstdc++, tinyxml2, qhull, ccd, lodepng, ...) statically
with hidden symbol visibility, so it does not clash with Choreonoid's own
runtime. This is why a prebuilt drop is safe across distributions and across
MSVC toolset versions.

## Why prebuilt instead of from source

The MuJoCo source build (`cmake/MujocoDependencies.cmake`) uses FetchContent to
download ~10 dependencies (abseil, Eigen, qhull, ccd, tinyxml2, tinyobjloader,
lodepng, MarchingCubeCpp, miniz, ...) at configure time. Bundling the source
alone would still require network access at build time and would pull in a
second copy of Eigen (Choreonoid already bundles eigen-3.4.0). The official
prebuilt binaries avoid all of that and build cleanly on Windows.

## Extraction base

- Upstream: https://github.com/google-deepmind/mujoco/releases/tag/3.9.0
- License: Apache License 2.0 (see `LICENSE`).

Download both per-platform release archives:

- `mujoco-3.9.0-linux-x86_64.tar.gz`
- `mujoco-3.9.0-windows-x86_64.zip`

## Directory layout

    include/mujoco/*.h            public C headers (shared by all platforms)
    lib/linux-x86_64/libmujoco.so.3.9.0
    lib/windows-x86_64/mujoco.dll
    lib/windows-x86_64/mujoco.lib (import library)
    LICENSE                       Apache 2.0 license text
    CMakeLists.txt                defines the IMPORTED "mujoco" target
    CHOREONOID_README.md          this file

## Assembly procedure (when updating)

1. Extract both archives.
2. Copy the headers from the **Linux** archive (LF line endings):
   `include/mujoco/` -> `thirdparty/mujoco-<ver>/include/mujoco/`.
   The Windows archive ships identical headers but with CRLF line endings;
   use the Linux ones for consistency.
3. Remove the parts not needed for simulation:
   - `include/mujoco/experimental/` (USD integration; needs OpenUSD)
   - `include/mujoco/.clang-format`
4. Copy `lib/libmujoco.so.<ver>` from the Linux archive into
   `lib/linux-x86_64/`. (The `libmujoco.so` dev symlink is not needed: the
   SONAME `libmujoco.so.<ver>` is recorded directly.)
5. Copy `bin/mujoco.dll` and `lib/mujoco.lib` from the Windows archive into
   `lib/windows-x86_64/`.
6. Copy `LICENSE` from either archive.
7. Bump the version strings in `CMakeLists.txt` (the `add_subdirectory`
   path in the top-level `CMakeLists.txt`, the directory name, and the
   `libmujoco.so.<ver>` SONAME) and in this file.

## Notes for agents working on this directory

- Apache 2.0 only requires keeping `LICENSE` and the in-source attribution
  notices intact. Bundling the binaries in this repository is permitted.
- Do not add the MuJoCo sample executables, models, or the `simulate` GUI
  sources; only the library and headers are needed.
- The prebuilt Linux `.so` depends only on core system libraries
  (libc/libm/libdl/libpthread/librt) and has no dynamic libstdc++ dependency.
  The Windows DLL depends on the standard VC++ 2015-2022 runtime
  (vcruntime140/msvcp140/UCRT).
