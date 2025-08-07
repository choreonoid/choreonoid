# Choreonoid Debian Package Build Script Manual

## Overview

The `build-packages.sh` script automates the process of building Debian packages for Choreonoid on multiple Ubuntu distributions. It supports both clean builds using cowbuilder and local builds using the current environment.

## Prerequisites

### Required Tools

- **git**: Version control system
- **gbp** (git-buildpackage): Git integration for Debian packaging
- **cowbuilder**: Clean build environment tool
- **devscripts**: Debian developer scripts (includes `dch`)
- **sudo**: Required for cowbuilder operations

### Installation

```bash
# Install required packages
sudo apt-get update
sudo apt-get install git-buildpackage cowbuilder devscripts
```

### Cowbuilder Environment Setup

Before using the script with cowbuilder, you need to create build environments:

```bash
# For Ubuntu 22.04 (jammy)
sudo cowbuilder --create --distribution jammy \
    --basepath /var/cache/pbuilder/jammy-base.cow \
    --mirror http://archive.ubuntu.com/ubuntu

# For Ubuntu 24.04 (noble)
sudo cowbuilder --create --distribution noble \
    --basepath /var/cache/pbuilder/noble-base.cow \
    --mirror http://archive.ubuntu.com/ubuntu
```

## Usage

### Basic Syntax

```bash
./debian/build-packages.sh [OPTIONS]
```

### Command Line Options

| Option | Long Form | Description |
|--------|-----------|-------------|
| `-d DISTRO` | `--distro DISTRO` | Build for specific distribution (jammy or noble) |
| `-c` | `--cowbuilder` | Use cowbuilder for clean build (default) |
| `-l` | `--local` | Build locally without cowbuilder |
| `-h` | `--help` | Show help message |

### Examples

#### Build for All Distributions (Default)
```bash
./debian/build-packages.sh
```
Builds packages for both Ubuntu 22.04 and 24.04 using cowbuilder.

#### Build for Specific Distribution
```bash
./debian/build-packages.sh -d jammy
```
Builds package only for Ubuntu 22.04.

#### Local Build
```bash
./debian/build-packages.sh -d noble -l
```
Builds package for Ubuntu 24.04 using the local environment (without cowbuilder).

#### Combined Options
```bash
./debian/build-packages.sh -d jammy -c
```
Explicitly uses cowbuilder to build for Ubuntu 22.04.

## Build Process

### 1. Initialization Phase

- **Git Repository Check**: Verifies the current directory is a git repository
- **Changelog Backup**: Creates a backup of `debian/changelog`
- **Version Extraction**: Reads the current version from the changelog

### 2. Distribution-Specific Build

For each target distribution, the script:

#### Version Management
- Original version: `2.3.0~git20250806.c272f4d-1`
- Distribution-specific: `2.3.0~git20250806.c272f4d~jammy1` (for Ubuntu 22.04)
- Distribution-specific: `2.3.0~git20250806.c272f4d~noble1` (for Ubuntu 24.04)

#### Build Directory Structure
```
../build-area-jammy/     # Ubuntu 22.04 build artifacts
../build-area-noble/     # Ubuntu 24.04 build artifacts
```

#### Build Methods

**Cowbuilder Mode (Recommended)**
- Uses clean, isolated build environment
- Ensures reproducible builds
- Automatically handles dependencies
- Requires sudo privileges
- Commands used:
  ```bash
  gbp buildpackage \
      --git-ignore-new \
      --git-upstream-tree=HEAD \
      --git-pbuilder \
      --git-dist=$DIST
  ```

**Local Mode**
- Builds in the current environment
- Faster but less reproducible
- Dependencies must be manually installed
- Commands used:
  ```bash
  gbp buildpackage \
      --git-ignore-new \
      --git-upstream-tree=HEAD \
      --git-export-dir="$BUILD_DIR" \
      --git-tarball-dir="$BUILD_DIR" \
      --git-builder='debuild -us -uc -b'
  ```

### 3. Post-Build Phase

- **Artifact Collection**: Copies .deb, .ddeb, .changes, and .buildinfo files
- **Permission Fix**: Changes ownership of build artifacts to current user
- **Changelog Restoration**: Restores original changelog from backup
- **Summary Display**: Shows list of generated packages with file sizes

## Output

### Build Artifacts

The script generates the following files in `../build-area-{distro}/`:

| File Type | Description |
|-----------|-------------|
| `*.deb` | Binary package files |
| `*.ddeb` | Debug symbol packages |
| `*.changes` | Change description file |
| `*.buildinfo` | Build information file |

### Console Output

The script uses colored output for better readability:
- 🟢 **[INFO]** - Informational messages
- 🔴 **[ERROR]** - Error messages
- ✓ - Build success indicator
- ✗ - Build failure indicator

## Troubleshooting

### Common Issues

#### Cowbuilder Environment Not Found
**Error**: `Cowbuilder environment not found: /var/cache/pbuilder/{distro}-base.cow`

**Solution**: Create the cowbuilder environment:
```bash
sudo cowbuilder --create --distribution {distro} \
    --basepath /var/cache/pbuilder/{distro}-base.cow
```

#### Build Dependencies Not Met (Local Build)
**Error**: Unmet build dependencies during local build

**Solution**: Install build dependencies:
```bash
sudo apt-get build-dep choreonoid
```
Or use cowbuilder mode which handles dependencies automatically.

#### Permission Denied
**Error**: Permission denied when accessing cowbuilder results

**Solution**: Ensure you have sudo privileges or run the script with appropriate permissions.

### Updating Cowbuilder Environments

Keep your build environments up to date:
```bash
# Update Ubuntu 22.04 environment
sudo cowbuilder --update --basepath /var/cache/pbuilder/jammy-base.cow

# Update Ubuntu 24.04 environment
sudo cowbuilder --update --basepath /var/cache/pbuilder/noble-base.cow
```

## Advanced Usage

### Custom Mirror

To use a local or custom mirror, update the cowbuilder environment:
```bash
sudo cowbuilder --create --distribution jammy \
    --basepath /var/cache/pbuilder/jammy-base.cow \
    --mirror http://your.mirror.com/ubuntu
```

### Build Hooks

You can add custom hooks to the cowbuilder environment by placing scripts in:
```
/var/cache/pbuilder/hook.d/
```

### Parallel Builds

The script respects the `DEB_BUILD_OPTIONS` environment variable:
```bash
export DEB_BUILD_OPTIONS="parallel=4"
./debian/build-packages.sh
```

## Best Practices

1. **Always Use Cowbuilder for Release Builds**: Ensures reproducible and clean builds
2. **Regular Environment Updates**: Update cowbuilder environments weekly or before important builds
3. **Version Control**: Commit all changes before building
4. **Test Locally First**: Use local mode for quick testing, then validate with cowbuilder
5. **Check Build Logs**: Review build logs for warnings even if build succeeds

## File Locations

| File/Directory | Purpose |
|----------------|---------|
| `debian/build-packages.sh` | This build script |
| `debian/changelog` | Package version and change history |
| `debian/control` | Package metadata and dependencies |
| `debian/rules` | Build configuration |
| `debian/gbp.conf` | Git-buildpackage configuration |
| `../build-area-{distro}/` | Build artifacts output directory |
| `/var/cache/pbuilder/{distro}-base.cow/` | Cowbuilder environment |

## Support

For issues or questions:
1. Check the build logs in the terminal output
2. Verify all prerequisites are installed
3. Ensure cowbuilder environments are properly created and updated
4. Consult the Choreonoid packaging documentation in `debian/README_PACKAGING.md`

---

*Last updated: 2025-08-14*