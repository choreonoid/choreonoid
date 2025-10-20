# Choreonoid Debian Package Build Script Manual

## Overview

The `build-packages.sh` script automates the process of building Debian packages for Choreonoid on multiple Ubuntu distributions. It supports both local builds for development and clean builds using cowbuilder for production.

## Prerequisites

### Required Tools

- **git**: Version control system
- **dpkg-dev**: Debian package development tools
- **devscripts**: Debian developer scripts (includes `dch`)
- **build-essential**: Basic compilation tools
- **cowbuilder** (optional): Clean build environment tool
- **sudo** (optional): Required for cowbuilder operations

### Installation

```bash
# Install required packages
sudo apt-get update
sudo apt-get install dpkg-dev devscripts build-essential

# Optional: Install cowbuilder for clean builds
sudo apt-get install cowbuilder
```

### Cowbuilder Environment Setup (Optional)

For production-quality builds, set up cowbuilder environments:

```bash
# For Ubuntu 22.04 (jammy)
sudo cowbuilder --create --distribution jammy \
    --basepath /var/cache/pbuilder/base-jammy.cow \
    --components "main universe" \
    --mirror http://jp.archive.ubuntu.com/ubuntu

# For Ubuntu 24.04 (noble)
sudo cowbuilder --create --distribution noble \
    --basepath /var/cache/pbuilder/base-noble.cow \
    --components "main universe" \
    --mirror http://jp.archive.ubuntu.com/ubuntu
```

## Usage

### Basic Syntax

```bash
./debian/build-packages.sh [OPTIONS]
```

### Command Line Options

| Option | Long Form | Description | Default |
|--------|-----------|-------------|---------|
| `-d DISTRO` | `--distro DISTRO` | Build for specific distribution (jammy or noble) | Both |
| `-c` | `--cowbuilder` | Use cowbuilder for clean build | No |
| `-l` | `--local` | Build locally (explicitly set) | Yes |
| `-r` | `--release` | Build release version (no git suffix) | No |
| `-h` | `--help` | Show help message | - |

### Build Types

#### Development Build (Default)
Includes git commit information in the version number:
- Version format: `2.4.0~git20250901.1713.202dedb8-1~jammy`
- Useful for: Testing, continuous integration, daily builds

#### Release Build (`-r`)
Uses the version from debian/changelog as-is:
- Version format: `2.4.0-1`
- Useful for: Official releases, stable versions

## Examples

### Common Use Cases

#### Quick Development Build
```bash
# Build for current distribution
./debian/build-packages.sh -d $(lsb_release -cs)
```

#### Build for All Distributions
```bash
# Development version (default)
./debian/build-packages.sh

# Release version
./debian/build-packages.sh -r
```

#### Build for Specific Distribution
```bash
# Ubuntu 22.04 development build
./debian/build-packages.sh -d jammy

# Ubuntu 24.04 release build
./debian/build-packages.sh -r -d noble
```

#### Production Build with Cowbuilder
```bash
# Clean build in isolated environment
./debian/build-packages.sh -c -d jammy

# Release version with cowbuilder
./debian/build-packages.sh -r -c -d noble
```

## Build Process

### 1. Initialization
- Verifies git repository
- Sets up maintainer information from changelog
- Creates temporary directory for backups
- Backs up original changelog

### 2. Version Determination
- **Development mode**: Generates version with git information
  - Format: `{base_version}~git{date}.{time}.{commit}-1~{distro}`
  - Example: `2.4.0~git20250901.1713.202dedb8-1~jammy`
- **Release mode**: Uses existing changelog version
  - Example: `2.4.0-1`

### 3. Changelog Update
- **Development builds**: Updates version with git information
- **Release builds**: Only updates distribution if needed
- Uses `dch` for proper formatting

### 4. Package Building

#### Local Build (Default)
- Uses `dpkg-buildpackage -us -uc -b`
- Builds in current environment
- Fast, suitable for development
- Warning shown if building for different distribution

#### Cowbuilder Build (`-c`)
- Creates isolated build environment
- Ensures clean dependencies
- Requires sudo privileges
- Steps:
  1. Creates source package with `dpkg-source`
  2. Builds with `cowbuilder --build`
  3. Fixes file permissions

### 5. Post-Build
- Moves packages to `../build-area-{distro}/`
- Restores original changelog
- Displays summary of generated packages

## Output Structure

```
~/packaging/
├── choreonoid/           # Source directory
│   └── debian/          # Packaging files
├── build-area-jammy/    # Ubuntu 22.04 packages
│   ├── choreonoid_*.deb
│   ├── choreonoid_*.buildinfo
│   └── choreonoid_*.changes
├── build-area-noble/    # Ubuntu 24.04 packages
│   └── ...
└── build-temp-{PID}/    # Temporary files (auto-cleaned)
```

## File Naming Convention

### Development Packages
```
choreonoid_2.4.0~git20250901.1713.202dedb8-1~jammy_amd64.deb
           └────────────────┬────────────────┘ └─┬─┘ └─┬─┘
                     version with git info    distro arch
```

### Release Packages
```
choreonoid_2.4.0-1_amd64.deb
           └──┬──┘ └─┬─┘
           version  arch
```

## Troubleshooting

### Build Fails with Missing Dependencies
**Local build**: Install missing packages manually
```bash
sudo apt-get install missing-package-name
```

**Cowbuilder**: Update the base environment
```bash
sudo cowbuilder --update --basepath /var/cache/pbuilder/base-jammy.cow
```

### Cowbuilder Environment Not Found
Create the environment:
```bash
sudo cowbuilder --create --distribution jammy \
    --basepath /var/cache/pbuilder/base-jammy.cow \
    --components "main universe" \
    --mirror http://jp.archive.ubuntu.com/ubuntu
```

### Version Number Conflicts
Use `-r` for release builds to avoid git suffixes, or clean the changelog:
```bash
# Reset to original version
git checkout debian/changelog
```

### Cross-Distribution Build Warning
When building for a different distribution than the host:
- **Solution 1**: Use cowbuilder (`-c`) for accurate builds
- **Solution 2**: Accept the warning for quick testing

## Environment Variables

The script respects these environment variables:

| Variable | Description | Example |
|----------|-------------|---------|
| `DEBEMAIL` | Maintainer email | `user@example.com` |
| `DEBFULLNAME` | Maintainer name | `John Doe` |
| `DEB_BUILD_OPTIONS` | Build options | `parallel=4` |

If not set, maintainer information is extracted from the existing changelog.

## Best Practices

1. **Development Workflow**
   - Use local builds without `-r` for rapid iteration
   - Version includes git commit for traceability

2. **Release Workflow**
   - Always use cowbuilder (`-c`) for final builds
   - Use `-r` flag for clean version numbers
   - Test on target distribution before release

3. **Multi-Distribution Support**
   - Build for all supported distributions before release
   - Use cowbuilder to ensure compatibility
   - Test packages on actual target systems

4. **Version Management**
   - Keep debian/changelog with generic version for development
   - Let the script handle version suffixes automatically
   - Use release mode for official versions

## Differences from Previous Version

### Removed Dependencies
- **git-buildpackage (gbp)**: No longer required
- Simpler, more predictable behavior
- Direct control over build process

### Improved Features
- Changelog modifications now always take effect
- Cleaner code structure (~270 lines vs ~300+)
- Faster local builds by default
- More intuitive version handling

### Migration Notes
- The script no longer uses gbp configuration
- debian/gbp.conf is not required
- All builds use standard Debian tools directly

## See Also

- `upload-to-ppa.sh`: Script for uploading to Launchpad PPA
- `dpkg-buildpackage(1)`: Manual page for the build command
- `dch(1)`: Manual page for changelog management
- `cowbuilder(8)`: Manual page for clean build environments