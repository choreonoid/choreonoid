#!/bin/bash
set -e

# Build script for Choreonoid Debian packages
# Builds packages for Ubuntu 22.04 (jammy) and 24.04 (noble)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse command line arguments
DISTRO=""
USE_COWBUILDER=false
LOCAL_BUILD=false
RELEASE_BUILD=false

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -d, --distro DISTRO     Build for specific distribution (jammy or noble)"
    echo "  -c, --cowbuilder        Use cowbuilder for clean build"
    echo "  -l, --local             Build locally (default)"
    echo "  -r, --release           Build release version (no git suffix)"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      # Build development version locally for both"
    echo "  $0 -r                   # Build release version locally for both"
    echo "  $0 -d jammy -c          # Build for Ubuntu 22.04 with cowbuilder"
    echo "  $0 -r -d noble          # Build release version for Ubuntu 24.04 locally"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--distro)
            DISTRO="$2"
            shift 2
            ;;
        -c|--cowbuilder)
            USE_COWBUILDER=true
            shift
            ;;
        -l|--local)
            LOCAL_BUILD=true
            shift
            ;;
        -r|--release)
            RELEASE_BUILD=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Default to local build if not specified
if [ "$USE_COWBUILDER" = false ] && [ "$LOCAL_BUILD" = false ]; then
    LOCAL_BUILD=true
fi

# List of distributions to build
if [ -z "$DISTRO" ]; then
    DISTROS=("jammy" "noble")
else
    DISTROS=("$DISTRO")
fi

# Change to project directory
cd "$PROJECT_DIR"

# Check git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    print_error "Not in a git repository"
    exit 1
fi

# Set email for dch if not already set
if [ -z "$DEBEMAIL" ] && [ -z "$EMAIL" ]; then
    # Get maintainer from current changelog
    MAINTAINER_EMAIL=$(dpkg-parsechangelog -S Maintainer | sed 's/.*<\(.*\)>/\1/')
    MAINTAINER_NAME=$(dpkg-parsechangelog -S Maintainer | sed 's/ <.*//')
    export DEBEMAIL="$MAINTAINER_EMAIL"
    export DEBFULLNAME="$MAINTAINER_NAME"
fi

# Create temp directory for backup files
TEMP_DIR="../build-temp-$$"
mkdir -p "$TEMP_DIR"
trap "rm -rf $TEMP_DIR" EXIT

# Backup current changelog
cp debian/changelog "$TEMP_DIR/changelog.original"

# Get version information
ORIG_VERSION=$(dpkg-parsechangelog -S Version)
BASE_VERSION=$(echo "$ORIG_VERSION" | sed 's/[~-].*//')

# Get git commit info for development builds
if [ "$RELEASE_BUILD" = false ]; then
    GIT_COMMIT=$(git rev-parse --short HEAD)
    GIT_DATE=$(git log -1 --format="%cd" --date=format:"%Y%m%d")
    GIT_TIME=$(git log -1 --format="%cd" --date=format:"%H%M")
fi

# Build for each distribution
for DIST in "${DISTROS[@]}"; do
    print_info "========================================="
    print_info "Building package for $DIST"
    print_info "========================================="
    
    # Clean up any previous build artifacts in debian directory
    # Note: obj-* is the build directory and should be cleaned by dpkg-buildpackage -b
    rm -rf debian/choreonoid debian/.debhelper debian/files debian/*.debhelper debian/*.substvars 2>/dev/null || true
    
    # Create build directory
    BUILD_DIR="../build-area-${DIST}"
    mkdir -p "$BUILD_DIR"
    
    # Determine version
    if [ "$RELEASE_BUILD" = true ]; then
        NEW_VERSION="${ORIG_VERSION}"
        print_info "Building release version: $NEW_VERSION"
    else
        UPSTREAM_VERSION="${BASE_VERSION}~git${GIT_DATE}.${GIT_TIME}.${GIT_COMMIT}"
        NEW_VERSION="${UPSTREAM_VERSION}-1~${DIST}"
        print_info "Building development version: $NEW_VERSION"
    fi
    
    # Update changelog
    if [ "$RELEASE_BUILD" = false ]; then
        # For development builds, update version
        rm -f debian/changelog.dch
        DCH_EDITOR=true dch -b --no-query --newversion "$NEW_VERSION" \
            --distribution "$DIST" --force-distribution \
            "Development snapshot from git commit ${GIT_COMMIT}"
    else
        # For release builds, only update distribution if needed
        CURRENT_DIST=$(dpkg-parsechangelog -S Distribution)
        if [ "$CURRENT_DIST" != "$DIST" ] && [ "$CURRENT_DIST" != "UNRELEASED" ]; then
            rm -f debian/changelog.dch
            DCH_EDITOR=true dch --no-query --distribution "$DIST" --force-distribution ""
        fi
    fi
    
    # Clean up any dch backup files
    rm -f debian/changelog.dch
    
    # Build the package
    if [ "$USE_COWBUILDER" = true ]; then
        # Build with cowbuilder
        print_info "Building with cowbuilder..."
        
        # Check cowbuilder environment exists
        COWBUILDER_BASE="/var/cache/pbuilder/base-${DIST}.cow"
        if [ ! -d "$COWBUILDER_BASE" ]; then
            COWBUILDER_BASE="/var/cache/pbuilder/${DIST}-base.cow"
        fi
        
        if [ ! -d "$COWBUILDER_BASE" ]; then
            print_error "Cowbuilder environment not found"
            print_info "Please create it with:"
            print_info "  sudo cowbuilder --create --distribution $DIST --basepath /var/cache/pbuilder/base-${DIST}.cow --components 'main universe' --mirror http://jp.archive.ubuntu.com/ubuntu/"
            cp "$TEMP_DIR/changelog.original" debian/changelog
            exit 1
        fi
        
        print_info "Using cowbuilder base: $COWBUILDER_BASE"
        print_info "This requires sudo, you may be prompted for password"
        
        # Create source package first
        if [ "$RELEASE_BUILD" = false ]; then
            # For development builds, create orig tarball
            ORIG_TARBALL="../choreonoid_${UPSTREAM_VERSION}.orig.tar.xz"
            ORIG_TARBALL_FINAL="${BUILD_DIR}/choreonoid_${UPSTREAM_VERSION}.orig.tar.xz"
            
            # Check if already exists in build directory
            if [ -f "$ORIG_TARBALL_FINAL" ]; then
                # Copy to parent for dpkg-source
                cp "$ORIG_TARBALL_FINAL" "$ORIG_TARBALL"
            elif [ ! -f "$ORIG_TARBALL" ]; then
                print_info "Creating orig tarball..."
                git archive --format=tar --prefix=choreonoid-${UPSTREAM_VERSION}/ HEAD | \
                    xz > "$ORIG_TARBALL"
            fi
        fi
        
        # Build source package
        dpkg-source -b .
        
        # Move ALL source files to build directory
        mv ../choreonoid_*.dsc "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid_*.debian.tar.* "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid_*.orig.tar.* "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid_*.tar.* "$BUILD_DIR/" 2>/dev/null || true
        
        # Build with cowbuilder
        DSC_FILE=$(ls "$BUILD_DIR"/choreonoid_*.dsc | head -1)
        sudo cowbuilder --build "$DSC_FILE" \
            --basepath "$COWBUILDER_BASE" \
            --distribution "$DIST" \
            --buildresult "$BUILD_DIR"
        
        # Fix permissions
        sudo chown $(whoami):$(whoami) "$BUILD_DIR"/* 2>/dev/null || true
        
    else
        # Build locally
        print_info "Building locally with dpkg-buildpackage..."
        
        if [ "$DIST" != "$(lsb_release -cs)" ]; then
            print_warning "Building for $DIST on $(lsb_release -cs) - package may have compatibility issues"
            print_warning "Consider using cowbuilder (-c) for production builds"
        fi
        
        # Build binary package only
        dpkg-buildpackage -us -uc -b
        
        # Move build results to build directory
        mv ../choreonoid*.deb "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid*.ddeb "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid_*.buildinfo "$BUILD_DIR/" 2>/dev/null || true
        mv ../choreonoid_*.changes "$BUILD_DIR/" 2>/dev/null || true
        
        # Clean up build directory to avoid conflicts with future cowbuilder builds
        print_info "Cleaning up build directory..."
        rm -rf obj-*
    fi
    
    # Check build results
    DEB_FILES=$(find "$BUILD_DIR" -name "*.deb" -type f 2>/dev/null)
    if [ -n "$DEB_FILES" ]; then
        print_info "✓ Build successful for $DIST"
        print_info "Generated packages:"
        for f in $DEB_FILES; do
            size=$(ls -lh "$f" | awk '{print $5}')
            echo "  - $(basename $f) [$size]"
        done
    else
        print_error "✗ Build failed for $DIST - no packages generated"
    fi
    
    echo ""
done

# Restore original changelog
print_info "Restoring original changelog..."
cp "$TEMP_DIR/changelog.original" debian/changelog

# Clean up any leftover files in parent directory
print_info "Cleaning up temporary files..."
rm -f ../choreonoid_*.dsc ../choreonoid_*.tar.* ../choreonoid_*.buildinfo ../choreonoid_*.changes ../choreonoid*.deb ../choreonoid*.ddeb 2>/dev/null || true

print_info "========================================="
print_info "Build complete"
print_info "========================================="

# Show summary
for DIST in "${DISTROS[@]}"; do
    BUILD_DIR="../build-area-${DIST}"
    if [ -d "$BUILD_DIR" ]; then
        echo ""
        print_info "Packages for $DIST in $BUILD_DIR:"
        ls -lh "$BUILD_DIR"/*.deb 2>/dev/null || echo "  No packages found"
    fi
done