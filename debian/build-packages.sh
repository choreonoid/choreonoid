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

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse command line arguments
DISTRO=""
USE_COWBUILDER=false
LOCAL_BUILD=false

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -d, --distro DISTRO     Build for specific distribution (jammy or noble)"
    echo "  -c, --cowbuilder        Use cowbuilder for clean build (default)"
    echo "  -l, --local             Build locally without cowbuilder"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      # Build for both distributions with cowbuilder"
    echo "  $0 -d jammy             # Build only for Ubuntu 22.04"
    echo "  $0 -d noble -l          # Build for Ubuntu 24.04 locally"
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

# Default to cowbuilder
if [ "$LOCAL_BUILD" = false ]; then
    USE_COWBUILDER=true
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

# Backup current changelog
cp debian/changelog debian/changelog.backup

# Get original version
ORIG_VERSION=$(dpkg-parsechangelog -S Version)
BASE_VERSION="${ORIG_VERSION%%-*}"

# Build for each distribution
for DIST in "${DISTROS[@]}"; do
    print_info "========================================="
    print_info "Building package for $DIST"
    print_info "========================================="
    
    # Add distribution info to version number
    NEW_VERSION="${BASE_VERSION}~${DIST}1"
    
    # Update changelog using dch
    print_info "Updating changelog: $NEW_VERSION"
    dch --newversion "$NEW_VERSION" --distribution "$DIST" --force-distribution "Build for $DIST"
    
    # Create build directory
    BUILD_DIR="../build-area-${DIST}"
    mkdir -p "$BUILD_DIR"
    
    if [ "$USE_COWBUILDER" = true ]; then
        # Build with cowbuilder
        print_info "Building with cowbuilder..."
        
        # Check cowbuilder environment exists
        COWBUILDER_BASE="/var/cache/pbuilder/${DIST}-base.cow"
        if [ ! -d "$COWBUILDER_BASE" ]; then
            print_error "Cowbuilder environment not found: $COWBUILDER_BASE"
            print_info "Please create it with:"
            print_info "  sudo cowbuilder --create --distribution $DIST --basepath $COWBUILDER_BASE"
            
            # Restore changelog
            mv debian/changelog.backup debian/changelog
            exit 1
        fi
        
        # Build with gbp
        print_info "Running: gbp buildpackage --git-ignore-new --git-upstream-tree=HEAD --git-pbuilder --git-dist=$DIST"
        print_info "This command requires sudo, you may be prompted for password"
        
        gbp buildpackage \
            --git-ignore-new \
            --git-upstream-tree=HEAD \
            --git-pbuilder \
            --git-dist=$DIST
        
        # Copy build results
        print_info "Copying build results..."
        if [ -d "/var/cache/pbuilder/${DIST}/result" ]; then
            sudo cp /var/cache/pbuilder/${DIST}/result/*.deb "$BUILD_DIR/" 2>/dev/null || true
            sudo cp /var/cache/pbuilder/${DIST}/result/*.ddeb "$BUILD_DIR/" 2>/dev/null || true
            sudo cp /var/cache/pbuilder/${DIST}/result/*.changes "$BUILD_DIR/" 2>/dev/null || true
            sudo cp /var/cache/pbuilder/${DIST}/result/*.buildinfo "$BUILD_DIR/" 2>/dev/null || true
            sudo chown $(whoami):$(whoami) "$BUILD_DIR"/*
        fi
        
    else
        # Build locally
        print_info "Building locally..."
        
        gbp buildpackage \
            --git-ignore-new \
            --git-upstream-tree=HEAD \
            --git-export-dir="$BUILD_DIR" \
            --git-tarball-dir="$BUILD_DIR" \
            --git-builder='debuild -us -uc -b'
    fi
    
    # Check build results
    DEB_FILES=$(find "$BUILD_DIR" -name "*.deb" -type f 2>/dev/null)
    if [ -n "$DEB_FILES" ]; then
        print_info "✓ Build successful for $DIST"
        print_info "Generated files:"
        for f in $DEB_FILES; do
            echo "  - $(basename $f)"
        done
    else
        print_error "✗ Build failed for $DIST"
    fi
    
    echo ""
done

# Restore changelog
print_info "Restoring changelog..."
mv debian/changelog.backup debian/changelog

print_info "========================================="
print_info "Build complete"
print_info "========================================="

# Show summary
for DIST in "${DISTROS[@]}"; do
    BUILD_DIR="../build-area-${DIST}"
    if [ -d "$BUILD_DIR" ]; then
        echo ""
        print_info "Packages for $DIST:"
        ls -lh "$BUILD_DIR"/*.deb 2>/dev/null || echo "  No packages found"
    fi
done