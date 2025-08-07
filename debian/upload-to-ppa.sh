#!/bin/bash
set -e

# Script for uploading Choreonoid packages to Launchpad PPA
# This script creates source packages and uploads them to PPA

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Default values
PPA_NAME=""
DISTROS=""
GPG_KEY=""
FORCE_UPLOAD=false
DRY_RUN=false
PPA_VERSION_SUFFIX="1"

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Upload Choreonoid source packages to Launchpad PPA"
    echo ""
    echo "Options:"
    echo "  -p, --ppa PPA           PPA name (e.g., 'username/ppa-name')"
    echo "  -d, --distro DISTRO     Distribution(s) to build for (jammy,noble or 'all')"
    echo "  -k, --key GPG_KEY       GPG key ID for signing"
    echo "  -s, --suffix SUFFIX     Debian revision number (default: 1, use 2,3... for re-uploads)"
    echo "  -f, --force             Force upload even if version exists"
    echo "  -n, --dry-run           Build packages but don't upload"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 -p myuser/choreonoid -d all -k ABCD1234"
    echo "  $0 -p myuser/test -d jammy -k ABCD1234 --dry-run"
    echo ""
    echo "Prerequisites:"
    echo "  1. Launchpad account with PPA created"
    echo "  2. GPG key registered with Launchpad"
    echo "  3. dput package installed (apt install dput)"
    echo "  4. ~/.dput.cf configured (optional)"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--ppa)
            PPA_NAME="$2"
            shift 2
            ;;
        -d|--distro)
            DISTROS="$2"
            shift 2
            ;;
        -k|--key)
            GPG_KEY="$2"
            shift 2
            ;;
        -s|--suffix)
            PPA_VERSION_SUFFIX="$2"
            shift 2
            ;;
        -f|--force)
            FORCE_UPLOAD=true
            shift
            ;;
        -n|--dry-run)
            DRY_RUN=true
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

# Validate required arguments
if [ -z "$PPA_NAME" ]; then
    print_error "PPA name is required (-p option)"
    show_help
    exit 1
fi

if [ -z "$DISTROS" ]; then
    print_error "Distribution is required (-d option)"
    show_help
    exit 1
fi

# Auto-detect GPG key if not provided
if [ -z "$GPG_KEY" ]; then
    print_info "Auto-detecting GPG key..."
    GPG_KEY=$(gpg --list-secret-keys --keyid-format LONG 2>/dev/null | grep '^sec' | head -1 | awk '{print $2}' | cut -d'/' -f2)
    if [ -z "$GPG_KEY" ]; then
        print_error "No GPG key found. Please specify with -k option"
        exit 1
    fi
    print_info "Using GPG key: $GPG_KEY"
fi

# Set default PPA version suffix if not provided
if [ -z "$PPA_VERSION_SUFFIX" ]; then
    PPA_VERSION_SUFFIX="1"
fi

# Determine distributions
if [ "$DISTROS" = "all" ]; then
    DISTRO_LIST=("jammy" "noble")
else
    IFS=',' read -ra DISTRO_LIST <<< "$DISTROS"
fi

# Check prerequisites
print_step "Checking prerequisites..."

if ! command -v dput &> /dev/null; then
    print_error "dput is not installed. Please install it with: sudo apt install dput"
    exit 1
fi

if ! command -v debuild &> /dev/null; then
    print_error "debuild is not installed. Please install it with: sudo apt install devscripts"
    exit 1
fi

if ! gpg --list-secret-keys "$GPG_KEY" &> /dev/null; then
    print_error "GPG key $GPG_KEY not found"
    exit 1
fi

# Change to project directory
cd "$PROJECT_DIR"

# Check git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    print_error "Not in a git repository"
    exit 1
fi

# Clean up any previous build artifacts
print_step "Cleaning up previous build artifacts..."
rm -f debian/files
rm -f ../*.dsc ../*.changes ../*.tar.gz ../*.tar.xz ../*.buildinfo ../*.upload

# No need for dput configuration - using ppa: prefix directly

# Backup changelog
print_step "Backing up changelog..."
cp debian/changelog debian/changelog.backup

# Get base version and maintainer
# Extract base version (e.g., 2.3.0) from changelog, handling various formats
ORIG_VERSION=$(dpkg-parsechangelog -S Version)
# Remove everything after ~ or - to get clean base version
BASE_VERSION=$(echo "$ORIG_VERSION" | sed 's/[~-].*//')
ORIG_MAINTAINER=$(dpkg-parsechangelog -S Maintainer)

# Create build area
BUILD_AREA="../ppa-build-area"
mkdir -p "$BUILD_AREA"

# Get git commit info (use commit timestamp, not current time)
GIT_COMMIT=$(git rev-parse --short HEAD)
GIT_DATE=$(git log -1 --format="%cd" --date=format:"%Y%m%d")
GIT_TIME=$(git log -1 --format="%cd" --date=format:"%H%M")

# Build for each distribution
for DIST in "${DISTRO_LIST[@]}"; do
    print_step "========================================="
    print_step "Building source package for $DIST"
    print_step "========================================="
    
    # Create version with PPA suffix
    # Format: upstream_version-debian_revision
    # Example: 2.3.0~git20250815.1630.166d2a13-1~jammy
    UPSTREAM_VERSION="${BASE_VERSION}~git${GIT_DATE}.${GIT_TIME}.${GIT_COMMIT}"
    DEBIAN_REVISION="${PPA_VERSION_SUFFIX}~${DIST}"
    PPA_VERSION="${UPSTREAM_VERSION}-${DEBIAN_REVISION}"
    
    print_info "Version: $PPA_VERSION"
    
    # Clean up any old files for this version in build area
    rm -f "$BUILD_AREA/choreonoid_${PPA_VERSION}"* 2>/dev/null || true
    
    # Update changelog
    print_info "Updating changelog..."
    cat > debian/changelog << EOF
choreonoid (${PPA_VERSION}) ${DIST}; urgency=medium

  * Development snapshot from git commit ${GIT_COMMIT}
  * Built for ${DIST}

 -- ${ORIG_MAINTAINER}  $(date -R)
EOF
    
    # Create source package
    print_info "Creating source package..."
    
    # Create orig tarball if needed (using upstream version only)
    ORIG_TARBALL="../choreonoid_${UPSTREAM_VERSION}.orig.tar.xz"
    if [ ! -f "$ORIG_TARBALL" ]; then
        print_info "Creating orig tarball..."
        git archive --format=tar --prefix=choreonoid-${UPSTREAM_VERSION}/ HEAD | \
            xz > "$ORIG_TARBALL"
    fi
    
    # Clean debian/files before each build (important!)
    rm -f debian/files
    
    # Build source package
    print_info "Building source package with debuild..."
    if debuild -S -sa -k"$GPG_KEY" -d; then
        print_info "✓ Source package created successfully"
    else
        print_error "✗ Failed to create source package"
        mv debian/changelog.backup debian/changelog
        exit 1
    fi
    
    # Move build files to build area immediately to avoid conflicts
    # Use specific filenames to avoid moving wrong files
    CHANGES_PATTERN="choreonoid_${PPA_VERSION}_source.changes"
    DSC_PATTERN="choreonoid_${PPA_VERSION}.dsc"
    DEBIAN_TAR_PATTERN="choreonoid_${PPA_VERSION}.debian.tar.*"
    BUILDINFO_PATTERN="choreonoid_${PPA_VERSION}_source.buildinfo"
    
    mv "../${CHANGES_PATTERN}" "$BUILD_AREA/" 2>/dev/null || true
    mv "../${DSC_PATTERN}" "$BUILD_AREA/" 2>/dev/null || true
    mv ../${DEBIAN_TAR_PATTERN} "$BUILD_AREA/" 2>/dev/null || true
    mv "../${BUILDINFO_PATTERN}" "$BUILD_AREA/" 2>/dev/null || true
    
    # Move orig tarball only if it's not already in build area
    if [ ! -f "$BUILD_AREA/$(basename "$ORIG_TARBALL")" ]; then
        cp "$ORIG_TARBALL" "$BUILD_AREA/" 2>/dev/null || true
    fi
    
    # Upload to PPA
    CHANGES_FILE="$BUILD_AREA/choreonoid_${PPA_VERSION}_source.changes"
    
    if [ -f "$CHANGES_FILE" ]; then
        if [ "$DRY_RUN" = true ]; then
            print_warning "DRY RUN: Would upload $CHANGES_FILE to ppa:$PPA_NAME"
        else
            print_info "Uploading to ppa:$PPA_NAME..."
            
            DPUT_OPTIONS=""
            if [ "$FORCE_UPLOAD" = true ]; then
                DPUT_OPTIONS="-f"
            fi
            
            if dput $DPUT_OPTIONS "ppa:$PPA_NAME" "$CHANGES_FILE"; then
                print_info "✓ Successfully uploaded to PPA for $DIST"
            else
                print_error "✗ Failed to upload to PPA"
                mv debian/changelog.backup debian/changelog
                exit 1
            fi
        fi
    else
        print_error "Changes file not found: $CHANGES_FILE"
        mv debian/changelog.backup debian/changelog
        exit 1
    fi
    
    echo ""
done

# Restore changelog
print_step "Restoring original changelog..."
mv debian/changelog.backup debian/changelog

print_step "========================================="
print_step "PPA upload complete!"
print_step "========================================="

if [ "$DRY_RUN" = false ]; then
    echo ""
    print_info "Packages uploaded to: https://launchpad.net/~${PPA_NAME%%/*}/+archive/ubuntu/${PPA_NAME##*/}"
    print_info "Build status: https://launchpad.net/~${PPA_NAME%%/*}/+archive/ubuntu/${PPA_NAME##*/}/+packages"
    echo ""
    print_info "Next steps:"
    print_info "1. Wait for Launchpad to process the upload (5-10 minutes)"
    print_info "2. Check build status on the PPA page"
    print_info "3. Once built, packages will be available via:"
    print_info "   sudo add-apt-repository ppa:$PPA_NAME"
    print_info "   sudo apt update"
    print_info "   sudo apt install choreonoid"
else
    print_warning "DRY RUN completed - no packages were uploaded"
fi

# Show summary
echo ""
print_info "Summary:"
for DIST in "${DISTRO_LIST[@]}"; do
    echo "  - ${DIST}: choreonoid_${BASE_VERSION}~git${GIT_DATE}.${GIT_TIME}.${GIT_COMMIT}~${DIST}~${PPA_VERSION_SUFFIX}"
done