#!/bin/bash

# ROS Package Build Script for cyclone_vision
# This script builds the ROS package with proper error handling and logging

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in a ROS workspace
check_workspace() {
    if [[ ! -f "package.xml" ]]; then
        print_error "package.xml not found. Make sure you're in the package root directory."
        exit 1
    fi
    
    if [[ ! -f "setup.py" ]]; then
        print_error "setup.py not found. This doesn't appear to be a Python ROS package."
        exit 1
    fi
}

# Clean previous builds
clean_build() {
    print_status "Cleaning previous build artifacts..."
    if [[ -d "build" ]]; then
        rm -rf build/
        print_status "Removed build/ directory"
    fi
    
    if [[ -d "install" ]]; then
        rm -rf install/
        print_status "Removed install/ directory"
    fi
    
    if [[ -d "log" ]]; then
        rm -rf log/
        print_status "Removed log/ directory"
    fi
}

# Build the package
build_package() {
    local package_name="cyclone_vision"  # Update this if you rename the package
    
    print_status "Building ROS package: $package_name"
    print_status "Starting colcon build..."
    
    # Build with verbose output and specific package selection
    if colcon build --packages-select "$package_name" --event-handlers console_direct+; then
        print_success "Package built successfully!"
        return 0
    else
        print_error "Build failed!"
        return 1
    fi
}

# Verify build results
verify_build() {
    print_status "Verifying build results..."
    
    # Check if install directory was created
    if [[ ! -d "install" ]]; then
        print_error "Install directory not found after build"
        return 1
    fi
    
    # Check if package was installed
    local package_install_dir="install/cyclone_vision"
    if [[ ! -d "$package_install_dir" ]]; then
        print_error "Package installation directory not found: $package_install_dir"
        return 1
    fi
    
    # Check for executables
    local lib_dir="$package_install_dir/lib/cyclone_vision"
    if [[ -d "$lib_dir" ]]; then
        print_success "Executable directory found: $lib_dir"
        if [[ -n "$(ls -A "$lib_dir" 2>/dev/null)" ]]; then
            print_status "Available executables:"
            ls -la "$lib_dir"
        else
            print_warning "No executables found in $lib_dir"
        fi
    else
        print_warning "Executable directory not found: $lib_dir"
    fi
    
    # Check for package resources
    local share_dir="$package_install_dir/share/cyclone_vision"
    if [[ -d "$share_dir" ]]; then
        print_success "Package share directory found: $share_dir"
    fi
    
    return 0
}

# Show build logs if build failed
show_build_logs() {
    if [[ -d "log/latest_build" ]]; then
        print_status "Recent build logs:"
        echo "----------------------------------------"
        if [[ -f "log/latest_build/cyclone_vision/stderr.log" ]]; then
            print_error "STDERR Log:"
            cat "log/latest_build/cyclone_vision/stderr.log"
        fi
        
        if [[ -f "log/latest_build/cyclone_vision/stdout.log" ]]; then
            print_status "STDOUT Log:"
            cat "log/latest_build/cyclone_vision/stdout.log"
        fi
        echo "----------------------------------------"
    fi
}

# Main execution
main() {
    print_status "Starting ROS package build process..."
    
    # Parse command line arguments
    CLEAN=false
    VERBOSE=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean|-c)
                CLEAN=true
                shift
                ;;
            --verbose|-v)
                VERBOSE=true
                shift
                ;;
            --help|-h)
                echo "Usage: $0 [OPTIONS]"
                echo "Options:"
                echo "  --clean, -c     Clean previous build artifacts before building"
                echo "  --verbose, -v   Enable verbose output"
                echo "  --help, -h      Show this help message"
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # Check workspace
    check_workspace
    
    # Clean if requested
    if [[ "$CLEAN" == "true" ]]; then
        clean_build
    fi
    
    # Build the package
    if build_package; then
        verify_build
        print_success "Build completed successfully!"
        print_status "You can now source the workspace with: source install/setup.bash"
    else
        print_error "Build failed!"
        show_build_logs
        exit 1
    fi
}

# Run main function
main "$@"