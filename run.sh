#!/bin/bash

# ROS Package Run Script for cyclone_vision
# This script helps run the ROS nodes with proper environment setup

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Package configuration
PACKAGE_NAME="cyclone_vision"
WORKSPACE_ROOT="$(pwd)"

# Available nodes (update these based on your actual entry points)
declare -A AVAILABLE_NODES
AVAILABLE_NODES=(
    ["camera"]="CamaraSubscriber"
    ["detection"]="BottomCamDetection"
)

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

# Check if workspace is built
check_workspace() {
    if [[ ! -d "install" ]]; then
        print_error "Workspace not built. Please run ./build.sh first."
        exit 1
    fi
    
    if [[ ! -f "install/setup.bash" ]]; then
        print_error "install/setup.bash not found. Please build the workspace first."
        exit 1
    fi
    
    if [[ ! -d "install/$PACKAGE_NAME" ]]; then
        print_error "Package $PACKAGE_NAME not found in install directory."
        print_status "Available packages:"
        ls install/ 2>/dev/null | grep -v -E "setup\.|local_setup\.|COLCON_IGNORE" || echo "None found"
        exit 1
    fi
}

# Source the workspace
source_workspace() {
    print_status "Sourcing ROS workspace..."
    
    # Source ROS2 if available
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        source /opt/ros/humble/setup.bash
        print_status "Sourced ROS2 Humble"
    elif [[ -f "/opt/ros/foxy/setup.bash" ]]; then
        source /opt/ros/foxy/setup.bash
        print_status "Sourced ROS2 Foxy"
    else
        print_warning "ROS2 installation not found in standard locations"
    fi
    
    # Source the workspace
    source install/setup.bash
    print_success "Workspace sourced successfully"
}

# List available nodes
list_nodes() {
    print_status "Available nodes in $PACKAGE_NAME:"
    echo "----------------------------------------"
    
    # Check what's actually installed
    local lib_dir="install/$PACKAGE_NAME/lib/$PACKAGE_NAME"
    if [[ -d "$lib_dir" ]] && [[ -n "$(ls -A "$lib_dir" 2>/dev/null)" ]]; then
        print_success "Installed executables:"
        ls -1 "$lib_dir" | while read -r executable; do
            echo "  - $executable"
        done
    else
        print_warning "No executables found in $lib_dir"
    fi
    
    echo ""
    print_status "Predefined shortcuts:"
    for key in "${!AVAILABLE_NODES[@]}"; do
        echo "  $key -> ${AVAILABLE_NODES[$key]}"
    done
    echo "----------------------------------------"
}

# Run a specific node
run_node() {
    local node_name="$1"
    shift  # Remove first argument, keep the rest as parameters
    local node_params="$@"
    
    print_status "Running node: $node_name"
    if [[ -n "$node_params" ]]; then
        print_status "With parameters: $node_params"
    fi
    
    # Check if it's a predefined shortcut
    if [[ -n "${AVAILABLE_NODES[$node_name]}" ]]; then
        node_name="${AVAILABLE_NODES[$node_name]}"
        print_status "Using shortcut, actual node: $node_name"
    fi
    
    # Run the node
    print_status "Executing: ros2 run $PACKAGE_NAME $node_name $node_params"
    exec ros2 run "$PACKAGE_NAME" "$node_name" $node_params
}

# Launch multiple nodes (basic example)
launch_all() {
    print_status "Launching all nodes..."
    print_warning "This will launch nodes in the background"
    print_status "Use 'pkill -f cyclone_vision' to stop all nodes"
    
    # Launch camera subscriber
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal -- bash -c "source install/setup.bash; ros2 run $PACKAGE_NAME CamaraSubscriber; exec bash"
        sleep 2
        gnome-terminal -- bash -c "source install/setup.bash; ros2 run $PACKAGE_NAME BottomCamDetection; exec bash"
    elif command -v xterm >/dev/null 2>&1; then
        xterm -e "source install/setup.bash; ros2 run $PACKAGE_NAME CamaraSubscriber" &
        sleep 2
        xterm -e "source install/setup.bash; ros2 run $PACKAGE_NAME BottomCamDetection" &
    else
        print_error "No terminal emulator found (gnome-terminal or xterm)"
        print_status "Please run nodes manually in separate terminals"
        exit 1
    fi
    
    print_success "Nodes launched in separate terminals"
}

# Show package info
show_info() {
    print_status "Package Information:"
    echo "----------------------------------------"
    echo "Package Name: $PACKAGE_NAME"
    echo "Workspace: $WORKSPACE_ROOT"
    
    if [[ -f "package.xml" ]]; then
        local version=$(grep -o '<version>.*</version>' package.xml | sed 's/<[^>]*>//g')
        local description=$(grep -o '<description>.*</description>' package.xml | sed 's/<[^>]*>//g')
        echo "Version: $version"
        echo "Description: $description"
    fi
    
    echo "----------------------------------------"
    
    # Check ROS environment
    if [[ -n "$ROS_DISTRO" ]]; then
        print_status "ROS Distribution: $ROS_DISTRO"
    else
        print_warning "ROS environment not detected"
    fi
}

# Show help
show_help() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  run <node_name> [params]  Run a specific node"
    echo "  list                      List available nodes"
    echo "  launch                    Launch all nodes in separate terminals"
    echo "  info                      Show package information"
    echo "  help                      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 run camera                    # Run camera subscriber node"
    echo "  $0 run CamaraSubscriber         # Run node by full name"
    echo "  $0 run detection --help         # Run detection node with --help"
    echo "  $0 list                         # List all available nodes"
    echo "  $0 launch                       # Launch all nodes"
    echo ""
    echo "Node shortcuts:"
    for key in "${!AVAILABLE_NODES[@]}"; do
        echo "  $key -> ${AVAILABLE_NODES[$key]}"
    done
}

# Main execution
main() {
    # Check if any arguments provided
    if [[ $# -eq 0 ]]; then
        show_help
        exit 0
    fi
    
    local command="$1"
    shift
    
    case "$command" in
        "run")
            if [[ $# -eq 0 ]]; then
                print_error "Node name required for run command"
                echo "Use '$0 list' to see available nodes"
                exit 1
            fi
            check_workspace
            source_workspace
            run_node "$@"
            ;;
        "list")
            check_workspace
            source_workspace
            list_nodes
            ;;
        "launch")
            check_workspace
            source_workspace
            launch_all
            ;;
        "info")
            show_info
            ;;
        "help"|"--help"|"-h")
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main function
main "$@"