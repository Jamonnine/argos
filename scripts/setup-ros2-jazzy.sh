#!/bin/bash

##############################################################################
# ROS 2 Jazzy Installation Script for Ubuntu 24.04
#
# This script automates the installation of ROS 2 Jazzy and essential tools
# for the ARGOS learning project.
#
# Usage: ./setup-ros2-jazzy.sh
# Prerequisites: Ubuntu 24.04 (native or WSL2)
##############################################################################

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check Ubuntu version
check_ubuntu_version() {
    log_info "Checking Ubuntu version..."

    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        if [[ "$VERSION_ID" == "24.04" ]]; then
            log_success "Ubuntu 24.04 detected"
        else
            log_error "This script requires Ubuntu 24.04. Detected: $VERSION_ID"
            exit 1
        fi
    else
        log_error "Cannot detect Ubuntu version"
        exit 1
    fi
}

# Update system
update_system() {
    log_info "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
    log_success "System updated"
}

# Install essential dependencies
install_dependencies() {
    log_info "Installing essential dependencies..."

    sudo apt install -y \
        software-properties-common \
        curl \
        wget \
        gnupg \
        lsb-release \
        ca-certificates \
        apt-transport-https \
        build-essential \
        git \
        python3-pip \
        python3-venv

    log_success "Dependencies installed"
}

# Setup ROS 2 repository
setup_ros_repository() {
    log_info "Setting up ROS 2 repository..."

    # Add ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add repository to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    log_success "ROS 2 repository configured"
}

# Install ROS 2 Jazzy
install_ros2_jazzy() {
    log_info "Installing ROS 2 Jazzy (Desktop Full)..."

    # Install ROS 2 Jazzy Desktop Full (includes RViz, demos, tutorials)
    sudo apt install -y ros-jazzy-desktop-full

    log_success "ROS 2 Jazzy installed"
}

# Install development tools
install_dev_tools() {
    log_info "Installing ROS 2 development tools..."

    sudo apt install -y \
        ros-jazzy-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-argcomplete

    log_success "Development tools installed"
}

# Initialize rosdep
initialize_rosdep() {
    log_info "Initializing rosdep..."

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    else
        log_warning "rosdep already initialized"
    fi

    rosdep update

    log_success "rosdep initialized"
}

# Setup environment
setup_environment() {
    log_info "Setting up ROS 2 environment..."

    BASHRC="$HOME/.bashrc"

    # Backup bashrc
    cp "$BASHRC" "$BASHRC.backup_$(date +%Y%m%d_%H%M%S)"

    # Add ROS 2 sourcing if not already present
    if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# ROS 2 Jazzy Setup" >> "$BASHRC"
        echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
        echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> "$BASHRC"
        log_success "ROS 2 environment added to .bashrc"
    else
        log_warning "ROS 2 environment already in .bashrc"
    fi

    # Add useful aliases
    if ! grep -q "alias cb='colcon build'" "$BASHRC"; then
        echo "" >> "$BASHRC"
        echo "# ROS 2 Aliases" >> "$BASHRC"
        echo "alias cb='colcon build'" >> "$BASHRC"
        echo "alias cbs='colcon build --symlink-install'" >> "$BASHRC"
        echo "alias cbt='colcon test'" >> "$BASHRC"
        echo "alias cbp='colcon build --packages-select'" >> "$BASHRC"
        echo "alias cba='colcon build --ament-cmake-args'" >> "$BASHRC"
        echo "alias rt='ros2 topic'" >> "$BASHRC"
        echo "alias rn='ros2 node'" >> "$BASHRC"
        echo "alias rr='ros2 run'" >> "$BASHRC"
        echo "alias rl='ros2 launch'" >> "$BASHRC"
        log_success "ROS 2 aliases added to .bashrc"
    fi

    # Source for current session
    source /opt/ros/jazzy/setup.bash
}

# Install additional useful packages
install_additional_packages() {
    log_info "Installing additional ROS 2 packages..."

    sudo apt install -y \
        ros-jazzy-turtlesim \
        ros-jazzy-rqt-* \
        ros-jazzy-gazebo-ros-pkgs \
        ros-jazzy-ros-gz \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-moveit \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro \
        ros-jazzy-tf2-tools \
        ros-jazzy-tf2-ros \
        ros-jazzy-image-transport \
        ros-jazzy-cv-bridge \
        ros-jazzy-vision-opencv

    log_success "Additional packages installed"
}

# Install Python packages
install_python_packages() {
    log_info "Installing Python packages for robotics development..."

    pip3 install --user \
        numpy \
        matplotlib \
        opencv-python \
        scipy \
        pyyaml \
        transforms3d \
        pytest \
        pytest-cov \
        black \
        flake8 \
        pylint

    log_success "Python packages installed"
}

# Create workspace
create_workspace() {
    log_info "Creating ROS 2 workspace..."

    WORKSPACE_DIR="$HOME/ros2_ws"

    if [ ! -d "$WORKSPACE_DIR" ]; then
        mkdir -p "$WORKSPACE_DIR/src"
        cd "$WORKSPACE_DIR"
        colcon build

        # Add workspace sourcing to bashrc
        BASHRC="$HOME/.bashrc"
        if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" "$BASHRC"; then
            echo "" >> "$BASHRC"
            echo "# ROS 2 Workspace" >> "$BASHRC"
            echo "source $WORKSPACE_DIR/install/setup.bash" >> "$BASHRC"
        fi

        log_success "Workspace created at $WORKSPACE_DIR"
    else
        log_warning "Workspace already exists at $WORKSPACE_DIR"
    fi
}

# Verify installation
verify_installation() {
    log_info "Verifying ROS 2 installation..."

    # Source ROS 2
    source /opt/ros/jazzy/setup.bash

    # Check ROS 2 commands
    if command -v ros2 &> /dev/null; then
        log_success "ros2 command available"

        # Check ROS 2 version
        ROS_VERSION=$(ros2 --version)
        log_info "Installed: $ROS_VERSION"
    else
        log_error "ros2 command not found!"
        exit 1
    fi

    # Check colcon
    if command -v colcon &> /dev/null; then
        log_success "colcon build tool available"
    else
        log_error "colcon not found!"
        exit 1
    fi
}

# Run test
run_test() {
    log_info "Running test: talker-listener demo..."

    echo ""
    log_info "Starting talker node (will run for 5 seconds)..."
    timeout 5s ros2 run demo_nodes_cpp talker || true

    echo ""
    log_success "If you saw messages above, ROS 2 is working correctly!"
}

# Print next steps
print_next_steps() {
    echo ""
    echo "========================================================================"
    log_success "ROS 2 Jazzy installation complete!"
    echo "========================================================================"
    echo ""
    echo "📋 Next Steps:"
    echo ""
    echo "1. Restart your terminal (or run: source ~/.bashrc)"
    echo ""
    echo "2. Verify installation:"
    echo "   $ ros2 --version"
    echo "   $ ros2 run demo_nodes_cpp talker"
    echo ""
    echo "3. Test with turtlesim:"
    echo "   Terminal 1: $ ros2 run turtlesim turtlesim_node"
    echo "   Terminal 2: $ ros2 run turtlesim turtle_teleop_key"
    echo ""
    echo "4. Start Week 1 learning:"
    echo "   $ cd ~/Desktop/ARGOS/learning-log/week01"
    echo "   $ cat plan.md"
    echo ""
    echo "📚 Useful Commands:"
    echo "   cb     = colcon build"
    echo "   cbs    = colcon build --symlink-install"
    echo "   rt     = ros2 topic"
    echo "   rn     = ros2 node"
    echo "   rr     = ros2 run"
    echo "   rl     = ros2 launch"
    echo ""
    echo "🔗 Resources:"
    echo "   Official Tutorials: https://docs.ros.org/en/jazzy/Tutorials.html"
    echo "   ROS Discourse: https://discourse.ros.org/"
    echo ""
    echo "========================================================================"
}

# Main installation flow
main() {
    echo "========================================================================"
    echo "         ROS 2 Jazzy Installation Script - ARGOS Learning Project"
    echo "========================================================================"
    echo ""

    check_ubuntu_version
    update_system
    install_dependencies
    setup_ros_repository
    install_ros2_jazzy
    install_dev_tools
    initialize_rosdep
    setup_environment
    install_additional_packages
    install_python_packages
    create_workspace
    verify_installation
    run_test
    print_next_steps

    echo ""
    log_success "Installation script completed successfully!"
    echo ""
}

# Run main function
main "$@"
