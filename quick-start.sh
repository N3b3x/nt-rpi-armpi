#!/usr/bin/env bash
set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if auto-setup.sh exists
if [[ ! -f "auto-setup.sh" ]]; then
    echo "Error: auto-setup.sh not found in current directory"
    echo "Please run this script from the ArmPi Mini Robot project directory"
    exit 1
fi

print_status "Starting ArmPi Mini Robot Quick Setup..."

# Run the auto-setup script
print_status "Running auto-setup.sh..."
./auto-setup.sh

# Check if setup was successful
if [[ $? -eq 0 ]]; then
    print_success "Setup completed successfully!"
    echo ""
    print_status "Next steps:"
    echo "1. Activate the virtual environment:"
    echo "   source .venv/bin/activate"
    echo ""
    echo "2. Run the main application:"
    echo "   python3 ArmPi_mini.py"
    echo ""
    echo "3. (Optional) Test the setup:"
    echo "   python3 test_setup.py"
    echo ""
    
    # Ask if user wants to activate now
    read -p "Would you like to activate the environment now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_status "Activating virtual environment..."
        source .venv/bin/activate
        print_success "Environment activated! You can now run: python3 ArmPi_mini.py"
    else
        print_warning "Remember to activate the environment before running the application:"
        echo "   source .venv/bin/activate"
    fi
else
    print_warning "Setup may have encountered issues. Please check the output above."
    echo "You can still try to activate the environment manually:"
    echo "   source .venv/bin/activate"
fi 