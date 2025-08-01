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

# Function to add auto-activation script to shell config
add_autoactivate_script() {
    local shell_config_file="$1"
    local shell_type="$2"
    local project_path="$3"

    # Check if the script is already in the config file
    if grep -q "armpi_mini_auto_activate" "$shell_config_file"; then
        print_warning "Auto-activation script already exists in $shell_config_file."
        return
    fi

    print_status "Adding auto-activation script to $shell_config_file..."

    if [[ "$shell_type" == "bash" ]]; then
        cat >> "$shell_config_file" << EOF

# Auto activate ArmPi Mini Robot virtual environment
function armpi_mini_auto_activate() {
    if [[ "\$PWD" == "$project_path"* ]] && [ -f ".venv/bin/activate" ]; then
        if [[ "\$VIRTUAL_ENV" != "$project_path/.venv" ]]; then
            source .venv/bin/activate
            echo "✓ ArmPi Mini Robot environment activated"
        fi
    elif [[ "\$VIRTUAL_ENV" == "$project_path/.venv" ]] && [[ "\$PWD" != "$project_path"* ]]; then
        deactivate
        echo "✓ ArmPi Mini Robot environment deactivated"
    fi
}

# Trigger auto_activate function on directory change
PROMPT_COMMAND="armpi_mini_auto_activate; \$PROMPT_COMMAND"
EOF

    elif [[ "$shell_type" == "zsh" ]]; then
        cat >> "$shell_config_file" << EOF

# Auto activate ArmPi Mini Robot virtual environment
function armpi_mini_auto_activate() {
    if [[ "\$PWD" == "$project_path"* ]] && [ -f ".venv/bin/activate" ]; then
        if [[ "\$VIRTUAL_ENV" != "$project_path/.venv" ]]; then
            source .venv/bin/activate
            echo "✓ ArmPi Mini Robot environment activated"
        fi
    elif [[ "\$VIRTUAL_ENV" == "$project_path/.venv" ]] && [[ "\$PWD" != "$project_path"* ]]; then
        deactivate
        echo "✓ ArmPi Mini Robot environment deactivated"
    fi
}

# Trigger auto_activate function on directory change
autoload -U add-zsh-hook
add-zsh-hook chpwd armpi_mini_auto_activate
EOF
    fi

    print_success "Auto-activation script added to $shell_config_file"
    print_warning "Please restart your terminal or run 'source $shell_config_file' to apply changes"
}

# Get the current project path
PROJECT_PATH=$(pwd)

# Check if we're in the right directory
if [[ ! -f "auto-setup.sh" ]] || [[ ! -f "ArmPi_mini.py" ]]; then
    echo "Error: This script must be run from the ArmPi Mini Robot project directory"
    echo "Current directory: $PROJECT_PATH"
    echo "Expected files: auto-setup.sh, ArmPi_mini.py"
    exit 1
fi

print_status "Setting up auto-activation for ArmPi Mini Robot project"
print_status "Project path: $PROJECT_PATH"

# Check if the user is using Bash or Zsh
SHELL_TYPE=$(basename "$SHELL")

if [[ "$SHELL_TYPE" == "bash" ]]; then
    SHELL_CONFIG_FILE="$HOME/.bashrc"
    add_autoactivate_script "$SHELL_CONFIG_FILE" "bash" "$PROJECT_PATH"
elif [[ "$SHELL_TYPE" == "zsh" ]]; then
    SHELL_CONFIG_FILE="$HOME/.zshrc"
    add_autoactivate_script "$SHELL_CONFIG_FILE" "zsh" "$PROJECT_PATH"
else
    print_warning "Unsupported shell detected: $SHELL_TYPE"
    print_status "This script currently supports only Bash and Zsh"
    print_status "You can manually activate the environment with:"
    echo "   source .venv/bin/activate"
    exit 1
fi

print_success "Auto-activation setup complete!"
echo ""
print_status "How it works:"
echo "- When you enter the ArmPi Mini Robot project directory, the virtual environment will automatically activate"
echo "- When you leave the project directory, the environment will automatically deactivate"
echo "- You'll see a message confirming the activation/deactivation"
echo ""
print_status "To apply changes immediately:"
echo "   source $SHELL_CONFIG_FILE"
echo ""
print_status "To disable auto-activation later:"
echo "   Remove the 'armpi_mini_auto_activate' function from $SHELL_CONFIG_FILE" 