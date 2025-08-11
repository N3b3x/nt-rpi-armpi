#!/bin/bash

# ArmPi Mini Web Server Service Installer
# Automatically detects repository location and installs service

echo "================================================"
echo "🔧 ArmPi Mini Web Server Service Installer"
echo "================================================"
echo ""

# Get the absolute path of the current directory (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PATH="$SCRIPT_DIR"

echo "📍 Detected repository location: $REPO_PATH"

# Check if web_server.py exists in this directory
if [ ! -f "$REPO_PATH/web_server.py" ]; then
    echo "❌ Error: web_server.py not found in $REPO_PATH"
    echo "   Please run this script from the ArmPi Mini repository directory"
    exit 1
fi

echo "✅ Found web_server.py in repository"

# Check if running as root (needed for systemd service installation)
if [ "$EUID" -eq 0 ]; then
    echo "⚠️  Warning: Running as root"
    echo "   This script will install the service for user 'pi'"
    echo "   If you want a different user, edit the service file after installation"
    SERVICE_USER="pi"
    SERVICE_GROUP="pi"
else
    echo "📝 Note: Not running as root"
    echo "   Service will be configured for current user: $(whoami)"
    SERVICE_USER="$(whoami)"
    SERVICE_GROUP="$(id -gn)"
fi

echo ""

# Get current user if not root
CURRENT_USER=$(whoami)
if [ "$CURRENT_USER" == "root" ]; then
    INSTALL_USER="pi"
else
    INSTALL_USER="$CURRENT_USER"
fi

# Create the service file with correct paths
SERVICE_FILE="/tmp/armpi-web.service"
echo "🔨 Creating service file with dynamic paths..."

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=ArmPi Mini Web Server
After=network.target
Wants=network.target

[Service]
Type=simple
User=$SERVICE_USER
Group=$SERVICE_GROUP
WorkingDirectory=$REPO_PATH
ExecStart=/usr/bin/python3 $REPO_PATH/web_server.py
Restart=always
RestartSec=10
Environment=PYTHONPATH=$REPO_PATH
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "✅ Service file created with paths:"
echo "   Repository: $REPO_PATH"
echo "   User/Group: $SERVICE_USER/$SERVICE_GROUP"
echo ""

# Function to install service (requires sudo)
install_service() {
    echo "🚀 Installing systemd service..."
    
    # Copy service file to systemd directory
    sudo cp "$SERVICE_FILE" /etc/systemd/system/armpi-web.service
    
    if [ $? -ne 0 ]; then
        echo "❌ Failed to copy service file"
        return 1
    fi
    
    # Set correct permissions
    sudo chmod 644 /etc/systemd/system/armpi-web.service
    
    # Reload systemd
    sudo systemctl daemon-reload
    
    if [ $? -ne 0 ]; then
        echo "❌ Failed to reload systemd"
        return 1
    fi
    
    echo "✅ Service installed successfully"
    
    # Ask if user wants to enable and start the service
    echo ""
    read -p "🤔 Do you want to enable the service to start on boot? (y/N): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "⚙️  Enabling service for auto-start on boot..."
        sudo systemctl enable armpi-web
        
        if [ $? -eq 0 ]; then
            echo "✅ Service enabled for auto-start"
        else
            echo "❌ Failed to enable service"
        fi
    fi
    
    echo ""
    read -p "🚀 Do you want to start the service now? (y/N): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "▶️  Starting ArmPi Mini Web Server service..."
        sudo systemctl start armpi-web
        
        if [ $? -eq 0 ]; then
            echo "✅ Service started successfully"
            echo ""
            echo "📊 Service Status:"
            sudo systemctl status armpi-web --no-pager -l
        else
            echo "❌ Failed to start service"
            echo "   Check logs with: sudo journalctl -u armpi-web -f"
        fi
    fi
}

# Check if we have sudo access for service installation
if sudo -n true 2>/dev/null; then
    echo "🔑 Sudo access available"
    install_service
else
    echo "🔑 Sudo access required for service installation"
    echo ""
    echo "Options:"
    echo "1. Run with sudo: sudo $0"
    echo "2. Install manually:"
    echo "   sudo cp $SERVICE_FILE /etc/systemd/system/armpi-web.service"
    echo "   sudo systemctl daemon-reload"
    echo "   sudo systemctl enable armpi-web"
    echo "   sudo systemctl start armpi-web"
    echo ""
    
    read -p "🤔 Do you want to proceed with sudo installation? (y/N): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_service
    else
        echo "📄 Service file created at: $SERVICE_FILE"
        echo "   You can install it manually when ready"
    fi
fi

# Cleanup temporary service file
rm -f "$SERVICE_FILE"

echo ""
echo "================================================"
echo "🎉 Installation Complete!"
echo "================================================"
echo ""
echo "📱 Access the web interface at:"
echo "   http://localhost:8000"
echo "   http://$(hostname -I | awk '{print $1}' 2>/dev/null || echo '[your-ip]'):8000"
echo ""
echo "🔧 Service Management Commands:"
echo "   sudo systemctl status armpi-web     # Check status"
echo "   sudo systemctl start armpi-web      # Start service"
echo "   sudo systemctl stop armpi-web       # Stop service"
echo "   sudo systemctl restart armpi-web    # Restart service"
echo "   sudo journalctl -u armpi-web -f     # View logs"
echo ""
echo "🗑️  To uninstall:"
echo "   sudo systemctl stop armpi-web"
echo "   sudo systemctl disable armpi-web"
echo "   sudo rm /etc/systemd/system/armpi-web.service"
echo "   sudo systemctl daemon-reload"
echo ""