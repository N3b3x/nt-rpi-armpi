# 🔧 ArmPi Mini Web Server Service Setup

This guide shows how to set up the ArmPi Mini Web Server as a system service that starts automatically on boot.

## 🚀 Quick Service Installation

```bash
# 1. Copy the service file to systemd
sudo cp armpi-web.service /etc/systemd/system/

# 2. Reload systemd configuration
sudo systemctl daemon-reload

# 3. Enable the service to start on boot
sudo systemctl enable armpi-web

# 4. Start the service immediately
sudo systemctl start armpi-web
```

## 📋 Service Management Commands

### Check Service Status
```bash
sudo systemctl status armpi-web
```

### Start the Service
```bash
sudo systemctl start armpi-web
```

### Stop the Service
```bash
sudo systemctl stop armpi-web
```

### Restart the Service
```bash
sudo systemctl restart armpi-web
```

### Enable Auto-start on Boot
```bash
sudo systemctl enable armpi-web
```

### Disable Auto-start on Boot
```bash
sudo systemctl disable armpi-web
```

### View Service Logs
```bash
# View recent logs
sudo journalctl -u armpi-web

# Follow logs in real-time
sudo journalctl -u armpi-web -f

# View logs since boot
sudo journalctl -u armpi-web -b
```

## 🔧 Configuration

### Custom Installation Path
If your ArmPi_mini installation is in a different location, edit the service file:

```bash
sudo nano /etc/systemd/system/armpi-web.service
```

Update these lines:
```ini
WorkingDirectory=/path/to/your/ArmPi_mini
ExecStart=/usr/bin/python3 /path/to/your/ArmPi_mini/web_server.py
```

### Custom User
To run the service as a different user, edit:
```ini
User=your_username
Group=your_group
```

## 🔍 Troubleshooting

### Service Won't Start
1. **Check file permissions**:
   ```bash
   ls -la /etc/systemd/system/armpi-web.service
   sudo chmod 644 /etc/systemd/system/armpi-web.service
   ```

2. **Verify Python path**:
   ```bash
   which python3
   ```

3. **Check working directory**:
   ```bash
   ls -la /home/pi/ArmPi_mini/web_server.py
   ```

### Service Fails to Start
1. **Check logs**:
   ```bash
   sudo journalctl -u armpi-web -n 50
   ```

2. **Test manual start**:
   ```bash
   cd /home/pi/ArmPi_mini
   python3 web_server.py
   ```

3. **Check dependencies**:
   ```bash
   python3 -c "import flask, cv2, numpy, jsonrpc2"
   ```

### Permission Issues
If you get permission errors:
```bash
# Make sure the user has access to camera and GPIO
sudo usermod -a -G video,gpio pi

# Restart the service
sudo systemctl restart armpi-web
```

## 🌐 Network Access

The service will bind to all network interfaces (0.0.0.0:8000). To access from other devices:

1. **Find the Pi's IP address**:
   ```bash
   hostname -I
   ```

2. **Access from browser**:
   ```
   http://[pi-ip-address]:8000
   ```

3. **Firewall configuration** (if needed):
   ```bash
   sudo ufw allow 8000
   ```

## 📊 Monitoring

### Service Health Check
Create a simple health check script:

```bash
#!/bin/bash
# health_check.sh
response=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000)
if [ $response = "200" ]; then
    echo "✅ ArmPi Web Server is healthy"
else
    echo "❌ ArmPi Web Server is not responding"
    sudo systemctl restart armpi-web
fi
```

### Automatic Monitoring with Cron
Add to crontab for automatic health checks:
```bash
# Edit crontab
crontab -e

# Add this line to check every 5 minutes
*/5 * * * * /path/to/health_check.sh
```

## 🔄 Updating the Service

When you update the web server code:

1. **Stop the service**:
   ```bash
   sudo systemctl stop armpi-web
   ```

2. **Update your code**:
   ```bash
   # Pull latest changes or update files
   ```

3. **Start the service**:
   ```bash
   sudo systemctl start armpi-web
   ```

4. **Check status**:
   ```bash
   sudo systemctl status armpi-web
   ```

## 🗑️ Complete Removal

To completely remove the service:

```bash
# Stop and disable the service
sudo systemctl stop armpi-web
sudo systemctl disable armpi-web

# Remove the service file
sudo rm /etc/systemd/system/armpi-web.service

# Reload systemd
sudo systemctl daemon-reload
```

## ⚠️ Important Notes

- The service runs with user privileges (not root) for security
- Camera and GPIO access requires the user to be in appropriate groups
- Service automatically restarts if it crashes (RestartSec=10)
- Logs are managed by systemd journal (no separate log files)
- The service waits for network before starting

## 🎯 Alternative: Running at Boot without systemd

For simpler setups, you can add to `/etc/rc.local`:

```bash
# Edit rc.local
sudo nano /etc/rc.local

# Add before 'exit 0':
cd /home/pi/ArmPi_mini && python3 web_server.py &
```

However, using systemd service is recommended for better process management and logging.