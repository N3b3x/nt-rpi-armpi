# ArmPi appliance v1

Non-ROS service for the cell brain. See
`experimental-test-setup/docs/architecture/appliance-armpi-v1.md`.

```bash
# Demo / CI (no Board required)
ARMPI_PORT=8000 python3 -m appliance

# Production: install armpi-appliance.service (or start from ArmPi_mini.py later)
# Token: export ARMPI_TOKEN=...
```

Do not install ROS 2 on the Pi for official cells (ADR-0007).
