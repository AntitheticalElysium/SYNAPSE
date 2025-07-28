#!/bin/bash

# Check for yay
if ! command -v yay &> /dev/null; then
    echo "'yay' not found. Install yay before running this script."
    exit 1
fi

# Install Python 3.10
yay -S --noconfirm python310

# Create virtual environment
/usr/bin/python3.10 -m venv snn-env

# Activate and install requirements
source snn-env/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

