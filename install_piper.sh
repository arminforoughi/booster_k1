#!/bin/bash
#
# Piper TTS Installation Script for ARM/Jetson
# Downloads and installs Piper TTS with a lightweight voice model
#

set -e

echo "========================================"
echo "Piper TTS Installation for ARM/Jetson"
echo "========================================"
echo

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Determine download URL based on architecture
if [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
    PIPER_ARCH="arm64"
elif [[ "$ARCH" == "armv7l" ]]; then
    PIPER_ARCH="armv7l"
else
    echo "❌ Unsupported architecture: $ARCH"
    echo "   Piper TTS is designed for ARM devices (aarch64/armv7l)"
    exit 1
fi

# Piper version
PIPER_VERSION="1.2.0"
PIPER_URL="https://github.com/rhasspy/piper/releases/download/${PIPER_VERSION}/piper_linux_${PIPER_ARCH}.tar.gz"

echo "Piper URL: $PIPER_URL"
echo

# Create installation directory
INSTALL_DIR="$HOME/.local/share/piper"
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

# Download Piper if not already installed
if [ -f "./piper/piper" ]; then
    echo "✓ Piper binary already exists"
else
    echo "Downloading Piper TTS..."
    wget -q --show-progress "$PIPER_URL" -O piper.tar.gz

    echo "Extracting..."
    tar -xzf piper.tar.gz
    rm piper.tar.gz

    echo "✓ Piper binary installed"
fi

# Create symlink to make piper available in PATH
echo
echo "Creating symlink..."
mkdir -p "$HOME/.local/bin"
ln -sf "$INSTALL_DIR/piper/piper" "$HOME/.local/bin/piper"

# Add to PATH if not already there
if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    export PATH="$HOME/.local/bin:$PATH"
    echo "✓ Added ~/.local/bin to PATH"
fi

# Download a lightweight voice model (en_US-lessac-medium)
echo
echo "Downloading voice model..."
VOICE_DIR="$INSTALL_DIR/voices"
mkdir -p "$VOICE_DIR"

VOICE_NAME="en_US-lessac-medium"
VOICE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx"
VOICE_CONFIG_URL="https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json"

if [ -f "$VOICE_DIR/$VOICE_NAME.onnx" ]; then
    echo "✓ Voice model already exists"
else
    echo "Downloading $VOICE_NAME..."
    wget -q --show-progress "$VOICE_URL" -O "$VOICE_DIR/$VOICE_NAME.onnx"
    wget -q "$VOICE_CONFIG_URL" -O "$VOICE_DIR/$VOICE_NAME.onnx.json"
    echo "✓ Voice model installed"
fi

# Create a symlink to default voice location
mkdir -p "$HOME/.local/share/piper-voices"
ln -sf "$VOICE_DIR/$VOICE_NAME.onnx" "$HOME/.local/share/piper-voices/default.onnx"
ln -sf "$VOICE_DIR/$VOICE_NAME.onnx.json" "$HOME/.local/share/piper-voices/default.onnx.json"

# Test installation
echo
echo "Testing Piper TTS..."
if echo "Hello, this is a test of Piper text to speech" | "$HOME/.local/bin/piper" --model "$VOICE_DIR/$VOICE_NAME.onnx" --output_file /tmp/piper_test.wav 2>/dev/null; then
    echo "✓ Piper TTS test successful"

    # Try to play the test audio
    if command -v aplay &> /dev/null; then
        echo "Playing test audio..."
        aplay -q /tmp/piper_test.wav 2>/dev/null || true
    fi

    rm -f /tmp/piper_test.wav
else
    echo "⚠️  Piper test had issues (but may still work)"
fi

echo
echo "========================================"
echo "✅ Piper TTS Installation Complete!"
echo "========================================"
echo
echo "Installation details:"
echo "  Binary: $HOME/.local/bin/piper"
echo "  Voice model: $VOICE_DIR/$VOICE_NAME.onnx"
echo "  Default voice: $HOME/.local/share/piper-voices/default.onnx"
echo
echo "To use Piper in a new terminal:"
echo "  1. Open a new terminal (to load updated PATH)"
echo "  2. Run: python src/smart_recognition.py"
echo
echo "Or in current terminal:"
echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
echo "  python src/smart_recognition.py"
echo
