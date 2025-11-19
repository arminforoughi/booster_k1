#!/bin/bash

# Deploy Booster K1 code to the robot

ROBOT_IP="192.168.88.153"
ROBOT_USER="booster"
ROBOT_PASSWORD="123456"
ROBOT_PATH="/home/booster/booster_k1"

echo "============================================================"
echo "🚀 DEPLOYING BOOSTER K1 CODE TO ROBOT"
echo "============================================================"
echo "Robot IP: $ROBOT_IP"
echo "Target path: $ROBOT_PATH"
echo ""

# Create directory on robot
echo "Creating directory on robot..."
sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" "mkdir -p $ROBOT_PATH"

# Copy all source files
echo "Copying source files..."
sshpass -p "$ROBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no \
    src/ \
    *.py \
    *.md \
    *.txt \
    *.sh \
    "$ROBOT_USER@$ROBOT_IP:$ROBOT_PATH/"

# Make scripts executable on robot
echo "Setting permissions..."
sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" "chmod +x $ROBOT_PATH/*.sh 2>/dev/null; chmod +x $ROBOT_PATH/src/*.py 2>/dev/null"

# Verify deployment
echo ""
echo "Verifying deployment..."
sshpass -p "$ROBOT_PASSWORD" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" "ls -la $ROBOT_PATH/src/ | head -10"

echo ""
echo "============================================================"
echo "✅ DEPLOYMENT COMPLETE"
echo "============================================================"
echo ""
echo "Files deployed to: $ROBOT_USER@$ROBOT_IP:$ROBOT_PATH"
echo ""
echo "Next steps:"
echo "1. Run tests: ./run_k1_tests.sh"
echo "2. Or SSH directly: sshpass -p '$ROBOT_PASSWORD' ssh $ROBOT_USER@$ROBOT_IP"
echo "   Then: cd $ROBOT_PATH && python3 src/basic_controls.py"
echo "============================================================"