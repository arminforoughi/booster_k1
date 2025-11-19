#!/usr/bin/env python3
"""
K-1 Robot Safe Test Script
Follows proper mode transitions: Damping -> Prepare -> Walking
"""

import subprocess
import time
import sys
from datetime import datetime

ROBOT_IP = "192.168.88.153"
ROBOT_USER = "booster"
ROBOT_PASSWORD = "123456"
ROBOT_PATH = "/home/booster/booster_k1"

def ssh_cmd(command, timeout=10):
    """Execute command on robot via SSH"""
    ssh_command = f"sshpass -p '{ROBOT_PASSWORD}' ssh -o StrictHostKeyChecking=no {ROBOT_USER}@{ROBOT_IP} '{command}'"
    try:
        result = subprocess.run(
            ssh_command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)

def print_header(text):
    print("\n" + "="*60)
    print(f"🤖 {text}")
    print("="*60)

def get_robot_mode():
    """Get current robot mode using basic_controls.py"""
    print("📊 Checking current robot mode...")
    cmd = f"""cd {ROBOT_PATH} && python3 -c "
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, GetModeResponse, RobotMode
import sys
ChannelFactory.Instance().Init(0, '127.0.0.1')
client = B1LocoClient()
client.Init()
gm = GetModeResponse()
res = client.GetMode(gm)
if res == 0:
    mode_names = {{
        RobotMode.kDamping: 'Damping',
        RobotMode.kPrepare: 'Prepare',
        RobotMode.kWalking: 'Walking',
        RobotMode.kCustom: 'Custom',
        RobotMode.kUnknown: 'Unknown'
    }}
    mode_name = mode_names.get(gm.mode, f'Unknown ({{gm.mode}})')
    print(f'Current mode: {{mode_name}}')
else:
    print(f'Error getting mode: {{res}}')
" 2>&1"""
    
    success, stdout, stderr = ssh_cmd(cmd)
    if success and "Current mode:" in stdout:
        for line in stdout.split('\n'):
            if "Current mode:" in line:
                print(f"   {line.strip()}")
                return line.strip()
    else:
        print(f"   ⚠️  Could not determine mode")
        return None

def main():
    print_header("K-1 ROBOT SAFE TEST SUITE")
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # SAFETY CHECK
    print("\n" + "⚠️ "*10)
    print("SAFETY CHECKLIST:")
    print("1. Robot has 2+ meter clear space around it")
    print("2. Emergency stop button is accessible")
    print("3. You are ready to press emergency stop if needed")
    print("4. Robot is on stable, flat surface")
    print("⚠️ "*10)
    
    response = input("\nType 'YES' if all safety conditions are met: ")
    if response.upper() != 'YES':
        print("❌ Test aborted for safety")
        return
    
    # Phase 1: Check connection
    print_header("PHASE 1: Connection Test")
    print("Testing SSH connection...")
    success, stdout, stderr = ssh_cmd("echo 'Connected'")
    if success:
        print("✅ SSH connection successful")
    else:
        print("❌ SSH connection failed")
        return
    
    # Phase 2: Check robot mode
    print_header("PHASE 2: Robot Mode Check")
    current_mode = get_robot_mode()
    
    if current_mode and "Damping" in current_mode:
        print("✅ Robot is in DAMPING mode (motors off) - Safe to proceed")
    elif current_mode and "Prepare" in current_mode:
        print("⚠️  Robot is in PREPARE mode (standing)")
        print("   The robot is already standing. This is OK.")
    elif current_mode and "Walking" in current_mode:
        print("⚠️  Robot is in WALKING mode")
        print("   For safety, we should return to Damping first")
        response = input("Return to Damping mode? (yes/no): ")
        if response.lower() == 'yes':
            print("Switching to Damping mode...")
            cmd = f"cd {ROBOT_PATH} && timeout 5 python3 -c \"from booster_robotics_sdk_python import *; import sys; ChannelFactory.Instance().Init(0, '127.0.0.1'); client = B1LocoClient(); client.Init(); client.ChangeMode(RobotMode.kDamping); print('Mode changed')\""
            ssh_cmd(cmd)
            time.sleep(2)
    
    # Phase 3: Test basic controls connection
    print_header("PHASE 3: Basic Controls Test")
    print("Testing basic_controls.py...")
    print("This will verify emergency stop system is active")
    
    cmd = f"cd {ROBOT_PATH} && timeout 3 python3 src/basic_controls.py 127.0.0.1 2>&1 | head -20"
    success, stdout, stderr = ssh_cmd(cmd, timeout=10)
    
    if "Emergency stop system active" in stdout:
        print("✅ Emergency stop system is active")
    else:
        print("⚠️  Could not verify emergency stop system")
    
    if "Connection successful" in stdout:
        print("✅ Robot connection verified")
    
    # Phase 4: Mode transition test (optional)
    print_header("PHASE 4: Mode Transition Test (OPTIONAL)")
    print("\n⚠️  This will change robot modes:")
    print("   Damping -> Prepare (robot stands) -> Damping")
    
    response = input("\nProceed with mode transition test? (yes/no): ")
    if response.lower() == 'yes':
        # Get current mode first
        current = get_robot_mode()
        
        if current and "Prepare" not in current:
            print("\n1. Switching to PREPARE mode (robot will stand)...")
            input("Press Enter when ready (have emergency stop ready)...")
            
            cmd = f"""cd {ROBOT_PATH} && python3 -c "
from booster_robotics_sdk_python import *
import sys, time
ChannelFactory.Instance().Init(0, '127.0.0.1')
client = B1LocoClient()
client.Init()
print('Changing to Prepare mode...')
res = client.ChangeMode(RobotMode.kPrepare)
print(f'Result: {{res}}')
time.sleep(3)
" 2>&1"""
            success, stdout, stderr = ssh_cmd(cmd, timeout=10)
            print(stdout)
            
            time.sleep(3)
            get_robot_mode()
        
        print("\n2. Returning to DAMPING mode (motors off)...")
        cmd = f"""cd {ROBOT_PATH} && python3 -c "
from booster_robotics_sdk_python import *
import sys, time
ChannelFactory.Instance().Init(0, '127.0.0.1')
client = B1LocoClient()
client.Init()
print('Changing to Damping mode...')
res = client.ChangeMode(RobotMode.kDamping)
print(f'Result: {{res}}')
" 2>&1"""
        success, stdout, stderr = ssh_cmd(cmd, timeout=10)
        print(stdout)
        
        time.sleep(2)
        get_robot_mode()
    
    # Phase 5: Movement test (HIGH RISK)
    print_header("PHASE 5: Movement Test (HIGH RISK)")
    print("\n" + "⚠️ "*10)
    print("MOVEMENT TEST - ROBOT WILL MOVE!")
    print("Required sequence:")
    print("1. Damping -> 2. Prepare -> 3. Walking -> 4. Send movement commands")
    print("⚠️ "*10)
    
    response = input("\nProceed with MOVEMENT test? (yes/no): ")
    if response.lower() != 'yes':
        print("Movement test skipped")
    else:
        print("\nFor movement test, you should:")
        print("1. SSH into robot: sshpass -p '123456' ssh booster@192.168.88.153")
        print("2. cd /home/booster/booster_k1")
        print("3. python3 src/basic_controls.py")
        print("4. Type 'mp' for Prepare mode (stand)")
        print("5. Type 'mw' for Walking mode")
        print("6. Use WASD keys for movement")
        print("7. Type 'md' to return to Damping mode")
        print("8. Ctrl+C to exit (triggers emergency stop)")
    
    # Summary
    print_header("TEST COMPLETE")
    print("✅ Basic tests completed")
    print("\nREMEMBER:")
    print("- Always start in Damping mode for safety")
    print("- Mode sequence: Damping -> Prepare -> Walking")
    print("- Have emergency stop ready at all times")
    print("- Test movement only with proper safety precautions")

if __name__ == "__main__":
    main()