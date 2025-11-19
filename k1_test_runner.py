#!/usr/bin/env python3
"""
K-1 Robot Test Runner
Executes safety checklist tests on the K-1 robot via SSH
"""

import subprocess
import time
import sys
import os
from datetime import datetime

# Configuration
ROBOT_IP = "192.168.88.153"
ROBOT_USER = "booster"
ROBOT_PASSWORD = "123456"
ROBOT_PATH = "/home/booster/booster_k1"  # Adjust if different

class K1TestRunner:
    def __init__(self):
        self.test_results = {}
        self.robot_connection = f"{ROBOT_USER}@{ROBOT_IP}"
        self.check_sshpass()
        
    def check_sshpass(self):
        """Check if sshpass is installed"""
        result = subprocess.run("which sshpass", shell=True, capture_output=True)
        if result.returncode != 0:
            print("⚠️  sshpass not found. Installing...")
            if sys.platform == "darwin":  # macOS
                subprocess.run("brew install hudochenkov/sshpass/sshpass", shell=True)
            else:  # Linux
                subprocess.run("sudo apt-get install -y sshpass", shell=True)
        
    def print_header(self, text):
        """Print formatted header"""
        print("\n" + "="*60)
        print(f"🔬 {text}")
        print("="*60)
        
    def print_status(self, test_name, success, message=""):
        """Print test status"""
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} - {test_name}")
        if message:
            print(f"   {message}")
        self.test_results[test_name] = success
        
    def run_ssh_command(self, command, timeout=10):
        """Run command on robot via SSH with password"""
        ssh_cmd = f"sshpass -p '{ROBOT_PASSWORD}' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 {self.robot_connection} '{command}'"
        try:
            result = subprocess.run(
                ssh_cmd,
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
    
    def test_phase1_environment(self):
        """Phase 1: Environment Setup Tests"""
        self.print_header("PHASE 1: Environment Setup")
        
        # Test 1.5: Can ping robot
        print("\n📍 Testing network connectivity...")
        ping_result = subprocess.run(
            f"ping -c 3 {ROBOT_IP}", 
            shell=True, 
            capture_output=True
        )
        self.print_status(
            "1.5 Can ping K1 robot", 
            ping_result.returncode == 0,
            f"Robot at {ROBOT_IP} is {'reachable' if ping_result.returncode == 0 else 'unreachable'}"
        )
        
        # Test SSH connection
        print("\n📍 Testing SSH connection...")
        success, stdout, stderr = self.run_ssh_command("echo 'SSH connection successful'")
        self.print_status(
            "SSH Connection", 
            success,
            stdout.strip() if success else stderr
        )
        
        return all([
            self.test_results.get("1.5 Can ping K1 robot", False),
            self.test_results.get("SSH Connection", False)
        ])
    
    def test_phase2_dependencies(self):
        """Phase 2: Dependency Check"""
        self.print_header("PHASE 2: Dependency Check")
        
        # Test 2.1: Python version
        print("\n📍 Checking Python version...")
        success, stdout, stderr = self.run_ssh_command("python3 --version")
        if success:
            version = stdout.strip()
            self.print_status("2.1 Python 3.8+", "3.8" in version or "3.9" in version or "3.10" in version or "3.11" in version, version)
        else:
            self.print_status("2.1 Python 3.8+", False, "Could not check Python version")
        
        # Test 2.4: ROS2 installed
        print("\n📍 Checking ROS2...")
        success, stdout, stderr = self.run_ssh_command("source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null && ros2 --version 2>/dev/null")
        self.print_status("2.4 ROS2 installed", success, stdout.strip() if success else "ROS2 not found")
        
        # Test 2.5: Camera topic exists
        print("\n📍 Checking camera topics...")
        success, stdout, stderr = self.run_ssh_command("source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | grep camera || source /opt/ros/foxy/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null | grep camera")
        self.print_status("2.5 Camera topic exists", success and "camera" in stdout, "Camera topics found" if success else "No camera topics")
        
        # Test 2.6: Check for TTS
        print("\n📍 Checking TTS installation...")
        success, stdout, stderr = self.run_ssh_command("which piper || which espeak")
        self.print_status("2.6 TTS installed", success, stdout.strip() if success else "No TTS found (optional)")
        
        return True  # Dependencies are informational
    
    def test_phase3_software(self):
        """Phase 3: Software Tests (Without Motors)"""
        self.print_header("PHASE 3: Software Tests (Without Motors)")
        
        # Test 3.1-3.4: Basic connection to robot
        print("\n📍 Testing basic_controls.py connection...")
        print("⚠️  IMPORTANT: Robot should be in DAMPING mode (motors off)")
        input("Press Enter when robot is in DAMPING mode...")
        
        # Run basic_controls.py briefly to test connection
        test_cmd = f"""cd {ROBOT_PATH} && timeout 3 python3 src/basic_controls.py 2>&1 | head -20"""
        success, stdout, stderr = self.run_ssh_command(test_cmd, timeout=10)
        
        # Check for expected outputs
        emergency_stop_active = "Emergency stop system active" in stdout
        connection_successful = "Connection successful" in stdout
        mode_shown = "Current robot mode" in stdout
        
        self.print_status("3.1 Basic controls connects", success or emergency_stop_active)
        self.print_status("3.2 Emergency stop message", emergency_stop_active)
        self.print_status("3.3 Connection verified", connection_successful or mode_shown)
        
        if mode_shown:
            # Extract and show current mode
            for line in stdout.split('\n'):
                if "Current robot mode" in line:
                    print(f"   📊 {line.strip()}")
                    if "Damping" not in line:
                        print("   ⚠️  WARNING: Robot is NOT in Damping mode! Please switch to Damping for safety.")
        
        # Test 3.5-3.7: Camera feed
        print("\n📍 Testing camera feed...")
        test_cmd = f"""cd {ROBOT_PATH} && timeout 3 python3 src/basic_cam.py 2>&1 | grep -E 'Camera topic|Web server'"""
        success, stdout, stderr = self.run_ssh_command(test_cmd, timeout=10)
        self.print_status("3.5 Camera feed starts", success and "Camera topic" in stdout)
        
        # Test 3.8-3.9: TTS System
        print("\n📍 Testing TTS system...")
        test_cmd = f"""cd {ROBOT_PATH} && python3 -c "
from src.tts_module import TextToSpeech
tts = TextToSpeech(engine='auto')
if tts.available:
    print('TTS available:', tts.engine)
    tts.speak('K1 robot test', blocking=True)
    print('TTS test complete')
else:
    print('TTS not available')
" 2>&1"""
        success, stdout, stderr = self.run_ssh_command(test_cmd, timeout=10)
        self.print_status("3.8-3.9 TTS system", "TTS available" in stdout or "TTS not available" in stdout, stdout.strip())
        
        return True
    
    def test_phase4_emergency_stop(self):
        """Phase 4: Safety Verification - Emergency Stop Test"""
        self.print_header("PHASE 4: Safety Verification - Emergency Stop")
        
        print("\n⚠️  CRITICAL: This will test the emergency stop system")
        print("   The robot MUST be in DAMPING mode for this test")
        response = input("Type 'yes' to proceed with emergency stop test: ")
        
        if response.lower() != 'yes':
            print("❌ Emergency stop test skipped")
            return False
        
        # Test Ctrl+C emergency stop
        print("\n📍 Testing Ctrl+C emergency stop...")
        test_cmd = f"""cd {ROBOT_PATH} && timeout 2 python3 src/basic_controls.py 2>&1"""
        success, stdout, stderr = self.run_ssh_command(test_cmd, timeout=5)
        
        emergency_triggered = "EMERGENCY STOP" in stdout or "Emergency stop commands sent" in stdout
        self.print_status("4.1 Emergency stop triggers", emergency_triggered)
        
        if emergency_triggered:
            print("   ✅ Emergency stop system is working correctly")
        else:
            print("   ⚠️  Could not verify emergency stop - manual verification required")
        
        return True
    
    def run_all_tests(self):
        """Run all test phases"""
        print("\n" + "="*60)
        print("🚨 K-1 ROBOT SAFETY TEST SUITE")
        print("="*60)
        print(f"Robot IP: {ROBOT_IP}")
        print(f"Test started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("\n⚠️  SAFETY REMINDER:")
        print("  - Robot should be in DAMPING mode initially")
        print("  - Have emergency stop button ready")
        print("  - Maintain 2+ meter clear space")
        print("="*60)
        
        # Phase 1: Environment
        if not self.test_phase1_environment():
            print("\n❌ Phase 1 failed - cannot continue")
            return False
        
        # Phase 2: Dependencies  
        self.test_phase2_dependencies()
        
        # Phase 3: Software Tests
        self.test_phase3_software()
        
        # Phase 4: Emergency Stop
        self.test_phase4_emergency_stop()
        
        # Summary
        self.print_summary()
        
        return True
    
    def print_summary(self):
        """Print test summary"""
        self.print_header("TEST SUMMARY")
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for v in self.test_results.values() if v)
        
        print(f"\nTotal Tests: {total_tests}")
        print(f"Passed: {passed_tests}")
        print(f"Failed: {total_tests - passed_tests}")
        
        if passed_tests == total_tests:
            print("\n✅ All automated tests PASSED")
            print("\n📋 NEXT STEPS:")
            print("1. Verify robot is elevated or has 2+ meter clear space")
            print("2. Have emergency stop button ready")
            print("3. Run Phase 5: Motor Test (HIGH RISK)")
            print("   ssh booster@192.168.88.153")
            print("   cd booster_k1")
            print("   python3 src/basic_controls.py")
            print("   Then type 'mp' to enter Prepare mode")
        else:
            print("\n⚠️  Some tests failed - review before proceeding")
            print("\nFailed tests:")
            for test, passed in self.test_results.items():
                if not passed:
                    print(f"  ❌ {test}")

def main():
    runner = K1TestRunner()
    
    try:
        runner.run_all_tests()
    except KeyboardInterrupt:
        print("\n\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()