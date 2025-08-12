#!/usr/bin/env python3
"""
Camera Calibration Fix Tool
This script helps diagnose and fix camera calibration issues that cause blurring.
"""
import os
import shutil
import sys
import datetime

def backup_calibration_files():
    """Backup existing calibration files"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = f"calibration_backup_{timestamp}"
    
    if not os.path.exists(backup_dir):
        os.makedirs(backup_dir)
    
    files_to_backup = [
        "/workspace/CameraCalibration/calibration_param.npz",
        "/workspace/imx477/calibration_data.npz"
    ]
    
    backed_up = []
    for file_path in files_to_backup:
        if os.path.exists(file_path):
            backup_path = os.path.join(backup_dir, os.path.basename(file_path))
            shutil.copy2(file_path, backup_path)
            backed_up.append(file_path)
            print(f"✅ Backed up: {file_path} -> {backup_path}")
    
    if backed_up:
        print(f"\n📁 Backup created in: {backup_dir}")
        return backup_dir, backed_up
    else:
        print("⚠️  No calibration files found to backup")
        return None, []

def remove_old_calibration():
    """Remove the problematic old calibration file"""
    old_calib_file = "/workspace/CameraCalibration/calibration_param.npz"
    
    if os.path.exists(old_calib_file):
        os.remove(old_calib_file)
        print(f"🗑️  Removed problematic calibration file: {old_calib_file}")
        print("   This will prevent automatic calibration correction in Camera.py")
        return True
    else:
        print("ℹ️  Old calibration file not found (already removed?)")
        return False

def check_calibration_status():
    """Check the current status of calibration files"""
    print("🔍 Checking calibration status...")
    print("=" * 50)
    
    # Check old system
    old_file = "/workspace/CameraCalibration/calibration_param.npz"
    if os.path.exists(old_file):
        size = os.path.getsize(old_file)
        print(f"📁 Old calibration file: EXISTS ({size} bytes)")
        print("   ⚠️  This file causes AUTOMATIC calibration in Camera.py")
        print("   ⚠️  This is likely the source of your blurring!")
    else:
        print("📁 Old calibration file: NOT FOUND")
        print("   ✅ Good! No automatic calibration will be applied")
    
    # Check new system  
    new_file = "/workspace/imx477/calibration_data.npz"
    if os.path.exists(new_file):
        size = os.path.getsize(new_file)
        print(f"📁 IMX477 calibration file: EXISTS ({size} bytes)")
        print("   ℹ️  This is used optionally in IMX477 modules (toggle with 'u' key)")
    else:
        print("📁 IMX477 calibration file: NOT FOUND")
    
    print()

def main():
    print("Camera Calibration Fix Tool")
    print("===========================")
    print("This tool helps fix blurring caused by bad camera calibration.")
    print()
    
    # Check current status
    check_calibration_status()
    
    print("📋 Available options:")
    print("1. Backup all calibration files (recommended first step)")
    print("2. Remove problematic old calibration (fixes blur in Camera.py)")
    print("3. Check status again")
    print("4. Restore from backup")
    print("q. Quit")
    print()
    
    while True:
        choice = input("Choose an option (1/2/3/4/q): ").strip().lower()
        
        if choice == '1':
            print("\n📦 Creating backup...")
            backup_dir, backed_up = backup_calibration_files()
            if backed_up:
                print(f"✅ Backup completed! Files saved in: {backup_dir}")
            
        elif choice == '2':
            print("\n🔧 Removing problematic calibration...")
            
            # First check if backup exists
            if not any(d.startswith('calibration_backup_') for d in os.listdir('.') if os.path.isdir(d)):
                print("⚠️  No backup found! Creating backup first...")
                backup_dir, backed_up = backup_calibration_files()
            
            removed = remove_old_calibration()
            if removed:
                print("\n✅ Fix applied! The old Camera.py will now use raw camera feed.")
                print("   🔄 You may need to restart any running camera applications.")
                print("   📝 You can still use calibration in IMX477 modules by pressing 'u'")
            
        elif choice == '3':
            print()
            check_calibration_status()
            
        elif choice == '4':
            print("\n📂 Available backups:")
            backups = [d for d in os.listdir('.') if d.startswith('calibration_backup_') and os.path.isdir(d)]
            if not backups:
                print("No backups found.")
            else:
                for i, backup in enumerate(backups, 1):
                    print(f"{i}. {backup}")
                
                try:
                    selection = int(input("Enter backup number to restore (0 to cancel): "))
                    if 1 <= selection <= len(backups):
                        backup_dir = backups[selection - 1]
                        print(f"Restoring from {backup_dir}...")
                        
                        # Restore files
                        for file_name in os.listdir(backup_dir):
                            src = os.path.join(backup_dir, file_name)
                            if file_name == "calibration_param.npz":
                                dst = "/workspace/CameraCalibration/calibration_param.npz"
                            elif file_name == "calibration_data.npz":
                                dst = "/workspace/imx477/calibration_data.npz"
                            else:
                                continue
                            
                            shutil.copy2(src, dst)
                            print(f"✅ Restored: {dst}")
                        
                        print("🔄 Calibration files restored!")
                    
                except ValueError:
                    print("Invalid selection.")
            
        elif choice == 'q':
            break
        else:
            print("Invalid choice, please try again.")
        
        print()
    
    print("🏁 Calibration fix tool completed!")
    print("\n💡 Recommendations:")
    print("   • Test camera with: python3 test_camera_calibration.py")
    print("   • If still blurry, run proper calibration with checkerboard")
    print("   • Use IMX477 modules for better camera control")

if __name__ == '__main__':
    main()