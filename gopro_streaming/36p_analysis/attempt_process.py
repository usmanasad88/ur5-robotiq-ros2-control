#!/usr/bin/env python3
"""
GoPro .36p Processing Attempts
Tries multiple methods to convert .36p to equirectangular JPG
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path
from datetime import datetime


def print_separator(title=""):
    """Print a formatted separator"""
    if title:
        print(f"\n{'='*70}")
        print(f" {title}")
        print('='*70)
    else:
        print('='*70)


def check_dependencies():
    """Check if required tools are installed"""
    print_separator("Checking Dependencies")
    
    tools = {
        'ffmpeg': 'sudo apt-get install ffmpeg',
        'convert': 'sudo apt-get install imagemagick',
        'heif-convert': 'sudo apt-get install libheif-examples'
    }
    
    available = {}
    for tool, install_cmd in tools.items():
        try:
            result = subprocess.run(['which', tool], capture_output=True, check=True)
            print(f"✓ {tool} found: {result.stdout.decode().strip()}")
            available[tool] = True
        except subprocess.CalledProcessError:
            print(f"✗ {tool} not found. Install with: {install_cmd}")
            available[tool] = False
    
    return available


def run_command(cmd, log_file=None):
    """Run a shell command and capture output"""
    print(f"\n🚀 Running: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60
        )
        
        # Print stdout
        if result.stdout:
            print("STDOUT:", result.stdout[:500])
        
        # Print stderr
        if result.stderr:
            print("STDERR:", result.stderr[:500])
        
        # Save to log if specified
        if log_file:
            with open(log_file, 'a') as f:
                f.write(f"\n{'='*70}\n")
                f.write(f"Command: {' '.join(cmd)}\n")
                f.write(f"Timestamp: {datetime.now()}\n")
                f.write(f"Return Code: {result.returncode}\n")
                f.write(f"\nSTDOUT:\n{result.stdout}\n")
                f.write(f"\nSTDERR:\n{result.stderr}\n")
        
        return result.returncode == 0, result
        
    except subprocess.TimeoutExpired:
        print("❌ Command timed out after 60 seconds")
        return False, None
    except Exception as e:
        print(f"❌ Error running command: {e}")
        return False, None


def method_a_direct_ffmpeg(input_file, output_dir, log_file):
    """Method A: Direct FFmpeg v360 filter"""
    print_separator("Method A: Direct FFmpeg v360 (EAC to Equirect)")
    
    output_file = output_dir / "output_A_direct.jpg"
    
    # Try assuming it's already in a processable format
    cmd = [
        'ffmpeg', '-y',
        '-i', str(input_file),
        '-vf', 'v360=eac:equirect',
        '-q:v', '2',
        str(output_file)
    ]
    
    success, result = run_command(cmd, log_file)
    
    if success and output_file.exists():
        print(f"✅ Success! Output: {output_file}")
        print(f"   Size: {output_file.stat().st_size:,} bytes")
        return True, output_file
    else:
        print(f"❌ Method A failed")
        return False, None


def method_b_copy_extension(input_file, output_dir, log_file):
    """Method B: Copy with .jpg extension and try FFmpeg"""
    print_separator("Method B: Copy as .jpg then FFmpeg")
    
    temp_jpg = output_dir / "test_B_input.jpg"
    output_file = output_dir / "output_B_copied.jpg"
    
    # Copy file
    print(f"📋 Copying {input_file} to {temp_jpg}")
    shutil.copy(input_file, temp_jpg)
    
    cmd = [
        'ffmpeg', '-y',
        '-i', str(temp_jpg),
        '-vf', 'v360=eac:equirect',
        '-q:v', '2',
        str(output_file)
    ]
    
    success, result = run_command(cmd, log_file)
    
    if success and output_file.exists():
        print(f"✅ Success! Output: {output_file}")
        print(f"   Size: {output_file.stat().st_size:,} bytes")
        return True, output_file
    else:
        print(f"❌ Method B failed")
        return False, None


def method_c_heic_convert(input_file, output_dir, log_file, tools_available):
    """Method C: Convert from HEIC to JPG first"""
    print_separator("Method C: HEIC Conversion First")
    
    if not tools_available.get('heif-convert', False):
        print("⚠️  Skipping - heif-convert not available")
        return False, None
    
    temp_jpg = output_dir / "test_C_converted.jpg"
    output_file = output_dir / "output_C_heic.jpg"
    
    # Try HEIC conversion
    cmd_convert = ['heif-convert', str(input_file), str(temp_jpg)]
    success, result = run_command(cmd_convert, log_file)
    
    if not success or not temp_jpg.exists():
        print("❌ HEIC conversion failed")
        return False, None
    
    print(f"✓ HEIC converted to {temp_jpg}")
    
    # Now try FFmpeg
    cmd_ffmpeg = [
        'ffmpeg', '-y',
        '-i', str(temp_jpg),
        '-vf', 'v360=eac:equirect',
        '-q:v', '2',
        str(output_file)
    ]
    
    success, result = run_command(cmd_ffmpeg, log_file)
    
    if success and output_file.exists():
        print(f"✅ Success! Output: {output_file}")
        print(f"   Size: {output_file.stat().st_size:,} bytes")
        return True, output_file
    else:
        print(f"❌ Method C failed")
        return False, None


def method_d_imagemagick(input_file, output_dir, log_file, tools_available):
    """Method D: ImageMagick conversion"""
    print_separator("Method D: ImageMagick Convert")
    
    if not tools_available.get('convert', False):
        print("⚠️  Skipping - ImageMagick not available")
        return False, None
    
    output_file = output_dir / "output_D_imagemagick.jpg"
    
    cmd = ['convert', str(input_file), str(output_file)]
    success, result = run_command(cmd, log_file)
    
    if success and output_file.exists():
        print(f"✅ Success! Output: {output_file}")
        print(f"   Size: {output_file.stat().st_size:,} bytes")
        
        # Try FFmpeg on the converted image
        output_file_stitched = output_dir / "output_D_stitched.jpg"
        cmd_stitch = [
            'ffmpeg', '-y',
            '-i', str(output_file),
            '-vf', 'v360=eac:equirect',
            '-q:v', '2',
            str(output_file_stitched)
        ]
        
        success2, _ = run_command(cmd_stitch, log_file)
        if success2:
            print(f"✅ Stitching also successful: {output_file_stitched}")
        
        return True, output_file
    else:
        print(f"❌ Method D failed")
        return False, None


def method_e_raw_extract(input_file, output_dir, log_file):
    """Method E: Extract JPEG if embedded"""
    print_separator("Method E: Extract Embedded JPEG")
    
    try:
        with open(input_file, 'rb') as f:
            content = f.read()
        
        # Look for JPEG markers
        jpeg_start = content.find(b'\xff\xd8\xff')
        jpeg_end = content.find(b'\xff\xd9', jpeg_start)
        
        if jpeg_start >= 0 and jpeg_end > jpeg_start:
            print(f"✓ Found JPEG data at offset {jpeg_start}")
            
            output_file = output_dir / "output_E_extracted.jpg"
            with open(output_file, 'wb') as out:
                out.write(content[jpeg_start:jpeg_end+2])
            
            print(f"💾 Extracted JPEG: {output_file}")
            print(f"   Size: {output_file.stat().st_size:,} bytes")
            
            # Try to stitch it
            output_stitched = output_dir / "output_E_stitched.jpg"
            cmd = [
                'ffmpeg', '-y',
                '-i', str(output_file),
                '-vf', 'v360=eac:equirect',
                '-q:v', '2',
                str(output_stitched)
            ]
            
            success, _ = run_command(cmd, log_file)
            if success:
                print(f"✅ Stitching successful: {output_stitched}")
                return True, output_stitched
            
            return True, output_file
        else:
            print("❌ No JPEG markers found")
            return False, None
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False, None


def main():
    if len(sys.argv) < 2:
        print("Usage: python attempt_process.py <path_to_36p_file>")
        print("\nExample:")
        print("  python attempt_process.py raw_samples/GS010013.36P")
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    
    if not input_file.exists():
        print(f"❌ Error: File not found: {input_file}")
        sys.exit(1)
    
    # Setup output directory
    output_dir = Path("output_attempts")
    output_dir.mkdir(exist_ok=True)
    
    log_file = output_dir / f"processing_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    print(f"\n🎬 GoPro .36p Processing Suite")
    print(f"Input: {input_file}")
    print(f"Output Directory: {output_dir}")
    print(f"Log File: {log_file}")
    
    # Check dependencies
    tools = check_dependencies()
    
    # Try all methods
    results = {}
    
    methods = [
        ("A: Direct FFmpeg", lambda: method_a_direct_ffmpeg(input_file, output_dir, log_file)),
        ("B: Copy as JPG", lambda: method_b_copy_extension(input_file, output_dir, log_file)),
        ("C: HEIC Convert", lambda: method_c_heic_convert(input_file, output_dir, log_file, tools)),
        ("D: ImageMagick", lambda: method_d_imagemagick(input_file, output_dir, log_file, tools)),
        ("E: Raw Extract", lambda: method_e_raw_extract(input_file, output_dir, log_file)),
    ]
    
    for method_name, method_func in methods:
        try:
            success, output = method_func()
            results[method_name] = (success, output)
        except Exception as e:
            print(f"❌ {method_name} crashed: {e}")
            results[method_name] = (False, None)
    
    # Summary
    print_separator("Final Summary")
    print(f"\nProcessing Results for: {input_file.name}")
    print(f"Log saved to: {log_file}\n")
    
    successful = []
    for method_name, (success, output) in results.items():
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"{status}: {method_name}")
        if output:
            print(f"         Output: {output}")
            successful.append(output)
    
    if successful:
        print(f"\n🎉 {len(successful)} method(s) succeeded!")
        print(f"\nView results with:")
        for output in successful:
            print(f"  eog {output}")
    else:
        print(f"\n⚠️  All methods failed. Check the log file for details.")
        print(f"\nNext steps:")
        print("  1. Review inspection_logs from inspect_format.py")
        print("  2. Check if file needs unpacking first")
        print("  3. Consult GoPro documentation for .36p format")


if __name__ == "__main__":
    main()
