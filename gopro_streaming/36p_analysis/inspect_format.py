#!/usr/bin/env python3
"""
GoPro .36p File Inspector
Analyzes file structure, magic numbers, and attempts to identify format
"""

import os
import sys
import subprocess
import zipfile
import struct
from pathlib import Path


def print_separator(title=""):
    """Print a formatted separator"""
    if title:
        print(f"\n{'='*70}")
        print(f" {title}")
        print('='*70)
    else:
        print('='*70)


def run_file_command(filepath):
    """Run the Linux 'file' command"""
    print_separator("Linux 'file' Command Output")
    try:
        result = subprocess.run(['file', '-b', filepath], 
                              capture_output=True, text=True, check=True)
        print(result.stdout.strip())
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Error running 'file' command: {e}")
        return None
    except FileNotFoundError:
        print("'file' command not found. Install with: sudo apt-get install file")
        return None


def read_magic_numbers(filepath, num_bytes=32):
    """Read and display the first N bytes (magic numbers) in hex"""
    print_separator(f"First {num_bytes} Bytes (Magic Numbers)")
    try:
        with open(filepath, 'rb') as f:
            magic_bytes = f.read(num_bytes)
        
        # Print hex dump
        hex_str = ' '.join(f'{b:02X}' for b in magic_bytes)
        print(f"Hex: {hex_str}")
        
        # Print ASCII interpretation (printable only)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in magic_bytes)
        print(f"ASCII: {ascii_str}")
        
        # Check common formats
        print("\n🔍 Format Detection:")
        if magic_bytes[:2] == b'\xff\xd8':
            print("  ✓ JPEG marker detected (FF D8)")
        if magic_bytes[:4] == b'\x00\x00\x00\x18' or magic_bytes[:4] == b'\x00\x00\x00\x1c':
            print("  ✓ HEIF/HEIC container detected")
        if b'ftyp' in magic_bytes[:12]:
            print("  ✓ ISO Base Media File Format (MP4/HEIC)")
        if magic_bytes[:2] == b'PK':
            print("  ✓ ZIP archive detected (PK)")
        if magic_bytes[:4] == b'\x89PNG':
            print("  ✓ PNG marker detected")
        
        return magic_bytes
    except Exception as e:
        print(f"Error reading file: {e}")
        return None


def check_zip_archive(filepath, output_dir):
    """Check if file is a ZIP archive and attempt extraction"""
    print_separator("ZIP Archive Check")
    
    try:
        if zipfile.is_zipfile(filepath):
            print(f"✓ File is a valid ZIP archive!")
            
            # Create extraction directory
            extract_path = Path(output_dir) / "temp_unzip" / Path(filepath).stem
            extract_path.mkdir(parents=True, exist_ok=True)
            
            print(f"📦 Extracting to: {extract_path}")
            
            with zipfile.ZipFile(filepath, 'r') as zip_ref:
                zip_ref.printdir()
                print(f"\n📂 Extracting {len(zip_ref.namelist())} files...")
                zip_ref.extractall(extract_path)
                
                print(f"\n✓ Extraction complete!")
                print(f"\nExtracted files:")
                for item in extract_path.rglob('*'):
                    if item.is_file():
                        size = item.stat().st_size
                        print(f"  - {item.relative_to(extract_path)} ({size:,} bytes)")
                
                return True, extract_path
        else:
            print("✗ File is NOT a ZIP archive")
            return False, None
            
    except Exception as e:
        print(f"Error checking ZIP: {e}")
        return False, None


def analyze_file_size(filepath):
    """Analyze file size and structure"""
    print_separator("File Size Analysis")
    
    size = os.path.getsize(filepath)
    print(f"Total size: {size:,} bytes ({size / (1024**2):.2f} MB)")
    
    # Check for multiple image indicators
    with open(filepath, 'rb') as f:
        content = f.read()
        
    jpeg_markers = content.count(b'\xff\xd8\xff')
    print(f"JPEG SOI markers found: {jpeg_markers}")
    
    if jpeg_markers > 1:
        print(f"⚠️  Multiple JPEG markers detected - may contain multiple images!")


def save_log(filepath, log_dir):
    """Save inspection results to log file"""
    import datetime
    
    log_file = Path(log_dir) / f"inspection_{Path(filepath).stem}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    print(f"\n💾 Log saved to: {log_file}")
    # In real implementation, redirect stdout to file
    

def main():
    if len(sys.argv) < 2:
        print("Usage: python inspect_format.py <path_to_36p_file>")
        print("\nExample:")
        print("  python inspect_format.py raw_samples/GS010013.36P")
        sys.exit(1)
    
    filepath = sys.argv[1]
    
    if not os.path.exists(filepath):
        print(f"❌ Error: File not found: {filepath}")
        sys.exit(1)
    
    print(f"\n🔬 Analyzing: {filepath}")
    print(f"Filename: {Path(filepath).name}")
    
    # Run analysis steps
    file_output = run_file_command(filepath)
    magic_bytes = read_magic_numbers(filepath, num_bytes=64)
    analyze_file_size(filepath)
    is_zip, extract_path = check_zip_archive(filepath, os.path.dirname(filepath))
    
    # Summary
    print_separator("Summary")
    print(f"File: {Path(filepath).name}")
    print(f"Size: {os.path.getsize(filepath):,} bytes")
    print(f"ZIP Archive: {'Yes' if is_zip else 'No'}")
    if extract_path:
        print(f"Extracted to: {extract_path}")
    
    print("\n✅ Inspection complete!")
    print("\nNext steps:")
    print("  1. Review the magic numbers to identify format")
    print("  2. If ZIP extracted, examine internal files")
    print("  3. Try processing with attempt_process.py")


if __name__ == "__main__":
    main()
