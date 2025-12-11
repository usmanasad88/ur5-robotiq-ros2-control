#!/usr/bin/env python3
"""
GoPro .36p File Analysis - Test Environment Setup
Creates directory structure and generates analysis scripts
"""

import os
import sys
import shutil
from pathlib import Path


def create_directory_structure(base_path="36p_analysis"):
    """Create the main analysis directory structure"""
    print(f"🔧 Creating directory structure in '{base_path}'...")
    
    # Define subdirectories
    subdirs = [
        "raw_samples",
        "inspection_logs",
        "output_attempts",
        "temp_unzip"
    ]
    
    # Create base directory
    base = Path(base_path)
    base.mkdir(exist_ok=True)
    
    # Create subdirectories
    for subdir in subdirs:
        (base / subdir).mkdir(exist_ok=True)
        print(f"  ✓ Created {base / subdir}")
    
    return base


def create_inspector_script(base_path):
    """Generate the inspect_format.py script"""
    print("\n📝 Generating inspect_format.py...")
    
    script_content = '''#!/usr/bin/env python3
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
        print(f"\\n{'='*70}")
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
        print("\\n🔍 Format Detection:")
        if magic_bytes[:2] == b'\\xff\\xd8':
            print("  ✓ JPEG marker detected (FF D8)")
        if magic_bytes[:4] == b'\\x00\\x00\\x00\\x18' or magic_bytes[:4] == b'\\x00\\x00\\x00\\x1c':
            print("  ✓ HEIF/HEIC container detected")
        if b'ftyp' in magic_bytes[:12]:
            print("  ✓ ISO Base Media File Format (MP4/HEIC)")
        if magic_bytes[:2] == b'PK':
            print("  ✓ ZIP archive detected (PK)")
        if magic_bytes[:4] == b'\\x89PNG':
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
                print(f"\\n📂 Extracting {len(zip_ref.namelist())} files...")
                zip_ref.extractall(extract_path)
                
                print(f"\\n✓ Extraction complete!")
                print(f"\\nExtracted files:")
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
        
    jpeg_markers = content.count(b'\\xff\\xd8\\xff')
    print(f"JPEG SOI markers found: {jpeg_markers}")
    
    if jpeg_markers > 1:
        print(f"⚠️  Multiple JPEG markers detected - may contain multiple images!")


def save_log(filepath, log_dir):
    """Save inspection results to log file"""
    import datetime
    
    log_file = Path(log_dir) / f"inspection_{Path(filepath).stem}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    print(f"\\n💾 Log saved to: {log_file}")
    # In real implementation, redirect stdout to file
    

def main():
    if len(sys.argv) < 2:
        print("Usage: python inspect_format.py <path_to_36p_file>")
        print("\\nExample:")
        print("  python inspect_format.py raw_samples/GS010013.36P")
        sys.exit(1)
    
    filepath = sys.argv[1]
    
    if not os.path.exists(filepath):
        print(f"❌ Error: File not found: {filepath}")
        sys.exit(1)
    
    print(f"\\n🔬 Analyzing: {filepath}")
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
    
    print("\\n✅ Inspection complete!")
    print("\\nNext steps:")
    print("  1. Review the magic numbers to identify format")
    print("  2. If ZIP extracted, examine internal files")
    print("  3. Try processing with attempt_process.py")


if __name__ == "__main__":
    main()
'''
    
    script_path = base_path / "inspect_format.py"
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    # Make executable
    os.chmod(script_path, 0o755)
    print(f"  ✓ Created {script_path}")


def create_processing_script(base_path):
    """Generate the attempt_process.py script"""
    print("\n📝 Generating attempt_process.py...")
    
    script_content = '''#!/usr/bin/env python3
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
        print(f"\\n{'='*70}")
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
    print(f"\\n🚀 Running: {' '.join(cmd)}")
    
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
                f.write(f"\\n{'='*70}\\n")
                f.write(f"Command: {' '.join(cmd)}\\n")
                f.write(f"Timestamp: {datetime.now()}\\n")
                f.write(f"Return Code: {result.returncode}\\n")
                f.write(f"\\nSTDOUT:\\n{result.stdout}\\n")
                f.write(f"\\nSTDERR:\\n{result.stderr}\\n")
        
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
        jpeg_start = content.find(b'\\xff\\xd8\\xff')
        jpeg_end = content.find(b'\\xff\\xd9', jpeg_start)
        
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
        print("\\nExample:")
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
    
    print(f"\\n🎬 GoPro .36p Processing Suite")
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
    print(f"\\nProcessing Results for: {input_file.name}")
    print(f"Log saved to: {log_file}\\n")
    
    successful = []
    for method_name, (success, output) in results.items():
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"{status}: {method_name}")
        if output:
            print(f"         Output: {output}")
            successful.append(output)
    
    if successful:
        print(f"\\n🎉 {len(successful)} method(s) succeeded!")
        print(f"\\nView results with:")
        for output in successful:
            print(f"  eog {output}")
    else:
        print(f"\\n⚠️  All methods failed. Check the log file for details.")
        print(f"\\nNext steps:")
        print("  1. Review inspection_logs from inspect_format.py")
        print("  2. Check if file needs unpacking first")
        print("  3. Consult GoPro documentation for .36p format")


if __name__ == "__main__":
    main()
'''
    
    script_path = base_path / "attempt_process.py"
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    # Make executable
    os.chmod(script_path, 0o755)
    print(f"  ✓ Created {script_path}")


def create_readme(base_path):
    """Generate README.md with instructions"""
    print("\n📝 Generating README.md...")
    
    readme_content = '''# GoPro .36p File Analysis Suite

This suite helps reverse-engineer GoPro Max 2 `.36p` files to extract viewable 360° equirectangular images.

## Directory Structure

```
36p_analysis/
├── raw_samples/          # Place your .36p files here
├── inspection_logs/      # Analysis logs will be saved here
├── output_attempts/      # Processed images will be saved here
├── temp_unzip/          # Temporary extraction directory
├── inspect_format.py    # File format analyzer
├── attempt_process.py   # Processing suite
└── README.md           # This file
```

## Prerequisites

### 1. Activate Conda Environment

```bash
conda activate ur5_python
```

### 2. Install Required Tools

```bash
# FFmpeg (video/image processing)
sudo apt-get update
sudo apt-get install ffmpeg

# ImageMagick (image conversion)
sudo apt-get install imagemagick

# libheif (HEIC/HEIF support)
sudo apt-get install libheif-examples

# file command (usually pre-installed)
sudo apt-get install file
```

## Quick Start

### Step 1: Copy a .36p File

Copy ONE `.36p` file into the `raw_samples/` directory:

```bash
cp /path/to/your/GS010013.36P raw_samples/
```

Or from your current location:

```bash
cp gopro_streaming/native_photos/GS010013.36P 36p_analysis/raw_samples/
```

### Step 2: Inspect the File Format

Run the inspector to analyze the file structure:

```bash
cd 36p_analysis
python inspect_format.py raw_samples/GS010013.36P
```

This will:
- Show the Linux `file` command output
- Display the first 64 bytes (magic numbers) in hex
- Detect JPEG, HEIC, ZIP, or other formats
- Attempt to unzip if it's an archive
- Count JPEG markers (may contain multiple images)

### Step 3: Attempt Processing

Try multiple conversion methods:

```bash
python attempt_process.py raw_samples/GS010013.36P
```

This will attempt 5 different methods:

**Method A**: Direct FFmpeg v360 filter (assumes EAC format)
**Method B**: Copy to .jpg extension first, then process
**Method C**: Convert from HEIC to JPG, then stitch
**Method D**: ImageMagick conversion, then stitch
**Method E**: Extract embedded JPEG data, then stitch

All outputs are saved to `output_attempts/` with detailed logs.

## Understanding the Output

### Successful Processing

If any method succeeds, you'll see:

```
✅ SUCCESS: Method A: Direct FFmpeg
         Output: output_attempts/output_A_direct.jpg
```

View the result:

```bash
eog output_attempts/output_A_direct.jpg
# or
firefox output_attempts/output_A_direct.jpg
```

### Failed Processing

If all methods fail, check:

1. **Inspection logs**: Review magic numbers to identify true format
2. **Processing log**: Check `output_attempts/processing_log_*.txt`
3. **Extracted files**: If ZIP, examine `temp_unzip/` contents

## Troubleshooting

### "HEIC conversion failed"

Install additional HEIC support:

```bash
sudo apt-get install libheif-dev libheif-examples
```

### "FFmpeg error: Invalid data found"

The file may need preprocessing:
- Check if it's a ZIP archive (inspector will tell you)
- Extract manually and process individual files
- File might be encrypted or proprietary format

### "All methods failed"

The `.36p` format may be:
1. **Proprietary container**: Requires GoPro Player or SDK
2. **Multiple fisheye images**: Need custom stitching
3. **Raw sensor data**: Requires decoding before stitching
4. **Encrypted**: Needs GoPro authentication

Try:
- GoPro Player (official software)
- GoPro Labs tools
- Community tools like `gopro2gpx` or `gopro-utils`

## Advanced Usage

### Process Multiple Files

```bash
for file in raw_samples/*.36P; do
    echo "Processing $file..."
    python attempt_process.py "$file"
done
```

### Custom FFmpeg Filters

Edit `attempt_process.py` and modify the v360 filter parameters:

```python
# Try different input/output projections
'-vf', 'v360=hequirect:equirect'  # Half equirectangular input
'-vf', 'v360=dfisheye:equirect'   # Dual fisheye input
'-vf', 'v360=eac:equirect:w=5760:h=2880'  # Specify output resolution
```

### Manual Extraction

If the inspector finds a ZIP:

```bash
unzip -d temp_unzip/ raw_samples/GS010013.36P
ls -lh temp_unzip/
```

Then process individual files inside.

## File Format Reference

Common GoPro 360 formats:

- **EAC**: Equi-Angular Cubemap (6 faces)
- **Dual Fisheye**: Two circular fisheye images side-by-side
- **Equirectangular**: Standard 360° panorama (2:1 ratio)
- **CMP**: Cubemap (6 square faces)

Magic numbers:
- `FF D8 FF`: JPEG Start of Image
- `00 00 00 18/1C ftyp`: HEIF/HEIC container
- `PK`: ZIP archive
- `00 00 00 20 ftyp mp4`: MP4 video

## Resources

- [GoPro Developer Documentation](https://gopro.github.io/OpenGoPro/)
- [FFmpeg v360 Filter](https://ffmpeg.org/ffmpeg-filters.html#v360)
- [GoPro Labs](https://gopro.github.io/labs/)
- [ImageMagick Documentation](https://imagemagick.org/)

## Next Steps

1. Run `inspect_format.py` on your file
2. Examine the magic numbers and file structure
3. Run `attempt_process.py` to try all methods
4. If successful, document which method worked
5. If all fail, share inspection results with GoPro community

## Support

If you discover the correct processing method:
1. Document it in this README
2. Update the scripts with the working approach
3. Share with the community!

Good luck! 🎬📸
'''
    
    readme_path = base_path / "README.md"
    with open(readme_path, 'w') as f:
        f.write(readme_content)
    
    print(f"  ✓ Created {readme_path}")


def print_next_steps(base_path):
    """Print instructions for the user"""
    print_separator("Setup Complete!")
    
    print(f"""
✅ Test environment created successfully!

📂 Directory: {base_path.absolute()}

Next Steps:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1️⃣  Copy your .36p file:
   cd {base_path.absolute()}
   cp ../native_photos/GS010013.36P raw_samples/

2️⃣  Inspect the file format:
   python inspect_format.py raw_samples/GS010013.36P

3️⃣  Attempt processing:
   python attempt_process.py raw_samples/GS010013.36P

4️⃣  Check results:
   ls -lh output_attempts/
   eog output_attempts/output_*.jpg

📖 Read README.md for detailed instructions and troubleshooting!

🔧 Make sure these tools are installed:
   - ffmpeg
   - imagemagick (convert command)
   - libheif-examples (heif-convert)
   
   Install with:
   sudo apt-get install ffmpeg imagemagick libheif-examples
""")


def print_separator(title=""):
    """Print a formatted separator"""
    if title:
        print(f"\n{'='*70}")
        print(f" {title}")
        print('='*70)
    else:
        print('='*70)


def main():
    print("🚀 GoPro .36p Analysis - Test Environment Setup")
    print_separator()
    
    # Create directory structure
    base_path = create_directory_structure()
    
    # Generate scripts
    create_inspector_script(base_path)
    create_processing_script(base_path)
    create_readme(base_path)
    
    # Print next steps
    print_next_steps(base_path)


if __name__ == "__main__":
    main()
