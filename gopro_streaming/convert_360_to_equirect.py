#!/usr/bin/env python3
"""
GoPro .360 to Equirectangular Converter

This script converts GoPro MAX/MAX2 .360 files to equirectangular format.

GoPro .360 files contain two video tracks (front and back lens) in EAC 
(Equi-Angular Cubemap) format. This script:
1. Extracts both video streams
2. Stacks them vertically to create a full EAC layout
3. Converts from EAC to equirectangular projection using ffmpeg's v360 filter

Usage:
    python convert_360_to_equirect.py <input.360> [output.mp4] [options]

Requirements:
    - ffmpeg with v360 filter support
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path
import tempfile
import shutil


def check_ffmpeg():
    """Check if ffmpeg is available and has v360 filter"""
    try:
        result = subprocess.run(['ffmpeg', '-version'], 
                              capture_output=True, text=True, check=True)
        print("✓ FFmpeg found")
        
        # Check for v360 filter
        result = subprocess.run(['ffmpeg', '-filters'], 
                              capture_output=True, text=True, check=True)
        if 'v360' in result.stdout:
            print("✓ v360 filter available")
            return True
        else:
            print("✗ v360 filter not available")
            return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("✗ FFmpeg not found")
        return False


def get_video_info(input_file):
    """Get video stream information using ffprobe"""
    cmd = [
        'ffprobe', '-v', 'error',
        '-select_streams', 'v',
        '-show_entries', 'stream=index,width,height,codec_name,avg_frame_rate',
        '-of', 'csv=p=0',
        str(input_file)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        streams = []
        for line in result.stdout.strip().split('\n'):
            if line:
                parts = line.split(',')
                if len(parts) >= 4:
                    streams.append({
                        'index': int(parts[0]),
                        'codec': parts[1],
                        'width': int(parts[2]),
                        'height': int(parts[3]),
                        'fps': parts[4] if len(parts) > 4 else '30'
                    })
        return streams
    except subprocess.CalledProcessError as e:
        print(f"Error getting video info: {e}")
        return []


def convert_360_to_equirect(input_file, output_file, output_width=4096, 
                             output_height=2048, quality='high', 
                             start_time=None, duration=None,
                             audio=True, verbose=False):
    """
    Convert GoPro .360 file to equirectangular format.
    
    The GoPro MAX/MAX2 .360 format stores two video tracks:
    - Stream 0:0 (or 0:v:0): Front lens EAC
    - Stream 0:4 (or 0:v:1): Back lens EAC
    
    Both are 4096x1344. When stacked vertically, they form a 4096x2688 EAC frame.
    The v360 filter then converts this to equirectangular.
    """
    
    # Get video info
    streams = get_video_info(input_file)
    video_streams = [s for s in streams if s['codec'] in ['hevc', 'h264', 'h265']]
    
    if len(video_streams) < 2:
        print(f"Error: Expected 2 video streams, found {len(video_streams)}")
        print("This might not be a valid GoPro .360 file")
        return False
    
    print(f"\nFound {len(video_streams)} video streams:")
    for s in video_streams:
        print(f"  Stream {s['index']}: {s['width']}x{s['height']} ({s['codec']})")
    
    # Quality presets
    quality_presets = {
        'low': {'crf': 28, 'preset': 'fast'},
        'medium': {'crf': 23, 'preset': 'medium'},
        'high': {'crf': 18, 'preset': 'slow'},
        'lossless': {'crf': 0, 'preset': 'slow'}
    }
    
    q = quality_presets.get(quality, quality_presets['high'])
    
    # Build ffmpeg command
    # The filter chain:
    # 1. Extract both video streams
    # 2. Stack them vertically (front on top, back on bottom for GoPro MAX)
    # 3. Apply v360 filter to convert EAC to equirectangular
    
    # Note: GoPro MAX2 uses a specific EAC layout. The two 4096x1344 streams
    # represent front and back lenses. When stacked, they form the full 
    # equi-angular cubemap which can be converted to equirectangular.
    
    filter_complex = (
        f"[0:v:0]setpts=PTS-STARTPTS[front];"
        f"[0:v:1]setpts=PTS-STARTPTS[back];"
        f"[front][back]vstack=inputs=2[stacked];"
        f"[stacked]v360=input=eac:output=equirect:w={output_width}:h={output_height}:interp=lanczos[out]"
    )
    
    cmd = ['ffmpeg', '-y']
    
    # Input options
    if start_time:
        cmd.extend(['-ss', str(start_time)])
    
    cmd.extend(['-i', str(input_file)])
    
    if duration:
        cmd.extend(['-t', str(duration)])
    
    # Filter
    cmd.extend(['-filter_complex', filter_complex])
    cmd.extend(['-map', '[out]'])
    
    # Audio handling
    if audio:
        # Try to map stereo audio (stream 0:1)
        cmd.extend(['-map', '0:a:0?'])
        cmd.extend(['-c:a', 'aac', '-b:a', '192k'])
    
    # Video encoding
    cmd.extend([
        '-c:v', 'libx265',  # HEVC for better compression
        '-crf', str(q['crf']),
        '-preset', q['preset'],
        '-pix_fmt', 'yuv420p',
        '-tag:v', 'hvc1',  # For better compatibility
    ])
    
    # Add metadata
    cmd.extend([
        '-metadata:s:v:0', 'stereo_mode=mono',
        '-metadata', 'comment=Converted from GoPro .360 to Equirectangular'
    ])
    
    cmd.append(str(output_file))
    
    if verbose:
        print("\nFFmpeg command:")
        print(' '.join(cmd))
    
    print(f"\nConverting to equirectangular ({output_width}x{output_height})...")
    print(f"Quality: {quality} (CRF={q['crf']}, preset={q['preset']})")
    
    try:
        if verbose:
            # Show progress
            subprocess.run(cmd, check=True)
        else:
            # Show only errors
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f"Error: {result.stderr}")
                return False
        
        if output_file.exists():
            size_mb = output_file.stat().st_size / (1024 * 1024)
            print(f"\n✓ Success! Output: {output_file}")
            print(f"  Size: {size_mb:.1f} MB")
            return True
        else:
            print("Error: Output file was not created")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"Error during conversion: {e}")
        return False


def extract_frame(input_file, output_image, time_offset=0, output_width=4096, 
                  output_height=2048):
    """Extract a single equirectangular frame from the .360 file"""
    
    filter_complex = (
        f"[0:v:0]setpts=PTS-STARTPTS[front];"
        f"[0:v:1]setpts=PTS-STARTPTS[back];"
        f"[front][back]vstack=inputs=2[stacked];"
        f"[stacked]v360=input=eac:output=equirect:w={output_width}:h={output_height}:interp=lanczos[out]"
    )
    
    cmd = [
        'ffmpeg', '-y',
        '-ss', str(time_offset),
        '-i', str(input_file),
        '-filter_complex', filter_complex,
        '-map', '[out]',
        '-frames:v', '1',
        '-q:v', '2',
        str(output_image)
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0 and output_image.exists():
            print(f"✓ Frame extracted: {output_image}")
            return True
        else:
            print(f"Error extracting frame: {result.stderr}")
            return False
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Convert GoPro .360 files to equirectangular format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic conversion
    python convert_360_to_equirect.py video.360
    
    # Specify output file and resolution
    python convert_360_to_equirect.py video.360 output.mp4 -w 5760 -h 2880
    
    # Extract first 30 seconds with high quality
    python convert_360_to_equirect.py video.360 -d 30 -q high
    
    # Extract a single frame at 5 seconds
    python convert_360_to_equirect.py video.360 --frame --time 5 -o frame.jpg
        """
    )
    
    parser.add_argument('input', help='Input .360 file')
    parser.add_argument('output', nargs='?', help='Output file (optional)')
    parser.add_argument('-w', '--width', type=int, default=4096,
                        help='Output width (default: 4096)')
    parser.add_argument('-H', '--height', type=int, default=2048,
                        help='Output height (default: 2048)')
    parser.add_argument('-q', '--quality', choices=['low', 'medium', 'high', 'lossless'],
                        default='high', help='Quality preset (default: high)')
    parser.add_argument('-s', '--start', type=float, 
                        help='Start time in seconds')
    parser.add_argument('-d', '--duration', type=float,
                        help='Duration in seconds')
    parser.add_argument('--no-audio', action='store_true',
                        help='Exclude audio from output')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Show detailed output')
    parser.add_argument('--frame', action='store_true',
                        help='Extract a single frame instead of video')
    parser.add_argument('-t', '--time', type=float, default=0,
                        help='Time offset for frame extraction (default: 0)')
    parser.add_argument('--info', action='store_true',
                        help='Show file information only')
    
    args = parser.parse_args()
    
    # Validate input file
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)
    
    # Check ffmpeg
    if not check_ffmpeg():
        print("\nPlease install ffmpeg with v360 filter support")
        sys.exit(1)
    
    # Show info only
    if args.info:
        print(f"\nFile: {input_path}")
        streams = get_video_info(input_path)
        for s in streams:
            print(f"  Stream {s['index']}: {s['width']}x{s['height']} {s['codec']} @ {s['fps']} fps")
        sys.exit(0)
    
    # Determine output file
    if args.output:
        output_path = Path(args.output)
    elif args.frame:
        output_path = input_path.with_suffix('.jpg')
    else:
        output_path = input_path.with_suffix('.equirect.mp4')
    
    print(f"\n{'='*60}")
    print(f"GoPro .360 to Equirectangular Converter")
    print(f"{'='*60}")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    
    if args.frame:
        # Extract single frame
        success = extract_frame(
            input_path, output_path, 
            time_offset=args.time,
            output_width=args.width,
            output_height=args.height
        )
    else:
        # Convert video
        success = convert_360_to_equirect(
            input_path, output_path,
            output_width=args.width,
            output_height=args.height,
            quality=args.quality,
            start_time=args.start,
            duration=args.duration,
            audio=not args.no_audio,
            verbose=args.verbose
        )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
