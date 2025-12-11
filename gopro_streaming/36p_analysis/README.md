# GoPro .36p File Analysis Suite

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
