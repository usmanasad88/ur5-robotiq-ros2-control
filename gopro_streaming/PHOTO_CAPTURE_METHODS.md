# GoPro Max 2 Photo Capture Methods

## Method 1: Native Photo Capture (RECOMMENDED)

Uses GoPro's built-in photo capture API - highest quality!

```bash
# Single native photo
python native_photo_capture.py

# Multiple photos
python native_photo_capture.py --count 5 --interval 3

# Custom output directory
python native_photo_capture.py --output-dir ./my_photos

# Capture only (don't download)
python native_photo_capture.py --no-download
```

**Pros:**
- ✅ **Highest quality**: ~5 MB native 360° photos
- ✅ **Native .36P format**: Full resolution dual fisheye
- ✅ **Proper camera capture**: Uses actual shutter, not stream
- ✅ **Auto-download**: Fetches photos from camera storage

**Cons:**
- ❌ Stops video stream (switches to photo mode)
- ❌ Slightly slower (2-3 seconds per capture)

**Output:** `.36P` files (GoPro Max 360 photo format)

---

## Method 2: Stream Frame Capture

Captures frames from live video stream.

```bash
# Single frame from stream
python capture_photo.py --filename test.jpg

# With 360° projection
python capture_photo.py --projection --filename test_360.jpg

# Multiple frames
python capture_photo.py --count 10 --interval 1.0
```

**Pros:**
- ✅ Fast capture (no camera mode switch)
- ✅ Works while streaming
- ✅ Optional 360° projection conversion
- ✅ Lightweight output (~20-30 KB)

**Cons:**
- ❌ Lower quality (video stream resolution)
- ❌ Small file size (compressed frames)
- ❌ Requires active stream

**Output:** `.jpg` files (can be raw dual fisheye or projected equirectangular)

---

## Comparison

| Feature | Native Capture | Stream Capture |
|---------|---------------|----------------|
| **Quality** | ⭐⭐⭐⭐⭐ (5 MB) | ⭐⭐ (25 KB) |
| **Speed** | ~3 sec | <1 sec |
| **Format** | .36P (native) | .jpg |
| **Stream Impact** | Stops stream | No impact |
| **Resolution** | Full camera | Stream quality |
| **Use Case** | High-quality stills | Quick snapshots |

---

## Recommendations

**For production/high-quality photos:**
```bash
python native_photo_capture.py --output-dir ./dataset
```

**For quick testing/debugging:**
```bash
python capture_photo.py --filename test.jpg
```

**For 360° projection testing:**
```bash
python capture_photo.py --projection --filename test_360.jpg
```

---

## File Formats

### .36P Files (Native)
- GoPro Max native 360° photo format
- Contains full resolution dual fisheye data
- Can be processed with GoPro Player or FFmpeg
- Convert to equirectangular: `ffmpeg -i photo.36P -vf v360=input=dfisheye:output=equirect:ih_fov=190:iv_fov=190 output.jpg`

### .jpg Files (Stream)
- Standard JPEG format
- Can be raw dual fisheye or projected
- Immediately viewable
- Lower quality but portable
