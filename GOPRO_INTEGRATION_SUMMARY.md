# GoPro Integration Summary

## Overview
Added GoPro video recording capability to the episode recording system, enabling synchronized multi-modal data capture.

## Files Modified

### 1. `/home/mani/Repos/ur_ws/gopro_recorder.py` (NEW)
**Purpose:** Helper module for GoPro video capture via FFmpeg

**Key features:**
- `GoProRecorder` class with start/stop recording methods
- UDP stream capture from `udp://0.0.0.0:8554`
- FFmpeg subprocess management
- Stream availability checking with `is_gopro_streaming()`

**Dependencies:**
- FFmpeg command-line tool
- GoPro Max 2 in webcam mode

### 2. `/home/mani/Repos/ur_ws/program_selector_ui.py` (MODIFIED)
**Changes:**
1. **Added GoPro imports:**
   ```python
   from gopro_recorder import GoProRecorder
   GOPRO_AVAILABLE = True
   ```

2. **Modified `EpisodeRecorder.__init__`:**
   - Added `self.gopro_recorder = None` attribute

3. **Modified `EpisodeRecorder.start_recording()`:**
   - Initialize `GoProRecorder` if available
   - Start GoPro recording to `gopro_video.mp4`
   - Print status messages for debugging
   - Gracefully handle GoPro not available/streaming

4. **Modified `EpisodeRecorder.stop_recording()`:**
   - Stop GoPro recording before saving metadata
   - Add error handling for GoPro stop

5. **Modified metadata saving:**
   - Add `gopro_video_file` entry if GoPro was used
   - Check if `gopro_video.mp4` exists before adding to metadata

6. **Added GoPro status indicator in UI sidebar:**
   - Shows "📹 GoPro: Streaming" when active
   - Shows "📹 GoPro: Not streaming" when not available
   - Shows "📹 GoPro: Not available" if module missing

### 3. `/home/mani/Repos/ur_ws/test_gopro_integration.py` (NEW)
**Purpose:** Test script for verifying GoPro recording functionality

**Features:**
- Check if GoPro stream is active
- Record 5-second test video
- Report file size and status
- Helpful error messages

### 4. `/home/mani/Repos/ur_ws/GOPRO_INTEGRATION.md` (NEW)
**Purpose:** Comprehensive documentation for GoPro integration

**Sections:**
- Prerequisites and setup
- UI usage instructions
- Troubleshooting guide
- Architecture overview
- Data synchronization details
- File format specifications
- Command reference

## Workflow

### Before Recording
1. Connect GoPro via USB
2. Run `python gopro_streaming/connect_gopro.py`
3. Verify stream with `./gopro_streaming/view_stream.sh`

### During Recording
1. Launch UI: `./run_program_ui.sh`
2. Check GoPro status indicator (should show "Streaming")
3. Enter experiment name
4. Click **🔴 Record**
5. Execute robot programs
6. Click **⏹️ End**

### Output Structure
```
recordings/{experiment_name}_{timestamp}/
├── video.mp4              # Webcam video (640x480, 30fps)
├── gopro_video.mp4        # GoPro video (if available)
├── joint_states.h5        # Robot joint data (50Hz)
├── metadata.json          # Recording metadata
└── program_events.json    # Program execution timeline
```

## Technical Details

### GoPro Stream Capture
- **Input:** UDP stream at `udp://0.0.0.0:8554`
- **Method:** FFmpeg subprocess
- **Video codec:** Copy (no re-encoding)
- **Audio codec:** AAC
- **Output format:** MP4

### Synchronization
All data sources share common timeline:
- t=0.0s: Recording start
- All timestamps relative to start time
- Program events logged with precise timestamps
- Joint states sampled at 50 Hz
- Webcam captured at 30 Hz
- GoPro captured at stream rate

### Error Handling
- Gracefully handles GoPro not available
- Continues recording with webcam only if GoPro fails
- Status messages printed for debugging
- Metadata only includes GoPro file if it exists

## Benefits

1. **Multi-camera recording:** Webcam + GoPro for different perspectives
2. **Higher quality video:** GoPro provides better resolution/quality
3. **Synchronized data:** All modalities share common timeline
4. **Graceful degradation:** Works even if GoPro unavailable
5. **Easy integration:** Minimal code changes, drop-in functionality

## Testing

### Test GoPro Connection
```bash
cd /home/mani/Repos/ur_ws
python test_gopro_integration.py
```

Expected output:
```
📹 Initializing GoPro recorder...
🔍 Checking if GoPro is streaming...
   ✓ GoPro stream is active!
🔴 Starting GoPro recording...
   ✓ Recording started!
⏱️  Recording for 5 seconds...
⏹️  Stopping recording...
   ✓ Recording stopped!
✅ Test completed successfully!
```

### Test Full Workflow
1. Start GoPro streaming
2. Launch UI
3. Verify status shows "GoPro: Streaming"
4. Record short episode
5. Check output directory for `gopro_video.mp4`

## Troubleshooting

### GoPro not detected
**Symptom:** UI shows "GoPro: Not streaming"

**Solution:**
```bash
cd /home/mani/Repos/ur_ws/gopro_streaming
conda activate ur5_python
python connect_gopro.py
```

### FFmpeg not found
**Symptom:** Error "ffmpeg: command not found"

**Solution:**
```bash
sudo apt install ffmpeg
```

### Video file missing after recording
**Symptom:** No `gopro_video.mp4` in output directory

**Possible causes:**
- GoPro stream dropped during recording
- FFmpeg process crashed
- Insufficient disk space

**Check terminal output for error messages**

## Future Improvements

Potential enhancements:
1. Auto-reconnect if stream drops
2. Configurable stream quality/resolution
3. Multi-GoPro support (multiple cameras)
4. Real-time stream preview in UI
5. Automatic GoPro connection on UI start
6. Stream health monitoring
7. Bandwidth usage indicators

## Dependencies

### System packages:
- FFmpeg (for video capture)

### Python modules:
- subprocess (built-in)
- socket (for stream detection)
- pathlib (built-in)

### Hardware:
- GoPro Max 2 camera
- USB 3.0 cable and port
- Sufficient disk space (GoPro videos are large)

## Integration Points

The GoPro integration touches these components:

1. **gopro_recorder.py**: Core GoPro capture logic
2. **program_selector_ui.py**: UI and recording orchestration
3. **EpisodeRecorder class**: Multi-modal data recording
4. **Metadata system**: Recording information tracking
5. **UI status indicators**: User feedback

## Code Locations

### GoPro capture logic:
- `/home/mani/Repos/ur_ws/gopro_recorder.py`

### UI integration:
- `/home/mani/Repos/ur_ws/program_selector_ui.py`
  - Lines ~15-20: Imports
  - Lines ~270: `EpisodeRecorder.__init__`
  - Lines ~310-340: `start_recording()` GoPro initialization
  - Lines ~440-450: `stop_recording()` GoPro cleanup
  - Lines ~560-570: Metadata GoPro entry
  - Lines ~670-685: UI status indicator

### Testing:
- `/home/mani/Repos/ur_ws/test_gopro_integration.py`

### Documentation:
- `/home/mani/Repos/ur_ws/GOPRO_INTEGRATION.md`
- This file: `/home/mani/Repos/ur_ws/GOPRO_INTEGRATION_SUMMARY.md`

## Implementation Notes

1. **Minimal changes:** Added GoPro as optional feature, doesn't break existing functionality
2. **Import error handling:** Uses try-except for graceful degradation if `gopro_recorder` not available
3. **Status indicator:** Provides real-time feedback about GoPro availability
4. **FFmpeg subprocess:** Runs in background, managed by `GoProRecorder` class
5. **UDP stream:** No re-encoding for efficiency, just copies stream to file
6. **Metadata tracking:** Automatically updates metadata if GoPro used

## Usage Statistics

**Lines of code added:**
- gopro_recorder.py: 127 lines
- program_selector_ui.py: ~40 lines modified/added
- test_gopro_integration.py: 73 lines
- GOPRO_INTEGRATION.md: 380 lines
- This summary: 250+ lines

**Total: ~870 lines** (including documentation)

## Validation Checklist

- [x] GoPro recording starts without errors
- [x] GoPro recording stops cleanly
- [x] Video file created and playable
- [x] Metadata includes gopro_video_file entry
- [x] UI status indicator shows correct state
- [x] Graceful handling when GoPro unavailable
- [x] Recording continues with webcam only if GoPro fails
- [ ] Test with full robot recording session (TODO)
- [ ] Verify video synchronization with joint states (TODO)
- [ ] Validate file sizes and quality (TODO)

## Next Steps

1. Test with actual robot recording session
2. Verify video synchronization accuracy
3. Validate GoPro video quality
4. Check disk space requirements
5. Test error scenarios (stream drop, disk full, etc.)
6. Consider adding stream preview to UI
7. Document known issues and limitations
