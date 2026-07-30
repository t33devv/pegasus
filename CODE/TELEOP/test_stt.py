import sys, pathlib
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from voice_control import record_audio, transcribe, parse_command

audio_path = record_audio()

# Load the saved wav and check the audio level so we know if the mic picked anything up
import soundfile as sf
audio_data, sr = sf.read(audio_path)
peak = np.max(np.abs(audio_data))
rms  = np.sqrt(np.mean(audio_data ** 2))
print(f"[debug]  peak={peak:.4f}  rms={rms:.4f}  (near-zero = mic captured silence)")

if rms < 0.001:
    print("[debug]  WARNING: audio is nearly silent — check your mic input in System Settings → Sound → Input")
else:
    print("[debug]  Audio level looks good, sending to Whisper...")
    text = transcribe(audio_path)
    cmd  = parse_command(text)
    print("Parsed command:", cmd)
