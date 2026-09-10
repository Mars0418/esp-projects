"""Convert generated mono 16-bit/16kHz WAVs to offline 8-bit/8kHz firmware clips."""
from pathlib import Path
import json
import struct
import wave

root = Path(__file__).resolve().parents[1]
out = ['#include <stdint.h>', '#include <stddef.h>']
manifest = []
for clip in range(6):
    with wave.open(str(root / f'audio/{clip}.wav'), 'rb') as w:
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate()) == (1, 2, 16000)
        samples = struct.unpack('<' + 'h' * w.getnframes(), w.readframes(w.getnframes()))
    pcm = bytes(max(0, min(255, (samples[j] + samples[j+1]) // 2 // 256 + 128))
                for j in range(0, len(samples)-1, 2))
    out += [f'static const uint8_t clip_{clip}[] = {{', ','.join(map(str, pcm)), '};']
    manifest.append(dict(id=clip, seconds=len(pcm)/8000, bytes=len(pcm)))
out += ['const uint8_t *const tour_speech_clips[6] = {' + ','.join(f'clip_{i}' for i in range(6)) + '};',
        'const size_t tour_speech_lengths[6] = {' + ','.join(f'sizeof(clip_{i})' for i in range(6)) + '};']
(root / 'main/tour_speech_data.c').write_text('\n'.join(out), encoding='utf-8')
(root / 'audio/manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(manifest)
