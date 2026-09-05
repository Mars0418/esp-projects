"""Request a camera snapshot from the car; inference stays on ESP32-S3."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
import numpy as np
from PIL import Image
import serial

def open_port(port):
    s=serial.Serial()
    s.port=port;s.baudrate=115200;s.timeout=.3
    s.dtr=False;s.rts=False
    s.open()
    s.set_buffer_size(rx_size=262144,tx_size=65536)
    return s

def serial_lines(s,deadline):
    """Read UART in blocks: byte-at-a-time readline can overrun Windows buffers."""
    pending=bytearray()
    while time.monotonic()<deadline:
        pending.extend(s.read(8192))
        while b'\n' in pending:
            line,_,rest=pending.partition(b'\n')
            pending=bytearray(rest)
            yield line.decode('ascii',errors='replace').strip()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',required=True)
    p.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'captures')
    a=p.parse_args()
    folder=a.output/datetime.now().strftime('%Y%m%d-%H%M%S')
    rows={};input_bytes=None;active=False;complete=False;logs=[]
    with open_port(a.port) as s:
        s.reset_input_buffer();s.write(b'C\n')
        deadline=time.monotonic()+25
        for line in serial_lines(s,deadline):
            if not line:continue
            if 'DIGIT_FRAME_BEGIN 160 120' in line:
                active=True;rows={};continue
            if not active:
                logs.append(line);continue
            try:
                if line.startswith('ROW '):
                    _,y,raw=line.split()
                    row=bytes.fromhex(raw)
                    if len(row)!=320:raise ValueError('invalid row')
                    rows[int(y)]=row
                elif line.startswith('INPUT '): input_bytes=bytes.fromhex(line[6:])
                elif line=='DIGIT_FRAME_END':complete=True;break
                else:logs.append(line)
            except ValueError as e:raise RuntimeError('Incomplete serial snapshot; retry C') from e
    if not complete or set(rows)!=set(range(120)) or input_bytes is None or len(input_bytes)!=784:
        raise RuntimeError(f'No complete snapshot: {len(rows)}/120 rows. Check camera stream and port.')
    data=b''.join(rows[y] for y in range(120))
    rgb565=np.frombuffer(data,dtype='>u2').reshape(120,160).astype(np.uint32)
    rgb=np.stack([((rgb565>>11)&31)*255//31,((rgb565>>5)&63)*255//63,(rgb565&31)*255//31],axis=-1).astype(np.uint8)
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'frame.rgb565').write_bytes(data)
    (folder/'input.bin').write_bytes(input_bytes)
    Image.fromarray(rgb).transpose(Image.Transpose.ROTATE_270).resize((480,640),Image.Resampling.NEAREST).save(folder/'camera.png')
    Image.frombytes('L',(28,28),input_bytes).resize((280,280),Image.Resampling.NEAREST).save(folder/'input.png')
    (folder/'serial.log').write_text('\n'.join(logs),encoding='utf-8')
    print(json.dumps({'folder':str(folder.resolve()),'camera':str((folder/'camera.png').resolve()),
                      'input':str((folder/'input.png').resolve())},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
