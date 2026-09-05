"""Inject MNIST inputs over UART and compare real chip logits against ONNX.

This verifies deployed inference, NOT real-camera accuracy. Motors remain disabled.
"""
import argparse
import gzip
import json
from pathlib import Path
import re
import time
import numpy as np
import onnxruntime as ort
from capture_camera import open_port

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',required=True)
    p.add_argument('--data-dir',type=Path,required=True)
    p.add_argument('--count',type=int,default=100)
    a=p.parse_args()
    if not 1<=a.count<=10000:p.error('--count must be 1..10000')
    images=np.frombuffer(gzip.decompress((a.data_dir/'t10k-images-idx3-ubyte.gz').read_bytes()),
                         np.uint8,offset=16).reshape(-1,28,28)
    labels=np.frombuffer(gzip.decompress((a.data_dir/'t10k-labels-idx1-ubyte.gz').read_bytes()),np.uint8,offset=8)
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(ROOT/'model/mnist-8.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    rows=[]
    with open_port(a.port) as s:
        s.reset_input_buffer()
        for index in range(a.count):
            image=images[index]
            expected=session.run(None,{'Input3':image.astype(np.float32)[None,None]/255})[0][0]
            s.write(b'I '+image.tobytes().hex().encode()+b'\n')
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                line=s.readline().decode('ascii',errors='replace')
                match=re.search(r'DIGIT_TEST digit=(\d) ms=([\d.]+) logits=([\d.,eE+\-]+)',line)
                if not match:continue
                actual=np.array([float(v) for v in match[3].split(',')])
                if actual.shape!=(10,):raise RuntimeError('Truncated test response')
                error=float(np.max(np.abs(expected-actual)))
                if error>0.002:raise AssertionError((index,error,actual,expected))
                predicted=int(match[1])
                if predicted!=int(expected.argmax()):raise AssertionError('Prediction mismatch')
                rows.append(dict(index=index,label=int(labels[index]),predicted=predicted,
                                 inference_ms=float(match[2]),max_logit_error=error))
                break
            else:raise TimeoutError(f'No board response for sample {index}')
    report=dict(count=len(rows),matching_predictions=len(rows),
                correct=sum(r['label']==r['predicted'] for r in rows),
                mean_inference_ms=float(np.mean([r['inference_ms'] for r in rows])),
                max_logit_error=max(r['max_logit_error'] for r in rows),samples=rows)
    (ROOT/'model/board-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))

if __name__=='__main__':main()
