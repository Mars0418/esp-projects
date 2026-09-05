"""Compare the exact firmware C kernels to ONNX Runtime, plus camera/gate cases."""
from pathlib import Path
import argparse
import ctypes as C
import gzip
import json
import numpy as np
import onnxruntime as ort
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
U8=C.POINTER(C.c_uint8)
F32=C.POINTER(C.c_float)
class Region(C.Structure):
    _fields_=[('valid',C.c_bool)]+[(n,C.c_int) for n in
              ['x','y','width','height','area','threshold','contrast']]+[('reason',C.c_char_p)]
class Gate(C.Structure):
    _fields_=[(n,C.c_int) for n in ['candidate','streak','empty_frames']]+[
        (n,C.c_int64) for n in ['last_ms','since_ms','empty_since_ms']]+[('latched',C.c_bool)]

def rgb565(gray):
    # Vision API accepts sensor orientation; the screen rotates clockwise.
    landscape=np.ascontiguousarray(np.rot90(gray,1),dtype=np.uint16)
    packed=((landscape>>3)<<11)|((landscape>>2)<<5)|(landscape>>3)
    return np.frombuffer(packed.astype('>u2').tobytes(),dtype=np.uint8).copy()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--data-dir',type=Path,required=True)
    a=parser.parse_args()
    lib=C.CDLL(str(ROOT/'build-host/digit.dll'))
    lib.digit_model_predict.argtypes=[C.c_void_p,U8,F32,F32]
    lib.digit_model_selftest.argtypes=[C.c_void_p,F32]
    lib.digit_vision_prepare.argtypes=[C.c_void_p,U8,C.c_uint,C.c_bool,U8]
    lib.digit_vision_prepare.restype=Region
    lib.digit_gate_reset.argtypes=[C.POINTER(Gate)]
    lib.digit_gate_update.argtypes=[C.POINTER(Gate),C.c_int,C.c_bool,C.c_float,C.c_float,C.c_int64]
    work=C.create_string_buffer((6272+1568+256)*4)
    vision=C.create_string_buffer(120*160*5)
    logits=np.zeros(10,np.float32); probs=np.zeros(10,np.float32)
    def predict(img):
        img=np.ascontiguousarray(img,np.uint8)
        lib.digit_model_predict(work,img.ctypes.data_as(U8),logits.ctypes.data_as(F32),probs.ctypes.data_as(F32))
        return logits.copy(),probs.copy()
    def prepare(gray):
        rgb=rgb565(gray); out=np.zeros(784,np.uint8)
        r=lib.digit_vision_prepare(vision,rgb.ctypes.data_as(U8),0,False,out.ctypes.data_as(U8))
        return r,out
    error=C.c_float()
    assert lib.digit_model_selftest(work,C.byref(error))==10 and error.value<0.002
    images=np.frombuffer(gzip.decompress((a.data_dir/'t10k-images-idx3-ubyte.gz').read_bytes()),
                         np.uint8,offset=16).reshape(-1,28,28)
    labels=np.frombuffer(gzip.decompress((a.data_dir/'t10k-labels-idx1-ubyte.gz').read_bytes()),np.uint8,offset=8)
    options=ort.SessionOptions(); options.intra_op_num_threads=1
    session=ort.InferenceSession(str(ROOT/'model/mnist-8.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    max_diff=0.0; matched=0; correct=0
    for img,label in zip(images[:1000],labels[:1000]):
        actual,_=predict(img)
        expected=session.run(None,{'Input3':img.astype(np.float32)[None,None]/255})[0][0]
        max_diff=max(max_diff,float(np.max(np.abs(actual-expected))))
        matched+=int(actual.argmax()==expected.argmax())
        correct+=int(actual.argmax()==label)
    assert max_diff<0.002 and matched==1000,(max_diff,matched)
    checks=[]
    for value in [0,80,180,255]:
        r,_=prepare(np.full((160,120),value,np.uint8))
        assert not r.valid
        checks.append(f'blank_{value}')
    # End-to-end synthetic white cards are separate from true camera validation.
    indices=json.loads((ROOT/'model/validation.json').read_text())['selftest_indices']
    card_results=[]
    montage=Image.new('RGB',(56,28*10),'#888888')
    for digit,index in enumerate(indices):
        gray=np.full((160,120),240,np.uint8)
        ink=np.array(Image.fromarray(images[index]).resize((56,56),Image.Resampling.BILINEAR))
        gray[52:108,32:88]=240-(ink.astype(np.uint16)*220//255).astype(np.uint8)
        r,out=prepare(gray)
        actual,p=predict(out)
        assert r.valid,(digit,r.reason)
        card_results.append(dict(label=digit,predicted=int(actual.argmax()),score=float(p.max())))
        montage.paste(Image.fromarray(images[index]),(0,28*digit))
        montage.paste(Image.fromarray(out.reshape(28,28)),(28,28*digit))
    montage.resize((224,1120),Image.Resampling.NEAREST).save(ROOT/'build-host/preprocess-debug.png')
    print('Synthetic card results:', card_results)
    assert all(r['label']==r['predicted'] for r in card_results),card_results
    checks.append('ten_synthetic_cards')
    gray=np.full((160,120),240,np.uint8)
    gray[55:100,28:32]=10;gray[55:100,87:91]=10
    r,_=prepare(gray)
    assert not r.valid and r.reason==b'MULTIPLE_OBJECTS'
    checks.append('multiple_digits_rejected')
    gray=np.full((160,120),240,np.uint8);gray[30:80,45:50]=10
    r,_=prepare(gray);assert not r.valid
    checks.append('clipped_digit_rejected')
    g=Gate();lib.digit_gate_reset(C.byref(g))
    update=lambda digit,valid,score,margin,t:lib.digit_gate_update(C.byref(g),digit,valid,score,margin,t)
    assert [update(2,True,.99,.98,t) for t in [100,200,300,400,500]]==[-1,-1,2,-1,-1]
    assert update(1,True,.99,.98,700)==-1
    assert update(1,True,.99,.98,800)==-1
    assert update(1,True,.99,.98,900)==-1
    assert update(-1,False,0,0,1000)==-1
    assert update(2,True,.99,.98,1100)==-1 and g.latched
    # A camera gap and a single blank frame must not release the command latch.
    assert update(2,True,.99,.98,2500)==-1 and g.latched
    for t in [2600,2700,2800,2900,3000,3100]: update(-1,False,0,0,t)
    assert not g.latched
    assert [update(1,True,.99,.98,t) for t in [3200,3300,3400]]==[-1,-1,1]
    lib.digit_gate_reset(C.byref(g))
    for t in range(100,1000,100): assert update(1,True,.6,.2,t)==-1
    lib.digit_gate_reset(C.byref(g))
    assert [update(2,True,.70,0,t) for t in [10,20,30]]==[-1,-1,2]
    lib.digit_gate_reset(C.byref(g))
    assert [update(2,True,score,0,t) for score,t in
            [(.72,10),(.69,20),(.75,30),(.70,40),(.71,50)]]==[-1,-1,-1,-1,2]
    lib.digit_gate_reset(C.byref(g))
    assert [update(d,True,.75,0,t) for d,t in
            [(2,10),(3,20),(2,30),(2,40),(2,50)]]==[-1,-1,-1,-1,2]
    checks+=['70_percent_inclusive_three_frames','below_70_resets_streak','different_digit_resets_streak']
    checks+=['one_event_per_presentation','no_rearm_on_uncertainty_or_gap','blank_rearms','low_score_rejected']
    report=dict(c_onnx_parity_count=1000,matching_predictions=matched,max_logit_error=max_diff,
                correct_first_1000=correct,checks=checks,synthetic_cards=card_results)
    (ROOT/'model/host-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
