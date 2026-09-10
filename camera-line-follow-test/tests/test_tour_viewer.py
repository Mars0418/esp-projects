import sys,json,zlib,queue,tkinter as tk
from pathlib import Path
repository = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(repository))
import push_debug_viewer as v
root=tk.Tk();root.withdraw()
app=v.PushDebugViewer(root,'',repository,False)
for i,phase in enumerate(('TOUR_DIGIT','LINE_FOLLOW')):
    m=dict(v=1,seq=i,width=80,height=60,format='RGB332',phase=phase,capture_us=100,processed_us=200,tour=dict(active=1,route=1,digit=1,score=92,corners=i,dwell_ms=1200,can_start=0))
    raw=json.dumps(m).encode('ascii');pixels=bytes(4800);body=raw+pixels
    packet=v.PREFIX.pack(v.MAGIC,len(raw),len(pixels),zlib.crc32(body))+body
    q=queue.Queue();worker=v.SerialWorker(q);buf=bytearray()
    for j in range(0,len(packet),31):
        buf.extend(packet[j:j+31]);worker._parse_packets(buf)
    kind,frame=q.get_nowait();assert kind=='frame'
    app._show_frame(frame)
    assert '92%' in app.mission_var.get()
    assert app.simple_start_button.cget('text')=='起步 / 继续'
    assert str(app.simple_start_button.cget('state'))=='disabled'
from types import SimpleNamespace
app.serial_worker.port=SimpleNamespace(is_open=True)
m.update(phase='TOUR_DIGIT',line={'control':{'state':'WAIT_START'}},tour={'active':1,'route':0,'can_start':1})
app._show_frame(v.DebugFrame(m,pixels))
assert app.simple_start_button.cget('text')=='开始'
assert str(app.simple_start_button.cget('state'))=='normal'
m['line']['control']['state']='WELCOME';m['tour']['can_start']=0
app._show_frame(v.DebugFrame(m,pixels))
assert str(app.simple_start_button.cget('state'))=='disabled'
sent=[]
app.serial_worker.write=sent.append
app.voice_command("XZ,ON")
app.voice_command("XZ,OFF")
app.voice_command("WIFI,phone,passwordXF P")
app.emergency_stop()
assert sent==[b"@XZ,ON\n",b"@XZ,OFF\n",b"@WIFI,phone,passwordXF P\n",b"\x03"]
m['xiaozhi']={'state':2,'binding_code':123}
app._show_frame(v.DebugFrame(m,pixels))
assert '正在监听' in app.xiaozhi_var.get() and '000123' in app.xiaozhi_var.get()
app.serial_worker.port=None
root.destroy()
print('PASS: fragmented CRC packets, digit/line UI render, tour labels, locked start')
