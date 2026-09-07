// Run with Node.js. Exercises the shipped page against simulated device replies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../main/wifi_debug.html'), 'utf8');
const code = html.match(/<script>([\s\S]*?)<\/script>/)[1];
let lastImage, mode = 'live', submitted;
function element() {
  return {value:'', textContent:'', className:'', style:{}, children:[],
    append(child){this.children.push(child);}, replaceChildren(){this.children=[];},
    get rows(){return this.children;}, get cells(){return this.children;},
    getContext(){return {createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),putImageData:image=>{lastImage=image;}}}};
}
const elements = Object.fromEntries(['view','pause','rotate','frameInfo','objects','health','network','wifi','ssid','password','save','message'].map(id=>[id,element()]));
function packet(age) {
  const json=Buffer.from(JSON.stringify({width:160,height:120,sequence:3,age_ms:age}));
  const bytes=Buffer.alloc(4+json.length+38400);
  bytes.writeUInt32LE(json.length);json.copy(bytes,4);
  bytes.writeUInt16BE(0xf800,4+json.length); // pure red, big endian
  bytes.writeUInt16BE(0x07e0,6+json.length); // pure green
  return bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.length);
}
const ball={found:true,predicted:false,confidence:100,x:70,y:69};
const ctx=vm.createContext({Uint8Array,Uint8ClampedArray,DataView,TextDecoder,TextEncoder,Date,JSON,Math,Error,AbortController,
  document:{getElementById:id=>elements[id],createElement:element},
  setTimeout:()=>1,clearTimeout:()=>{},
  fetch:async(url,options)=>{
    if(mode==='offline')throw Error('offline');
    if(url==='/api/wifi'){submitted=JSON.parse(options.body);assert.equal(options.headers['X-Car-Config'],'1');return {ok:true};}
    if(url==='/api/frame')return {ok:true,arrayBuffer:async()=>mode==='broken'?new ArrayBuffer(3):packet(mode==='stale'?3000:80)};
    return {ok:true,json:async()=>({camera_fresh:mode!=='stale',ap_ssid:'BallCar-test',ap_ip:'192.168.4.1',sta_connected:false,sta_ssid:'',config_error:'ESP_OK',internal_free:60000,
      red:{...ball,found:false},white:{...ball,predicted:true},purple:ball,goal:{found:false,confidence:0}})};
  }});
vm.runInContext(code,ctx);
async function tick(){await new Promise(resolve=>setImmediate(resolve));}
(async()=>{
  await tick();
  assert.deepEqual(Array.from(lastImage.data.slice(0,8)),[255,0,0,255,0,255,0,255]);
  assert.equal(elements.objects.rows[0].cells[1].textContent,'未检出');
  assert.equal(elements.objects.rows[1].cells[1].textContent,'预测（非本帧）');
  assert.equal(elements.objects.rows[2].cells[1].textContent,'本帧检出');
  mode='stale';await vm.runInContext('frames()',ctx);await vm.runInContext('status()',ctx);
  assert.match(elements.frameInfo.textContent,/过期/);assert.equal(elements.objects.rows[2].cells[1].textContent,'画面过期');
  mode='broken';await vm.runInContext('frames()',ctx);assert.match(elements.frameInfo.textContent,/数据不完整/);
  mode='offline';await vm.runInContext('status()',ctx);assert.equal(elements.objects.rows[2].cells[1].textContent,'连接中断');
  mode='live';elements.ssid.value='test';elements.password.value='short';
  await elements.wifi.onsubmit({preventDefault(){}});assert.equal(submitted,undefined);
  elements.password.value='local-test-password';await elements.wifi.onsubmit({preventDefault(){}});
  assert.equal(submitted.ssid,'test');assert.equal(elements.password.value,'');assert.equal(elements.save.disabled,false);
  console.log('PASS: RGB565 colors, real/predicted/missing states, stale frames, corrupt packets, disconnect, Wi-Fi validation');
})().catch(error=>{console.error(error);process.exitCode=1;});
