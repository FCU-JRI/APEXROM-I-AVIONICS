import { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { MapContainer, TileLayer, Polyline, Marker, useMap } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import { Rocket3D } from './components/Rocket3D';
import { TeleCard } from './components/TeleCard';
import { SensorValue } from './components/SensorValue';

// STATENUM — 對應 StateMachine.hpp enum（值 0-17）
const STATES = {
    0:  { name:"STBY_IDLE",         label:"待機",              group:"standby", next:[1,2,7,15,16,17], critical:false },
    1:  { name:"STBY_BIT",          label:"自我測試",           group:"standby", next:[0],            critical:false },
    2:  { name:"CAL_GYRO",          label:"陀螺儀校準",         group:"cal",     next:[3],            critical:false },
    3:  { name:"CAL_ACCEL",         label:"加速計校準",         group:"cal",     next:[4],            critical:false },
    4:  { name:"CAL_MAG",           label:"磁力計校準",         group:"cal",     next:[5],            critical:false },
    5:  { name:"CAL_BARO",          label:"氣壓計校準",         group:"cal",     next:[6],            critical:false },
    6:  { name:"CAL_TEMP",          label:"溫度校準",           group:"cal",     next:[0],            critical:false },
    7:  { name:"FLIGHT_IGNITION",   label:"點火",               group:"flight",  next:[8],            critical:true  },
    8:  { name:"FLIGHT_POWERED",    label:"動力上升",           group:"flight",  next:[9],            critical:false },
    9:  { name:"FLIGHT_INERTIAL",   label:"慣性滑行",           group:"flight",  next:[10],           critical:false },
    10: { name:"FLIGHT_APOGEE",     label:"頂點 / 減速傘",      group:"flight",  next:[11],           critical:true  },
    11: { name:"FLIGHT_DESCENT",    label:"下降",               group:"flight",  next:[12,13],        critical:false },
    12: { name:"MAIN_CHUTE_DEPLOY", label:"分離引擎 / 開主傘", group:"flight",  next:[14],           critical:true  },
    13: { name:"SKIP_MAIN_CHUTE",   label:"跳過主傘",           group:"flight",  next:[14],           critical:true  },
    14: { name:"TERMINATE",         label:"任務終止",           group:"flight",  next:[],             critical:true  },
    15: { name:"DBG_COMM",          label:"Debug: 通訊",        group:"debug",   next:[0],            critical:false },
    16: { name:"DBG_SENSOR",        label:"Debug: 感測器",      group:"debug",   next:[0],            critical:false },
    17: { name:"DBG_STORAGE",       label:"Debug: 儲存",        group:"debug",   next:[0],            critical:false },
};

// === GPS MAP COMPONENT ===
const MapUpdater = ({ center }) => {
    const map = useMap();
    useEffect(() => {
        if (center[0] !== 0) map.setView(center, map.getZoom());
    }, [center, map]);
    return null;
};

const customIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
});

const GpsMap = ({ gps, trajectory }) => {
    if (gps.lat === 0 && gps.lon === 0) {
        return <div className="absolute inset-0 flex items-center justify-center text-gray-700 text-sm font-bold tracking-widest z-20 bg-black/80">AWAITING SATELLITE LOCK</div>;
    }
    return (
        <MapContainer center={[gps.lat, gps.lon]} zoom={18} style={{ height: '100%', width: '100%', backgroundColor: '#000' }} zoomControl={false} attributionControl={false}>
            <TileLayer url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png" />
            {trajectory.length > 0 && <Polyline positions={trajectory} color="#00f2fe" weight={4} opacity={0.8} />}
            <Marker position={[gps.lat, gps.lon]} icon={customIcon} />
            <MapUpdater center={[gps.lat, gps.lon]} />
        </MapContainer>
    );
};

function App() {
    const [currStateId, setCurrStateId] = useState(0);
    const [wsConnected, setWsConnected] = useState(false);
    const [isLive,      setIsLive]      = useState(true);
    const [histIdx,     setHistIdx]     = useState(0);
    const [history,     setHistory]     = useState([]);
    const [cmdLog,      setCmdLog]      = useState([]);
    const wsRef = useRef(null);

    const addCmdLog = useCallback((msg) => {
        const t = new Date().toLocaleTimeString('zh-TW', { hour12:false });
        setCmdLog(prev => [`[${t}] ${msg}`, ...prev].slice(0, 100));
    }, []);

    // WebSocket — 永久連線，僅初始化一次
    useEffect(() => {
        let ws, timer;
        const connect = () => {
            ws = new WebSocket("ws://localhost:8765");
            wsRef.current = ws;
            ws.onopen  = () => { setWsConnected(true);  addCmdLog("✅ 已連線至 Monitor WebSocket"); };
            ws.onclose = () => { setWsConnected(false); addCmdLog("❌ WebSocket 斷線，3 秒後重連…"); timer = setTimeout(connect, 3000); };
            ws.onerror = () => {};
            ws.onmessage = (e) => {
                try {
                    const d = JSON.parse(e.data);
                    if (d.batch) {
                        setHistory(h => [...h, d].slice(-1000));
                        // 攔截 LOG 封包，同步火箭端狀態
                        d.batch.forEach(b => {
                            if (b.type === "LOG" && b.data && b.data.msg) {
                                const transitionMatch = b.data.msg.match(/State:\s*\d+\s*->\s*(\d+)/);
                                const currentMatch = b.data.msg.match(/\[SM\] Current State:\s*(\d+)/);
                                if (transitionMatch) {
                                    setCurrStateId(parseInt(transitionMatch[1], 10));
                                } else if (currentMatch) {
                                    setCurrStateId(parseInt(currentMatch[1], 10));
                                }
                            }
                        });
                    }
                } catch (err) {}
            };
        };
        connect();
        return () => { clearTimeout(timer); if (ws) ws.close(); };
    }, [addCmdLog]);

    useEffect(() => {
        if (isLive && history.length > 0) setHistIdx(history.length - 1);
    }, [history.length, isLive]);

    // 發送狀態切換指令
    const sendStateCommand = useCallback((stateId) => {
        const s = STATES[stateId];
        if (!s) return;
        if (s.critical) {
            if (!window.confirm(`⚠️ 警告：即將發送飛行器指令\n\n→ [${stateId}] ${s.name}  (${s.label})\n\n此操作將直接影響飛行器，請再次確認！`)) return;
        }
        const ws = wsRef.current;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type:"cmd", action:"setState", stateId }));
            setCurrStateId(stateId);
            addCmdLog(`📡 [GND→FC] 切換至 ${s.name} (${stateId})`);
        } else {
            addCmdLog("❌ WebSocket 未連線，無法發送指令");
        }
    }, [addCmdLog]);

    // 衍生資料
    const currState  = STATES[currStateId];
    const nextStates = currState.next.map(id => ({ id, ...STATES[id] }));
    const normalNext   = nextStates.filter(s => !s.critical && s.group !== "debug");
    const criticalNext = nextStates.filter(s =>  s.critical);
    const debugNext    = nextStates.filter(s =>  s.group === "debug");

    const view   = isLive ? (history[history.length-1] || null) : history[histIdx];
    const getVal = (t) => view?.batch?.find(b => b.type === t)?.data;
    
    // 往前搜尋歷史紀錄，找出最近一筆指定類型的資料（解決低頻資料如 GPS 會閃動歸零的問題）
    const getLastVal = (t) => {
        const endIdx = isLive ? history.length - 1 : histIdx;
        if (endIdx < 0 || !history) return undefined;
        // 往前找最多 100 筆 (約數秒內)，避免無止盡搜尋
        const limit = Math.max(0, endIdx - 100);
        for (let i = endIdx; i >= limit; i--) {
            const d = history[i]?.batch?.find(b => b.type === t)?.data;
            if (d !== undefined) return d;
        }
        return undefined;
    };

    // 高頻資料可直接取當前 batch，或是為了穩定也使用 getLastVal
    const alt    = getLastVal("KALMAN_ALTITUDE")?.alt ?? 0;
    const vel    = getLastVal("KALMAN_ALTITUDE")?.vz  ?? 0;
    const acc    = getLastVal("IMU")?.az               ?? 0;
    const q      = getLastVal("KALMAN_QUATERNION")?.q  ?? [1,0,0,0];
    
    // GPS 是低頻資料，必須使用 getLastVal
    const gps    = getLastVal("KALMAN_GPS") ?? getLastVal("GPS") ?? { lat: 0, lon: 0 };
    const fcLogs = view?.batch?.filter(b => b.type === "LOG").map(b => b.data.msg) ?? [];
    
    const imu    = getLastVal("IMU");
    const bmp    = getLastVal("BMP");
    const gpsRaw = getLastVal("GPS");

    // 計算歷史軌跡 (Trajectory)
    const trajectory = useMemo(() => {
        const path = [];
        const endIdx = isLive ? history.length - 1 : histIdx;
        for (let i = 0; i <= endIdx; i++) {
            const batch = history[i]?.batch;
            if (!batch) continue;
            const gpsData = batch.find(b => b.type === "KALMAN_GPS" || b.type === "GPS")?.data;
            if (gpsData && gpsData.lat !== 0 && gpsData.lon !== 0) {
                const pt = [gpsData.lat, gpsData.lon];
                if (path.length === 0 || (path[path.length-1][0] !== pt[0] || path[path.length-1][1] !== pt[1])) {
                    path.push(pt);
                }
            }
        }
        return path;
    }, [history, histIdx, isLive]);

    return (
        <div className="h-full flex flex-col gap-3 relative">
            <div className="scanline"></div>

            {/* Header */}
            <header className="glass-panel p-2 flex justify-between items-center glow-cyan">
                <div className="text-lg font-bold text-neon uppercase tracking-tighter">JRI P2026 GND V2.1</div>
                <div className="flex gap-3 items-center">
                    {!isLive && <div className="text-sm text-yellow-500 font-bold animate-pulse">HISTORICAL PLAYBACK</div>}
                    <div id="ws-status" className={`px-2 py-0.5 rounded text-xs font-bold border ${wsConnected ? 'bg-green-900/50 text-green-400 border-green-700' : 'bg-red-900/50 text-red-400 border-red-700 animate-pulse'}`}>
                        {wsConnected ? '● WS CONNECTED' : '○ WS DISCONNECTED'}
                    </div>
                    <div className="px-3 py-1 bg-black/60 border border-cyan-900 text-sm font-bold text-cyan-400">
                        [{currStateId}] {currState.name}
                    </div>
                </div>
            </header>

            <div className="flex-1 flex gap-3 overflow-hidden">

                {/* 左側：遙測 + 3D + GPS */}
                <div className="w-1/3 flex flex-col gap-3">
                    <div className="grid grid-cols-2 gap-2">
                        <TeleCard label="Alt"     val={alt.toFixed(1)} unit="m"   />
                        <TeleCard label="Vel"     val={vel.toFixed(0)} unit="m/s" />
                        <TeleCard label="Accel"   val={acc.toFixed(2)} unit="g"   />
                        <TeleCard label="Packets" val={view?.pkt ?? 0} unit="idx" />
                    </div>
                    <div className="glass-panel flex-1 flex flex-col items-center justify-center relative min-h-[140px]">
                        <div className="text-sm text-cyan-400 font-bold uppercase absolute top-2 left-2 flex flex-col gap-0.5 z-10">
                            <span><span className="text-gray-500">QW:</span> {q[0].toFixed(3)}</span>
                            <span><span className="text-gray-500">QX:</span> {q[1].toFixed(3)}</span>
                            <span><span className="text-gray-500">QY:</span> {q[2].toFixed(3)}</span>
                            <span><span className="text-gray-500">QZ:</span> {q[3].toFixed(3)}</span>
                        </div>
                        <Rocket3D q={q} />
                        <div className="text-xs text-gray-600 uppercase absolute bottom-2">Live Orientation</div>
                    </div>
                    {/* GPS MAP */}
                    <div className="glass-panel h-[160px] flex flex-col p-2 relative overflow-hidden">
                        <div className="text-sm text-cyan-400 font-bold uppercase mb-1 flex justify-between">
                            <span>GPS MAP</span>
                            <span className="text-gray-500 font-mono text-xs">{gps.lat !== 0 ? `${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}` : 'NO FIX'}</span>
                        </div>
                        <div className="flex-1 rounded border border-cyan-900/30 overflow-hidden relative bg-black/50 z-0">
                            <GpsMap gps={gps} trajectory={trajectory} />
                        </div>
                    </div>
                </div>

                {/* 中間：Raw Sensors / 串流 / FC Log / GND CMD Log */}
                <div className="flex-1 flex flex-col gap-3 overflow-hidden">
                    
                    {/* Raw Sensor Telemetry */}
                    <div className="glass-panel p-2 flex flex-col gap-1 shrink-0">
                        <div className="p-1 bg-cyan-900/10 border-b border-cyan-900/20 text-xs font-bold text-cyan-400 uppercase mb-1">Raw Sensor Telemetry</div>
                        <div className="grid grid-cols-3 gap-2">
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="ACC_X" val={imu?.ax?.toFixed(2) ?? '0.00'} unit="g" />
                                <SensorValue label="ACC_Y" val={imu?.ay?.toFixed(2) ?? '0.00'} unit="g" />
                                <SensorValue label="ACC_Z" val={imu?.az?.toFixed(2) ?? '0.00'} unit="g" />
                            </div>
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="GYR_X" val={imu?.gx?.toFixed(1) ?? '0.0'} unit="°/s" />
                                <SensorValue label="GYR_Y" val={imu?.gy?.toFixed(1) ?? '0.0'} unit="°/s" />
                                <SensorValue label="GYR_Z" val={imu?.gz?.toFixed(1) ?? '0.0'} unit="°/s" />
                            </div>
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="MAG_X" val={imu?.mx?.toFixed(1) ?? '0.0'} unit="uT" />
                                <SensorValue label="MAG_Y" val={imu?.my?.toFixed(1) ?? '0.0'} unit="uT" />
                                <SensorValue label="MAG_Z" val={imu?.mz?.toFixed(1) ?? '0.0'} unit="uT" />
                            </div>
                        </div>
                        <div className="grid grid-cols-2 gap-2 mt-1">
                             <div className="flex flex-col gap-0.5">
                                <SensorValue label="BMP_PRES" val={bmp?.pressure?.toFixed(2) ?? '0.00'} unit="hPa" />
                                <SensorValue label="BMP_TEMP" val={bmp?.temp?.toFixed(1) ?? '0.0'} unit="°C" />
                             </div>
                             <div className="flex flex-col gap-0.5">
                                <SensorValue label="GPS_LAT" val={gpsRaw?.lat?.toFixed(6) ?? '0.000000'} unit="°" />
                                <SensorValue label="GPS_LON" val={gpsRaw?.lon?.toFixed(6) ?? '0.000000'} unit="°" />
                             </div>
                        </div>
                    </div>

                    <div className="glass-panel flex flex-col overflow-hidden shrink-0" style={{height:"90px"}}>
                        <div className="p-1.5 bg-blue-900/10 border-b border-blue-900/20 text-sm font-bold text-blue-400 uppercase">Parsed JSON Batch Stream</div>
                        <div className="terminal-box custom-scrollbar p-2">
                            {history.length === 0 
                                ? <div className="text-gray-800 text-xs">— 尚無資料 —</div>
                                : history.slice(-10).reverse().map((h,i) => (
                                    <div key={i} className="terminal-line text-blue-300 opacity-80 whitespace-nowrap overflow-hidden">{JSON.stringify(h.batch)}</div>
                                ))
                            }
                        </div>
                    </div>

                    <div className="glass-panel flex-1 p-2 overflow-y-auto font-mono text-sm custom-scrollbar">
                        <div className="text-xs text-gray-600 mb-1 font-bold">FC EVENT LOGS</div>
                        {fcLogs.length === 0
                            ? <div className="text-gray-800 text-xs">— 尚無資料 —</div>
                            : fcLogs.map((m,i) => <div key={i} className="text-cyan-900">[{new Date().toLocaleTimeString()}] {m}</div>)
                        }
                    </div>

                    <div className="glass-panel p-2 overflow-y-auto custom-scrollbar" style={{height:"100px"}}>
                        <div className="text-xs text-yellow-600 mb-1 font-bold uppercase">GND CMD Log</div>
                        {cmdLog.length === 0
                            ? <div className="text-gray-800 text-xs">— 等待操作 —</div>
                            : cmdLog.map((m,i) => <div key={i} className="text-yellow-500/70 text-xs">{m}</div>)
                        }
                    </div>
                </div>

                {/* 右側：狀態控制面板 */}
                <div className="w-1/4 flex flex-col gap-3 relative">
                    {!isLive && (
                        <div className="absolute inset-0 bg-black/80 z-20 flex items-center justify-center">
                            <button onClick={() => setIsLive(true)} className="px-4 py-1 bg-yellow-600 text-black font-bold text-sm rounded">RETURN LIVE</button>
                        </div>
                    )}

                    {/* Mission Sequence */}
                    <div className="glass-panel flex flex-col overflow-hidden flex-1">
                        <div className="p-2 bg-cyan-900/10 border-b border-cyan-900/20 text-sm font-bold text-cyan-500 uppercase">Mission Sequence</div>
                        <div className="p-2 flex flex-col gap-2 overflow-y-auto custom-scrollbar flex-1">
                            {normalNext.length === 0 && <div className="text-xs text-gray-700 text-center py-2 border border-dashed border-gray-800">NO NEXT STEP</div>}
                            {normalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-sequence w-full py-2">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {debugNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-debug w-full py-1.5">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                        </div>
                    </div>

                    {/* Critical Operations */}
                    <div className="glass-panel flex flex-col overflow-hidden" style={{minHeight:"150px"}}>
                        <div className="p-2 bg-red-900/5 border-b border-red-900/20 text-sm font-bold text-red-500 uppercase">⚠ Critical Operations</div>
                        <div className="p-2 flex flex-col gap-2 flex-1">
                            {criticalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-emergency w-full py-3">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {criticalNext.length === 0 && currStateId !== 14 && (
                                <div className="text-xs text-gray-700 text-center py-2 border border-dashed border-gray-800">— 無緊急操作 —</div>
                            )}
                            {currStateId === 14 && (
                                <div className="text-xs text-red-900 text-center py-4 border border-dashed border-red-900">☠ MISSION TERMINATED</div>
                            )}
                            {currStateId !== 14 && (
                                <button id="btn-force-terminate" onClick={() => sendStateCommand(14)}
                                    className="btn-base btn-emergency w-full py-1.5 mt-auto opacity-50 hover:opacity-100 text-sm">
                                    ☠ FORCE TERMINATE [14]
                                </button>
                            )}
                        </div>
                    </div>
                </div>
            </div>

            {/* Timeline Scrubber */}
            <div className="glass-panel p-2 flex items-center gap-3 h-9">
                <span className="text-xs text-gray-600 shrink-0">TIMELINE</span>
                <input type="range" min="0" max={Math.max(0, history.length-1)} value={histIdx}
                    onChange={e => { setIsLive(false); setHistIdx(parseInt(e.target.value)); }}
                    className="w-full h-1" />
                <span className="text-xs text-gray-600 shrink-0">{history.length} pkt</span>
            </div>
        </div>
    );
}

export default App;
