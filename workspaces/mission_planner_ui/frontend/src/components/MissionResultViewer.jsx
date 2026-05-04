import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, CatmullRomLine, Html, Sphere } from '@react-three/drei';
import * as THREE from 'three';
import { ArrowLeft, CheckCircle, XCircle, Layers, ChevronRight, Calendar } from 'lucide-react';
import ReactMarkdown from 'react-markdown';

import { API_BASE } from '../lib/config'

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`${API_BASE}${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const purposeColors = {
    none: '#64748b', fire_extinguisher: '#ef4444', gauge_reading: '#3b82f6',
    staircase_start: '#f59e0b', staircase_end: '#f59e0b',
    slope_start: '#22c55e', slope_end: '#22c55e',
    human_analyse: '#a855f7', elevator_door: '#06b6d4',
    elevator_inside: '#0891b2', elevator_exit: '#0e7490',
};

const safeParseJson = (data) => {
    if (!data) return null;
    if (typeof data === 'object') return data;
    if (typeof data === 'string') {
        try {
            // Sometimes it's double-stringified
            let parsed = JSON.parse(data);
            if (typeof parsed === 'string') {
                parsed = JSON.parse(parsed);
            }
            return parsed;
        } catch (e) {
            return data;
        }
    }
    return null;
};

// Formats JSON analysis output nicely
const AnalysisRenderer = ({ analysisData }) => {
    const analysis = safeParseJson(analysisData);
    if (!analysis) return null;
    if (typeof analysis === 'string') return <p className="text-[9px] text-zinc-500 leading-relaxed">{analysis}</p>;

    return (
        <div className="space-y-2 mt-2 bg-black/20 p-2.5 rounded-lg border border-white/5">
            {analysis.summary && <p className="text-[10px] text-zinc-300 font-medium leading-relaxed">{analysis.summary}</p>}
            
            {analysis.findings && Array.isArray(analysis.findings) && analysis.findings.length > 0 && (
                <div className="space-y-1">
                    <span className="text-[8px] font-bold uppercase tracking-wider text-zinc-600">Findings</span>
                    <ul className="list-disc pl-3 text-[9px] text-zinc-500 space-y-0.5">
                        {analysis.findings.map((f, i) => <li key={i}>{f}</li>)}
                    </ul>
                </div>
            )}
            
            <div className="flex flex-wrap gap-1.5 mt-2">
                {Object.entries(analysis).map(([k, v]) => {
                    // Skip keys we already explicitly rendered or are basic top-level status info
                    if (['summary', 'findings', 'decision', 'confidence', 'task', 'vlm_raw_response'].includes(k)) return null;
                    let displayVal = typeof v === 'object' ? JSON.stringify(v) : String(v);
                    return (
                        <div key={k} className="px-1.5 py-0.5 bg-zinc-800/40 rounded border border-zinc-700/50 text-[8px] text-zinc-400">
                            <span className="font-bold text-zinc-500 mr-1">{k}:</span>{displayVal}
                        </div>
                    );
                })}
            </div>
        </div>
    );
};

// Interactive waypoint sphere
const WaypointMarker = ({ wp, result, index, isSelected, isHovered, onSelect, onHover }) => {
    const meshRef = useRef();
    const isSuccess = result?.success === 'yes';
    const hasResult = !!result;
    const baseColor = hasResult ? (isSuccess ? '#22c55e' : '#ef4444') : '#64748b';

    useFrame((state) => {
        if (!meshRef.current) return;
        const t = state.clock.getElapsedTime();
        if (isSelected) {
            meshRef.current.scale.setScalar(1.4 + Math.sin(t * 3) * 0.1);
        } else if (isHovered) {
            meshRef.current.scale.setScalar(1.2);
        } else {
            meshRef.current.scale.setScalar(1.0);
        }
    });

    return (
        <group position={[wp.x, wp.y, wp.z]}>
            {/* Glow ring for selected */}
            {isSelected && (
                <mesh rotation={[Math.PI / 2, 0, 0]}>
                    <ringGeometry args={[0.28, 0.38, 32]} />
                    <meshBasicMaterial color={baseColor} transparent opacity={0.5} side={THREE.DoubleSide} />
                </mesh>
            )}
            <mesh
                ref={meshRef}
                onClick={(e) => { e.stopPropagation(); onSelect(wp, result); }}
                onPointerOver={(e) => { e.stopPropagation(); onHover(wp.ID); document.body.style.cursor = 'pointer'; }}
                onPointerOut={() => { onHover(null); document.body.style.cursor = 'default'; }}
            >
                <sphereGeometry args={[0.2, 16, 16]} />
                <meshStandardMaterial
                    color={baseColor}
                    emissive={baseColor}
                    emissiveIntensity={isSelected ? 0.6 : isHovered ? 0.4 : 0.2}
                    roughness={0.3}
                    metalness={0.5}
                />
            </mesh>

            {/* Always show number label */}
            <Html center position={[0, 0.45, 0]} style={{ pointerEvents: 'none' }}>
                <div className={`text-[8px] font-black px-1.5 py-0.5 rounded-full border backdrop-blur-sm ${hasResult
                    ? isSuccess ? 'bg-emerald-500/20 text-emerald-300 border-emerald-500/30' : 'bg-red-500/20 text-red-300 border-red-500/30'
                    : 'bg-zinc-800/80 text-zinc-400 border-zinc-700'}`}>
                    {index + 1}
                </div>
            </Html>

            {/* Hover popup */}
            {isHovered && (
                <Html center position={[0, 0.9, 0]} style={{ pointerEvents: 'none' }} distanceFactor={12}>
                    <div className="px-2.5 py-1.5 rounded-xl border border-white/10 bg-[#0d0d14]/95 backdrop-blur-xl shadow-xl whitespace-nowrap text-center">
                        <div className="text-[10px] font-bold text-white">{wp.name}</div>
                        {result && (
                            <>
                                <div className={`text-[8px] font-bold ${isSuccess ? 'text-emerald-400' : 'text-red-400'}`}>
                                    {isSuccess ? '✓ PASS' : '✗ FAIL'} · {Math.round((result.confidence || 0) * 100)}%
                                </div>
                                {(() => {
                                    const parsed = safeParseJson(result.analysis);
                                    if (parsed && typeof parsed === 'object' && parsed.summary) {
                                        return <div className="text-[8px] text-zinc-400 max-w-[150px] truncate mt-0.5 whitespace-normal leading-tight">{parsed.summary}</div>;
                                    }
                                    return null;
                                })()}
                                {result.image && (
                                    <img src={result.image} alt="" className="w-20 h-12 rounded-lg object-cover mt-1 border border-white/10"
                                        onError={e => e.target.style.display = 'none'} />
                                )}
                            </>
                        )}
                    </div>
                </Html>
            )}
        </group>
    );
};

// Camera focus helper
const CameraFocus = ({ target }) => {
    const { camera } = useThree();
    useEffect(() => {
        if (target) {
            const goal = new THREE.Vector3(target.x + 2, target.y + 2, target.z + 2);
            camera.position.lerp(goal, 0.5);
            camera.lookAt(target.x, target.y, target.z);
        }
    }, [target, camera]);
    return null;
};

const MissionResultViewer = ({ onBack }) => {
    const [missions, setMissions] = useState([]);
    const [selectedMission, setSelectedMission] = useState(null);
    const [dates, setDates] = useState([]);
    const [selectedDate, setSelectedDate] = useState('');
    const [results, setResults] = useState([]);
    const [waypoints, setWaypoints] = useState([]);
    const [selectedWp, setSelectedWp] = useState(null);
    const [hoveredWpId, setHoveredWpId] = useState(null);
    const [cameraTarget, setCameraTarget] = useState(null);
    const controlsRef = useRef();

    useEffect(() => {
        apiCall('/missions').then(d => setMissions(d || [])).catch(console.error);
    }, []);

    useEffect(() => {
        if (!selectedMission) { setDates([]); setSelectedDate(''); setWaypoints([]); return; }
        setWaypoints(selectedMission.waypoints || []);
        apiCall(`/missions/${selectedMission.ID}/results/dates`).then(d => {
            setDates(d || []);
            if (d && d.length > 0) setSelectedDate(d[0]);
        }).catch(console.error);
    }, [selectedMission]);

    useEffect(() => {
        if (!selectedMission || !selectedDate) { setResults([]); return; }
        apiCall(`/missions/${selectedMission.ID}/results?date=${selectedDate}`).then(d => setResults(d || [])).catch(console.error);
    }, [selectedMission, selectedDate]);

    const resultMap = {};
    results.forEach(r => { resultMap[r.mission_waypoint_id] = r; });

    const passCount = results.filter(r => r.success === 'yes').length;
    const failCount = results.filter(r => r.success !== 'yes').length;
    const avgConf = results.length > 0 ? results.reduce((s, r) => s + (r.confidence || 0), 0) / results.length : 0;

    const handleWpSelect = (wp, result) => {
        setSelectedWp(prev => prev?.ID === wp.ID ? null : wp);
        setCameraTarget(prev => prev && prev.x === wp.x ? null : new THREE.Vector3(wp.x, wp.y, wp.z));
    };

    return (
        <div className="h-screen bg-[#0a0a0f] text-white font-sans flex flex-col overflow-hidden">
            {/* Header */}
            <header className="border-b border-white/5 bg-[#0d0d14]/80 backdrop-blur-xl flex-shrink-0">
                <div className="px-6 py-3.5 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                        <button onClick={onBack} className="p-2 rounded-lg hover:bg-white/5 text-zinc-400 hover:text-white transition-colors"><ArrowLeft size={16} /></button>
                        <div>
                            <h1 className="text-sm font-bold tracking-wide">Mission Results</h1>
                            <p className="text-[9px] text-zinc-600 uppercase tracking-widest">Inspection Analysis & 3D Visualization</p>
                        </div>
                    </div>
                    {results.length > 0 && (
                        <div className="flex items-center gap-4 text-xs">
                            <span className="flex items-center gap-1.5 text-emerald-400"><CheckCircle size={12} />{passCount} Pass</span>
                            <span className="flex items-center gap-1.5 text-red-400"><XCircle size={12} />{failCount} Fail</span>
                            <span className="text-zinc-500">Avg: <span className="text-white font-bold">{Math.round(avgConf * 100)}%</span></span>
                        </div>
                    )}
                </div>
            </header>

            <div className="flex-1 flex overflow-hidden">
                {/* Sidebar */}
                <aside className="w-64 border-r border-white/5 bg-[#0d0d14]/50 flex flex-col flex-shrink-0 overflow-hidden">
                    <div className="p-4 flex-shrink-0">
                        <div className="text-[9px] font-bold uppercase tracking-widest text-zinc-600 flex items-center gap-1.5 mb-3"><Layers size={9} /> Missions</div>
                        <div className="space-y-1 max-h-48 overflow-auto">
                            {missions.map(m => (
                                <button key={m.ID} onClick={() => { setSelectedMission(m); setSelectedWp(null); }}
                                    className={`w-full text-left px-3 py-2 rounded-xl transition-all group text-xs ${selectedMission?.ID === m.ID ? 'bg-sky-600/20 border border-sky-500/30 text-white' : 'hover:bg-white/[0.03] border border-transparent text-zinc-400 hover:text-white'}`}>
                                    <div className="flex items-center justify-between">
                                        <span className="font-bold truncate">{m.name}</span>
                                        <ChevronRight size={10} className={`flex-shrink-0 ${selectedMission?.ID === m.ID ? 'text-sky-400' : 'text-zinc-700'}`} />
                                    </div>
                                    <div className="text-[9px] text-zinc-600 mt-0.5">{(m.waypoints || []).length} waypoints</div>
                                </button>
                            ))}
                        </div>
                    </div>

                    {selectedMission && dates.length > 0 && (
                        <div className="px-4 pb-4 border-t border-white/5 pt-4 flex-shrink-0">
                            <div className="text-[9px] font-bold uppercase tracking-widest text-zinc-600 flex items-center gap-1.5 mb-3"><Calendar size={9} /> Dates</div>
                            <div className="space-y-1 max-h-32 overflow-auto">
                                {dates.map(d => (
                                    <button key={d} onClick={() => setSelectedDate(d)}
                                        className={`w-full text-left px-2.5 py-1.5 rounded-lg text-[10px] transition-colors ${selectedDate === d ? 'bg-sky-600/15 text-sky-400 border border-sky-500/20' : 'text-zinc-500 hover:text-white hover:bg-white/[0.03]'}`}>
                                        {d}
                                    </button>
                                ))}
                            </div>
                        </div>
                    )}

                    {/* Selected waypoint detail */}
                    {selectedWp && (
                        <div className="flex-1 p-4 border-t border-white/5 overflow-auto">
                            <div className="text-[9px] font-bold uppercase tracking-widest text-zinc-600 mb-3">Selected Point</div>
                            <div className="space-y-2">
                                <div className="text-xs font-bold text-white">{selectedWp.name}</div>
                                <div className="text-[9px] text-zinc-500">{selectedWp.purpose || 'none'}</div>
                                {resultMap[selectedWp.ID] ? (() => {
                                    const r = resultMap[selectedWp.ID];
                                    const ok = r.success === 'yes';
                                    return (
                                        <div className="mt-3 space-y-2">
                                            <div className={`inline-flex items-center gap-1 px-2 py-1 rounded-full text-[9px] font-black uppercase border ${ok ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30' : 'bg-red-500/20 text-red-400 border-red-500/30'}`}>
                                                {ok ? <CheckCircle size={8} /> : <XCircle size={8} />} {ok ? 'PASS' : 'FAIL'}
                                            </div>
                                            <div className="text-[9px] text-zinc-400">Confidence: <span className="text-white font-bold">{Math.round((r.confidence || 0) * 100)}%</span></div>
                                            <AnalysisRenderer analysisData={r.analysis} />
                                            {r.image && (
                                                <img src={r.image} alt="" className="w-full rounded-lg border border-zinc-800 object-cover"
                                                    onError={e => e.target.style.display = 'none'} />
                                            )}
                                        </div>
                                    );
                                })() : (
                                    <p className="text-[9px] text-zinc-700 italic mt-2">No result recorded.</p>
                                )}
                                <button onClick={() => { setSelectedWp(null); setCameraTarget(null); }}
                                    className="text-[9px] text-zinc-600 hover:text-white transition-colors mt-2">Clear selection</button>
                            </div>
                        </div>
                    )}
                </aside>

                {/* Main content */}
                <div className="flex-1 flex flex-col overflow-hidden">
                    {!selectedMission ? (
                        <div className="flex-1 flex items-center justify-center text-zinc-700 text-sm italic">Select a mission.</div>
                    ) : (
                        <>
                            {/* 3D Canvas */}
                            <div className="flex-1 relative min-h-0" style={{ flexBasis: results.length > 0 ? '55%' : '100%' }}>
                                <Canvas camera={{ position: [5, 5, 5], fov: 50 }}>
                                    <color attach="background" args={['#0a0a0f']} />
                                    <fog attach="fog" args={['#0a0a0f', 40, 120]} />
                                    <ambientLight intensity={0.6} />
                                    <pointLight position={[10, 10, 10]} intensity={0.8} />
                                    <pointLight position={[-10, -10, -10]} intensity={0.3} color="#6366f1" />
                                    <gridHelper args={[40, 40, '#111122', '#111122']} />
                                    <axesHelper args={[2]} />
                                    <OrbitControls ref={controlsRef} enableDamping dampingFactor={0.05} makeDefault />
                                    <CameraFocus target={cameraTarget} />

                                    <group rotation={[-Math.PI / 2, 0, 0]}>
                                        {/* Path line */}
                                        {waypoints.length >= 2 && (
                                            <CatmullRomLine
                                                points={waypoints.map(p => [p.x, p.y, p.z])}
                                                color={results.length > 0 ? '#38bdf8' : '#334155'}
                                                lineWidth={2}
                                                segments={80}
                                                curveType="catmullrom"
                                                tension={0.5}
                                            />
                                        )}
                                        {/* Waypoint markers */}
                                        {waypoints.map((wp, i) => (
                                            <WaypointMarker
                                                key={wp.ID}
                                                wp={wp}
                                                result={resultMap[wp.ID]}
                                                index={i}
                                                isSelected={selectedWp?.ID === wp.ID}
                                                isHovered={hoveredWpId === wp.ID}
                                                onSelect={handleWpSelect}
                                                onHover={setHoveredWpId}
                                            />
                                        ))}
                                    </group>
                                </Canvas>

                                {/* Hint overlay */}
                                <div className="absolute bottom-3 left-1/2 -translate-x-1/2 text-[9px] text-zinc-700 pointer-events-none">
                                    Click waypoint to select · Drag to orbit · Scroll to zoom
                                </div>
                            </div>

                            {/* Results table */}
                            {results.length > 0 && (
                                <div className="border-t border-white/5 overflow-auto" style={{ flexBasis: '45%', flexShrink: 0 }}>
                                    <table className="w-full text-xs">
                                        <thead className="sticky top-0 bg-[#0a0a0f]/95 backdrop-blur-sm">
                                            <tr className="text-left text-zinc-600 uppercase tracking-wider border-b border-white/5 text-[9px]">
                                                <th className="px-4 py-2.5 font-bold">#</th>
                                                <th className="px-4 py-2.5 font-bold">Waypoint</th>
                                                <th className="px-4 py-2.5 font-bold">Purpose</th>
                                                <th className="px-4 py-2.5 font-bold">Result</th>
                                                <th className="px-4 py-2.5 font-bold">Confidence</th>
                                                <th className="px-4 py-2.5 font-bold">Analysis</th>
                                                <th className="px-4 py-2.5 font-bold">Image</th>
                                            </tr>
                                        </thead>
                                        <tbody>
                                            {waypoints.map((wp, idx) => {
                                                const r = resultMap[wp.ID];
                                                const isSuccess = r?.success === 'yes';
                                                const isActive = selectedWp?.ID === wp.ID;
                                                return (
                                                    <tr key={wp.ID} onClick={() => handleWpSelect(wp, r)}
                                                        className={`border-b border-white/[0.03] transition-colors cursor-pointer ${isActive ? 'bg-sky-600/10' : 'hover:bg-white/[0.02]'}`}>
                                                        <td className="px-4 py-2.5 text-zinc-600 font-mono">{idx + 1}</td>
                                                        <td className="px-4 py-2.5 font-bold text-zinc-200">{wp.name}</td>
                                                        <td className="px-4 py-2.5">
                                                            <span className="px-1.5 py-0.5 rounded-full bg-zinc-800 text-zinc-400 text-[8px] font-bold uppercase">{wp.purpose || 'none'}</span>
                                                        </td>
                                                        <td className="px-4 py-2.5">
                                                            {r ? (
                                                                <span className={`inline-flex items-center gap-1 px-1.5 py-0.5 rounded-full text-[8px] font-black uppercase ${isSuccess ? 'bg-emerald-500/20 text-emerald-400' : 'bg-red-500/20 text-red-400'}`}>
                                                                    {isSuccess ? <CheckCircle size={8} /> : <XCircle size={8} />} {isSuccess ? 'PASS' : 'FAIL'}
                                                                </span>
                                                            ) : <span className="text-zinc-700 text-[9px]">—</span>}
                                                        </td>
                                                        <td className="px-4 py-2.5">
                                                            {r ? (
                                                                <div className="flex items-center gap-2">
                                                                    <div className="w-14 h-1 rounded-full bg-zinc-800 overflow-hidden">
                                                                        <div className={`h-full rounded-full ${(r.confidence || 0) > 0.7 ? 'bg-emerald-500' : (r.confidence || 0) > 0.4 ? 'bg-amber-500' : 'bg-red-500'}`}
                                                                            style={{ width: `${(r.confidence || 0) * 100}%` }} />
                                                                    </div>
                                                                    <span className="text-zinc-400 font-mono text-[9px]">{Math.round((r.confidence || 0) * 100)}%</span>
                                                                </div>
                                                            ) : <span className="text-zinc-700">—</span>}
                                                        </td>
                                                        <td className="px-4 py-2.5 text-zinc-500 max-w-[160px] truncate text-[9px]">
                                                            {(() => {
                                                                const parsed = safeParseJson(r?.analysis);
                                                                if (!parsed) return '—';
                                                                return typeof parsed === 'object' ? (parsed.summary || JSON.stringify(parsed)) : parsed;
                                                            })()}
                                                        </td>
                                                        <td className="px-4 py-2.5">
                                                            {r?.image ? (
                                                                <img src={r.image} alt="" className="h-7 w-10 rounded border border-zinc-800 object-cover"
                                                                    onError={e => e.target.style.display = 'none'} />
                                                            ) : <span className="text-zinc-700">—</span>}
                                                        </td>
                                                    </tr>
                                                );
                                            })}
                                        </tbody>
                                    </table>
                                </div>
                            )}

                            {!selectedDate && (
                                <div className="flex-1 flex items-center justify-center text-zinc-600 text-xs italic">No results recorded yet for this mission.</div>
                            )}
                        </>
                    )}
                </div>
            </div>
        </div>
    );
};

export default MissionResultViewer;
