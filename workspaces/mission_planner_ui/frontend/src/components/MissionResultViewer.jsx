import React, { useState, useEffect, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, CatmullRomLine, Html } from '@react-three/drei';
import { ArrowLeft, Calendar, MessageCircle, Send, X as XIcon, CheckCircle, XCircle, Bot } from 'lucide-react';
import ReactMarkdown from 'react-markdown';

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const HOST = window.location.hostname;
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`http://${HOST}:8000${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const ResultMarker = ({ wp, result }) => {
    const isSuccess = result?.success === 'yes';
    const color = isSuccess ? '#22c55e' : '#ef4444';
    return (
        <group position={[wp.x, wp.y, wp.z]}>
            <mesh>
                <sphereGeometry args={[0.18]} />
                <meshBasicMaterial color={color} transparent opacity={0.85} />
            </mesh>
            <Html position={[0, 0, 0.4]} center distanceFactor={18} style={{ pointerEvents: 'none' }}>
                <div className="flex flex-col items-center gap-1">
                    <div className={`px-2 py-0.5 rounded-full text-[7px] font-black uppercase tracking-wider whitespace-nowrap border backdrop-blur-md shadow-lg ${isSuccess ? 'bg-emerald-500/20 text-emerald-300 border-emerald-500/30' : 'bg-red-500/20 text-red-300 border-red-500/30'}`}>
                        {isSuccess ? '✓' : '✗'} {wp.name}
                    </div>
                    {result?.image_url && (
                        <img src={result.image_url} alt="" className="w-16 h-10 rounded border border-white/20 object-cover shadow-lg" />
                    )}
                </div>
            </Html>
        </group>
    );
};

const MissionResultViewer = ({ onBack }) => {
    const [missions, setMissions] = useState([]);
    const [selectedMissionId, setSelectedMissionId] = useState(null);
    const [dates, setDates] = useState([]);
    const [selectedDate, setSelectedDate] = useState('');
    const [results, setResults] = useState([]);
    const [waypoints, setWaypoints] = useState([]);
    const [chatOpen, setChatOpen] = useState(false);
    const [chatMessages, setChatMessages] = useState([]);
    const [chatInput, setChatInput] = useState('');
    const [chatLoading, setChatLoading] = useState(false);
    const chatEndRef = useRef(null);

    useEffect(() => {
        apiCall('/missions').then(d => setMissions(d || [])).catch(console.error);
    }, []);

    const selectedMission = missions.find(m => m.ID === selectedMissionId);

    useEffect(() => {
        if (!selectedMissionId) { setDates([]); setSelectedDate(''); return; }
        apiCall(`/missions/${selectedMissionId}/results/dates`).then(d => {
            setDates(d || []);
            if (d && d.length > 0) setSelectedDate(d[0]);
        }).catch(console.error);
        setWaypoints(selectedMission?.waypoints || []);
    }, [selectedMissionId, selectedMission]);

    useEffect(() => {
        if (!selectedMissionId || !selectedDate) { setResults([]); return; }
        apiCall(`/missions/${selectedMissionId}/results?date=${selectedDate}`).then(d => setResults(d || [])).catch(console.error);
    }, [selectedMissionId, selectedDate]);

    useEffect(() => {
        chatEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [chatMessages]);

    const resultMap = {};
    results.forEach(r => { resultMap[r.mission_waypoint_id] = r; });

    const passCount = results.filter(r => r.success === 'yes').length;
    const failCount = results.filter(r => r.success !== 'yes').length;
    const avgConfidence = results.length > 0 ? (results.reduce((s, r) => s + (r.confidence || 0), 0) / results.length) : 0;

    const sendChat = async () => {
        if (!chatInput.trim()) return;
        const msg = chatInput.trim();
        setChatMessages(prev => [...prev, { role: 'user', text: msg }]);
        setChatInput('');
        setChatLoading(true);
        try {
            const data = await apiCall('/chat', 'POST', {
                mission_id: selectedMissionId || 0,
                date: selectedDate || '',
                message: msg,
            });
            setChatMessages(prev => [...prev, { role: 'assistant', text: data.reply }]);
        } catch (e) {
            setChatMessages(prev => [...prev, { role: 'assistant', text: 'Error: Could not reach AI assistant. Check your API key.' }]);
        }
        setChatLoading(false);
    };

    return (
        <div className="min-h-screen bg-[#0a0a0f] text-white font-sans flex flex-col">
            {/* Header */}
            <header className="border-b border-white/5 bg-[#0d0d14]/80 backdrop-blur-xl sticky top-0 z-50">
                <div className="max-w-[100rem] mx-auto px-6 py-3 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                        <button onClick={onBack} className="p-2 rounded-lg hover:bg-white/5 text-zinc-400 hover:text-white transition-colors"><ArrowLeft size={18} /></button>
                        <div>
                            <h1 className="text-lg font-bold tracking-wide">Mission Results</h1>
                            <p className="text-[10px] text-zinc-500 uppercase tracking-widest">Inspection Analysis & AI Insights</p>
                        </div>
                    </div>
                    <div className="flex items-center gap-3">
                        {/* Mission Selector */}
                        <select value={selectedMissionId || ''} onChange={e => setSelectedMissionId(e.target.value ? parseInt(e.target.value) : null)}
                            className="bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none">
                            <option value="">Select Mission</option>
                            {missions.map(m => <option key={m.ID} value={m.ID} className="bg-zinc-900">{m.name}</option>)}
                        </select>
                        {/* Date Selector */}
                        {dates.length > 0 && (
                            <select value={selectedDate} onChange={e => setSelectedDate(e.target.value)}
                                className="bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none">
                                {dates.map(d => <option key={d} value={d} className="bg-zinc-900">{d}</option>)}
                            </select>
                        )}
                        <button onClick={() => setChatOpen(!chatOpen)}
                            className={`p-2.5 rounded-lg text-xs font-bold flex items-center gap-1.5 transition-all ${chatOpen ? 'bg-sky-600 text-white shadow-lg shadow-sky-500/20' : 'bg-zinc-800 text-zinc-400 hover:text-white'}`}>
                            <Bot size={14} /> AI Chat
                        </button>
                    </div>
                </div>
            </header>

            <div className="flex-1 flex overflow-hidden">
                {/* Main Content */}
                <div className={`flex-1 flex flex-col overflow-hidden transition-all ${chatOpen ? 'mr-0' : ''}`}>
                    {!selectedMissionId ? (
                        <div className="flex-1 flex items-center justify-center text-zinc-600 text-sm italic">Select a mission to view results.</div>
                    ) : results.length === 0 ? (
                        <div className="flex-1 flex flex-col">
                            {/* 3D View placeholder */}
                            <div className="h-[40vh] relative border-b border-white/5">
                                <Canvas camera={{ position: [5, 5, 5], fov: 50 }}>
                                    <color attach="background" args={['#0a0a0f']} />
                                    <ambientLight intensity={0.5} />
                                    <gridHelper args={[30, 30, '#1a1a2e', '#1a1a2e']} />
                                    <OrbitControls enableDamping dampingFactor={0.05} />
                                    <group rotation={[-Math.PI / 2, 0, 0]}>
                                        {waypoints.length >= 2 && (
                                            <CatmullRomLine points={waypoints.map(p => [p.x, p.y, p.z])} color="#334155" lineWidth={2} segments={64} curveType="catmullrom" tension={0.5} />
                                        )}
                                    </group>
                                </Canvas>
                            </div>
                            <div className="flex-1 flex items-center justify-center text-zinc-600 text-sm italic">No results found for this date.</div>
                        </div>
                    ) : (
                        <>
                            {/* Summary Stats */}
                            <div className="px-6 py-3 border-b border-white/5 flex items-center gap-6">
                                <div className="flex items-center gap-2 text-emerald-400"><CheckCircle size={14} /><span className="text-sm font-bold">{passCount} Pass</span></div>
                                <div className="flex items-center gap-2 text-red-400"><XCircle size={14} /><span className="text-sm font-bold">{failCount} Fail</span></div>
                                <div className="text-zinc-500 text-xs">Avg Confidence: <span className="text-white font-bold">{Math.round(avgConfidence * 100)}%</span></div>
                                <div className="text-zinc-500 text-xs">Date: <span className="text-white font-bold">{selectedDate}</span></div>
                            </div>

                            {/* 3D Visualization */}
                            <div className="h-[40vh] relative border-b border-white/5">
                                <Canvas camera={{ position: [5, 5, 5], fov: 50 }}>
                                    <color attach="background" args={['#0a0a0f']} />
                                    <fog attach="fog" args={['#0a0a0f', 30, 100]} />
                                    <ambientLight intensity={0.5} />
                                    <pointLight position={[10, 10, 10]} intensity={0.5} />
                                    <gridHelper args={[30, 30, '#1a1a2e', '#1a1a2e']} />
                                    <axesHelper args={[2]} />
                                    <OrbitControls enableDamping dampingFactor={0.05} />
                                    <group rotation={[-Math.PI / 2, 0, 0]}>
                                        {waypoints.length >= 2 && (
                                            <CatmullRomLine points={waypoints.map(p => [p.x, p.y, p.z])} color="#38bdf8" lineWidth={2} segments={64} curveType="catmullrom" tension={0.5} />
                                        )}
                                        {waypoints.map((wp, i) => (
                                            <ResultMarker key={i} wp={wp} result={resultMap[wp.ID]} />
                                        ))}
                                    </group>
                                </Canvas>
                            </div>

                            {/* Results Table */}
                            <div className="flex-1 overflow-auto p-6">
                                <table className="w-full text-xs">
                                    <thead>
                                        <tr className="text-left text-zinc-500 uppercase tracking-wider border-b border-white/5">
                                            <th className="pb-3 font-bold">#</th>
                                            <th className="pb-3 font-bold">Waypoint</th>
                                            <th className="pb-3 font-bold">Purpose</th>
                                            <th className="pb-3 font-bold">Result</th>
                                            <th className="pb-3 font-bold">Confidence</th>
                                            <th className="pb-3 font-bold">Analysis</th>
                                            <th className="pb-3 font-bold">Image</th>
                                        </tr>
                                    </thead>
                                    <tbody>
                                        {waypoints.map((wp, idx) => {
                                            const r = resultMap[wp.ID];
                                            const isSuccess = r?.success === 'yes';
                                            return (
                                                <tr key={wp.ID} className="border-b border-white/5 hover:bg-white/[0.02] transition-colors">
                                                    <td className="py-3 text-zinc-500 font-mono">{idx + 1}</td>
                                                    <td className="py-3 font-bold text-zinc-200">{wp.name}</td>
                                                    <td className="py-3"><span className="px-2 py-0.5 rounded-full bg-zinc-800 text-zinc-400 text-[10px] font-bold uppercase">{wp.purpose || 'none'}</span></td>
                                                    <td className="py-3">
                                                        {r ? (
                                                            <span className={`inline-flex items-center gap-1 px-2 py-0.5 rounded-full text-[10px] font-black uppercase ${isSuccess ? 'bg-emerald-500/20 text-emerald-400' : 'bg-red-500/20 text-red-400'}`}>
                                                                {isSuccess ? <CheckCircle size={10} /> : <XCircle size={10} />} {isSuccess ? 'PASS' : 'FAIL'}
                                                            </span>
                                                        ) : <span className="text-zinc-700">—</span>}
                                                    </td>
                                                    <td className="py-3">
                                                        {r ? (
                                                            <div className="flex items-center gap-2">
                                                                <div className="w-16 h-1.5 rounded-full bg-zinc-800 overflow-hidden">
                                                                    <div className={`h-full rounded-full ${(r.confidence || 0) > 0.7 ? 'bg-emerald-500' : (r.confidence || 0) > 0.4 ? 'bg-amber-500' : 'bg-red-500'}`} style={{ width: `${(r.confidence || 0) * 100}%` }} />
                                                                </div>
                                                                <span className="text-zinc-400 font-mono">{Math.round((r.confidence || 0) * 100)}%</span>
                                                            </div>
                                                        ) : <span className="text-zinc-700">—</span>}
                                                    </td>
                                                    <td className="py-3 text-zinc-400 max-w-[200px] truncate">{r?.analysis || '—'}</td>
                                                    <td className="py-3">
                                                        {r?.image_url ? (
                                                            <img src={r.image_url} alt="" className="h-8 w-12 rounded border border-zinc-800 object-cover" />
                                                        ) : <span className="text-zinc-700">—</span>}
                                                    </td>
                                                </tr>
                                            );
                                        })}
                                    </tbody>
                                </table>
                            </div>
                        </>
                    )}
                </div>

                {/* Chat Sidebar */}
                {chatOpen && (
                    <div className="w-96 border-l border-white/5 bg-[#0d0d14] flex flex-col">
                        <div className="p-4 border-b border-white/5 flex items-center justify-between">
                            <div className="flex items-center gap-2">
                                <Bot size={16} className="text-sky-400" />
                                <span className="text-sm font-bold">Mission Analyst</span>
                            </div>
                            <button onClick={() => setChatOpen(false)} className="p-1 text-zinc-500 hover:text-white"><XIcon size={14} /></button>
                        </div>

                        {/* Messages */}
                        <div className="flex-1 overflow-auto p-4 space-y-3">
                            {chatMessages.length === 0 && (
                                <div className="text-center py-8 text-zinc-600 text-xs">
                                    <Bot size={24} className="mx-auto mb-2 opacity-30" />
                                    Ask me about inspection results.
                                    <br /><span className="text-zinc-700">e.g. "Summarize today's results"</span>
                                </div>
                            )}
                            {chatMessages.map((msg, i) => (
                                <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                                    <div className={`max-w-[85%] px-3 py-2 rounded-xl text-xs leading-relaxed overflow-hidden ${msg.role === 'user' ? 'bg-sky-600 text-white rounded-br-sm' : 'bg-zinc-800 text-zinc-300 rounded-bl-sm [&_p]:mb-2 [&_p:last-child]:mb-0 [&_ul]:list-disc [&_ul]:pl-4 [&_ol]:list-decimal [&_ol]:pl-4 [&_li]:mb-1 [&_strong]:text-white'}`}>
                                        {msg.role === 'user' ? (
                                            msg.text.split('\n').map((line, j) => <p key={j} className={j > 0 ? 'mt-1' : ''}>{line}</p>)
                                        ) : (
                                            <ReactMarkdown>{msg.text}</ReactMarkdown>
                                        )}
                                    </div>
                                </div>
                            ))}
                            {chatLoading && (
                                <div className="flex justify-start">
                                    <div className="bg-zinc-800 text-zinc-500 px-3 py-2 rounded-xl text-xs animate-pulse">Thinking...</div>
                                </div>
                            )}
                            <div ref={chatEndRef} />
                        </div>

                        {/* Input */}
                        <div className="p-3 border-t border-white/5">
                            <div className="flex gap-2">
                                <input value={chatInput} onChange={e => setChatInput(e.target.value)}
                                    onKeyDown={e => e.key === 'Enter' && !e.shiftKey && sendChat()}
                                    placeholder="Ask about results..."
                                    className="flex-1 bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none focus:border-zinc-600 placeholder-zinc-700" />
                                <button onClick={sendChat} disabled={chatLoading || !chatInput.trim()}
                                    className="px-3 py-2 bg-sky-600 hover:bg-sky-500 disabled:bg-zinc-800 disabled:text-zinc-600 text-white rounded-lg transition-colors">
                                    <Send size={12} />
                                </button>
                            </div>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
};

export default MissionResultViewer;
