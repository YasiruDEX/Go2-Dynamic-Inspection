import React, { useState, useEffect } from 'react';
import { ArrowLeft, Save, Check, X, Calendar, Image, ChevronRight, Layers, AlertTriangle, CheckCircle2 } from 'lucide-react';

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const HOST = window.location.hostname;
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`http://${HOST}:8000${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const purposeColors = {
    none: 'bg-zinc-800 text-zinc-400',
    fire_extinguisher: 'bg-red-500/20 text-red-400',
    gauge_reading: 'bg-blue-500/20 text-blue-400',
    staircase_start: 'bg-amber-500/20 text-amber-400',
    staircase_end: 'bg-amber-500/20 text-amber-400',
    slope_start: 'bg-emerald-500/20 text-emerald-400',
    slope_end: 'bg-emerald-500/20 text-emerald-400',
    human_analyse: 'bg-violet-500/20 text-violet-400',
    elevator_door: 'bg-cyan-500/20 text-cyan-400',
    elevator_inside: 'bg-cyan-500/20 text-cyan-400',
    elevator_exit: 'bg-cyan-500/20 text-cyan-400',
};

const MissionResultEditor = ({ onBack }) => {
    const [missions, setMissions] = useState([]);
    const [selectedMission, setSelectedMission] = useState(null);
    const [selectedDate, setSelectedDate] = useState(new Date().toISOString().split('T')[0]);
    const [existingDates, setExistingDates] = useState([]);
    const [waypoints, setWaypoints] = useState([]);
    const [results, setResults] = useState({});
    const [saving, setSaving] = useState({});
    const [saved, setSaved] = useState({});
    const [bulkSaving, setBulkSaving] = useState(false);

    useEffect(() => {
        apiCall('/missions').then(d => setMissions(d || [])).catch(console.error);
    }, []);

    useEffect(() => {
        if (!selectedMission) { setWaypoints([]); setResults({}); setExistingDates([]); return; }
        const wps = selectedMission.waypoints || [];
        setWaypoints(wps);
        apiCall(`/missions/${selectedMission.ID}/results/dates`).then(d => setExistingDates(d || [])).catch(console.error);
    }, [selectedMission]);

    useEffect(() => {
        if (!selectedMission || !selectedDate) { setResults({}); return; }
        apiCall(`/missions/${selectedMission.ID}/results?date=${selectedDate}`).then(data => {
            const map = {};
            (data || []).forEach(r => {
                map[r.mission_waypoint_id] = { id: r.ID, image_url: r.image_url || '', success: r.success || 'no', analysis: r.analysis || '', confidence: r.confidence || 0 };
            });
            setResults(map);
        }).catch(console.error);
    }, [selectedMission, selectedDate]);

    const updateResult = (wpId, field, value) => {
        setResults(prev => ({ ...prev, [wpId]: { ...(prev[wpId] || { image_url: '', success: 'no', analysis: '', confidence: 0 }), [field]: value } }));
    };

    const saveResult = async (wp) => {
        if (!selectedMission) return;
        const r = results[wp.ID] || {};
        setSaving(prev => ({ ...prev, [wp.ID]: true }));
        try {
            await apiCall(`/missions/${selectedMission.ID}/results`, 'POST', {
                mission_waypoint_id: wp.ID,
                date: selectedDate,
                image_url: r.image_url || '',
                success: r.success || 'no',
                analysis: r.analysis || '',
                confidence: parseFloat(r.confidence) || 0,
            });
            setSaved(prev => ({ ...prev, [wp.ID]: true }));
            setTimeout(() => setSaved(prev => ({ ...prev, [wp.ID]: false })), 2000);
            // refresh dates
            apiCall(`/missions/${selectedMission.ID}/results/dates`).then(d => setExistingDates(d || [])).catch(() => {});
        } catch (e) { console.error(e); }
        setSaving(prev => ({ ...prev, [wp.ID]: false }));
    };

    const saveAll = async () => {
        if (!selectedMission) return;
        setBulkSaving(true);
        for (const wp of waypoints) await saveResult(wp);
        setBulkSaving(false);
    };

    const passCount = waypoints.filter(wp => results[wp.ID]?.success === 'yes').length;
    const failCount = waypoints.filter(wp => results[wp.ID] && results[wp.ID].success !== 'yes').length;

    return (
        <div className="h-screen bg-[#0a0a0f] text-white font-sans flex flex-col overflow-hidden">
            {/* Header */}
            <header className="border-b border-white/5 bg-[#0d0d14]/80 backdrop-blur-xl flex-shrink-0">
                <div className="px-6 py-3.5 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                        <button onClick={onBack} className="p-2 rounded-lg hover:bg-white/5 text-zinc-400 hover:text-white transition-colors"><ArrowLeft size={16} /></button>
                        <div>
                            <h1 className="text-sm font-bold tracking-wide">Result Editor</h1>
                            <p className="text-[9px] text-zinc-600 uppercase tracking-widest">Record daily inspection outcomes</p>
                        </div>
                    </div>
                    <div className="flex items-center gap-3">
                        {selectedMission && waypoints.length > 0 && (
                            <div className="flex items-center gap-3">
                                {passCount > 0 && <span className="text-[10px] text-emerald-400 font-bold flex items-center gap-1"><CheckCircle2 size={10} />{passCount} Pass</span>}
                                {failCount > 0 && <span className="text-[10px] text-red-400 font-bold flex items-center gap-1"><AlertTriangle size={10} />{failCount} Fail</span>}
                                <button onClick={saveAll} disabled={bulkSaving}
                                    className="flex items-center gap-1.5 px-4 py-2 rounded-lg bg-emerald-600 hover:bg-emerald-500 disabled:bg-zinc-800 disabled:text-zinc-600 text-white text-[10px] font-bold uppercase tracking-wider transition-colors shadow-lg shadow-emerald-500/20">
                                    <Save size={11} /> {bulkSaving ? 'Saving...' : 'Save All'}
                                </button>
                            </div>
                        )}
                    </div>
                </div>
            </header>

            <div className="flex-1 flex overflow-hidden">
                {/* Mission Sidebar */}
                <aside className="w-72 border-r border-white/5 bg-[#0d0d14]/50 flex flex-col flex-shrink-0 overflow-hidden">
                    <div className="p-4 border-b border-white/5">
                        <div className="text-[9px] font-bold uppercase tracking-widest text-zinc-600 flex items-center gap-1.5 mb-3">
                            <Layers size={9} /> Missions
                        </div>
                        <div className="space-y-1">
                            {missions.map(m => (
                                <button key={m.ID} onClick={() => setSelectedMission(m)}
                                    className={`w-full text-left px-3 py-2.5 rounded-xl transition-all group ${selectedMission?.ID === m.ID ? 'bg-sky-600/20 border border-sky-500/30 text-white' : 'hover:bg-white/[0.03] border border-transparent text-zinc-400 hover:text-white'}`}>
                                    <div className="flex items-center justify-between">
                                        <span className="text-xs font-bold truncate">{m.name}</span>
                                        <ChevronRight size={11} className={`transition-transform ${selectedMission?.ID === m.ID ? 'text-sky-400 translate-x-0.5' : 'text-zinc-700 group-hover:text-zinc-500'}`} />
                                    </div>
                                    <div className="text-[9px] text-zinc-600 mt-0.5">{(m.waypoints || []).length} waypoints · {m.status}</div>
                                </button>
                            ))}
                            {missions.length === 0 && <p className="text-xs text-zinc-700 italic px-3 py-2">No missions found.</p>}
                        </div>
                    </div>

                    {/* Date selector */}
                    {selectedMission && (
                        <div className="p-4 border-b border-white/5">
                            <div className="text-[9px] font-bold uppercase tracking-widest text-zinc-600 flex items-center gap-1.5 mb-3">
                                <Calendar size={9} /> Inspection Date
                            </div>
                            <input type="date" value={selectedDate} onChange={e => setSelectedDate(e.target.value)}
                                className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none focus:border-zinc-700 transition-colors" />

                            {existingDates.length > 0 && (
                                <div className="mt-3">
                                    <div className="text-[8px] text-zinc-700 uppercase tracking-wider mb-1.5">Recorded dates</div>
                                    <div className="space-y-1">
                                        {existingDates.map(d => (
                                            <button key={d} onClick={() => setSelectedDate(d)}
                                                className={`w-full text-left px-2.5 py-1.5 rounded-lg text-[10px] transition-colors ${selectedDate === d ? 'bg-sky-600/15 text-sky-400 border border-sky-500/20' : 'text-zinc-500 hover:text-white hover:bg-white/[0.03]'}`}>
                                                {d}
                                            </button>
                                        ))}
                                    </div>
                                </div>
                            )}
                        </div>
                    )}
                </aside>

                {/* Main form area */}
                <main className="flex-1 overflow-auto">
                    {!selectedMission ? (
                        <div className="h-full flex flex-col items-center justify-center text-center px-8">
                            <Layers size={40} className="text-zinc-800 mb-4" />
                            <p className="text-zinc-600 text-sm">Select a mission from the sidebar to record results.</p>
                        </div>
                    ) : waypoints.length === 0 ? (
                        <div className="h-full flex items-center justify-center text-zinc-600 text-sm italic">This mission has no waypoints.</div>
                    ) : (
                        <div className="p-6 space-y-3 max-w-4xl mx-auto">
                            <div className="text-xs text-zinc-500 mb-4">
                                <span className="font-bold text-white">{selectedMission.name}</span> · {selectedDate}
                            </div>
                            {waypoints.map((wp, idx) => {
                                const r = results[wp.ID] || { image_url: '', success: 'no', analysis: '', confidence: 0 };
                                const isSaving = saving[wp.ID];
                                const isSaved = saved[wp.ID];
                                const isPass = r.success === 'yes';

                                return (
                                    <div key={wp.ID} className={`rounded-xl border overflow-hidden transition-all ${isPass ? 'border-emerald-500/15' : 'border-white/5'} bg-[#0d0d14]/50`}>
                                        {/* Waypoint header bar */}
                                        <div className={`flex items-center justify-between px-4 py-2.5 ${isPass ? 'bg-emerald-500/5 border-b border-emerald-500/10' : 'border-b border-white/5'}`}>
                                            <div className="flex items-center gap-3">
                                                <div className={`w-6 h-6 rounded-full flex items-center justify-center text-[9px] font-black border ${isPass ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30' : 'bg-zinc-800 text-zinc-500 border-zinc-700'}`}>{idx + 1}</div>
                                                <div>
                                                    <div className="text-xs font-bold text-white">{wp.name}</div>
                                                    <div className="flex items-center gap-2 mt-0.5">
                                                        <span className={`text-[8px] font-bold uppercase px-1.5 py-0.5 rounded-full ${purposeColors[wp.purpose] || purposeColors.none}`}>{wp.purpose || 'none'}</span>
                                                        <span className="text-[9px] text-zinc-700 font-mono">({wp.x?.toFixed(1)}, {wp.y?.toFixed(1)}, {wp.z?.toFixed(1)})</span>
                                                    </div>
                                                </div>
                                            </div>
                                            <div className="flex items-center gap-2">
                                                {isSaved && <span className="text-[9px] text-emerald-400 font-bold">✓ Saved</span>}
                                                <button onClick={() => saveResult(wp)} disabled={isSaving}
                                                    className="px-3 py-1.5 rounded-lg text-[9px] font-bold uppercase tracking-wider bg-zinc-800 hover:bg-zinc-700 disabled:opacity-40 text-zinc-300 flex items-center gap-1 transition-colors">
                                                    <Save size={9} /> {isSaving ? '...' : 'Save'}
                                                </button>
                                            </div>
                                        </div>

                                        {/* Form fields */}
                                        <div className="p-4 grid grid-cols-12 gap-3">
                                            {/* Pass/Fail */}
                                            <div className="col-span-12 md:col-span-3">
                                                <label className="text-[8px] font-bold uppercase tracking-wider text-zinc-600 mb-1.5 block">Result</label>
                                                <div className="flex gap-1.5">
                                                    <button onClick={() => updateResult(wp.ID, 'success', 'yes')}
                                                        className={`flex-1 py-2 rounded-lg text-[10px] font-bold flex items-center justify-center gap-1 transition-all border ${r.success === 'yes' ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30' : 'bg-zinc-900/50 text-zinc-600 border-zinc-800 hover:border-zinc-700'}`}>
                                                        <Check size={10} /> Pass
                                                    </button>
                                                    <button onClick={() => updateResult(wp.ID, 'success', 'no')}
                                                        className={`flex-1 py-2 rounded-lg text-[10px] font-bold flex items-center justify-center gap-1 transition-all border ${r.success === 'no' ? 'bg-red-500/20 text-red-400 border-red-500/30' : 'bg-zinc-900/50 text-zinc-600 border-zinc-800 hover:border-zinc-700'}`}>
                                                        <X size={10} /> Fail
                                                    </button>
                                                </div>
                                            </div>

                                            {/* Confidence */}
                                            <div className="col-span-12 md:col-span-3">
                                                <label className="text-[8px] font-bold uppercase tracking-wider text-zinc-600 mb-1.5 block">Confidence: <span className="text-white">{Math.round((r.confidence || 0) * 100)}%</span></label>
                                                <input type="range" min="0" max="1" step="0.01" value={r.confidence || 0}
                                                    onChange={e => updateResult(wp.ID, 'confidence', parseFloat(e.target.value))}
                                                    className="w-full h-1.5 rounded-full appearance-none bg-zinc-800 cursor-pointer mt-3 [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-3 [&::-webkit-slider-thumb]:h-3 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-sky-500" />
                                            </div>

                                            {/* Image URL */}
                                            <div className="col-span-12 md:col-span-6">
                                                <label className="text-[8px] font-bold uppercase tracking-wider text-zinc-600 mb-1.5 flex items-center gap-1"><Image size={8} /> Image URL</label>
                                                <div className="flex gap-2">
                                                    <input type="text" value={r.image_url} onChange={e => updateResult(wp.ID, 'image_url', e.target.value)} placeholder="https://..."
                                                        className="flex-1 bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-1.5 text-[10px] text-white focus:outline-none focus:border-zinc-700 placeholder-zinc-700 font-mono" />
                                                    {r.image_url && (
                                                        <img src={r.image_url} alt="" className="h-8 w-12 rounded-lg border border-zinc-800 object-cover flex-shrink-0"
                                                            onError={e => e.target.style.display = 'none'} />
                                                    )}
                                                </div>
                                            </div>

                                            {/* Analysis */}
                                            <div className="col-span-12">
                                                <label className="text-[8px] font-bold uppercase tracking-wider text-zinc-600 mb-1.5 block">Analysis Notes</label>
                                                <textarea value={r.analysis} onChange={e => updateResult(wp.ID, 'analysis', e.target.value)} placeholder="Describe inspection findings..."
                                                    rows={2} className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-[10px] text-white focus:outline-none focus:border-zinc-700 placeholder-zinc-700 resize-none" />
                                            </div>
                                        </div>
                                    </div>
                                );
                            })}
                        </div>
                    )}
                </main>
            </div>
        </div>
    );
};

export default MissionResultEditor;
