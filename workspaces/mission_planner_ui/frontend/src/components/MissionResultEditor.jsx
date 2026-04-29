import React, { useState, useEffect, useCallback } from 'react';
import { ArrowLeft, Save, Check, X, Calendar, Clipboard, Image } from 'lucide-react';

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const HOST = window.location.hostname;
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`http://${HOST}:8000${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const MissionResultEditor = ({ onBack }) => {
    const [missions, setMissions] = useState([]);
    const [selectedMissionId, setSelectedMissionId] = useState(null);
    const [selectedDate, setSelectedDate] = useState(new Date().toISOString().split('T')[0]);
    const [waypoints, setWaypoints] = useState([]);
    const [results, setResults] = useState({});
    const [saving, setSaving] = useState({});
    const [saved, setSaved] = useState({});

    useEffect(() => {
        apiCall('/missions').then(data => setMissions(data || [])).catch(console.error);
    }, []);

    const selectedMission = missions.find(m => m.ID === selectedMissionId);

    useEffect(() => {
        if (!selectedMission) { setWaypoints([]); setResults({}); return; }
        const wps = selectedMission.waypoints || [];
        setWaypoints(wps);

        // Load existing results for this date
        apiCall(`/missions/${selectedMissionId}/results?date=${selectedDate}`).then(data => {
            const map = {};
            (data || []).forEach(r => {
                map[r.mission_waypoint_id] = {
                    id: r.ID,
                    image_url: r.image_url || '',
                    success: r.success || 'no',
                    analysis: r.analysis || '',
                    confidence: r.confidence || 0,
                };
            });
            setResults(map);
        }).catch(console.error);
    }, [selectedMissionId, selectedDate, selectedMission]);

    const updateResult = (wpId, field, value) => {
        setResults(prev => ({
            ...prev,
            [wpId]: { ...(prev[wpId] || { image_url: '', success: 'no', analysis: '', confidence: 0 }), [field]: value }
        }));
    };

    const saveResult = async (wp) => {
        const r = results[wp.ID] || {};
        setSaving(prev => ({ ...prev, [wp.ID]: true }));
        try {
            await apiCall(`/missions/${selectedMissionId}/results`, 'POST', {
                mission_waypoint_id: wp.ID,
                date: selectedDate,
                image_url: r.image_url || '',
                success: r.success || 'no',
                analysis: r.analysis || '',
                confidence: parseFloat(r.confidence) || 0,
            });
            setSaved(prev => ({ ...prev, [wp.ID]: true }));
            setTimeout(() => setSaved(prev => ({ ...prev, [wp.ID]: false })), 2000);
        } catch (e) { console.error(e); }
        setSaving(prev => ({ ...prev, [wp.ID]: false }));
    };

    const saveAll = async () => {
        for (const wp of waypoints) {
            await saveResult(wp);
        }
    };

    return (
        <div className="min-h-screen bg-[#0a0a0f] text-white font-sans">
            {/* Header */}
            <header className="border-b border-white/5 bg-[#0d0d14]/80 backdrop-blur-xl sticky top-0 z-50">
                <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                        <button onClick={onBack} className="p-2 rounded-lg hover:bg-white/5 text-zinc-400 hover:text-white transition-colors">
                            <ArrowLeft size={18} />
                        </button>
                        <div>
                            <h1 className="text-lg font-bold tracking-wide">Mission Result Editor</h1>
                            <p className="text-[10px] text-zinc-500 uppercase tracking-widest">Record inspection outcomes</p>
                        </div>
                    </div>
                    {waypoints.length > 0 && (
                        <button onClick={saveAll} className="px-4 py-2 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white text-xs font-bold uppercase tracking-wider flex items-center gap-2 transition-colors shadow-lg shadow-emerald-500/20">
                            <Save size={14} /> Save All Results
                        </button>
                    )}
                </div>
            </header>

            <div className="max-w-7xl mx-auto px-6 py-8">
                {/* Selectors */}
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-8">
                    {/* Mission Selector */}
                    <div className="p-4 rounded-xl border border-white/5 bg-white/[0.02]">
                        <label className="text-[10px] font-bold uppercase tracking-widest text-zinc-500 flex items-center gap-1.5 mb-2">
                            <Clipboard size={10} /> Select Mission
                        </label>
                        <select
                            value={selectedMissionId || ''}
                            onChange={e => setSelectedMissionId(e.target.value ? parseInt(e.target.value) : null)}
                            className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2.5 text-sm text-white focus:outline-none focus:border-zinc-600 transition-colors"
                        >
                            <option value="">— Choose a mission —</option>
                            {missions.map(m => (
                                <option key={m.ID} value={m.ID} className="bg-zinc-900">{m.name} ({(m.waypoints || []).length} waypoints)</option>
                            ))}
                        </select>
                    </div>

                    {/* Date Picker */}
                    <div className="p-4 rounded-xl border border-white/5 bg-white/[0.02]">
                        <label className="text-[10px] font-bold uppercase tracking-widest text-zinc-500 flex items-center gap-1.5 mb-2">
                            <Calendar size={10} /> Inspection Date
                        </label>
                        <input
                            type="date"
                            value={selectedDate}
                            onChange={e => setSelectedDate(e.target.value)}
                            className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2.5 text-sm text-white focus:outline-none focus:border-zinc-600 transition-colors"
                        />
                    </div>
                </div>

                {/* Waypoint Results Form */}
                {!selectedMissionId ? (
                    <div className="text-center py-20 text-zinc-600 text-sm italic">Select a mission to begin recording results.</div>
                ) : waypoints.length === 0 ? (
                    <div className="text-center py-20 text-zinc-600 text-sm italic">This mission has no waypoints yet.</div>
                ) : (
                    <div className="space-y-4">
                        {waypoints.map((wp, idx) => {
                            const r = results[wp.ID] || { image_url: '', success: 'no', analysis: '', confidence: 0 };
                            const isSaving = saving[wp.ID];
                            const isSaved = saved[wp.ID];

                            return (
                                <div key={wp.ID} className={`rounded-xl border overflow-hidden transition-all ${r.success === 'yes' ? 'border-emerald-500/20 bg-emerald-500/[0.03]' : 'border-white/5 bg-white/[0.02]'}`}>
                                    {/* Waypoint Header */}
                                    <div className="flex items-center justify-between px-5 py-3 border-b border-white/5">
                                        <div className="flex items-center gap-3">
                                            <div className={`w-7 h-7 rounded-full flex items-center justify-center text-[10px] font-black ${r.success === 'yes' ? 'bg-emerald-500/20 text-emerald-400' : 'bg-zinc-800 text-zinc-400'}`}>{idx + 1}</div>
                                            <div>
                                                <div className="text-sm font-bold">{wp.name}</div>
                                                <div className="text-[10px] text-zinc-500 font-mono">({wp.x.toFixed(1)}, {wp.y.toFixed(1)}, {wp.z.toFixed(1)}) · {wp.purpose || 'none'}</div>
                                            </div>
                                        </div>
                                        <div className="flex items-center gap-2">
                                            {isSaved && <span className="text-[10px] text-emerald-400 font-bold animate-pulse">✓ SAVED</span>}
                                            <button onClick={() => saveResult(wp)} disabled={isSaving}
                                                className={`px-3 py-1.5 rounded-lg text-[10px] font-bold uppercase flex items-center gap-1.5 transition-colors ${isSaving ? 'bg-zinc-800 text-zinc-600' : 'bg-zinc-800 hover:bg-zinc-700 text-zinc-300'}`}>
                                                <Save size={10} /> {isSaving ? 'Saving...' : 'Save'}
                                            </button>
                                        </div>
                                    </div>

                                    {/* Form Fields */}
                                    <div className="p-5 grid grid-cols-1 md:grid-cols-2 gap-4">
                                        {/* Success Toggle */}
                                        <div>
                                            <label className="text-[9px] font-bold uppercase tracking-wider text-zinc-500 mb-1.5 block">Result</label>
                                            <div className="flex gap-2">
                                                <button onClick={() => updateResult(wp.ID, 'success', 'yes')}
                                                    className={`flex-1 py-2.5 rounded-lg text-xs font-bold uppercase flex items-center justify-center gap-1.5 transition-all border ${r.success === 'yes' ? 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30 shadow-lg shadow-emerald-500/10' : 'bg-zinc-900/50 text-zinc-500 border-zinc-800 hover:border-zinc-700'}`}>
                                                    <Check size={12} /> Pass
                                                </button>
                                                <button onClick={() => updateResult(wp.ID, 'success', 'no')}
                                                    className={`flex-1 py-2.5 rounded-lg text-xs font-bold uppercase flex items-center justify-center gap-1.5 transition-all border ${r.success === 'no' ? 'bg-red-500/20 text-red-400 border-red-500/30 shadow-lg shadow-red-500/10' : 'bg-zinc-900/50 text-zinc-500 border-zinc-800 hover:border-zinc-700'}`}>
                                                    <X size={12} /> Fail
                                                </button>
                                            </div>
                                        </div>

                                        {/* Confidence */}
                                        <div>
                                            <label className="text-[9px] font-bold uppercase tracking-wider text-zinc-500 mb-1.5 block">Confidence: {Math.round((r.confidence || 0) * 100)}%</label>
                                            <input type="range" min="0" max="1" step="0.01" value={r.confidence || 0} onChange={e => updateResult(wp.ID, 'confidence', parseFloat(e.target.value))}
                                                className="w-full h-2 rounded-full appearance-none bg-zinc-800 [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-sky-500 [&::-webkit-slider-thumb]:shadow-lg cursor-pointer" />
                                        </div>

                                        {/* Image URL */}
                                        <div>
                                            <label className="text-[9px] font-bold uppercase tracking-wider text-zinc-500 mb-1.5 flex items-center gap-1"><Image size={9} /> Image URL</label>
                                            <input type="text" value={r.image_url} onChange={e => updateResult(wp.ID, 'image_url', e.target.value)} placeholder="https://..."
                                                className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none focus:border-zinc-600 placeholder-zinc-700 font-mono" />
                                        </div>

                                        {/* Preview */}
                                        <div>
                                            <label className="text-[9px] font-bold uppercase tracking-wider text-zinc-500 mb-1.5 block">Preview</label>
                                            {r.image_url ? (
                                                <img src={r.image_url} alt="preview" className="h-16 rounded-lg border border-zinc-800 object-cover" onError={e => e.target.style.display = 'none'} />
                                            ) : (
                                                <div className="h-16 rounded-lg border border-dashed border-zinc-800 flex items-center justify-center text-[10px] text-zinc-700">No image</div>
                                            )}
                                        </div>

                                        {/* Analysis */}
                                        <div className="md:col-span-2">
                                            <label className="text-[9px] font-bold uppercase tracking-wider text-zinc-500 mb-1.5 block">Analysis Notes</label>
                                            <textarea value={r.analysis} onChange={e => updateResult(wp.ID, 'analysis', e.target.value)} placeholder="Describe inspection findings..."
                                                rows={2} className="w-full bg-zinc-900/50 border border-zinc-800 rounded-lg px-3 py-2 text-xs text-white focus:outline-none focus:border-zinc-600 placeholder-zinc-700 resize-none" />
                                        </div>
                                    </div>
                                </div>
                            );
                        })}
                    </div>
                )}
            </div>
        </div>
    );
};

export default MissionResultEditor;
