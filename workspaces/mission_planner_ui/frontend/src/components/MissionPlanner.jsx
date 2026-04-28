import React, { useState, useEffect, useCallback } from 'react';
import { Plus, Trash2, Play, Square, Map, ChevronDown, ChevronRight, Crosshair, Home, MapPin, ArrowUpDown } from 'lucide-react';

const PURPOSE_OPTIONS = [
    { value: 'none', label: 'None' },
    { value: 'fire_extinguisher', label: 'Fire Extinguisher' },
    { value: 'gauge_reading', label: 'Gauge Reading' },
    { value: 'staircase_start', label: 'Staircase Start' },
    { value: 'staircase_end', label: 'Staircase End' },
    { value: 'slope_start', label: 'Slope Start' },
    { value: 'slope_end', label: 'Slope End' },
    { value: 'human_analyse', label: 'Human Analyse' },
    { value: 'elevator_door', label: 'Elevator Door' },
    { value: 'elevator_inside', label: 'Elevator Inside' },
    { value: 'elevator_exit', label: 'Elevator Exit' },
];

const ELEVATOR_PURPOSES = ['elevator_door', 'elevator_inside', 'elevator_exit'];

const PURPOSE_COLORS = {
    none: 'zinc', fire_extinguisher: 'red', gauge_reading: 'blue',
    staircase_start: 'amber', staircase_end: 'amber',
    slope_start: 'purple', slope_end: 'purple', human_analyse: 'emerald',
    elevator_door: 'indigo', elevator_inside: 'indigo', elevator_exit: 'indigo',
};

const STATUS_CFG = {
    created: { dark: 'bg-zinc-600/30 text-zinc-300', light: 'bg-zinc-100 text-zinc-600', label: 'CREATED' },
    mapping: { dark: 'bg-cyan-500/20 text-cyan-400 animate-pulse', light: 'bg-cyan-50 text-cyan-600 animate-pulse', label: 'MAPPING' },
    ready: { dark: 'bg-emerald-500/20 text-emerald-400', light: 'bg-emerald-50 text-emerald-600', label: 'READY' },
    running: { dark: 'bg-blue-500/20 text-blue-400 animate-pulse', light: 'bg-blue-50 text-blue-600 animate-pulse', label: 'RUNNING' },
    terminated: { dark: 'bg-red-500/20 text-red-400', light: 'bg-red-50 text-red-600', label: 'TERMINATED' },
};

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const HOST = window.location.hostname;
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`http://${HOST}:8000${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const MissionPlanner = ({ isDark, selectedPoint, onClearSelectedPoint, savedLocations = [], onMissionWaypointsChange, onStatusChange }) => {
    const [missions, setMissions] = useState([]);
    const [newMissionName, setNewMissionName] = useState('');
    const [activeMissionId, setActiveMissionId] = useState(null);
    const [wpName, setWpName] = useState('');
    const [wpPurpose, setWpPurpose] = useState('none');
    const [wpX, setWpX] = useState('');
    const [wpY, setWpY] = useState('');
    const [wpZ, setWpZ] = useState('');
    const [homeX, setHomeX] = useState('0');
    const [homeY, setHomeY] = useState('0');
    const [homeZ, setHomeZ] = useState('0');
    const [loading, setLoading] = useState(false);
    const [showCreate, setShowCreate] = useState(false);
    const [showSavedPicker, setShowSavedPicker] = useState(false);
    const [showElevator, setShowElevator] = useState(false);
    const [elevDoorX, setElevDoorX] = useState('');
    const [elevDoorY, setElevDoorY] = useState('');
    const [elevDoorZ, setElevDoorZ] = useState('');
    const [elevInsideX, setElevInsideX] = useState('');
    const [elevInsideY, setElevInsideY] = useState('');
    const [elevFloorDelta, setElevFloorDelta] = useState('3.0');
    const [elevExitX, setElevExitX] = useState('');
    const [elevExitY, setElevExitY] = useState('');

    const thm = isDark ? 'dark' : 'light';

    const fetchMissions = useCallback(async () => {
        try {
            const data = await apiCall('/missions');
            setMissions(data || []);
        } catch (e) { console.error(e); }
    }, []);

    useEffect(() => {
        fetchMissions();
        const iv = setInterval(fetchMissions, 3000);
        return () => clearInterval(iv);
    }, [fetchMissions]);

    // Auto-fill coords from 3D click
    useEffect(() => {
        if (selectedPoint && activeMissionId) {
            setWpX(selectedPoint.x.toFixed(2));
            setWpY(selectedPoint.y.toFixed(2));
            setWpZ(selectedPoint.z.toFixed(2));
        }
    }, [selectedPoint, activeMissionId]);

    const activeMission = missions.find(m => m.ID === activeMissionId);

    // Notify parent of active mission waypoints + home for 3D spline
    useEffect(() => {
        if (onMissionWaypointsChange) {
            if (activeMission && activeMission.waypoints) {
                const home = { x: parseFloat(homeX) || 0, y: parseFloat(homeY) || 0, z: parseFloat(homeZ) || 0, name: 'HOME', purpose: 'home' };
                const pts = [home, ...activeMission.waypoints, home];
                onMissionWaypointsChange(pts);
            } else {
                onMissionWaypointsChange([]);
            }
        }
    }, [activeMission, homeX, homeY, homeZ, onMissionWaypointsChange]);

    // Notify parent of status changes for HUD
    useEffect(() => {
        if (onStatusChange && activeMission) {
            onStatusChange({ status: activeMission.status, name: activeMission.name });
        } else if (onStatusChange) {
            onStatusChange(null);
        }
    }, [activeMission?.status, activeMission?.name, onStatusChange]);

    const handleCreate = async () => {
        if (!newMissionName.trim()) return;
        setLoading(true);
        try {
            const c = await apiCall('/missions', 'POST', { name: newMissionName.trim() });
            setNewMissionName(''); setShowCreate(false);
            await fetchMissions();
            setActiveMissionId(c.ID);
        } catch (e) { console.error(e); }
        setLoading(false);
    };

    const handleDelete = async (id) => {
        setLoading(true);
        try {
            await apiCall(`/missions/${id}`, 'DELETE');
            if (activeMissionId === id) setActiveMissionId(null);
            await fetchMissions();
        } catch (e) { console.error(e); }
        setLoading(false);
    };

    const handleAddWp = async () => {
        if (!activeMissionId || !wpName.trim()) return;
        setLoading(true);
        try {
            await apiCall(`/missions/${activeMissionId}/waypoints`, 'POST', {
                name: wpName.trim(), x: parseFloat(wpX) || 0, y: parseFloat(wpY) || 0, z: parseFloat(wpZ) || 0, purpose: wpPurpose,
            });
            setWpName(''); setWpPurpose('none'); setWpX(''); setWpY(''); setWpZ('');
            if (onClearSelectedPoint) onClearSelectedPoint();
            await fetchMissions();
        } catch (e) { console.error(e); }
        setLoading(false);
    };

    const handleDeleteWp = async (wpId) => {
        if (!activeMissionId) return;
        setLoading(true);
        try { await apiCall(`/missions/${activeMissionId}/waypoints/${wpId}`, 'DELETE'); await fetchMissions(); } catch (e) { console.error(e); }
        setLoading(false);
    };

    const handleAction = async (action) => {
        if (!activeMissionId) return;
        setLoading(true);
        try { await apiCall(`/missions/${activeMissionId}/${action}`, 'POST'); await fetchMissions(); } catch (e) { console.error(e); }
        setLoading(false);
    };

    const addFromSaved = (loc) => {
        setWpName(loc.name);
        setWpX(loc.x.toString());
        setWpY(loc.y.toString());
        setWpZ(loc.z.toString());
        setShowSavedPicker(false);
    };

    // Compute current floor Z from last elevator_exit in the stack
    const getCurrentFloorZ = (wps) => {
        for (let i = wps.length - 1; i >= 0; i--) {
            if (wps[i].purpose === 'elevator_exit') return wps[i].z;
        }
        return parseFloat(homeZ) || 0;
    };

    const handleAddElevatorSequence = async () => {
        if (!activeMissionId) return;
        setLoading(true);
        try {
            const curZ = activeMission ? getCurrentFloorZ(activeMission.waypoints || []) : 0;
            const delta = parseFloat(elevFloorDelta) || 0;
            const newZ = curZ + delta;
            // 1. Door front
            await apiCall(`/missions/${activeMissionId}/waypoints`, 'POST', {
                name: 'Elevator Door', x: parseFloat(elevDoorX)||0, y: parseFloat(elevDoorY)||0, z: parseFloat(elevDoorZ)||curZ, purpose: 'elevator_door',
            });
            // 2. Inside
            await apiCall(`/missions/${activeMissionId}/waypoints`, 'POST', {
                name: 'Elevator Inside', x: parseFloat(elevInsideX)||0, y: parseFloat(elevInsideY)||0, z: curZ, purpose: 'elevator_inside',
            });
            // 3. Exit on new floor
            await apiCall(`/missions/${activeMissionId}/waypoints`, 'POST', {
                name: `Elevator Exit (${delta > 0 ? '+' : ''}${delta}m)`, x: parseFloat(elevExitX)||0, y: parseFloat(elevExitY)||0, z: newZ, purpose: 'elevator_exit',
            });
            setShowElevator(false);
            setElevDoorX(''); setElevDoorY(''); setElevDoorZ('');
            setElevInsideX(''); setElevInsideY('');
            setElevFloorDelta('3.0'); setElevExitX(''); setElevExitY('');
            await fetchMissions();
        } catch (e) { console.error(e); }
        setLoading(false);
    };

    const inputCls = `w-full text-xs px-2.5 py-1.5 rounded-lg border bg-transparent focus:outline-none transition-colors ${isDark ? 'border-zinc-700 focus:border-zinc-500 text-white placeholder-zinc-600' : 'border-zinc-300 focus:border-orange-400 text-zinc-800 placeholder-zinc-400'}`;
    const coordCls = `w-full text-[10px] font-mono px-2 py-1.5 rounded border bg-transparent focus:outline-none ${isDark ? 'border-zinc-700 focus:border-zinc-500 text-white' : 'border-zinc-300 focus:border-orange-400 text-zinc-800'}`;

    return (
        <div className="flex flex-col gap-3">
            {/* Create Mission */}
            {!showCreate ? (
                <button onClick={() => setShowCreate(true)}
                    className={`w-full py-2.5 rounded-lg text-xs font-bold uppercase tracking-wider flex items-center justify-center gap-2 transition-all border ${isDark ? 'bg-zinc-800/80 hover:bg-zinc-700 text-zinc-200 border-zinc-700/50' : 'bg-orange-500 hover:bg-orange-400 text-white border-orange-400'}`}>
                    <Plus size={14} /> New Mission
                </button>
            ) : (
                <div className={`p-3 rounded-lg border ${isDark ? 'bg-zinc-800/50 border-white/5' : 'bg-white border-zinc-200'}`}>
                    <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Mission Name</div>
                    <input value={newMissionName} onChange={e => setNewMissionName(e.target.value)} onKeyDown={e => e.key === 'Enter' && handleCreate()} placeholder="e.g. Floor_1_Inspection" className={inputCls} autoFocus />
                    <div className="flex gap-1.5 mt-2">
                        <button onClick={handleCreate} disabled={loading || !newMissionName.trim()} className={`flex-1 py-1.5 rounded-lg text-[10px] font-bold uppercase disabled:opacity-40 ${isDark ? 'bg-zinc-600 hover:bg-zinc-500 text-white' : 'bg-orange-500 hover:bg-orange-400 text-white'}`}>Create</button>
                        <button onClick={() => { setShowCreate(false); setNewMissionName(''); }} className={`px-3 py-1.5 rounded-lg text-[10px] font-bold uppercase ${isDark ? 'bg-zinc-800 text-zinc-400' : 'bg-zinc-100 text-zinc-600'}`}>Cancel</button>
                    </div>
                </div>
            )}

            {/* Mission List */}
            {missions.map((mission) => {
                const isActive = activeMissionId === mission.ID;
                const st = STATUS_CFG[mission.status] || STATUS_CFG.created;
                const wps = mission.waypoints || [];
                const canEdit = ['created', 'mapping', 'ready'].includes(mission.status);

                return (
                    <div key={mission.ID} className={`rounded-lg border overflow-hidden transition-all ${isActive ? (isDark ? 'border-zinc-500/40 bg-zinc-800/30' : 'border-orange-300 bg-orange-50/30') : (isDark ? 'border-white/5 bg-zinc-800/20' : 'border-zinc-200 bg-white')}`}>
                        {/* Header */}
                        <button onClick={() => setActiveMissionId(isActive ? null : mission.ID)}
                            className={`w-full flex items-center justify-between p-3 transition-colors text-left ${isDark ? 'hover:bg-white/5' : 'hover:bg-zinc-50'}`}>
                            <div className="flex items-center gap-2 min-w-0 flex-1">
                                {isActive ? <ChevronDown size={12} className="opacity-50" /> : <ChevronRight size={12} className="opacity-50" />}
                                <div className="min-w-0 flex-1">
                                    <div className={`text-xs font-bold truncate ${isDark ? 'text-zinc-200' : 'text-zinc-800'}`}>{mission.name}</div>
                                    <div className={`text-[10px] ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{wps.length} waypoint{wps.length !== 1 ? 's' : ''}</div>
                                </div>
                            </div>
                            <span className={`text-[9px] font-black uppercase px-2 py-0.5 rounded-full border ${st[thm]}`}>{st.label}</span>
                        </button>

                        {/* Expanded */}
                        {isActive && (
                            <div className={`px-3 pb-3 ${isDark ? 'bg-zinc-800/20' : 'bg-zinc-50/50'}`}>
                                {/* Action Buttons */}
                                <div className="flex gap-1.5 mb-3 pt-1">
                                    {mission.status === 'mapping' ? (
                                        <button onClick={() => handleAction('mapping/stop')} disabled={loading}
                                            className="flex-1 py-2 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 bg-cyan-500 hover:bg-cyan-400 text-white shadow-[0_0_12px_rgba(6,182,212,0.3)]">
                                            <Square size={10} /> Stop Mapping
                                        </button>
                                    ) : (
                                        <button onClick={() => handleAction('mapping/start')} disabled={loading || mission.status === 'running'}
                                            className={`flex-1 py-2 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 disabled:opacity-30 border ${isDark ? 'bg-zinc-700/50 text-cyan-400 border-cyan-500/30' : 'bg-cyan-50 text-cyan-600 border-cyan-200'}`}>
                                            <Map size={10} /> Start Mapping
                                        </button>
                                    )}
                                    {mission.status === 'running' ? (
                                        <button onClick={() => handleAction('terminate')} disabled={loading}
                                            className="flex-1 py-2 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 bg-red-500 hover:bg-red-400 text-white shadow-[0_0_12px_rgba(239,68,68,0.3)]">
                                            <Square size={10} /> Terminate
                                        </button>
                                    ) : (
                                        <button onClick={() => handleAction('start')} disabled={loading || mission.status === 'mapping' || wps.length === 0}
                                            className={`flex-1 py-2 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 disabled:opacity-30 border ${isDark ? 'bg-zinc-700/50 text-emerald-400 border-emerald-500/30' : 'bg-emerald-50 text-emerald-600 border-emerald-200'}`}>
                                            <Play size={10} /> Start Mission
                                        </button>
                                    )}
                                </div>

                                {/* Home Position */}
                                <div className={`p-2.5 rounded-lg border mb-3 ${isDark ? 'bg-zinc-900/50 border-white/5' : 'bg-white border-zinc-200'}`}>
                                    <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 flex items-center gap-1.5 ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>
                                        <Home size={10} /> Home Position (Start & End)
                                    </div>
                                    <div className="flex gap-1.5">
                                        {[['X', homeX, setHomeX], ['Y', homeY, setHomeY], ['Z', homeZ, setHomeZ]].map(([l, v, s]) => (
                                            <div key={l} className="flex-1">
                                                <label className={`text-[9px] font-bold uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{l}</label>
                                                <input type="number" step="0.1" value={v} onChange={e => s(e.target.value)} className={coordCls} />
                                            </div>
                                        ))}
                                    </div>
                                </div>

                                {/* Waypoint Stack */}
                                <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
                                    Mission Stack ({wps.length + 2} pts)
                                </div>

                                <div className={`rounded-lg border overflow-hidden mb-3 ${isDark ? 'border-white/5' : 'border-zinc-200'}`}>
                                    {/* Home start */}
                                    <div className={`flex items-center gap-2 p-2 ${isDark ? 'bg-amber-500/5' : 'bg-amber-50/50'}`}>
                                        <div className="w-5 h-5 rounded-full flex items-center justify-center shrink-0 bg-amber-500/20 text-amber-400"><Home size={9} /></div>
                                        <div className="flex-1 min-w-0">
                                            <div className={`text-[11px] font-bold ${isDark ? 'text-amber-300' : 'text-amber-700'}`}>HOME (Start)</div>
                                            <div className={`text-[9px] font-mono ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>({parseFloat(homeX || 0).toFixed(1)}, {parseFloat(homeY || 0).toFixed(1)}, {parseFloat(homeZ || 0).toFixed(1)})</div>
                                        </div>
                                    </div>

                                    {/* Mission waypoints */}
                                    {wps.map((wp, idx) => {
                                        const isElev = ELEVATOR_PURPOSES.includes(wp.purpose);
                                        const prevWp = idx > 0 ? wps[idx - 1] : null;
                                        const showFloorChange = wp.purpose === 'elevator_exit' && prevWp;
                                        return (
                                            <React.Fragment key={wp.ID}>
                                                {showFloorChange && (
                                                    <div className={`flex items-center gap-2 px-2 py-1 ${isDark ? 'bg-indigo-500/10 border-t border-indigo-500/20' : 'bg-indigo-50 border-t border-indigo-200'}`}>
                                                        <ArrowUpDown size={9} className={isDark ? 'text-indigo-400' : 'text-indigo-500'} />
                                                        <span className={`text-[8px] font-black uppercase tracking-wider ${isDark ? 'text-indigo-400' : 'text-indigo-600'}`}>
                                                            Floor Change → Z: {wp.z.toFixed(1)}m ({wp.z > (prevWp?.z || 0) ? '↑' : '↓'}{Math.abs(wp.z - (prevWp?.z || 0)).toFixed(1)}m)
                                                        </span>
                                                    </div>
                                                )}
                                                <div className={`flex items-center gap-2 p-2 group transition-colors ${isDark ? 'hover:bg-white/5 border-t border-white/5' : 'hover:bg-zinc-50 border-t border-zinc-100'} ${isElev ? (isDark ? 'bg-indigo-500/5' : 'bg-indigo-50/30') : ''}`}>
                                                    <div className={`w-5 h-5 rounded-full flex items-center justify-center text-[9px] font-black shrink-0 ${isElev ? 'bg-indigo-500/20 text-indigo-400' : (isDark ? 'bg-zinc-700 text-zinc-300' : 'bg-zinc-200 text-zinc-600')}`}>
                                                        {isElev ? <ArrowUpDown size={9} /> : idx + 1}
                                                    </div>
                                                    <div className="flex-1 min-w-0">
                                                        <div className={`text-[11px] font-bold truncate ${isElev ? (isDark ? 'text-indigo-300' : 'text-indigo-700') : (isDark ? 'text-zinc-200' : 'text-zinc-700')}`}>{wp.name}</div>
                                                        <div className={`text-[9px] font-mono ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>({wp.x.toFixed(1)}, {wp.y.toFixed(1)}, {wp.z.toFixed(1)})</div>
                                                    </div>
                                                    {wp.purpose && wp.purpose !== 'none' && (
                                                        <span className={`text-[8px] font-bold uppercase px-1.5 py-0.5 rounded-full border shrink-0 ${isElev ? (isDark ? 'bg-indigo-500/20 text-indigo-400 border-indigo-500/30' : 'bg-indigo-50 text-indigo-600 border-indigo-200') : (isDark ? 'bg-zinc-700/50 text-zinc-400 border-zinc-600/50' : 'bg-zinc-100 text-zinc-500 border-zinc-200')}`}>
                                                            {PURPOSE_OPTIONS.find(p => p.value === wp.purpose)?.label || wp.purpose}
                                                        </span>
                                                    )}
                                                    {canEdit && (
                                                        <button onClick={() => handleDeleteWp(wp.ID)} className="opacity-0 group-hover:opacity-100 p-1 text-red-500 hover:text-red-400"><Trash2 size={11} /></button>
                                                    )}
                                                </div>
                                            </React.Fragment>
                                        );
                                    })}

                                    {/* Home end */}
                                    <div className={`flex items-center gap-2 p-2 ${isDark ? 'bg-amber-500/5 border-t border-white/5' : 'bg-amber-50/50 border-t border-zinc-100'}`}>
                                        <div className="w-5 h-5 rounded-full flex items-center justify-center shrink-0 bg-amber-500/20 text-amber-400"><Home size={9} /></div>
                                        <div className="flex-1 min-w-0">
                                            <div className={`text-[11px] font-bold ${isDark ? 'text-amber-300' : 'text-amber-700'}`}>HOME (Return)</div>
                                        </div>
                                    </div>
                                </div>

                                {/* Add Waypoint Form */}
                                {canEdit && (
                                    <div className={`p-2.5 rounded-lg border ${isDark ? 'bg-zinc-900/50 border-white/5' : 'bg-white border-zinc-200'}`}>
                                        <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 flex items-center justify-between ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
                                            <span className="flex items-center gap-1.5"><Plus size={10} /> Add Waypoint</span>
                                            {savedLocations.length > 0 && (
                                                <button onClick={() => setShowSavedPicker(!showSavedPicker)} className={`flex items-center gap-1 px-2 py-0.5 rounded text-[9px] font-bold border transition-all ${showSavedPicker ? (isDark ? 'bg-zinc-700 border-zinc-600 text-white' : 'bg-orange-100 border-orange-300 text-orange-600') : (isDark ? 'bg-zinc-800 border-zinc-700 text-zinc-400 hover:text-zinc-300' : 'bg-zinc-50 border-zinc-200 text-zinc-500 hover:text-zinc-700')}`}>
                                                    <MapPin size={8} /> From Saved
                                                </button>
                                            )}
                                        </div>

                                        {/* Saved Location Picker */}
                                        {showSavedPicker && savedLocations.length > 0 && (
                                            <div className={`mb-2 rounded-lg border overflow-hidden max-h-32 overflow-y-auto ${isDark ? 'border-zinc-700 bg-zinc-800/50' : 'border-zinc-200 bg-zinc-50'}`}>
                                                {savedLocations.map((loc, i) => (
                                                    <button key={i} onClick={() => addFromSaved(loc)}
                                                        className={`w-full text-left px-2.5 py-1.5 text-[10px] flex items-center justify-between transition-colors ${isDark ? 'hover:bg-zinc-700 text-zinc-300' : 'hover:bg-zinc-100 text-zinc-700'} ${i > 0 ? (isDark ? 'border-t border-zinc-700/50' : 'border-t border-zinc-100') : ''}`}>
                                                        <span className="font-bold">{loc.name}</span>
                                                        <span className={`font-mono ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>({loc.x.toFixed(1)}, {loc.y.toFixed(1)})</span>
                                                    </button>
                                                ))}
                                            </div>
                                        )}

                                        <input value={wpName} onChange={e => setWpName(e.target.value)} placeholder="Waypoint name" className={`${inputCls} mb-2`} />
                                        <select value={wpPurpose} onChange={e => setWpPurpose(e.target.value)} className={`${inputCls} mb-2`}>
                                            {PURPOSE_OPTIONS.map(o => <option key={o.value} value={o.value} className={isDark ? 'bg-zinc-900' : 'bg-white'}>{o.label}</option>)}
                                        </select>

                                        <div className="flex gap-1.5 mb-2">
                                            {[['X', wpX, setWpX], ['Y', wpY, setWpY], ['Z', wpZ, setWpZ]].map(([l, v, s]) => (
                                                <div key={l} className="flex-1">
                                                    <label className={`text-[9px] font-bold uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{l}</label>
                                                    <input type="number" step="0.1" value={v} onChange={e => s(e.target.value)} placeholder="0.0" className={coordCls} />
                                                </div>
                                            ))}
                                        </div>

                                        {selectedPoint && (
                                            <div className={`text-[9px] mb-2 px-2 py-1 rounded flex items-center gap-1 ${isDark ? 'bg-emerald-500/10 text-emerald-400' : 'bg-emerald-50 text-emerald-600'}`}>
                                                <Crosshair size={9} /> Coordinates filled from 3D click
                                            </div>
                                        )}

                                        <button onClick={handleAddWp} disabled={loading || !wpName.trim()}
                                            className={`w-full py-1.5 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 disabled:opacity-30 ${isDark ? 'bg-zinc-700 hover:bg-zinc-600 text-white' : 'bg-orange-500 hover:bg-orange-400 text-white'}`}>
                                            <Plus size={10} /> Add to Stack
                                        </button>
                                    </div>
                                )}

                                {/* Add Elevator Sequence */}
                                {canEdit && (
                                    <div className={`mt-2 rounded-lg border ${isDark ? 'bg-indigo-500/5 border-indigo-500/20' : 'bg-indigo-50/50 border-indigo-200'}`}>
                                        <button onClick={() => setShowElevator(!showElevator)}
                                            className={`w-full p-2.5 flex items-center justify-between text-[10px] font-bold uppercase tracking-wider ${isDark ? 'text-indigo-400' : 'text-indigo-600'}`}>
                                            <span className="flex items-center gap-1.5"><ArrowUpDown size={10} /> Add Elevator Sequence</span>
                                            {showElevator ? <ChevronDown size={10} /> : <ChevronRight size={10} />}
                                        </button>

                                        {showElevator && (
                                            <div className="px-2.5 pb-2.5 space-y-2">
                                                {/* Door Front */}
                                                <div>
                                                    <div className={`text-[9px] font-bold uppercase mb-1 ${isDark ? 'text-indigo-300' : 'text-indigo-500'}`}>1. Door Front Position</div>
                                                    <div className="flex gap-1.5">
                                                        {[['X', elevDoorX, setElevDoorX], ['Y', elevDoorY, setElevDoorY], ['Z', elevDoorZ, setElevDoorZ]].map(([l,v,s]) => (
                                                            <div key={l} className="flex-1">
                                                                <label className={`text-[8px] font-bold ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{l}</label>
                                                                <input type="number" step="0.1" value={v} onChange={e => s(e.target.value)} placeholder="0.0" className={coordCls} />
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>
                                                {/* Inside */}
                                                <div>
                                                    <div className={`text-[9px] font-bold uppercase mb-1 ${isDark ? 'text-indigo-300' : 'text-indigo-500'}`}>2. Inside Elevator Position</div>
                                                    <div className="flex gap-1.5">
                                                        {[['X', elevInsideX, setElevInsideX], ['Y', elevInsideY, setElevInsideY]].map(([l,v,s]) => (
                                                            <div key={l} className="flex-1">
                                                                <label className={`text-[8px] font-bold ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{l}</label>
                                                                <input type="number" step="0.1" value={v} onChange={e => s(e.target.value)} placeholder="0.0" className={coordCls} />
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>
                                                {/* Floor Delta */}
                                                <div>
                                                    <div className={`text-[9px] font-bold uppercase mb-1 ${isDark ? 'text-indigo-300' : 'text-indigo-500'}`}>3. Floor Height Change (m)</div>
                                                    <input type="number" step="0.5" value={elevFloorDelta} onChange={e => setElevFloorDelta(e.target.value)}
                                                        className={`${coordCls} ${isDark ? 'text-indigo-300' : 'text-indigo-600'}`} placeholder="+3.0 or -3.0" />
                                                    <div className={`text-[8px] mt-0.5 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Positive = up, Negative = down</div>
                                                </div>
                                                {/* Exit Door */}
                                                <div>
                                                    <div className={`text-[9px] font-bold uppercase mb-1 ${isDark ? 'text-indigo-300' : 'text-indigo-500'}`}>4. Exit Door Position (new floor)</div>
                                                    <div className="flex gap-1.5">
                                                        {[['X', elevExitX, setElevExitX], ['Y', elevExitY, setElevExitY]].map(([l,v,s]) => (
                                                            <div key={l} className="flex-1">
                                                                <label className={`text-[8px] font-bold ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>{l}</label>
                                                                <input type="number" step="0.1" value={v} onChange={e => s(e.target.value)} placeholder="0.0" className={coordCls} />
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>
                                                <button onClick={handleAddElevatorSequence} disabled={loading}
                                                    className={`w-full py-1.5 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 disabled:opacity-30 ${isDark ? 'bg-indigo-600 hover:bg-indigo-500 text-white' : 'bg-indigo-500 hover:bg-indigo-400 text-white'}`}>
                                                    <ArrowUpDown size={10} /> Add Elevator to Stack
                                                </button>
                                            </div>
                                        )}
                                    </div>
                                )}

                                {/* Delete Mission */}
                                <button onClick={() => handleDelete(mission.ID)} disabled={loading || mission.status === 'running'}
                                    className={`w-full mt-2 py-1.5 rounded-lg text-[10px] font-bold uppercase flex items-center justify-center gap-1.5 disabled:opacity-30 border ${isDark ? 'bg-transparent text-red-400/70 border-red-500/20' : 'bg-transparent text-red-400 border-red-200'}`}>
                                    <Trash2 size={10} /> Delete Mission
                                </button>
                            </div>
                        )}
                    </div>
                );
            })}

            {missions.length === 0 && !showCreate && (
                <div className={`text-center py-6 text-xs italic ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
                    No missions yet. Create one to get started.
                </div>
            )}
        </div>
    );
};

export default MissionPlanner;
