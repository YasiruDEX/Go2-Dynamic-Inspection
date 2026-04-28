import React, { useState, useEffect, useCallback } from 'react';
import { Plus, Trash2, Play, Square, Map, ChevronDown, ChevronRight, Navigation, Crosshair } from 'lucide-react';

const PURPOSE_OPTIONS = [
    { value: 'none', label: 'None', color: 'zinc' },
    { value: 'fire_extinguisher', label: 'Fire Extinguisher', color: 'red' },
    { value: 'gauge_reading', label: 'Gauge Reading', color: 'blue' },
    { value: 'staircase_start', label: 'Staircase Start', color: 'amber' },
    { value: 'staircase_end', label: 'Staircase End', color: 'amber' },
    { value: 'slope_start', label: 'Slope Start', color: 'purple' },
    { value: 'slope_end', label: 'Slope End', color: 'purple' },
    { value: 'human_analyse', label: 'Human Analyse', color: 'emerald' },
];

const PURPOSE_BADGE_CLASSES = {
    none: { dark: 'bg-zinc-700/50 text-zinc-400 border-zinc-600/50', light: 'bg-zinc-100 text-zinc-500 border-zinc-200' },
    fire_extinguisher: { dark: 'bg-red-500/20 text-red-400 border-red-500/30', light: 'bg-red-50 text-red-600 border-red-200' },
    gauge_reading: { dark: 'bg-blue-500/20 text-blue-400 border-blue-500/30', light: 'bg-blue-50 text-blue-600 border-blue-200' },
    staircase_start: { dark: 'bg-amber-500/20 text-amber-400 border-amber-500/30', light: 'bg-amber-50 text-amber-600 border-amber-200' },
    staircase_end: { dark: 'bg-amber-500/20 text-amber-400 border-amber-500/30', light: 'bg-amber-50 text-amber-600 border-amber-200' },
    slope_start: { dark: 'bg-purple-500/20 text-purple-400 border-purple-500/30', light: 'bg-purple-50 text-purple-600 border-purple-200' },
    slope_end: { dark: 'bg-purple-500/20 text-purple-400 border-purple-500/30', light: 'bg-purple-50 text-purple-600 border-purple-200' },
    human_analyse: { dark: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30', light: 'bg-emerald-50 text-emerald-600 border-emerald-200' },
};

const STATUS_STYLES = {
    created: { dark: 'bg-zinc-600/30 text-zinc-300 border-zinc-500/30', light: 'bg-zinc-100 text-zinc-600 border-zinc-200', label: 'CREATED' },
    mapping: { dark: 'bg-cyan-500/20 text-cyan-400 border-cyan-500/30 animate-pulse', light: 'bg-cyan-50 text-cyan-600 border-cyan-200 animate-pulse', label: 'MAPPING' },
    ready: { dark: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30', light: 'bg-emerald-50 text-emerald-600 border-emerald-200', label: 'READY' },
    running: { dark: 'bg-blue-500/20 text-blue-400 border-blue-500/30 animate-pulse', light: 'bg-blue-50 text-blue-600 border-blue-200 animate-pulse', label: 'RUNNING' },
    terminated: { dark: 'bg-red-500/20 text-red-400 border-red-500/30', light: 'bg-red-50 text-red-600 border-red-200', label: 'TERMINATED' },
};

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const HOST = window.location.hostname;
    const opts = {
        method,
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`,
        },
    };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`http://${HOST}:8000${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const MissionPlanner = ({ isDark, selectedPoint, onClearSelectedPoint }) => {
    const [missions, setMissions] = useState([]);
    const [newMissionName, setNewMissionName] = useState('');
    const [activeMissionId, setActiveMissionId] = useState(null);
    const [wpName, setWpName] = useState('');
    const [wpPurpose, setWpPurpose] = useState('none');
    const [wpX, setWpX] = useState('');
    const [wpY, setWpY] = useState('');
    const [wpZ, setWpZ] = useState('');
    const [loading, setLoading] = useState(false);
    const [showCreateForm, setShowCreateForm] = useState(false);

    const fetchMissions = useCallback(async () => {
        try {
            const data = await apiCall('/missions');
            setMissions(data || []);
        } catch (e) {
            console.error('Failed to fetch missions:', e);
        }
    }, []);

    useEffect(() => {
        fetchMissions();
        const interval = setInterval(fetchMissions, 3000);
        return () => clearInterval(interval);
    }, [fetchMissions]);

    // Auto-fill coordinates from 3D click
    useEffect(() => {
        if (selectedPoint && activeMissionId) {
            setWpX(selectedPoint.x.toFixed(2));
            setWpY(selectedPoint.y.toFixed(2));
            setWpZ(selectedPoint.z.toFixed(2));
        }
    }, [selectedPoint, activeMissionId]);

    const activeMission = missions.find(m => m.ID === activeMissionId);

    const handleCreateMission = async () => {
        if (!newMissionName.trim()) return;
        setLoading(true);
        try {
            const created = await apiCall('/missions', 'POST', { name: newMissionName.trim() });
            setNewMissionName('');
            setShowCreateForm(false);
            await fetchMissions();
            setActiveMissionId(created.ID);
        } catch (e) {
            console.error('Failed to create mission:', e);
        }
        setLoading(false);
    };

    const handleDeleteMission = async (id) => {
        setLoading(true);
        try {
            await apiCall(`/missions/${id}`, 'DELETE');
            if (activeMissionId === id) setActiveMissionId(null);
            await fetchMissions();
        } catch (e) {
            console.error('Failed to delete mission:', e);
        }
        setLoading(false);
    };

    const handleAddWaypoint = async () => {
        if (!activeMissionId || !wpName.trim()) return;
        setLoading(true);
        try {
            await apiCall(`/missions/${activeMissionId}/waypoints`, 'POST', {
                name: wpName.trim(),
                x: parseFloat(wpX) || 0,
                y: parseFloat(wpY) || 0,
                z: parseFloat(wpZ) || 0,
                purpose: wpPurpose,
            });
            setWpName('');
            setWpPurpose('none');
            setWpX('');
            setWpY('');
            setWpZ('');
            if (onClearSelectedPoint) onClearSelectedPoint();
            await fetchMissions();
        } catch (e) {
            console.error('Failed to add waypoint:', e);
        }
        setLoading(false);
    };

    const handleDeleteWaypoint = async (wpId) => {
        if (!activeMissionId) return;
        setLoading(true);
        try {
            await apiCall(`/missions/${activeMissionId}/waypoints/${wpId}`, 'DELETE');
            await fetchMissions();
        } catch (e) {
            console.error('Failed to delete waypoint:', e);
        }
        setLoading(false);
    };

    const handleMissionAction = async (action) => {
        if (!activeMissionId) return;
        setLoading(true);
        try {
            await apiCall(`/missions/${activeMissionId}/${action}`, 'POST');
            await fetchMissions();
        } catch (e) {
            console.error(`Failed to ${action} mission:`, e);
        }
        setLoading(false);
    };

    const themeMode = isDark ? 'dark' : 'light';

    return (
        <div className="flex flex-col gap-3">
            {/* Create Mission Button / Form */}
            {!showCreateForm ? (
                <button
                    onClick={() => setShowCreateForm(true)}
                    className={`w-full py-2.5 rounded-lg text-xs font-bold uppercase tracking-wider flex items-center justify-center gap-2 transition-all border ${isDark
                            ? 'bg-zinc-800/80 hover:bg-zinc-700 text-zinc-200 border-zinc-700/50 hover:border-zinc-600'
                            : 'bg-orange-500 hover:bg-orange-400 text-white border-orange-400'
                        }`}
                >
                    <Plus size={14} />
                    New Mission
                </button>
            ) : (
                <div className={`p-3 rounded-lg border ${isDark ? 'bg-zinc-800/50 border-white/5' : 'bg-white border-zinc-200'}`}>
                    <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
                        Mission Name
                    </div>
                    <div className="flex gap-1.5">
                        <input
                            value={newMissionName}
                            onChange={e => setNewMissionName(e.target.value)}
                            onKeyDown={e => e.key === 'Enter' && handleCreateMission()}
                            placeholder="e.g. Floor_1_Inspection"
                            className={`flex-1 text-xs px-2.5 py-2 rounded-lg border bg-transparent focus:outline-none transition-colors ${isDark
                                    ? 'border-zinc-600 focus:border-zinc-400 text-white placeholder-zinc-600'
                                    : 'border-zinc-300 focus:border-orange-500 text-zinc-800 placeholder-zinc-400'
                                }`}
                            autoFocus
                        />
                    </div>
                    <div className="flex gap-1.5 mt-2">
                        <button
                            onClick={handleCreateMission}
                            disabled={loading || !newMissionName.trim()}
                            className={`flex-1 py-1.5 rounded-lg text-[10px] font-bold uppercase tracking-wider transition-all disabled:opacity-40 ${isDark
                                    ? 'bg-zinc-600 hover:bg-zinc-500 text-white'
                                    : 'bg-orange-500 hover:bg-orange-400 text-white'
                                }`}
                        >
                            Create
                        </button>
                        <button
                            onClick={() => { setShowCreateForm(false); setNewMissionName(''); }}
                            className={`px-3 py-1.5 rounded-lg text-[10px] font-bold uppercase ${isDark
                                    ? 'bg-zinc-800 hover:bg-zinc-700 text-zinc-400'
                                    : 'bg-zinc-100 hover:bg-zinc-200 text-zinc-600'
                                }`}
                        >
                            Cancel
                        </button>
                    </div>
                </div>
            )}

            {/* Mission List */}
            {missions.length > 0 && (
                <div className={`rounded-lg border overflow-hidden ${isDark ? 'bg-zinc-800/30 border-white/5' : 'bg-white border-zinc-200'}`}>
                    {missions.map((mission) => {
                        const isActive = activeMissionId === mission.ID;
                        const status = STATUS_STYLES[mission.status] || STATUS_STYLES.created;
                        const wpCount = mission.waypoints?.length || 0;

                        return (
                            <div key={mission.ID} className={`border-b last:border-b-0 ${isDark ? 'border-white/5' : 'border-zinc-100'}`}>
                                {/* Mission Header */}
                                <button
                                    onClick={() => setActiveMissionId(isActive ? null : mission.ID)}
                                    className={`w-full flex items-center justify-between p-3 transition-colors text-left ${isActive
                                            ? isDark ? 'bg-zinc-700/30' : 'bg-orange-50'
                                            : isDark ? 'hover:bg-white/5' : 'hover:bg-zinc-50'
                                        }`}
                                >
                                    <div className="flex items-center gap-2.5 min-w-0 flex-1">
                                        {isActive ? <ChevronDown size={12} className="shrink-0 opacity-50" /> : <ChevronRight size={12} className="shrink-0 opacity-50" />}
                                        <div className="min-w-0 flex-1">
                                            <div className={`text-xs font-bold truncate ${isDark ? 'text-zinc-200' : 'text-zinc-800'}`}>
                                                {mission.name}
                                            </div>
                                            <div className={`text-[10px] ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
                                                {wpCount} waypoint{wpCount !== 1 ? 's' : ''}
                                            </div>
                                        </div>
                                    </div>
                                    <div className="flex items-center gap-2 shrink-0">
                                        <span className={`text-[9px] font-black uppercase tracking-wider px-2 py-0.5 rounded-full border ${status[themeMode]}`}>
                                            {status.label}
                                        </span>
                                    </div>
                                </button>

                                {/* Expanded Mission Panel */}
                                {isActive && (
                                    <div className={`px-3 pb-3 ${isDark ? 'bg-zinc-800/20' : 'bg-zinc-50/50'}`}>
                                        {/* Action Buttons Row */}
                                        <div className="flex gap-1.5 mb-3 pt-1">
                                            {/* Mapping Toggle */}
                                            {mission.status === 'mapping' ? (
                                                <button
                                                    onClick={() => handleMissionAction('mapping/stop')}
                                                    disabled={loading}
                                                    className="flex-1 py-2 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 bg-cyan-500 hover:bg-cyan-400 text-white transition-all disabled:opacity-50 shadow-[0_0_12px_rgba(6,182,212,0.3)]"
                                                >
                                                    <Square size={10} />
                                                    Stop Mapping
                                                </button>
                                            ) : (
                                                <button
                                                    onClick={() => handleMissionAction('mapping/start')}
                                                    disabled={loading || mission.status === 'running'}
                                                    className={`flex-1 py-2 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 transition-all disabled:opacity-30 border ${isDark
                                                            ? 'bg-zinc-700/50 hover:bg-zinc-600/50 text-cyan-400 border-cyan-500/30'
                                                            : 'bg-cyan-50 hover:bg-cyan-100 text-cyan-600 border-cyan-200'
                                                        }`}
                                                >
                                                    <Map size={10} />
                                                    Start Mapping
                                                </button>
                                            )}

                                            {/* Mission Start/Terminate Toggle */}
                                            {mission.status === 'running' ? (
                                                <button
                                                    onClick={() => handleMissionAction('terminate')}
                                                    disabled={loading}
                                                    className="flex-1 py-2 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 bg-red-500 hover:bg-red-400 text-white transition-all disabled:opacity-50 shadow-[0_0_12px_rgba(239,68,68,0.3)]"
                                                >
                                                    <Square size={10} />
                                                    Terminate
                                                </button>
                                            ) : (
                                                <button
                                                    onClick={() => handleMissionAction('start')}
                                                    disabled={loading || mission.status === 'mapping' || wpCount === 0}
                                                    className={`flex-1 py-2 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 transition-all disabled:opacity-30 border ${isDark
                                                            ? 'bg-zinc-700/50 hover:bg-zinc-600/50 text-emerald-400 border-emerald-500/30'
                                                            : 'bg-emerald-50 hover:bg-emerald-100 text-emerald-600 border-emerald-200'
                                                        }`}
                                                >
                                                    <Play size={10} />
                                                    Start Mission
                                                </button>
                                            )}
                                        </div>

                                        {/* Waypoint Stack */}
                                        <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
                                            Waypoint Stack
                                        </div>

                                        {wpCount === 0 ? (
                                            <div className={`text-center py-4 text-[10px] italic rounded-lg border-dashed border ${isDark ? 'text-zinc-600 border-zinc-700' : 'text-zinc-400 border-zinc-300'}`}>
                                                No waypoints added yet
                                            </div>
                                        ) : (
                                            <div className={`rounded-lg border overflow-hidden mb-3 ${isDark ? 'border-white/5' : 'border-zinc-200'}`}>
                                                {mission.waypoints.map((wp, idx) => (
                                                    <div
                                                        key={wp.ID}
                                                        className={`flex items-center gap-2 p-2 group transition-colors ${isDark ? 'hover:bg-white/5' : 'hover:bg-zinc-50'} ${idx > 0 ? (isDark ? 'border-t border-white/5' : 'border-t border-zinc-100') : ''}`}
                                                    >
                                                        {/* Order Number */}
                                                        <div className={`w-5 h-5 rounded-full flex items-center justify-center text-[9px] font-black shrink-0 ${isDark ? 'bg-zinc-700 text-zinc-300' : 'bg-zinc-200 text-zinc-600'}`}>
                                                            {idx + 1}
                                                        </div>

                                                        {/* Waypoint Info */}
                                                        <div className="flex-1 min-w-0">
                                                            <div className={`text-[11px] font-bold truncate ${isDark ? 'text-zinc-200' : 'text-zinc-700'}`}>
                                                                {wp.name}
                                                            </div>
                                                            <div className={`text-[9px] font-mono ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
                                                                ({wp.x.toFixed(1)}, {wp.y.toFixed(1)}, {wp.z.toFixed(1)})
                                                            </div>
                                                        </div>

                                                        {/* Purpose Badge */}
                                                        {wp.purpose && wp.purpose !== 'none' && (
                                                            <span className={`text-[8px] font-bold uppercase tracking-wider px-1.5 py-0.5 rounded-full border shrink-0 ${(PURPOSE_BADGE_CLASSES[wp.purpose] || PURPOSE_BADGE_CLASSES.none)[themeMode]}`}>
                                                                {PURPOSE_OPTIONS.find(p => p.value === wp.purpose)?.label || wp.purpose}
                                                            </span>
                                                        )}

                                                        {/* Delete */}
                                                        <button
                                                            onClick={() => handleDeleteWaypoint(wp.ID)}
                                                            className="opacity-0 group-hover:opacity-100 transition-opacity p-1 text-red-500 hover:text-red-400"
                                                        >
                                                            <Trash2 size={11} />
                                                        </button>
                                                    </div>
                                                ))}
                                            </div>
                                        )}

                                        {/* Add Waypoint Form */}
                                        {(mission.status === 'created' || mission.status === 'mapping' || mission.status === 'ready') && (
                                            <div className={`p-2.5 rounded-lg border ${isDark ? 'bg-zinc-900/50 border-white/5' : 'bg-white border-zinc-200'}`}>
                                                <div className={`text-[10px] font-bold uppercase tracking-wider mb-2 flex items-center gap-1.5 ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
                                                    <Plus size={10} />
                                                    Add Waypoint
                                                </div>

                                                <input
                                                    value={wpName}
                                                    onChange={e => setWpName(e.target.value)}
                                                    placeholder="Waypoint name"
                                                    className={`w-full text-xs px-2.5 py-1.5 rounded-lg border bg-transparent focus:outline-none mb-2 transition-colors ${isDark
                                                            ? 'border-zinc-700 focus:border-zinc-500 text-white placeholder-zinc-600'
                                                            : 'border-zinc-300 focus:border-orange-400 text-zinc-800 placeholder-zinc-400'
                                                        }`}
                                                />

                                                <select
                                                    value={wpPurpose}
                                                    onChange={e => setWpPurpose(e.target.value)}
                                                    className={`w-full text-xs px-2.5 py-1.5 rounded-lg border bg-transparent focus:outline-none mb-2 transition-colors ${isDark
                                                            ? 'border-zinc-700 focus:border-zinc-500 text-white'
                                                            : 'border-zinc-300 focus:border-orange-400 text-zinc-800'
                                                        }`}
                                                >
                                                    {PURPOSE_OPTIONS.map(opt => (
                                                        <option key={opt.value} value={opt.value} className={isDark ? 'bg-zinc-900' : 'bg-white'}>
                                                            {opt.label}
                                                        </option>
                                                    ))}
                                                </select>

                                                {/* Coordinates */}
                                                <div className="flex gap-1.5 mb-2">
                                                    <div className="flex-1">
                                                        <label className={`text-[9px] font-bold uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>X</label>
                                                        <input
                                                            type="number"
                                                            step="0.1"
                                                            value={wpX}
                                                            onChange={e => setWpX(e.target.value)}
                                                            placeholder="0.0"
                                                            className={`w-full text-[10px] font-mono px-2 py-1.5 rounded border bg-transparent focus:outline-none ${isDark
                                                                    ? 'border-zinc-700 focus:border-zinc-500 text-white'
                                                                    : 'border-zinc-300 focus:border-orange-400 text-zinc-800'
                                                                }`}
                                                        />
                                                    </div>
                                                    <div className="flex-1">
                                                        <label className={`text-[9px] font-bold uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>Y</label>
                                                        <input
                                                            type="number"
                                                            step="0.1"
                                                            value={wpY}
                                                            onChange={e => setWpY(e.target.value)}
                                                            placeholder="0.0"
                                                            className={`w-full text-[10px] font-mono px-2 py-1.5 rounded border bg-transparent focus:outline-none ${isDark
                                                                    ? 'border-zinc-700 focus:border-zinc-500 text-white'
                                                                    : 'border-zinc-300 focus:border-orange-400 text-zinc-800'
                                                                }`}
                                                        />
                                                    </div>
                                                    <div className="flex-1">
                                                        <label className={`text-[9px] font-bold uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>Z</label>
                                                        <input
                                                            type="number"
                                                            step="0.1"
                                                            value={wpZ}
                                                            onChange={e => setWpZ(e.target.value)}
                                                            placeholder="0.0"
                                                            className={`w-full text-[10px] font-mono px-2 py-1.5 rounded border bg-transparent focus:outline-none ${isDark
                                                                    ? 'border-zinc-700 focus:border-zinc-500 text-white'
                                                                    : 'border-zinc-300 focus:border-orange-400 text-zinc-800'
                                                                }`}
                                                        />
                                                    </div>
                                                </div>

                                                {selectedPoint && (
                                                    <div className={`text-[9px] mb-2 px-2 py-1 rounded flex items-center gap-1 ${isDark ? 'bg-emerald-500/10 text-emerald-400' : 'bg-emerald-50 text-emerald-600'}`}>
                                                        <Crosshair size={9} />
                                                        Coordinates filled from 3D click
                                                    </div>
                                                )}

                                                <button
                                                    onClick={handleAddWaypoint}
                                                    disabled={loading || !wpName.trim()}
                                                    className={`w-full py-1.5 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 transition-all disabled:opacity-30 ${isDark
                                                            ? 'bg-zinc-700 hover:bg-zinc-600 text-white'
                                                            : 'bg-orange-500 hover:bg-orange-400 text-white'
                                                        }`}
                                                >
                                                    <Plus size={10} />
                                                    Add to Stack
                                                </button>
                                            </div>
                                        )}

                                        {/* Delete Mission */}
                                        <button
                                            onClick={() => handleDeleteMission(mission.ID)}
                                            disabled={loading || mission.status === 'running'}
                                            className={`w-full mt-2 py-1.5 rounded-lg text-[10px] font-bold uppercase tracking-wider flex items-center justify-center gap-1.5 transition-all disabled:opacity-30 border ${isDark
                                                    ? 'bg-transparent hover:bg-red-500/10 text-red-400/70 hover:text-red-400 border-red-500/20'
                                                    : 'bg-transparent hover:bg-red-50 text-red-400 hover:text-red-500 border-red-200'
                                                }`}
                                        >
                                            <Trash2 size={10} />
                                            Delete Mission
                                        </button>
                                    </div>
                                )}
                            </div>
                        );
                    })}
                </div>
            )}

            {missions.length === 0 && !showCreateForm && (
                <div className={`text-center py-6 text-xs italic ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
                    No missions yet. Create one to get started.
                </div>
            )}
        </div>
    );
};

export default MissionPlanner;
