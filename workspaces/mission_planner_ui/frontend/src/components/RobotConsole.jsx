import React, { useState, useEffect, useRef } from 'react';
import { Terminal, Activity, Map, Navigation, ShieldAlert, Cpu, CheckCircle2, AlertTriangle, AlertCircle, X, Maximize2, Minimize2 } from 'lucide-react';
import { API_BASE } from '../lib/config';

const WS_BASE = API_BASE.replace(/^http/, 'ws');

const RobotConsole = ({ isDark = true, onClose }) => {
    const [events, setEvents] = useState([]);
    const [isConnected, setIsConnected] = useState(false);
    const [isExpanded, setIsExpanded] = useState(false);
    const wsRef = useRef(null);
    const logsEndRef = useRef(null);

    useEffect(() => {
        const connectWs = () => {
            const ws = new WebSocket(`${WS_BASE}/ws/robot_status`);
            
            ws.onopen = () => setIsConnected(true);
            ws.onclose = () => {
                setIsConnected(false);
                setTimeout(connectWs, 3000);
            };
            ws.onerror = (e) => console.error("RobotConsole WS error", e);
            
            ws.onmessage = (msg) => {
                try {
                    const data = JSON.parse(msg.data);
                    // data: { topic: string, payload: object, timestamp: string }
                    setEvents(prev => [...prev, data].slice(-50)); // Keep last 50 events
                } catch (e) {
                    console.error("Failed to parse status message", e);
                }
            };
            
            wsRef.current = ws;
        };
        
        connectWs();
        
        return () => {
            if (wsRef.current) wsRef.current.close();
        };
    }, []);

    useEffect(() => {
        logsEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [events]);

    const getTopicIcon = (topic) => {
        if (topic.includes('mission')) return <Navigation size={12} className="text-blue-400" />;
        if (topic.includes('waypoint')) return <Map size={12} className="text-emerald-400" />;
        if (topic.includes('mapping')) return <Activity size={12} className="text-violet-400" />;
        if (topic.includes('system')) return <Cpu size={12} className="text-amber-400" />;
        return <Terminal size={12} className="text-zinc-400" />;
    };

    const getStatusColor = (status) => {
        switch (status?.toLowerCase()) {
            case 'active': case 'running': case 'navigating': return 'text-blue-400 border-blue-400/30 bg-blue-400/10';
            case 'completed': case 'reached': return 'text-emerald-400 border-emerald-400/30 bg-emerald-400/10';
            case 'aborted': return 'text-orange-400 border-orange-400/30 bg-orange-400/10';
            case 'failed': case 'estop': return 'text-red-400 border-red-400/30 bg-red-400/10';
            case 'recording': return 'text-cyan-400 border-cyan-400/30 bg-cyan-400/10';
            case 'generating': return 'text-violet-400 border-violet-400/30 bg-violet-400/10';
            default: return 'text-zinc-400 border-zinc-700 bg-zinc-800';
        }
    };

    return (
        <div className={`flex flex-col border-l border-white/10 bg-[#0A0A0F] text-white transition-all duration-300 ease-in-out ${isExpanded ? 'w-[600px]' : 'w-[350px]'} h-full shadow-2xl`}>
            {/* Header */}
            <div className="flex items-center justify-between px-4 py-3 border-b border-white/10 bg-black/40">
                <div className="flex items-center gap-2">
                    <div className="w-6 h-6 rounded bg-sky-500/20 flex items-center justify-center border border-sky-500/30">
                        <Terminal size={12} className="text-sky-400" />
                    </div>
                    <div>
                        <div className="text-xs font-bold tracking-wide flex items-center gap-2">
                            ROBOT CONSOLE
                            <span className="flex h-2 w-2 relative">
                                <span className={`animate-ping absolute inline-flex h-full w-full rounded-full opacity-75 ${isConnected ? 'bg-emerald-400' : 'bg-red-400'}`}></span>
                                <span className={`relative inline-flex rounded-full h-2 w-2 ${isConnected ? 'bg-emerald-500' : 'bg-red-500'}`}></span>
                            </span>
                        </div>
                        <div className="text-[9px] text-zinc-500 font-mono tracking-wider">{isConnected ? 'WS CONNECTED' : 'WS DISCONNECTED'}</div>
                    </div>
                </div>
                <div className="flex items-center gap-1">
                    <button onClick={() => setIsExpanded(!isExpanded)} className="p-1.5 hover:bg-white/10 rounded text-zinc-400 hover:text-white transition-colors">
                        {isExpanded ? <Minimize2 size={12} /> : <Maximize2 size={12} />}
                    </button>
                    {onClose && (
                        <button onClick={onClose} className="p-1.5 hover:bg-red-500/20 rounded text-zinc-400 hover:text-red-400 transition-colors">
                            <X size={12} />
                        </button>
                    )}
                </div>
            </div>

            {/* Event Log */}
            <div className="flex-1 overflow-y-auto p-3 space-y-2 font-mono text-[10px] custom-scrollbar bg-black/20">
                {events.length === 0 ? (
                    <div className="flex flex-col items-center justify-center h-full text-zinc-600 space-y-2">
                        <Activity size={24} className="opacity-20" />
                        <span>Waiting for telemetry...</span>
                    </div>
                ) : (
                    events.map((evt, idx) => {
                        const { topic, payload, timestamp } = evt;
                        const shortTopic = topic.replace('robot/robot_01/', '');
                        const stColor = getStatusColor(payload.status);
                        
                        return (
                            <div key={idx} className="rounded border border-white/5 bg-[#111118] overflow-hidden group hover:border-white/10 transition-colors">
                                {/* Event Header */}
                                <div className="flex items-center justify-between px-2 py-1.5 bg-black/40 border-b border-white/5">
                                    <div className="flex items-center gap-1.5 text-zinc-400">
                                        {getTopicIcon(topic)}
                                        <span className="text-[9px] font-bold tracking-wider">{shortTopic}</span>
                                    </div>
                                    <div className="text-[8px] text-zinc-600">
                                        {new Date(timestamp).toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })}
                                    </div>
                                </div>
                                
                                {/* Event Body */}
                                <div className="p-2 flex flex-col gap-1.5">
                                    <div className="flex flex-wrap gap-2 items-center">
                                        {payload.status && (
                                            <span className={`px-1.5 py-0.5 rounded text-[9px] font-bold uppercase border ${stColor}`}>
                                                {payload.status}
                                            </span>
                                        )}
                                        {payload.mission_id && (
                                            <span className="text-zinc-300">
                                                ID: <span className="text-sky-400">{payload.mission_id}</span>
                                            </span>
                                        )}
                                        {payload.waypoint_id && (
                                            <span className="text-zinc-300">
                                                WP: <span className="text-emerald-400">{payload.waypoint_id}</span>
                                            </span>
                                        )}
                                        {payload.session_id && (
                                            <span className="text-zinc-300">
                                                Session: <span className="text-violet-400 text-[8px]">{payload.session_id.substring(0,8)}...</span>
                                            </span>
                                        )}
                                    </div>
                                    
                                    {/* Detailed payload dump for expanded view or complex objects */}
                                    {(payload.label || payload.inspection || payload.waypoint_index !== undefined) && (
                                        <div className="text-[9px] text-zinc-500 mt-1 pl-2 border-l-2 border-white/10 space-y-0.5">
                                            {payload.label && <div>Label: {payload.label}</div>}
                                            {payload.waypoint_index !== undefined && payload.total_waypoints !== undefined && 
                                                <div>Progress: {payload.waypoint_index} / {payload.total_waypoints}</div>}
                                            {payload.inspection && <div>Inspection: {payload.inspection}</div>}
                                        </div>
                                    )}
                                </div>
                            </div>
                        );
                    })
                )}
                <div ref={logsEndRef} />
            </div>
            
            {/* Legend / Status Bar */}
            <div className="px-4 py-2 bg-[#0A0A0F] border-t border-white/10 flex flex-wrap gap-2 text-[8px] font-bold uppercase tracking-widest text-zinc-600">
                <span className="flex items-center gap-1"><Navigation size={8}/> Mission</span>
                <span className="flex items-center gap-1"><Map size={8}/> Waypoint</span>
                <span className="flex items-center gap-1"><Activity size={8}/> Mapping</span>
            </div>
        </div>
    );
};

export default RobotConsole;
