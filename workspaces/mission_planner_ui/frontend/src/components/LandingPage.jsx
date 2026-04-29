import React, { useState, useEffect, useRef } from 'react';
import { Activity, Map, Cpu, Zap, Shield, ChevronRight, Radio, Database, Layers, Navigation, GitBranch, BarChart3, Settings, LogOut, User, Menu, X } from 'lucide-react';

const FEATURES = [
    {
        icon: <Navigation size={20} className="text-sky-400" />,
        title: 'Mission Planner',
        desc: 'Build multi-waypoint inspection missions with elevator traversal, custom waypoint functions, and real-time SLAM-based path planning.',
        color: 'sky',
    },
    {
        icon: <Map size={20} className="text-violet-400" />,
        title: 'MOLA LO Mapping',
        desc: 'CUDA-accelerated LiDAR odometry via MOLA framework. Real-time 3D map construction with IMU integration and deskewed scan processing.',
        color: 'violet',
    },
    {
        icon: <Cpu size={20} className="text-emerald-400" />,
        title: 'Far Planner (CUDA)',
        desc: 'GPU-accelerated global path planning using visibility graph computation. CUDA kernels for collision checking at real-time performance.',
        color: 'emerald',
    },
    {
        icon: <Shield size={20} className="text-amber-400" />,
        title: 'Terrain Analysis',
        desc: 'Dual-layer traversability assessment. Near-field terrain mapping with slope analysis and dynamic obstacle boundary extraction.',
        color: 'amber',
    },
    {
        icon: <BarChart3 size={20} className="text-rose-400" />,
        title: 'Mission Results',
        desc: 'Date-keyed inspection outcome tracking per waypoint. AI-powered analysis with confidence scoring, image logging, and Gemini insights.',
        color: 'rose',
    },
    {
        icon: <Radio size={20} className="text-cyan-400" />,
        title: 'Live Telemetry',
        desc: 'Real-time WebSocket streams for point clouds (/ws/points), robot transforms (/ws/tf), and live camera feed (/ws/video).',
        color: 'cyan',
    },
];

const TECH_STACK = [
    { name: 'ROS2 Humble', tag: 'Middleware' },
    { name: 'MOLA LO', tag: 'Odometry' },
    { name: 'CUDA 11+', tag: 'GPU' },
    { name: 'Unitree Go2', tag: 'Hardware' },
    { name: 'Livox Mid-360', tag: 'LiDAR' },
    { name: 'Go (Gin)', tag: 'Backend' },
    { name: 'React + Vite', tag: 'Frontend' },
    { name: 'PostgreSQL', tag: 'Database' },
    { name: 'MQTT', tag: 'Comms' },
    { name: 'Gemini AI', tag: 'Intelligence' },
    { name: 'Three.js', tag: '3D Viz' },
    { name: 'JWT Auth', tag: 'Security' },
];

const PIPELINE = [
    { label: 'LiDAR + IMU', sub: 'Livox Mid-360' },
    { label: 'MOLA LO', sub: 'Odometry & Mapping' },
    { label: 'Terrain Analysis', sub: 'Traversability' },
    { label: 'Far Planner', sub: 'CUDA Global Path' },
    { label: 'Local Planner', sub: 'Reactive Avoidance' },
    { label: 'Go2 Robot', sub: 'Execution' },
];

const NavItem = ({ label, onClick, active }) => (
    <button onClick={onClick} className={`text-xs font-medium tracking-wide transition-colors px-3 py-1.5 rounded-lg ${active ? 'text-white bg-white/5' : 'text-zinc-400 hover:text-white'}`}>
        {label}
    </button>
);

const colorMap = {
    sky: 'border-sky-500/20 bg-sky-500/5 hover:border-sky-500/40',
    violet: 'border-violet-500/20 bg-violet-500/5 hover:border-violet-500/40',
    emerald: 'border-emerald-500/20 bg-emerald-500/5 hover:border-emerald-500/40',
    amber: 'border-amber-500/20 bg-amber-500/5 hover:border-amber-500/40',
    rose: 'border-rose-500/20 bg-rose-500/5 hover:border-rose-500/40',
    cyan: 'border-cyan-500/20 bg-cyan-500/5 hover:border-cyan-500/40',
};

const LandingPage = ({ onOperate, onResultEditor, onResultViewer, onLogout, user }) => {
    const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
    const [scrolled, setScrolled] = useState(false);

    useEffect(() => {
        const handler = () => setScrolled(window.scrollY > 20);
        window.addEventListener('scroll', handler);
        return () => window.removeEventListener('scroll', handler);
    }, []);

    return (
        <div className="min-h-screen bg-[#070710] text-white font-sans overflow-x-hidden relative">
            {/* Ambient background */}
            <div className="fixed inset-0 pointer-events-none overflow-hidden">
                <div className="absolute top-0 left-1/4 w-[600px] h-[600px] bg-sky-600/8 rounded-full blur-[120px]" />
                <div className="absolute bottom-1/4 right-1/4 w-[400px] h-[400px] bg-violet-600/8 rounded-full blur-[100px]" />
                <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[800px] h-[300px] bg-blue-800/5 rounded-full blur-[80px]" />
            </div>

            {/* Grid pattern */}
            <div className="fixed inset-0 pointer-events-none opacity-[0.015]"
                style={{ backgroundImage: 'linear-gradient(#fff 1px, transparent 1px), linear-gradient(90deg, #fff 1px, transparent 1px)', backgroundSize: '60px 60px' }} />

            {/* Top Navigation */}
            <nav className={`fixed top-0 left-0 right-0 z-50 transition-all duration-300 ${scrolled ? 'bg-[#070710]/90 backdrop-blur-xl border-b border-white/5 shadow-xl shadow-black/20' : ''}`}>
                <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
                    {/* Logo */}
                    <div className="flex items-center gap-3">
                        <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-sky-500 to-blue-700 flex items-center justify-center shadow-lg shadow-sky-500/20">
                            <Activity size={16} className="text-white" />
                        </div>
                        <div>
                            <div className="text-sm font-bold tracking-wide">Go2 Planner Suite</div>
                            <div className="text-[9px] text-zinc-600 uppercase tracking-widest">RoboticGen Labs</div>
                        </div>
                    </div>

                    {/* Desktop Nav Links */}
                    <div className="hidden md:flex items-center gap-1">
                        <NavItem label="Mission Control" onClick={onOperate} />
                        <NavItem label="Result Editor" onClick={onResultEditor} />
                        <NavItem label="Mission Results" onClick={onResultViewer} />
                    </div>

                    {/* Right side */}
                    <div className="hidden md:flex items-center gap-3">
                        {user && (
                            <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-white/[0.03] border border-white/5">
                                <div className="w-5 h-5 rounded-full bg-sky-500/20 flex items-center justify-center">
                                    <User size={10} className="text-sky-400" />
                                </div>
                                <span className="text-[10px] text-zinc-400">{user}</span>
                            </div>
                        )}
                        {onLogout && (
                            <button onClick={onLogout} className="p-2 rounded-lg text-zinc-600 hover:text-red-400 hover:bg-red-500/5 transition-colors">
                                <LogOut size={14} />
                            </button>
                        )}
                    </div>

                    {/* Mobile menu toggle */}
                    <button className="md:hidden p-2 text-zinc-400" onClick={() => setMobileMenuOpen(!mobileMenuOpen)}>
                        {mobileMenuOpen ? <X size={18} /> : <Menu size={18} />}
                    </button>
                </div>

                {/* Mobile dropdown */}
                {mobileMenuOpen && (
                    <div className="md:hidden border-t border-white/5 bg-[#070710]/95 backdrop-blur-xl px-6 py-4 space-y-2">
                        <button onClick={() => { onOperate(); setMobileMenuOpen(false); }} className="w-full text-left text-sm text-zinc-300 py-2 hover:text-white transition-colors">Mission Control</button>
                        <button onClick={() => { onResultEditor(); setMobileMenuOpen(false); }} className="w-full text-left text-sm text-zinc-300 py-2 hover:text-white transition-colors">Result Editor</button>
                        <button onClick={() => { onResultViewer(); setMobileMenuOpen(false); }} className="w-full text-left text-sm text-zinc-300 py-2 hover:text-white transition-colors">Mission Results</button>
                    </div>
                )}
            </nav>

            {/* Hero */}
            <section className="relative z-10 pt-36 pb-24 px-6 text-center max-w-5xl mx-auto">
                {/* Badge */}
                <div className="inline-flex items-center gap-2 px-3 py-1.5 rounded-full border border-sky-500/20 bg-sky-500/5 text-sky-400 text-[10px] font-bold uppercase tracking-widest mb-8 shadow-lg shadow-sky-500/5">
                    <span className="w-1.5 h-1.5 rounded-full bg-sky-400 animate-pulse" />
                    Unofficial ROS2 SDK · Unitree Go2 AIR/PRO/EDU
                </div>

                <h1 className="text-5xl md:text-7xl font-light tracking-tight mb-6 leading-[1.1]">
                    CUDA-Accelerated<br />
                    <span className="font-bold bg-gradient-to-r from-sky-400 via-blue-400 to-violet-400 bg-clip-text text-transparent">
                        Autonomous Navigation
                    </span>
                </h1>

                <p className="text-zinc-400 text-base md:text-lg max-w-2xl mx-auto mb-4 leading-relaxed">
                    Complete autonomous navigation stack for the Unitree Go2 quadruped. MOLA LO odometry, GPU-accelerated Far Planner visibility graphs, real-time terrain analysis, and a web-based mission control interface.
                </p>

                <div className="flex flex-wrap items-center justify-center gap-3 text-[10px] text-zinc-600 mb-12">
                    <span>67 ⭐ GitHub</span><span className="text-zinc-800">·</span>
                    <span>5 Forks</span><span className="text-zinc-800">·</span>
                    <span>MIT License</span><span className="text-zinc-800">·</span>
                    <span>ROS2 Humble</span><span className="text-zinc-800">·</span>
                    <span>CUDA 11+</span>
                </div>

                {/* CTA Buttons */}
                <div className="flex flex-wrap items-center justify-center gap-4">
                    <button onClick={onOperate}
                        className="group flex items-center gap-2 px-6 py-3 rounded-xl bg-sky-600 hover:bg-sky-500 text-white text-sm font-bold tracking-wide transition-all shadow-lg shadow-sky-500/20 hover:shadow-sky-500/40 hover:scale-105">
                        <Navigation size={16} /> Mission Control
                        <ChevronRight size={14} className="group-hover:translate-x-1 transition-transform" />
                    </button>
                    <button onClick={onResultEditor}
                        className="group flex items-center gap-2 px-6 py-3 rounded-xl bg-white/5 border border-white/10 hover:bg-emerald-500/10 hover:border-emerald-500/30 text-zinc-300 hover:text-emerald-300 text-sm font-medium tracking-wide transition-all">
                        <Database size={16} /> Result Editor
                    </button>
                    <button onClick={onResultViewer}
                        className="group flex items-center gap-2 px-6 py-3 rounded-xl bg-white/5 border border-white/10 hover:bg-violet-500/10 hover:border-violet-500/30 text-zinc-300 hover:text-violet-300 text-sm font-medium tracking-wide transition-all">
                        <BarChart3 size={16} /> Mission Results
                    </button>
                </div>
            </section>

            {/* Data Pipeline */}
            <section className="relative z-10 px-6 pb-24 max-w-6xl mx-auto">
                <div className="text-center mb-10">
                    <div className="text-[10px] font-bold uppercase tracking-widest text-zinc-600 mb-2">System Architecture</div>
                    <h2 className="text-2xl font-light text-white">Autonomous Navigation Pipeline</h2>
                </div>
                <div className="flex flex-wrap items-center justify-center gap-0">
                    {PIPELINE.map((step, i) => (
                        <React.Fragment key={i}>
                            <div className="flex flex-col items-center group">
                                <div className="px-4 py-3 rounded-xl border border-sky-500/20 bg-sky-500/5 hover:border-sky-500/40 hover:bg-sky-500/10 transition-all cursor-default min-w-[120px] text-center">
                                    <div className="text-xs font-bold text-sky-300">{step.label}</div>
                                    <div className="text-[9px] text-zinc-600 mt-0.5">{step.sub}</div>
                                </div>
                            </div>
                            {i < PIPELINE.length - 1 && (
                                <div className="flex items-center text-zinc-700 px-1">
                                    <div className="w-6 h-px bg-zinc-700" />
                                    <div className="text-zinc-700">›</div>
                                </div>
                            )}
                        </React.Fragment>
                    ))}
                </div>

                {/* ROS Topics */}
                <div className="mt-8 flex flex-wrap justify-center gap-2">
                    {['/lidar_odometry/pose', '/terrain_map', '/terrain_map_ext', '/ws/points', '/ws/tf', '/ws/video'].map(t => (
                        <code key={t} className="text-[10px] px-2 py-1 rounded-md bg-zinc-900/60 border border-zinc-800 text-zinc-400 font-mono">{t}</code>
                    ))}
                </div>
            </section>

            {/* Features Grid */}
            <section className="relative z-10 px-6 pb-24 max-w-6xl mx-auto">
                <div className="text-center mb-12">
                    <div className="text-[10px] font-bold uppercase tracking-widest text-zinc-600 mb-2">Capabilities</div>
                    <h2 className="text-2xl font-light text-white">Everything You Need</h2>
                </div>
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                    {FEATURES.map((f, i) => (
                        <div key={i} className={`p-5 rounded-2xl border transition-all duration-300 cursor-default group ${colorMap[f.color]}`}>
                            <div className="mb-3">{f.icon}</div>
                            <h3 className="text-sm font-bold text-white mb-1.5">{f.title}</h3>
                            <p className="text-xs text-zinc-500 leading-relaxed">{f.desc}</p>
                        </div>
                    ))}
                </div>
            </section>

            {/* CUDA Highlight */}
            <section className="relative z-10 px-6 pb-24 max-w-6xl mx-auto">
                <div className="rounded-2xl border border-amber-500/10 bg-amber-500/5 p-8 md:p-12">
                    <div className="grid md:grid-cols-2 gap-8 items-center">
                        <div>
                            <div className="text-[10px] font-bold uppercase tracking-widest text-amber-500/60 mb-3 flex items-center gap-1.5">
                                <Zap size={10} /> GPU Acceleration
                            </div>
                            <h2 className="text-2xl md:text-3xl font-light text-white mb-4">
                                CUDA-Powered <br /><span className="font-bold text-amber-400">Visibility Graphs</span>
                            </h2>
                            <p className="text-zinc-400 text-sm leading-relaxed mb-4">
                                Far Planner computes collision-free navigation connections using massively parallel CUDA kernels. 
                                Achieves 10–100× speedup over CPU-only planning in complex environments.
                            </p>
                            <p className="text-zinc-600 text-xs">Requires CUDA 11+ · GPU compute capability 6.0+ (Pascal or newer) · GTX 1060 minimum</p>
                        </div>
                        <div className="space-y-2">
                            {[
                                { fn: 'ComputeVisibilityConnections()', role: 'Graph edge computation' },
                                { fn: 'IsEdgeCollidePolygons_GPU()', role: 'Obstacle collision check' },
                                { fn: 'doIntersect_GPU()', role: 'Geometry intersection' },
                            ].map((k, i) => (
                                <div key={i} className="p-3 rounded-xl border border-amber-500/10 bg-black/20">
                                    <code className="text-xs text-amber-300 font-mono block">{k.fn}</code>
                                    <span className="text-[10px] text-zinc-600">{k.role}</span>
                                </div>
                            ))}
                        </div>
                    </div>
                </div>
            </section>

            {/* Tech Stack */}
            <section className="relative z-10 px-6 pb-24 max-w-6xl mx-auto">
                <div className="text-center mb-10">
                    <div className="text-[10px] font-bold uppercase tracking-widest text-zinc-600 mb-2">Technology</div>
                    <h2 className="text-2xl font-light text-white">Full-Stack Integration</h2>
                </div>
                <div className="flex flex-wrap justify-center gap-3">
                    {TECH_STACK.map((t, i) => (
                        <div key={i} className="flex items-center gap-2 px-4 py-2.5 rounded-xl border border-white/5 bg-white/[0.02] hover:bg-white/[0.04] hover:border-white/10 transition-all cursor-default">
                            <span className="text-xs font-bold text-white">{t.name}</span>
                            <span className="text-[9px] text-zinc-600 uppercase tracking-wider">{t.tag}</span>
                        </div>
                    ))}
                </div>
            </section>

            {/* Mission Control CTA strip */}
            <section className="relative z-10 px-6 pb-24 max-w-6xl mx-auto">
                <div className="rounded-2xl border border-sky-500/15 bg-gradient-to-r from-sky-900/20 to-violet-900/10 p-8 md:p-12 text-center">
                    <div className="text-[10px] font-bold uppercase tracking-widest text-sky-500/50 mb-3">Ready to deploy</div>
                    <h2 className="text-2xl md:text-3xl font-light text-white mb-4">
                        Launch <span className="font-bold bg-gradient-to-r from-sky-400 to-violet-400 bg-clip-text text-transparent">Mission Control</span>
                    </h2>
                    <p className="text-zinc-500 text-sm max-w-xl mx-auto mb-8">
                        Plan multi-waypoint inspection missions, monitor telemetry in real time, record inspection results, and query AI insights — all from one unified interface.
                    </p>
                    <div className="flex flex-wrap items-center justify-center gap-4">
                        <button onClick={onOperate}
                            className="flex items-center gap-2 px-8 py-3.5 rounded-xl bg-sky-600 hover:bg-sky-500 text-white text-sm font-bold tracking-wide transition-all shadow-lg shadow-sky-500/20 hover:scale-105">
                            <Navigation size={16} /> Open Mission Control
                        </button>
                        <button onClick={onResultViewer}
                            className="flex items-center gap-2 px-8 py-3.5 rounded-xl border border-white/10 hover:border-violet-500/30 text-zinc-400 hover:text-violet-300 text-sm font-medium tracking-wide transition-all">
                            <BarChart3 size={16} /> View Results
                        </button>
                    </div>
                </div>
            </section>

            {/* Footer */}
            <footer className="relative z-10 border-t border-white/5 py-8 px-6 max-w-6xl mx-auto">
                <div className="flex flex-col md:flex-row items-center justify-between gap-4 text-[10px] text-zinc-700">
                    <span>© 2025 RoboticGen Labs · Go2 Planner Suite · MIT License</span>
                    <div className="flex items-center gap-4">
                        <a href="https://github.com/YasiruDEX/Go2-Dynamic-Inspection" target="_blank" rel="noopener noreferrer" className="hover:text-zinc-400 transition-colors">GitHub</a>
                        <span>Unitree Go2 AIR/PRO/EDU</span>
                        <span>CUDA 11+ Required</span>
                    </div>
                </div>
            </footer>
        </div>
    );
};

export default LandingPage;
