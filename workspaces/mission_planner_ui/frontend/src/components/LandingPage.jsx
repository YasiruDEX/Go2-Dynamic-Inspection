import React from 'react';

const LandingPage = ({ onOperate, onResultEditor, onResultViewer }) => {
    return (
        <div className="min-h-screen bg-midnight text-white font-sans overflow-x-hidden relative">
            {/* Background Gradients */}
            <div className="absolute inset-0 overflow-hidden pointer-events-none">
                <div className="absolute top-[-10%] left-1/2 -translate-x-1/2 w-[80vw] h-[60vh] bg-blue-900/20 rounded-full blur-[120px]" />
                <div className="absolute bottom-[-10%] left-1/2 -translate-x-1/2 w-[60vw] h-[40vh] bg-blue-500/10 rounded-full blur-[100px]" />
            </div>

            {/* Navbar */}
            <nav className="relative z-10 flex justify-between items-center px-12 py-6 max-w-7xl mx-auto">
                <div className="flex items-center gap-2">
                    <div className="text-xl font-bold tracking-wider">
                        RoboticGen<span className="text-xs ml-1 opacity-60">LABS</span>
                    </div>
                </div>
            </nav>

            {/* Hero Section */}
            <main className="relative z-10 flex flex-col items-center justify-center pt-20 pb-32 text-center px-4">
                <h1 className="text-5xl md:text-7xl font-light tracking-tight mb-6 max-w-5xl leading-tight">
                    Intelligent Machines For <br />
                    <span className="font-medium bg-gradient-to-r from-gray-100 to-gray-500 bg-clip-text text-transparent">
                        A Better Tomorrow
                    </span>
                </h1>

                <p className="text-gray-400 text-lg max-w-2xl mb-12 opacity-80">
                    Sri Lanka's first Robotics & AI Lab, driving innovation in smart mobility, physical AI, and industrial automation.
                </p>

                <div className="flex flex-wrap items-center justify-center gap-4">
                    <button
                        onClick={onOperate}
                        className="group relative px-8 py-3 rounded-full bg-slate-800/50 border border-slate-700/50 text-blue-200 text-sm font-medium tracking-wide hover:bg-slate-700/50 hover:border-blue-500/30 transition-all duration-300 backdrop-blur-md overflow-hidden"
                    >
                        <div className="absolute inset-0 bg-gradient-to-r from-blue-500/10 to-purple-500/10 opacity-0 group-hover:opacity-100 transition-opacity" />
                        Operate Obo Dog
                    </button>

                    <button
                        onClick={onResultEditor}
                        className="group relative px-8 py-3 rounded-full bg-emerald-900/30 border border-emerald-800/40 text-emerald-200 text-sm font-medium tracking-wide hover:bg-emerald-800/40 hover:border-emerald-500/30 transition-all duration-300 backdrop-blur-md overflow-hidden"
                    >
                        <div className="absolute inset-0 bg-gradient-to-r from-emerald-500/10 to-teal-500/10 opacity-0 group-hover:opacity-100 transition-opacity" />
                        Result Editor
                    </button>

                    <button
                        onClick={onResultViewer}
                        className="group relative px-8 py-3 rounded-full bg-sky-900/30 border border-sky-800/40 text-sky-200 text-sm font-medium tracking-wide hover:bg-sky-800/40 hover:border-sky-500/30 transition-all duration-300 backdrop-blur-md overflow-hidden"
                    >
                        <div className="absolute inset-0 bg-gradient-to-r from-sky-500/10 to-indigo-500/10 opacity-0 group-hover:opacity-100 transition-opacity" />
                        Mission Results
                    </button>
                </div>
            </main>

            {/* Info Section - Dynamic Inspection Pipeline */}
            <section className="relative z-10 max-w-6xl mx-auto px-6 pb-20 grid grid-cols-1 md:grid-cols-2 gap-12 items-center">
                <div className="space-y-6">
                    <div className="w-12 h-[1px] bg-blue-500" />
                    <h2 className="text-3xl font-light text-white">Dynamic Inspection Pipeline</h2>
                    <p className="text-gray-300 leading-relaxed text-lg">
                        Our state-of-the-art voxelization and SLAM pipeline enables real-time environmental mapping and navigation for the Obo Dog platform.
                        Utilizing raw LiDAR data processed into efficient voxel grids, we ensure robust obstacle detection and path planning.
                    </p>
                </div>

                {/* Robot Image */}
                <div className="relative h-[400px] flex items-center justify-center">
                    <div className="absolute inset-0 bg-gradient-to-tr from-blue-500 to-purple-500 rounded-full opacity-20 blur-3xl" />
                    <img
                        src="/robot.png"
                        alt="Unitree Go1"
                        className="relative z-10 w-full max-w-md drop-shadow-2xl hover:scale-105 transition-transform duration-500"
                    />
                </div>
            </section>

            <footer className="relative z-10 border-t border-white/5 py-12 text-center text-gray-600 text-sm">
                &copy; 2025 RoboticGen Labs. All rights reserved.
            </footer>
        </div>
    );
};

export default LandingPage;
