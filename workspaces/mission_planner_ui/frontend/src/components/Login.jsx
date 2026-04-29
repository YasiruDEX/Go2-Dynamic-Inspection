import React, { useState } from 'react';
import axios from 'axios';
import { Lock, User } from 'lucide-react';

const Login = ({ onLoginSuccess }) => {
    const [username, setUsername] = useState('');
    const [password, setPassword] = useState('');
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);

    const handleLogin = async (e) => {
        e.preventDefault();
        setError('');
        setLoading(true);

        try {
            const formData = new URLSearchParams();
            formData.append('username', username);
            formData.append('password', password);

            const response = await axios.post(`/login`, formData, {
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' }
            });

            const token = response.data.access_token;
            localStorage.setItem('auth_token', token);
            axios.defaults.headers.common['Authorization'] = `Bearer ${token}`;

            onLoginSuccess();
        } catch (err) {
            if (err.response && err.response.status === 401) {
                setError('Invalid username or password');
            } else {
                setError('Failed to connect to server');
            }
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="min-h-screen bg-midnight text-white font-sans flex items-center justify-center relative overflow-hidden">
            {/* Background Gradients */}
            <div className="absolute inset-0 pointer-events-none">
                <div className="absolute top-[-10%] left-[-10%] w-[50vw] h-[50vw] bg-blue-900/20 rounded-full blur-[120px]" />
                <div className="absolute bottom-[-10%] right-[-10%] w-[40vw] h-[40vw] bg-purple-500/10 rounded-full blur-[100px]" />
            </div>

            <div className="relative z-10 w-full max-w-4xl flex flex-col md:flex-row bg-slate-900/50 backdrop-blur-xl border border-white/10 rounded-2xl shadow-[0_8px_32px_rgba(0,0,0,0.5)] overflow-hidden">

                {/* Robot Image Side */}
                <div className="w-full md:w-1/2 p-12 bg-gradient-to-br from-blue-900/40 to-black/60 flex flex-col items-center justify-center relative border-b md:border-b-0 md:border-r border-white/5">
                    <div className="absolute inset-0 bg-grid-white/[0.02] bg-[size:30px_30px]" />
                    <h2 className="text-3xl font-light tracking-wide mb-2 text-center relative z-10">RoboticGen<span className="font-bold">LABS</span></h2>
                    <p className="text-slate-400 text-sm mb-12 text-center max-w-xs relative z-10">Enter your credentials to access the Mission Planner interface.</p>

                    <div className="relative h-64 w-full flex items-center justify-center group perspective-1000">
                        <div className="absolute inset-0 bg-blue-500/20 rounded-full blur-3xl opacity-50 group-hover:opacity-80 transition-opacity duration-700" />
                        <img
                            src="/robot.png"
                            alt="Unitree Go2"
                            className="relative z-10 max-h-full object-contain drop-shadow-2xl transition-transform duration-700 ease-in-out transform group-hover:scale-110 group-hover:rotate-y-12"
                        />
                    </div>
                </div>

                {/* Login Form Side */}
                <div className="w-full md:w-1/2 p-8 md:p-12 flex flex-col justify-center">
                    <div className="mb-8">
                        <h3 className="text-2xl font-semibold mb-1">Welcome Back</h3>
                        <p className="text-slate-400 text-sm">Sign in to initialize systems.</p>
                    </div>

                    <form onSubmit={handleLogin} className="space-y-6">
                        {error && (
                            <div className="p-3 rounded-lg bg-red-500/10 border border-red-500/30 text-red-400 text-sm font-medium animate-in slide-in-from-top-2">
                                {error}
                            </div>
                        )}

                        <div className="space-y-4">
                            <div className="relative">
                                <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none text-slate-500">
                                    <User size={18} />
                                </div>
                                <input
                                    type="text"
                                    value={username}
                                    onChange={(e) => setUsername(e.target.value)}
                                    placeholder="Operator ID"
                                    required
                                    className="w-full pl-10 pr-4 py-3 bg-black/40 border border-slate-700 rounded-xl focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all outline-none text-slate-200 placeholder-slate-500"
                                />
                            </div>

                            <div className="relative">
                                <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none text-slate-500">
                                    <Lock size={18} />
                                </div>
                                <input
                                    type="password"
                                    value={password}
                                    onChange={(e) => setPassword(e.target.value)}
                                    placeholder="Access Code"
                                    required
                                    className="w-full pl-10 pr-4 py-3 bg-black/40 border border-slate-700 rounded-xl focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-all outline-none text-slate-200 placeholder-slate-500"
                                />
                            </div>
                        </div>

                        <button
                            type="submit"
                            disabled={loading}
                            className={`w-full py-3 px-4 rounded-xl flex items-center justify-center text-sm font-bold tracking-wider transition-all duration-300
                                ${loading
                                    ? 'bg-blue-600/50 text-white/50 cursor-not-allowed'
                                    : 'bg-blue-600 hover:bg-blue-500 text-white shadow-[0_0_20px_rgba(37,99,235,0.3)] hover:shadow-[0_0_25px_rgba(37,99,235,0.5)] transform hover:-translate-y-0.5'
                                }`}
                        >
                            {loading ? (
                                <div className="w-5 h-5 border-2 border-white/20 border-t-white rounded-full animate-spin" />
                            ) : (
                                "INITIALIZE LINK"
                            )}
                        </button>
                    </form>

                    <div className="mt-8 text-center text-xs text-slate-500 font-mono opacity-60">
                        AUTH :: POSTGRESQL // SECURE CONNECTION
                    </div>
                </div>
            </div>
        </div>
    );
};

export default Login;
