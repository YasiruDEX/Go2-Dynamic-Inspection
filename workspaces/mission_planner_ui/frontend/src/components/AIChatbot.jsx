import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import { Bot, Send, X, Minimize2, ChevronDown } from 'lucide-react';

import { API_BASE } from '../lib/config'

const apiCall = async (path, method = 'GET', body = null) => {
    const token = localStorage.getItem('auth_token');
    const opts = { method, headers: { 'Content-Type': 'application/json', 'Authorization': `Bearer ${token}` } };
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`${API_BASE}${path}`, opts);
    if (!res.ok) throw new Error(`API error: ${res.status}`);
    return res.json();
};

const AIChatbot = ({ contextMissionId, contextDate }) => {
    const [open, setOpen] = useState(false);
    const [minimized, setMinimized] = useState(false);
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [loading, setLoading] = useState(false);
    const [missions, setMissions] = useState([]);
    const [selectedMission, setSelectedMission] = useState(contextMissionId || '');
    const [selectedDate, setSelectedDate] = useState(contextDate || '');
    const [dates, setDates] = useState([]);
    const endRef = useRef(null);

    useEffect(() => {
        apiCall('/missions').then(d => setMissions(d || [])).catch(() => {});
    }, []);

    useEffect(() => {
        if (contextMissionId) setSelectedMission(contextMissionId);
        if (contextDate) setSelectedDate(contextDate);
    }, [contextMissionId, contextDate]);

    useEffect(() => {
        if (!selectedMission) { setDates([]); return; }
        apiCall(`/missions/${selectedMission}/results/dates`).then(d => {
            setDates(d || []);
            if (!selectedDate && d && d.length > 0) setSelectedDate(d[0]);
        }).catch(() => {});
    }, [selectedMission]);

    useEffect(() => {
        endRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    const send = async () => {
        if (!input.trim()) return;
        const msg = input.trim();
        setMessages(prev => [...prev, { role: 'user', text: msg }]);
        setInput('');
        setLoading(true);
        try {
            const data = await apiCall('/chat', 'POST', {
                mission_id: selectedMission ? parseInt(selectedMission) : 0,
                date: selectedDate || '',
                message: msg,
            });
            setMessages(prev => [...prev, { role: 'assistant', text: data.reply }]);
        } catch {
            setMessages(prev => [...prev, { role: 'assistant', text: 'Unable to reach AI. Check backend connection.' }]);
        }
        setLoading(false);
    };

    return (
        <div className="fixed bottom-6 right-6 z-[9999] flex flex-col items-end gap-3">
            {/* Chat Window */}
            {open && !minimized && (
                <div className="w-[380px] h-[520px] flex flex-col rounded-2xl border border-sky-500/20 bg-[#0d0d14]/95 backdrop-blur-2xl shadow-2xl shadow-sky-500/10 overflow-hidden">
                    {/* Header */}
                    <div className="flex items-center justify-between px-4 py-3 border-b border-white/5 bg-sky-600/10">
                        <div className="flex items-center gap-2.5">
                            <div className="w-7 h-7 rounded-full bg-sky-500/20 border border-sky-500/30 flex items-center justify-center">
                                <Bot size={14} className="text-sky-400" />
                            </div>
                            <div>
                                <div className="text-xs font-bold text-white">Mission Analyst AI</div>
                                <div className="text-[9px] text-sky-400/60 uppercase tracking-wider">Powered by Gemini</div>
                            </div>
                        </div>
                        <div className="flex items-center gap-1">
                            <button onClick={() => setMinimized(true)} className="p-1.5 rounded-lg text-zinc-500 hover:text-white hover:bg-white/5 transition-colors">
                                <Minimize2 size={12} />
                            </button>
                            <button onClick={() => setOpen(false)} className="p-1.5 rounded-lg text-zinc-500 hover:text-white hover:bg-white/5 transition-colors">
                                <X size={12} />
                            </button>
                        </div>
                    </div>

                    {/* Context selectors */}
                    <div className="flex gap-2 px-3 py-2 border-b border-white/5 bg-zinc-900/30">
                        <select value={selectedMission} onChange={e => { setSelectedMission(e.target.value); setSelectedDate(''); }}
                            className="flex-1 bg-zinc-900/80 border border-zinc-800 rounded-lg px-2 py-1.5 text-[10px] text-white focus:outline-none truncate">
                            <option value="">All missions</option>
                            {missions.map(m => <option key={m.ID} value={m.ID} className="bg-zinc-900">{m.name}</option>)}
                        </select>
                        {dates.length > 0 && (
                            <select value={selectedDate} onChange={e => setSelectedDate(e.target.value)}
                                className="bg-zinc-900/80 border border-zinc-800 rounded-lg px-2 py-1.5 text-[10px] text-white focus:outline-none">
                                <option value="">All dates</option>
                                {dates.map(d => <option key={d} value={d} className="bg-zinc-900">{d}</option>)}
                            </select>
                        )}
                    </div>

                    {/* Messages */}
                    <div className="flex-1 overflow-auto p-3 space-y-3">
                        {messages.length === 0 && (
                            <div className="text-center py-10 text-zinc-600 text-xs">
                                <Bot size={28} className="mx-auto mb-3 opacity-20" />
                                <p>Ask me anything about inspection results.</p>
                                <p className="text-zinc-700 mt-1">Select a mission for specific analysis.</p>
                            </div>
                        )}
                        {messages.map((msg, i) => (
                            <div key={i} className={`flex ${msg.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                                <div className={`max-w-[88%] px-3 py-2 rounded-xl text-xs leading-relaxed overflow-hidden ${msg.role === 'user' ? 'bg-sky-600 text-white rounded-br-sm' : 'bg-zinc-800/80 text-zinc-300 rounded-bl-sm [&_p]:mb-1.5 [&_p:last-child]:mb-0 [&_ul]:list-disc [&_ul]:pl-3.5 [&_ol]:list-decimal [&_ol]:pl-3.5 [&_li]:mb-1 [&_strong]:text-sky-300 [&_h3]:font-bold [&_h3]:text-white [&_h3]:mb-1'}`}>
                                    {msg.role === 'user'
                                        ? msg.text
                                        : <ReactMarkdown>{msg.text}</ReactMarkdown>
                                    }
                                </div>
                            </div>
                        ))}
                        {loading && (
                            <div className="flex justify-start">
                                <div className="bg-zinc-800/80 text-sky-400 px-3 py-2 rounded-xl text-xs flex items-center gap-1.5">
                                    <span className="w-1.5 h-1.5 rounded-full bg-sky-400 animate-bounce" style={{ animationDelay: '0ms' }} />
                                    <span className="w-1.5 h-1.5 rounded-full bg-sky-400 animate-bounce" style={{ animationDelay: '150ms' }} />
                                    <span className="w-1.5 h-1.5 rounded-full bg-sky-400 animate-bounce" style={{ animationDelay: '300ms' }} />
                                </div>
                            </div>
                        )}
                        <div ref={endRef} />
                    </div>

                    {/* Input */}
                    <div className="p-3 border-t border-white/5">
                        <div className="flex gap-2">
                            <input value={input} onChange={e => setInput(e.target.value)}
                                onKeyDown={e => e.key === 'Enter' && !e.shiftKey && send()}
                                placeholder="Ask about missions & results..."
                                className="flex-1 bg-zinc-900/80 border border-zinc-800 rounded-xl px-3 py-2 text-xs text-white focus:outline-none focus:border-sky-700/50 placeholder-zinc-700 transition-colors" />
                            <button onClick={send} disabled={loading || !input.trim()}
                                className="px-3 py-2 bg-sky-600 hover:bg-sky-500 disabled:bg-zinc-800 disabled:text-zinc-600 text-white rounded-xl transition-all shadow-lg shadow-sky-500/20">
                                <Send size={12} />
                            </button>
                        </div>
                    </div>
                </div>
            )}

            {/* Minimized bar */}
            {open && minimized && (
                <button onClick={() => setMinimized(false)}
                    className="flex items-center gap-2 px-4 py-2.5 bg-[#0d0d14]/95 border border-sky-500/20 rounded-2xl text-xs text-sky-400 font-bold shadow-xl backdrop-blur-xl hover:border-sky-500/40 transition-all">
                    <Bot size={14} /> Mission Analyst AI
                    <ChevronDown size={12} />
                </button>
            )}

            {/* FAB Button */}
            <button onClick={() => { setOpen(!open); setMinimized(false); }}
                className={`relative w-14 h-14 rounded-2xl flex items-center justify-center shadow-2xl transition-all duration-300 ${open ? 'bg-zinc-800 rotate-0' : 'bg-gradient-to-br from-sky-600 to-blue-700 hover:scale-110 hover:shadow-sky-500/30'}`}>
                {open ? <X size={20} className="text-zinc-400" /> : <Bot size={22} className="text-white" />}
                {!open && messages.length > 0 && (
                    <span className="absolute -top-1 -right-1 w-5 h-5 rounded-full bg-emerald-500 text-[9px] text-white font-black flex items-center justify-center border-2 border-[#0a0a0f]">
                        {messages.filter(m => m.role === 'assistant').length}
                    </span>
                )}
            </button>
        </div>
    );
};

export default AIChatbot;
