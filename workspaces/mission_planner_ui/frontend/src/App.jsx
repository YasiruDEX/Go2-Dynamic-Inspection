import React, { useState, useEffect } from 'react';
import Viewer3D from './components/Viewer3D';
import LandingPage from './components/LandingPage';
import Login from './components/Login';
import MissionResultEditor from './components/MissionResultEditor';
import MissionResultViewer from './components/MissionResultViewer';
import AIChatbot from './components/AIChatbot';
import RobotConsole from './components/RobotConsole';

function App() {
  const [view, setView] = useState('landing');
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState(null);
  const [showConsole, setShowConsole] = useState(false);

  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      setIsAuthenticated(true);
      // Decode display name from JWT if available
      try {
        const payload = JSON.parse(atob(token.split('.')[1]));
        setUser(payload.sub || null);
      } catch {}
    }
  }, []);

  const handleLoginSuccess = () => {
    setIsAuthenticated(true);
    setView('landing');
    try {
      const token = localStorage.getItem('auth_token');
      const payload = JSON.parse(atob(token.split('.')[1]));
      setUser(payload.sub || null);
    } catch {}
  };

  const handleLogout = () => {
    localStorage.removeItem('auth_token');
    setIsAuthenticated(false);
    setUser(null);
    setView('landing');
  };

  if (!isAuthenticated) {
    return <Login onLoginSuccess={handleLoginSuccess} />;
  }

  return (
    <div className="App w-full h-full relative">
      {view === 'landing' && (
        <LandingPage
          onOperate={() => setView('viz')}
          onResultEditor={() => setView('result-editor')}
          onResultViewer={() => setView('result-viewer')}
          onConsole={() => setShowConsole(true)}
          onLogout={handleLogout}
          user={user}
        />
      )}
      {view === 'viz' && (
        <Viewer3D onBack={() => setView('landing')} onLogout={handleLogout} />
      )}
      {view === 'result-editor' && (
        <MissionResultEditor onBack={() => setView('landing')} />
      )}
      {view === 'result-viewer' && (
        <MissionResultViewer onBack={() => setView('landing')} />
      )}

      {/* Global floating AI chatbot — visible on all authenticated pages */}
      <AIChatbot />

      {/* Global Robot Console overlay */}
      {showConsole && (
        <div className="fixed top-0 right-0 h-full z-[100]">
            <RobotConsole onClose={() => setShowConsole(false)} />
        </div>
      )}
    </div>
  );
}

export default App;
