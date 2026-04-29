import React, { useState, useEffect } from 'react';
import Viewer3D from './components/Viewer3D';
import LandingPage from './components/LandingPage';
import Login from './components/Login';
import MissionResultEditor from './components/MissionResultEditor';
import MissionResultViewer from './components/MissionResultViewer';

function App() {
  const [view, setView] = useState('landing');
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      setIsAuthenticated(true);
    }
  }, []);

  const handleLoginSuccess = () => {
    setIsAuthenticated(true);
    setView('landing');
  };

  const handleLogout = () => {
    localStorage.removeItem('auth_token');
    setIsAuthenticated(false);
    setView('landing');
  };

  // If not authenticated, force login view
  if (!isAuthenticated) {
    return <Login onLoginSuccess={handleLoginSuccess} />;
  }

  return (
    <div className="App w-full h-full relative">
      {view === 'landing' ? (
        <LandingPage
          onOperate={() => setView('viz')}
          onResultEditor={() => setView('result-editor')}
          onResultViewer={() => setView('result-viewer')}
        />
      ) : view === 'result-editor' ? (
        <MissionResultEditor onBack={() => setView('landing')} />
      ) : view === 'result-viewer' ? (
        <MissionResultViewer onBack={() => setView('landing')} />
      ) : (
        <Viewer3D onBack={() => setView('landing')} onLogout={handleLogout} />
      )}
    </div>
  );
}

export default App;
