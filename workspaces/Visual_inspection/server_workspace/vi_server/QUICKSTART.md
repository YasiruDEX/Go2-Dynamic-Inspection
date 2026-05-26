# Visual Inspection Server - Quick Reference

## 🚀 Quick Start

### 1. First Time Setup
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
venv\Scripts\activate  # Windows
source venv/bin/activate  # Linux/Mac

# Install dependencies
pip install -e .

# Create .env file
cp .env.example .env
```

### 2. Start Server
```bash
# Option A: Using batch script (Windows)
start_server.bat

# Option B: Manual
venv\Scripts\activate
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

Server will be available at:
- **Local**: http://localhost:8000
- **LAN**: http://YOUR_IP:8000
- **API Docs**: http://localhost:8000/docs

### 3. Test Upload
```bash
# Option A: Using batch script (Windows)
test_upload.bat sample_images\sample_gauge.jpg gauge

# Option B: Manual
python scripts\upload_test.py --image sample_images\sample_gauge.jpg --object-type gauge
```

## 📡 API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/jobs` | Upload ROI image |
| GET | `/api/v1/jobs` | List all jobs |
| GET | `/api/v1/jobs/{job_id}` | Get job details |
| GET | `/api/v1/jobs/{job_id}/roi` | Download ROI image |
| GET | `/api/v1/health` | Health check |

## 🔧 Common Commands

```bash
# Check server health
curl http://localhost:8000/api/v1/health

# List all jobs
curl http://localhost:8000/api/v1/jobs

# Get specific job
curl http://localhost:8000/api/v1/jobs/{job_id}

# Upload image (using curl)
curl -X POST http://localhost:8000/api/v1/jobs \
  -F "file=@sample_images/sample_gauge.jpg" \
  -F "object_type=gauge"
```

## 📁 Project Structure

```
vi_server/
├── app/                    # Main application code
│   ├── main.py            # FastAPI app & routes
│   ├── queue_worker.py    # Background job processor
│   ├── pipelines/         # Processing pipelines
│   │   ├── gauge_pipeline.py   # Gauge reading (placeholder)
│   │   └── vlm_stub.py         # VLM stub
│   └── ...
├── data/                  # Runtime data (created automatically)
│   ├── vi_server.db      # SQLite database
│   └── jobs/             # Job files
├── scripts/              # Utility scripts
│   └── upload_test.py    # Test client
├── sample_images/        # Sample test images
├── start_server.bat      # Quick start script
└── test_upload.bat       # Quick test script
```

## 🔌 Integrating Your Gauge Pipeline

Edit `app/pipelines/gauge_pipeline.py`:

```python
# Replace the placeholder with your actual pipeline
from your_gauge_module import read_gauge

def run_gauge_pipeline(roi_path: str) -> dict:
    result = read_gauge(roi_path)
    return {
        "reading": result.value,
        "confidence": result.confidence,
        "method": "gauge_pipeline",
        "unit": result.unit,
    }
```

## 🌐 LAN Access (for Jetson)

1. Find your computer's IP:
   ```bash
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   ```

2. Ensure firewall allows port 8000

3. From Jetson:
   ```python
   import requests
   
   url = "http://YOUR_IP:8000/api/v1/jobs"
   files = {"file": open("roi.jpg", "rb")}
   data = {"object_type": "gauge"}
   
   response = requests.post(url, files=files, data=data)
   job_id = response.json()["job_id"]
   ```

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| Port 8000 in use | Change port in `.env`: `SERVER_PORT=8001` |
| Can't access from LAN | Check firewall, ensure `SERVER_HOST=0.0.0.0` |
| Jobs stuck in QUEUED | Check server logs for worker errors |
| Import errors | Ensure venv is activated |

## 📊 Job Status Flow

```
RECEIVED → QUEUED → RUNNING → DONE
                            ↘ FAILED
```

## 🧪 Testing

```bash
# Run tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=app
```

## 📝 Environment Variables

Key settings in `.env`:

```bash
DATABASE_URL=sqlite+aiosqlite:///./data/vi_server.db
STORAGE_ROOT=./data/jobs
MAX_UPLOAD_SIZE_MB=10
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
ALLOWED_OBJECT_TYPES=gauge,door,fire_extinguisher,unknown
```

## 📚 Full Documentation

See `README.md` for complete documentation.
