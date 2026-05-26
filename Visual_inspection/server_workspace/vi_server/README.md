# Visual Inspection Server

Production-ready local server backend for robot visual inspection system. Receives ROI images from Jetson devices over Wi-Fi, processes them asynchronously using specialized pipelines, and stores results in a local SQLite database.

## Features

- **Fast HTTP API** - Immediate job acknowledgment with async background processing
- **Multi-Pipeline Support** - Route jobs by object type (gauge, door, fire extinguisher, unknown)
- **Local Storage** - SQLite database + file-based image storage
- **Background Worker** - Async job queue with graceful error handling
- **Production Ready** - Structured logging, type hints, comprehensive error handling
- **LAN Accessible** - Bind to 0.0.0.0 for Jetson device access

## Architecture

```
┌─────────────┐
│   Jetson    │ ──upload ROI──▶ ┌──────────────────┐
│   Device    │                  │   FastAPI API    │
└─────────────┘                  │   (Port 8000)    │
                                 └────────┬─────────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
              ┌─────▼──────┐      ┌──────▼──────┐      ┌──────▼──────┐
              │  SQLite DB │      │ File Storage│      │ Job Queue   │
              │  (Jobs)    │      │ (ROI Images)│      │ (Async)     │
              └────────────┘      └─────────────┘      └──────┬──────┘
                                                               │
                                                        ┌──────▼──────┐
                                                        │  Pipelines  │
                                                        ├─────────────┤
                                                        │ • Gauge     │
                                                        │ • VLM (stub)│
                                                        └─────────────┘
```

## Quick Start

### 1. Setup Environment

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install -e .
```

### 2. Configure Environment

```bash
# Copy example config
cp .env.example .env

# Edit .env if needed (defaults work for local testing)
```

### 3. Run Server

```bash
# Start server (binds to 0.0.0.0:8000 for LAN access)
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

Server will be accessible at:
- Local: `http://localhost:8000`
- LAN: `http://<your-ip>:8000`

### 4. Test Upload

```bash
# Upload a gauge image
python scripts/upload_test.py --image path/to/gauge.jpg --object-type gauge

# Upload with metadata
python scripts/upload_test.py \
  --image path/to/door.jpg \
  --object-type door \
  --metadata '{"location":"Room A","camera_id":"cam_01"}'
```

## API Endpoints

### POST /api/v1/jobs
Upload ROI image for inspection.

**Request:**
- `file`: ROI image (JPEG/PNG, max 10MB)
- `object_type`: One of `gauge`, `door`, `fire_extinguisher`, `unknown`
- `metadata_json`: Optional JSON string with additional metadata

**Response:**
```json
{
  "accepted": true,
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "QUEUED"
}
```

### GET /api/v1/jobs
List jobs with pagination and filtering.

**Query Parameters:**
- `limit`: Number of jobs (default: 20, max: 100)
- `offset`: Pagination offset (default: 0)
- `status`: Filter by status (optional)
- `object_type`: Filter by object type (optional)

**Response:**
```json
{
  "total": 42,
  "limit": 20,
  "offset": 0,
  "jobs": [
    {
      "job_id": "...",
      "created_at": "2026-01-06T13:30:00",
      "object_type": "gauge",
      "status": "DONE",
      "error_message": null
    }
  ]
}
```

### GET /api/v1/jobs/{job_id}
Get detailed job information.

**Response:**
```json
{
  "job_id": "...",
  "created_at": "2026-01-06T13:30:00",
  "updated_at": "2026-01-06T13:30:05",
  "object_type": "gauge",
  "status": "DONE",
  "roi_filename": "550e8400.../roi.jpg",
  "result_json": "{\"reading\":\"5.2\",\"confidence\":0.92}",
  "error_message": null,
  "blur_score": null,
  "brightness_mean": null,
  "detection_conf": null,
  "metadata_json": null
}
```

### GET /api/v1/jobs/{job_id}/roi
Download ROI image.

Returns the original uploaded ROI image file.

### GET /api/v1/health
Health check endpoint.

**Response:**
```json
{
  "status": "ok",
  "timestamp": "2026-01-06T13:30:00",
  "version": "0.1.0"
}
```

## Job Status Flow

```
RECEIVED → QUEUED → RUNNING → DONE
                            ↘ FAILED
```

- **RECEIVED**: Job created, image saved
- **QUEUED**: Job in processing queue
- **RUNNING**: Pipeline executing
- **DONE**: Processing completed successfully
- **FAILED**: Processing failed (see `error_message`)

## Pipeline Integration

### Gauge Pipeline

Currently uses a **placeholder implementation** in `app/pipelines/gauge_pipeline.py`.

**To integrate your existing gauge pipeline:**

1. **Option A: Import as module** (recommended)
   ```python
   # In app/pipelines/gauge_pipeline.py
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

2. **Option B: Copy files**
   - Copy your gauge pipeline files to `app/pipelines/gauge/`
   - Import and call from `gauge_pipeline.py`

3. **Option C: Subprocess call**
   ```python
   import subprocess
   import json
   
   def run_gauge_pipeline(roi_path: str) -> dict:
       result = subprocess.run(
           ["python", "path/to/gauge_script.py", roi_path],
           capture_output=True, text=True
       )
       return json.loads(result.stdout)
   ```

### VLM Pipeline (Future)

Currently returns stub data. Located in `app/pipelines/vlm_stub.py`.

When ready to implement:
- Replace stub with actual VLM model inference
- Consider GPT-4 Vision, LLaVA, BLIP-2, or custom models
- Maintain consistent result format

## Project Structure

```
vi_server/
├── app/
│   ├── main.py              # FastAPI app & routes
│   ├── settings.py          # Configuration
│   ├── db.py                # Database engine
│   ├── models.py            # SQLAlchemy models
│   ├── schemas.py           # Pydantic schemas
│   ├── storage.py           # File operations
│   ├── queue_worker.py      # Background worker
│   ├── pipelines/
│   │   ├── gauge_pipeline.py
│   │   └── vlm_stub.py
│   └── utils/
│       ├── logging.py
│       └── image_utils.py
├── data/
│   ├── vi_server.db         # SQLite database
│   └── jobs/                # Job storage
│       └── <job_id>/
│           └── roi.jpg
├── scripts/
│   ├── upload_test.py       # Test client
│   └── seed_sample_job.py   # DB seeding
├── tests/
│   └── test_api.py          # API tests
├── pyproject.toml
├── .env.example
└── README.md
```

## Development

### Run Tests

```bash
# Install dev dependencies
pip install -e ".[dev]"

# Run tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=app --cov-report=html
```

### Code Quality

```bash
# Format code
black app/ scripts/ tests/

# Lint
ruff check app/ scripts/ tests/
```

### Database Management

```bash
# Seed sample jobs
python scripts/seed_sample_job.py

# Access database directly
sqlite3 data/vi_server.db
```

## Configuration

Edit `.env` file:

```bash
# Database
DATABASE_URL=sqlite+aiosqlite:///./data/vi_server.db

# Storage
STORAGE_ROOT=./data/jobs
MAX_UPLOAD_SIZE_MB=10

# Server
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
LOG_LEVEL=INFO

# Object Types
ALLOWED_OBJECT_TYPES=gauge,door,fire_extinguisher,unknown

# Processing
MAX_QUEUE_SIZE=1000
WORKER_POLL_INTERVAL=0.1
```

## LAN Access for Jetson

1. **Find your computer's IP:**
   ```bash
   # Windows
   ipconfig
   
   # Linux/Mac
   ifconfig
   ```

2. **Ensure firewall allows port 8000**

3. **From Jetson, test connection:**
   ```bash
   curl http://<your-ip>:8000/api/v1/health
   ```

4. **Upload from Jetson:**
   ```python
   import requests
   
   url = "http://<your-ip>:8000/api/v1/jobs"
   files = {"file": open("roi.jpg", "rb")}
   data = {"object_type": "gauge"}
   
   response = requests.post(url, files=files, data=data)
   job_id = response.json()["job_id"]
   ```

## Future Expansion

This server is designed for easy expansion:

- **Camera-Lidar Mapping**: Add new endpoints and models
- **Additional Pipelines**: Add new object types and handlers
- **Result Visualization**: Serve annotated images
- **Metrics & Monitoring**: Add Prometheus/Grafana integration
- **Multi-Worker**: Scale with multiple worker processes
- **Cloud Sync**: Optional cloud backup/sync

## Troubleshooting

### Server won't start
- Check port 8000 is not in use: `netstat -ano | findstr :8000`
- Verify virtual environment is activated
- Check `.env` file exists

### Jobs stuck in QUEUED
- Check server logs for worker errors
- Verify background worker started (check startup logs)
- Check database permissions

### Can't access from Jetson
- Verify server bound to 0.0.0.0 (not 127.0.0.1)
- Check firewall settings
- Ensure devices on same network
- Test with `curl http://<ip>:8000/api/v1/health`

### Image upload fails
- Check file size < 10MB
- Verify file is valid JPEG/PNG
- Check disk space in `data/jobs/`

## License

MIT

## Support

For issues or questions, check the logs:
```bash
# Server logs show in console when running with --reload
# Look for ERROR or WARNING messages
```
