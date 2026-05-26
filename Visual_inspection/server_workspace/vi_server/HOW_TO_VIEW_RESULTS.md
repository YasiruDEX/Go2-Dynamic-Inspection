# Quick Guide: How to View Gauge Inspection Results

## 📊 Where Everything is Stored

### Images
```
./data/jobs/<job_id>/roi.jpg
```

### Database
```
./data/vi_server.db
```

---

## 🔍 View Results

### Method 1: Using API (Recommended)

**Get job result:**
```bash
curl http://localhost:8000/api/v1/jobs/<job_id>
```

**Get image:**
```bash
curl http://localhost:8000/api/v1/jobs/<job_id>/roi --output gauge_image.jpg
```

### Method 2: Direct Database Query

**Install SQLite (if not installed):**
```bash
# Windows: Download from https://www.sqlite.org/download.html
# Or use DB Browser for SQLite: https://sqlitebrowser.org/
```

**Query database:**
```bash
cd "e:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\vi_server"

# View all gauge jobs
sqlite3 data/vi_server.db "SELECT job_id, status, result_json FROM jobs WHERE object_type='gauge';"

# View specific job
sqlite3 data/vi_server.db "SELECT * FROM jobs WHERE job_id='<job_id>';"

# View latest gauge job
sqlite3 data/vi_server.db "SELECT job_id, status, result_json FROM jobs WHERE object_type='gauge' ORDER BY created_at DESC LIMIT 1;"
```

### Method 3: Python Script

**Create `view_results.py`:**
```python
import sqlite3
import json
from pathlib import Path

# Connect to database
db_path = "data/vi_server.db"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get latest gauge job
cursor.execute("""
    SELECT job_id, object_type, status, result_json, roi_filename, created_at
    FROM jobs 
    WHERE object_type='gauge' 
    ORDER BY created_at DESC 
    LIMIT 5
""")

print("Latest 5 Gauge Jobs:")
print("-" * 80)

for row in cursor.fetchall():
    job_id, obj_type, status, result_json, roi_filename, created_at = row
    
    print(f"\nJob ID: {job_id}")
    print(f"Status: {status}")
    print(f"Created: {created_at}")
    print(f"Image: ./data/jobs/{roi_filename}")
    
    if result_json:
        result = json.loads(result_json)
        print(f"Reading: {result.get('reading')}")
        print(f"Unit: {result.get('unit')}")
        print(f"Confidence: {result.get('confidence')}")
    print("-" * 80)

conn.close()
```

**Run:**
```bash
python view_results.py
```

---

## 📸 View Images

### Option 1: File Explorer
```
Navigate to: e:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\vi_server\data\jobs\
Open: <job_id>\roi.jpg
```

### Option 2: Command Line
```bash
# Windows
start data\jobs\<job_id>\roi.jpg

# Or copy to desktop
copy data\jobs\<job_id>\roi.jpg %USERPROFILE%\Desktop\gauge_result.jpg
```

---

## 🐛 Debug Images

Gauge pipeline creates detailed debug images:

```
app/pipelines/gauge/runs/server_run_<uuid>/
├── bbox_results.jpg           # Shows detected gauge bounding box
├── key_point_results.jpg      # Shows detected key points on gauge
├── ellipse_results_*.jpg      # Shows fitted ellipse
├── ocr_results_numbers.jpg    # Shows detected numbers
├── segmentation_results.jpg   # Shows detected needle
└── reading_line_fit.jpg       # Shows final reading calculation
```

**View latest debug images:**
```bash
cd app/pipelines/gauge/runs
dir /od  # List directories by date
cd server_run_<latest>
start .  # Open folder in explorer
```

---

## 📊 Example Output

### Database Result:
```
job_id: abc123-def456-789
status: DONE
result_json: {"reading": 1.987, "unit": "unknown", "confidence": 0.85, "method": "analog_gauge_reader"}
roi_filename: abc123-def456-789/roi.jpg
created_at: 2026-01-07 09:42:00
```

### API Response:
```json
{
  "job_id": "abc123-def456-789",
  "created_at": "2026-01-07T09:42:00",
  "updated_at": "2026-01-07T09:43:30",
  "object_type": "gauge",
  "status": "DONE",
  "roi_filename": "abc123-def456-789/roi.jpg",
  "result_json": "{\"reading\": 1.987, \"unit\": \"unknown\", \"confidence\": 0.85, \"method\": \"analog_gauge_reader\"}",
  "error_message": null
}
```

---

## 🔧 Troubleshooting

### Can't find database
```bash
# Check if database exists
dir data\vi_server.db

# If not, server hasn't been started yet
start_server.bat
```

### Can't find images
```bash
# Check storage directory
dir data\jobs

# If empty, no jobs have been processed yet
python scripts/upload_test.py --image <gauge_image> --object-type gauge
```

### Empty result_json
- Job might still be processing (status: QUEUED or RUNNING)
- Check error_message field for errors
- Check server logs

---

## 📝 Quick Commands

```bash
# View latest gauge job
sqlite3 data/vi_server.db "SELECT job_id, status, result_json FROM jobs WHERE object_type='gauge' ORDER BY created_at DESC LIMIT 1;"

# Count total gauge jobs
sqlite3 data/vi_server.db "SELECT COUNT(*) FROM jobs WHERE object_type='gauge';"

# View failed jobs
sqlite3 data/vi_server.db "SELECT job_id, error_message FROM jobs WHERE status='FAILED';"

# List all images
dir /s /b data\jobs\*.jpg
```

---

**Everything is stored and accessible!** 🎉
