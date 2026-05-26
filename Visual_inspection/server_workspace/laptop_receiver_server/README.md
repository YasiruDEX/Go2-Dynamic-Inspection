# Laptop Receiver Server — Design Notes

## Purpose
Receives ROI image batches sent from the **Jetson** (via a ROS service → HTTP POST)
and stores them on this laptop in an organised folder hierarchy for later inspection.

---

## Folder Structure

```
Local_database/
└── <EventID>/                  ← e.g. E29Apr261201
    ├── <WaypointID>/           ← e.g. Way01001
    │   ├── metadata.json
    │   ├── roi_0.jpg
    │   └── roi_1.jpg
    └── <WaypointID_2>/         ← same event, different waypoint
        ├── metadata.json
        └── roi_0.jpg

└── <EventID_2>/                ← new event (even if waypoint IDs repeat)
    └── <WaypointID>/
        └── ...
```

**Rule:** A new event ID always creates a new top-level folder.
Waypoint IDs are *scoped inside* their event folder, so the same waypoint ID can exist
under multiple events without collision.

---

## Metadata Label Format
The metadata JSON sent by the Jetson must contain a field named one of:
- `location_label`
- `section_label`
- `label`

With a value like: `"E29Apr261201,Way01001"` — *EventID comma WaypointID*.

---

## API Endpoints

| Method | Path      | Description                         |
|--------|-----------|-------------------------------------|
| POST   | `/upload` | Receive images + metadata from Jetson |
| GET    | `/health` | Check that the server is running    |

### POST `/upload`  (multipart/form-data)
| Field      | Type   | Description                          |
|------------|--------|--------------------------------------|
| `metadata` | string | JSON-encoded metadata from Jetson    |
| `image_0`  | file   | First ROI image                      |
| `image_1`  | file   | Second ROI image (optional)          |
| `image_N`  | file   | ...up to N images                    |

---

## Running

```bash
# 1. Install dependency (only once)
pip install flask

# 2. Run the server
python laptop_receiver.py
```

Server listens on **port 5050** on all network interfaces (`0.0.0.0`).
Find your laptop's local IP and tell the Jetson to POST to:
```
http://<LAPTOP_LOCAL_IP>:5050/upload
```

---

## Notes
- No SQLite, no job IDs — just flat files, organised by EventID/WaypointID.
- The server does **not** trigger any inspection pipeline automatically.
  Inspection (gauge / VLM) is a separate step run manually when needed.
