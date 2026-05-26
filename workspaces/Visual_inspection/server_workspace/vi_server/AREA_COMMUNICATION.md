# Area Information in Robot Communication

## Summary

✅ **Areas and inspection points are NOT fixed** - users can create unlimited areas and points from the UI
✅ **Area information is ALWAYS sent** to the robot in inspection responses
✅ **System is fully dynamic** - no hardcoded limits

## How Area Information is Sent to Robot

### 1. Known Location Inspection

**Request from UI:**
```json
POST /api/v1/inspections/known
{
  "area_id": 1,
  "point_id": 5
}
```

**Response to Robot:**
```json
{
  "job_id": "pending",
  "status": "NAVIGATION_REQUESTED",
  "message": "Navigate to Gauge 1 and capture ROI",
  "coordinates": {
    "x": 5.2,
    "y": 3.8,
    "z": 0.0,
    "orientation": 0.0,
    "point_id": 5,
    "point_name": "Gauge 1",
    "object_type": "gauge",
    // ↓ AREA INFORMATION INCLUDED ↓
    "area_id": 1,
    "area_name": "Floor 1 - Main Hall",
    "floor_number": 1
  }
}
```

### 2. Ad-hoc Location Inspection

**Request from UI:**
```json
POST /api/v1/inspections/adhoc
{
  "area_id": 1,
  "x": 10.5,
  "y": 5.3,
  "object_type": "unknown"
}
```

**Response to Robot:**
```json
{
  "job_id": "pending",
  "status": "NAVIGATION_REQUESTED",
  "message": "Navigate to coordinates in Floor 1 - Main Hall and capture ROI",
  "coordinates": {
    "x": 10.5,
    "y": 5.3,
    "z": 0.0,
    "orientation": 0.0,
    // ↓ AREA INFORMATION INCLUDED ↓
    "area_id": 1,
    "area_name": "Floor 1 - Main Hall",
    "floor_number": 1,
    "description": "Main production floor",
    "object_type": "unknown"
  }
}
```

## What Robot Receives

The robot ALWAYS receives:
- **Coordinates**: x, y, z, orientation (where to go)
- **Area ID**: Which area/floor
- **Area Name**: Human-readable area name
- **Floor Number**: Which floor level
- **Object Type**: What to inspect
- **Point Name** (known locations only): Inspection point identifier

## Dynamic System - No Limits

### Creating Areas (Unlimited)
```bash
# User can create as many areas as needed
POST /api/v1/areas
{
  "name": "Floor 1",
  "floor_number": 1
}

POST /api/v1/areas
{
  "name": "Floor 2",
  "floor_number": 2
}

POST /api/v1/areas
{
  "name": "Basement",
  "floor_number": -1
}
# ... unlimited areas
```

### Creating Inspection Points (Unlimited per Area)
```bash
# User can create as many points as needed in each area
POST /api/v1/areas/1/points
{
  "name": "Gauge 1",
  "object_type": "gauge",
  "x": 5.2,
  "y": 3.8
}

POST /api/v1/areas/1/points
{
  "name": "Gauge 2",
  "object_type": "gauge",
  "x": 12.8,
  "y": 7.5
}
# ... unlimited points per area
```

## Robot Integration Example

When robot receives inspection request:

```python
# Robot receives this from server
response = {
    "coordinates": {
        "x": 5.2,
        "y": 3.8,
        "area_id": 1,
        "area_name": "Floor 1",
        "floor_number": 1,
        "point_name": "Gauge 1",
        "object_type": "gauge"
    }
}

# Robot can use area information for:
# 1. Navigation context
robot.navigate_to_floor(response["coordinates"]["floor_number"])
robot.navigate_to_area(response["coordinates"]["area_id"])
robot.navigate_to_position(
    x=response["coordinates"]["x"],
    y=response["coordinates"]["y"]
)

# 2. Logging and tracking
log.info(f"Navigating to {response['coordinates']['point_name']} "
         f"in {response['coordinates']['area_name']}")

# 3. Metadata for inspection job
metadata = {
    "area_id": response["coordinates"]["area_id"],
    "area_name": response["coordinates"]["area_name"],
    "floor": response["coordinates"]["floor_number"],
    "location": response["coordinates"]["point_name"]
}
```

## Testing Without Robot

The `populate_sample_data.py` script creates sample data:
- 3 areas (Floor 1, Floor 2, Basement)
- 10 inspection points across areas

This is ONLY for testing - users can create their own areas and points from the UI.

## Commands to Test

```powershell
# 1. Run migration
python migrate_add_locations.py

# 2. Populate sample data (optional - for testing only)
python populate_sample_data.py

# 3. Start server
python -m uvicorn app.main:app --host 0.0.0.0 --port 8001

# 4. Open UI and create your own areas/points
# Double-click location_ui.html
```

## Key Points

✅ **Unlimited areas** - create as many floors/zones as needed
✅ **Unlimited points per area** - no restrictions
✅ **Area info always sent** - robot knows context
✅ **Dynamic system** - nothing is hardcoded
✅ **Sample data is optional** - just for testing
