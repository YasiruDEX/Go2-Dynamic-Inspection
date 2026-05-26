# Quick Start Guide - Location Management System

## Prerequisites
- Server running on Windows
- Python virtual environment activated
- Database initialized

## Step-by-Step Setup

### 1. Activate Virtual Environment
```powershell
cd "e:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\vi_server"
.\venv\Scripts\activate
```

### 2. Run Database Migration
```powershell
python migrate_add_locations.py
```

**Expected Output:**
```
Starting database migration...
Migration completed successfully!

New tables created:
  - areas
  - inspection_points

Existing tables updated:
  - jobs (added inspection_point_id column)
```

### 3. Start the Server
```powershell
python -m uvicorn app.main:app --host 0.0.0.0 --port 8001
```

**Expected Output:**
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8001
```

### 4. Test the API (New Terminal)
```powershell
# Open new terminal
cd "e:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\vi_server"
.\venv\Scripts\activate
python test_location_system.py
```

**Expected Output:**
```
======================================================================
TESTING LOCATION MANAGEMENT SYSTEM
======================================================================

1. Creating test area...
   ✓ Area created: Test Floor 1 (ID: 1)

2. Listing all areas...
   ✓ Found 1 area(s)
      - Test Floor 1 (Floor 1)

3. Creating inspection points...
   ✓ Created: Gauge 1 at (5.2, 3.8)
   ✓ Created: Fire Extinguisher 1 at (8.5, 2.1)
   ✓ Created: Emergency Exit at (12.0, 7.5)

... (more tests)

ALL TESTS COMPLETED SUCCESSFULLY!
```

### 5. Open Test UI
```
1. Open your web browser
2. Navigate to: file:///e:/sem7/FYP/Main%20repo%20github%20upload/Visual_inspection/server_workspace/vi_server/location_ui.html
3. Or simply double-click location_ui.html
```

## Using the UI

### Create an Area
1. Click "Create New Area"
2. Enter name (e.g., "Floor 1")
3. Enter floor number (e.g., 1)
4. Area appears in dropdown

### Add Inspection Points
1. Select area from dropdown
2. Click "Request Robot Position"
3. Enter point name (e.g., "Gauge 1")
4. Enter object type (e.g., "gauge")
5. Point appears on map and in list

### Inspect Known Location
1. Select area
2. Click on a point marker on the map
   OR
   Click on a point in the list
3. Check console for navigation coordinates

### Inspect Ad-hoc Location
1. Select area
2. Click anywhere on the map
3. Coordinates appear below
4. Click "Inspect Location"
5. Check console for navigation coordinates

## API Examples

### Create Area
```bash
curl -X POST http://localhost:8001/api/v1/areas \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Floor 1",
    "floor_number": 1,
    "map_width": 20.0,
    "map_height": 15.0
  }'
```

### Add Inspection Point
```bash
curl -X POST http://localhost:8001/api/v1/areas/1/points \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Gauge 1",
    "object_type": "gauge",
    "x": 5.2,
    "y": 3.8
  }'
```

### Inspect Known Location
```bash
curl -X POST http://localhost:8001/api/v1/inspections/known \
  -H "Content-Type: application/json" \
  -d '{
    "area_id": 1,
    "point_id": 1
  }'
```

### Inspect Ad-hoc Location
```bash
curl -X POST http://localhost:8001/api/v1/inspections/adhoc \
  -H "Content-Type: application/json" \
  -d '{
    "area_id": 1,
    "x": 10.5,
    "y": 5.3,
    "object_type": "unknown"
  }'
```

## Troubleshooting

### Server won't start
- Check if port 8001 is already in use
- Kill existing process: `netstat -ano | findstr :8001` then `taskkill /F /PID <PID>`

### Migration fails
- Make sure virtual environment is activated
- Check database file exists: `data/vi_server.db`
- Delete database and restart server to recreate

### UI can't connect
- Verify server is running on port 8001
- Check browser console for errors
- Make sure CORS is enabled (already configured)

### Test script fails
- Ensure server is running first
- Check API_BASE URL in test script matches server

## What's Next?

1. **Integrate with Robot**: Modify robot code to call these endpoints
2. **Add Map Images**: Upload actual floor plan images
3. **ROS Services**: Convert to ROS services when ready
4. **Analytics**: Add inspection history and reporting

## Documentation

- **LOCATION_MANAGEMENT.md**: Complete system documentation
- **IMPLEMENTATION_SUMMARY.md**: What was implemented
- **QUICKSTART.md**: This file

## Support

Check the logs for detailed error messages:
- Server logs appear in terminal where uvicorn is running
- Browser console (F12) for UI errors
- Test script output for API errors

---

**Status**: Ready for testing! 🚀
