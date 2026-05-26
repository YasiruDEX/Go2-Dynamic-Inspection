"""
Test script for location management system.

This script tests all the location management endpoints.
"""

import requests
import json

BASE_URL = "http://localhost:8001/api/v1"


def test_location_system():
    """Test the complete location management system."""
    
    print("=" * 70)
    print("TESTING LOCATION MANAGEMENT SYSTEM")
    print("=" * 70)
    
    # Test 1: Create Area
    print("\n1. Creating test area...")
    area_data = {
        "name": "Test Floor 1",
        "description": "Test area for location management",
        "floor_number": 1,
        "map_width": 20.0,
        "map_height": 15.0,
        "origin_x": 0.0,
        "origin_y": 0.0
    }
    
    response = requests.post(f"{BASE_URL}/areas", json=area_data)
    if response.status_code == 201:
        area = response.json()
        area_id = area['id']
        print(f"   ✓ Area created: {area['name']} (ID: {area_id})")
    else:
        print(f"   ✗ Failed to create area: {response.text}")
        return
    
    # Test 2: List Areas
    print("\n2. Listing all areas...")
    response = requests.get(f"{BASE_URL}/areas")
    if response.ok:
        areas = response.json()
        print(f"   ✓ Found {len(areas)} area(s)")
        for a in areas:
            print(f"      - {a['name']} (Floor {a['floor_number']})")
    
    # Test 3: Create Inspection Points
    print("\n3. Creating inspection points...")
    points_data = [
        {
            "name": "Gauge 1",
            "object_type": "gauge",
            "x": 5.2,
            "y": 3.8,
            "z": 0.0,
            "orientation": 0.0,
            "notes": "Main pressure gauge"
        },
        {
            "name": "Fire Extinguisher 1",
            "object_type": "fire_extinguisher",
            "x": 8.5,
            "y": 2.1,
            "z": 0.0,
            "orientation": 1.57,
            "notes": "Near entrance"
        },
        {
            "name": "Emergency Exit",
            "object_type": "emergency_exit",
            "x": 12.0,
            "y": 7.5,
            "z": 0.0,
            "orientation": 3.14,
            "notes": "Main exit"
        }
    ]
    
    point_ids = []
    for point_data in points_data:
        response = requests.post(f"{BASE_URL}/areas/{area_id}/points", json=point_data)
        if response.status_code == 201:
            point = response.json()
            point_ids.append(point['id'])
            print(f"   ✓ Created: {point['name']} at ({point['x']}, {point['y']})")
        else:
            print(f"   ✗ Failed to create point: {response.text}")
    
    # Test 4: List Inspection Points
    print("\n4. Listing inspection points...")
    response = requests.get(f"{BASE_URL}/areas/{area_id}/points")
    if response.ok:
        points = response.json()
        print(f"   ✓ Found {len(points)} inspection point(s) in {area['name']}")
        for p in points:
            print(f"      - {p['name']} ({p['object_type']}) at ({p['x']}, {p['y']})")
    
    # Test 5: Request Coordinates
    print("\n5. Testing coordinate request...")
    response = requests.post(f"{BASE_URL}/areas/{area_id}/request-coordinates")
    if response.ok:
        data = response.json()
        print(f"   ✓ Coordinate request: {data['message']}")
    
    # Test 6: Save Coordinates (simulating robot response)
    print("\n6. Saving coordinates from robot...")
    coord_data = {
        "point_name": "Door 1",
        "object_type": "door",
        "x": 3.5,
        "y": 9.2,
        "z": 0.0,
        "orientation": 0.0,
        "notes": "Saved from robot position"
    }
    
    response = requests.post(f"{BASE_URL}/areas/{area_id}/save-coordinates", json=coord_data)
    if response.status_code == 201:
        point = response.json()
        print(f"   ✓ Saved: {point['name']} at ({point['x']}, {point['y']})")
    
    # Test 7: Inspect Known Location
    print("\n7. Testing known location inspection...")
    if point_ids:
        inspection_data = {
            "area_id": area_id,
            "point_id": point_ids[0]
        }
        
        response = requests.post(f"{BASE_URL}/inspections/known", json=inspection_data)
        if response.ok:
            result = response.json()
            print(f"   ✓ Inspection requested")
            print(f"      Status: {result['status']}")
            print(f"      Message: {result['message']}")
            print(f"      Coordinates: {json.dumps(result['coordinates'], indent=6)}")
    
    # Test 8: Inspect Ad-hoc Location
    print("\n8. Testing ad-hoc location inspection...")
    adhoc_data = {
        "area_id": area_id,
        "x": 10.5,
        "y": 5.3,
        "z": 0.0,
        "orientation": 0.0,
        "object_type": "unknown"
    }
    
    response = requests.post(f"{BASE_URL}/inspections/adhoc", json=adhoc_data)
    if response.ok:
        result = response.json()
        print(f"   ✓ Ad-hoc inspection requested")
        print(f"      Status: {result['status']}")
        print(f"      Message: {result['message']}")
        print(f"      Coordinates: {json.dumps(result['coordinates'], indent=6)}")
    
    # Test 9: Update Inspection Point
    print("\n9. Testing inspection point update...")
    if point_ids:
        update_data = {
            "x": 5.5,
            "y": 4.0,
            "notes": "Updated coordinates"
        }
        
        response = requests.put(
            f"{BASE_URL}/areas/{area_id}/points/{point_ids[0]}",
            json=update_data
        )
        if response.ok:
            point = response.json()
            print(f"   ✓ Updated: {point['name']} to ({point['x']}, {point['y']})")
    
    # Test 10: Get Area Details
    print("\n10. Getting area details...")
    response = requests.get(f"{BASE_URL}/areas/{area_id}")
    if response.ok:
        area_details = response.json()
        print(f"   ✓ Area: {area_details['name']}")
        print(f"      Floor: {area_details['floor_number']}")
        print(f"      Map size: {area_details['map_width']}m x {area_details['map_height']}m")
        print(f"      Created: {area_details['created_at']}")
    
    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 70)
    print("\nNext steps:")
    print("1. Open location_ui.html in your browser")
    print("2. Test the UI with the created area and points")
    print("3. Try clicking on the map for ad-hoc inspections")
    print("=" * 70)


if __name__ == "__main__":
    try:
        test_location_system()
    except requests.exceptions.ConnectionError:
        print("\n✗ ERROR: Could not connect to server!")
        print("   Make sure the server is running:")
        print("   python -m uvicorn app.main:app --host 0.0.0.0 --port 8001")
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
