"""
Universal Test Script - Simulates Jetson Sending Data to Server

This script tests the complete flow:
1. Jetson sends: Image + object_type + metadata (JSON)
2. Server receives and stores in database
3. Server routes to correct pipeline (Gauge or VLM)
4. Result is stored in database
5. UI/Jetson can retrieve result

Usage:
    python test_server.py --image path/to/image.jpg --type gauge
    python test_server.py --image path/to/image.jpg --type door
    python test_server.py --image path/to/image.jpg --type fire_extinguisher
"""

import requests
import json
import time
import argparse

SERVER_URL = "http://localhost:8001"

def test_inspection(image_path, object_type, custom_prompt=None):
    """
    Test the complete inspection flow.
    
    Args:
        image_path: Path to image file
        object_type: Type of object (gauge, door, fire_extinguisher, etc.)
        custom_prompt: Custom inspection prompt for 'unknown' type
    """
    print("="*70)
    print(f"TESTING: {object_type.upper()} INSPECTION")
    print("="*70)
    
    # Step 1: Prepare data (simulating Jetson)
    print("\nStep 1: Preparing data (simulating Jetson)...")
    
    metadata = {
        'robot_id': 'test_robot_01',
        'location': 'pg semina room',
        'timestamp': '2026-01-08T11:00:00'
    }
    
    # Add custom prompt for unknown type
    if object_type == 'unknown' and custom_prompt:
        metadata['inspection_prompt'] = custom_prompt
    
    print(f"   Image: {image_path}")
    print(f"   Object Type: {object_type}")
    print(f"   Metadata: {json.dumps(metadata, indent=6)}")
    
    # Step 2: Upload to server
    print("\nStep 2: Sending to server...")
    
    try:
        files = {'file': (f'{object_type}.jpg', open(image_path, 'rb'), 'image/jpeg')}
        data = {
            'object_type': object_type,
            'metadata_json': json.dumps(metadata)
        }
        
        response = requests.post(f'{SERVER_URL}/api/v1/jobs', files=files, data=data)
        
        if response.status_code != 200:
            print(f"    Upload failed!")
            print(f"   Status: {response.status_code}")
            print(f"   Error: {response.text}")
            return
        
        result = response.json()
        job_id = result['job_id']
        print(f"    Job created: {job_id}")
        print(f"   Status: {result['status']}")
        
    except Exception as e:
        print(f"    Error: {e}")
        return
    
    # Step 3: Wait for processing
    print("\n Step 3: Processing...")
    
    # Gauge takes ~60-90 seconds, VLM takes ~30-60 seconds
    wait_time = 90 if object_type == 'gauge' else 60
    
    print(f"   Waiting {wait_time} seconds for {object_type} pipeline...")
    for i in range(wait_time // 10):
        time.sleep(10)
        print(f"   {(i+1)*10}s elapsed...")
    
    # Step 4: Retrieve result
    print("\n Step 4: Retrieving result from database...")
    
    try:
        response = requests.get(f'{SERVER_URL}/api/v1/jobs/{job_id}')
        job_data = response.json()
        
        print("\n" + "="*70)
        print("RESULT FROM DATABASE")
        print("="*70)
        
        print(f"\n Job Information:")
        print(f"   Job ID: {job_data['job_id']}")
        print(f"   Status: {job_data['status']}")
        print(f"   Object Type: {job_data['object_type']}")
        print(f"   Created: {job_data['created_at']}")
        print(f"   Updated: {job_data.get('updated_at', 'N/A')}")
        
        print(f"\n Storage:")
        print(f"   Image: ./data/jobs/{job_data['roi_filename']}")
        print(f"   Metadata: {job_data['metadata_json']}")
        
        # Display result based on type
        if job_data['status'] == 'DONE' and job_data['result_json']:
            result = json.loads(job_data['result_json'])
            
            print(f"\n Inspection Result:")
            
            if object_type == 'gauge':
                # Gauge result format
                print(f"   Reading: {result.get('reading')}")
                print(f"   Unit: {result.get('unit')}")
                print(f"   Confidence: {result.get('confidence')}")
                print(f"   Method: {result.get('method')}")
            else:
                # VLM result format
                print(f"   Decision: {result.get('decision')}")
                print(f"   Confidence: {result.get('confidence')}")
                print(f"   Method: {result.get('method', 'vlm_reasoning')}")
                
                if 'summary' in result:
                    print(f"\n   Summary: {result['summary']}")
                
                if 'findings' in result and result['findings']:
                    print(f"\n   Findings:")
                    for finding in result['findings']:
                        print(f"      - {finding}")
                
                if 'evidence' in result:
                    print(f"\n   Evidence:")
                    for key, value in result['evidence'].items():
                        print(f"      {key}: {value}")
                
                if 'extracted_objects' in result and result['extracted_objects']:
                    print(f"\n   Detected Objects: {', '.join(result['extracted_objects'])}")
                
                # Show raw VLM response
                if 'vlm_raw_response' in result:
                    print(f"\n   Raw VLM Response:")
                    print(f"   {'-'*60}")
                    print(f"   {result['vlm_raw_response']}")
                    print(f"   {'-'*60}")
                
                if 'reasoning' in result:
                    print(f"\n   Reasoning: {result['reasoning']}")
        
        elif job_data['status'] == 'FAILED':
            print(f"\n Processing Failed:")
            print(f"   Error: {job_data.get('error_message', 'Unknown error')}")
        
        else:
            print(f"\n Status: {job_data['status']}")
            print(f"   Still processing or queued...")
        
        print("\n" + "="*70)
        
    except Exception as e:
        print(f"    Error retrieving result: {e}")

def main():
    parser = argparse.ArgumentParser(description='Test visual inspection server')
    parser.add_argument('--image', required=True, help='Path to image file')
    parser.add_argument('--type', required=True, 
                       choices=['gauge', 'door', 'fire_extinguisher', 'emergency_exit', 
                               'main_cylinder', 'unknown'],
                       help='Object type to inspect')
    parser.add_argument('--prompt', help='Custom inspection prompt (for unknown type)')
    
    args = parser.parse_args()
    
    
    test_inspection(args.image, args.type, args.prompt)

if __name__ == '__main__':
    main()
