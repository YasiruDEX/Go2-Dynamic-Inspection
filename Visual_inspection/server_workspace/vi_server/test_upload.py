import requests
import time
import json

# Image path
IMAGE_PATH = r"E:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\vi_server\app\pipelines\gauge\test_images\original_image.jpg"

print("Uploading gauge image...")

# Upload with explicit content type
files = {'file': ('gauge.jpg', open(IMAGE_PATH, 'rb'), 'image/jpeg')}
data = {
    'object_type': 'gauge',
    'metadata_json': json.dumps({
        'robot_id': 'test_robot',
        'location': 'test_area',
        'timestamp': '2026-01-08T08:00:00'
    })
}

response = requests.post('http://localhost:8001/api/v1/jobs', files=files, data=data)

print(f"Status Code: {response.status_code}")
print(f"Response: {response.text}")

if response.status_code == 200:
    result = response.json()
    job_id = result['job_id']
    print(f"\n✓ Job created: {job_id}")
    print(f"✓ Status: {result['status']}")

    # Wait for processing
    print("\nWaiting for processing (90 seconds)...")
    for i in range(9):
        time.sleep(10)
        print(f"  {(i+1)*10} seconds...")

    # Get result
    print("\nFetching result...")
    response = requests.get(f'http://localhost:8001/api/v1/jobs/{job_id}')
    job_data = response.json()

    print("\n" + "="*60)
    print("GAUGE READING RESULT")
    print("="*60)
    print(f"Job ID: {job_data['job_id']}")
    print(f"Status: {job_data['status']}")
    print(f"Object Type: {job_data['object_type']}")
    print(f"Created: {job_data['created_at']}")
    
    if job_data['result_json']:
        result = json.loads(job_data['result_json'])
        print(f"\n📊 Gauge Reading:")
        print(f"   Reading: {result.get('reading')}")
        print(f"   Unit: {result.get('unit')}")
        print(f"   Confidence: {result.get('confidence')}")
        print(f"   Method: {result.get('method')}")
    else:
        print(f"\n⚠ No result yet. Status: {job_data['status']}")
        if job_data.get('error_message'):
            print(f"   Error: {job_data['error_message']}")
    
    print(f"\n📁 Image stored at: ./data/jobs/{job_data['roi_filename']}")
    print(f"📋 Metadata: {job_data['metadata_json']}")
    print("="*60)
else:
    print("\n✗ Upload failed!")
    print(f"Error: {response.text}")