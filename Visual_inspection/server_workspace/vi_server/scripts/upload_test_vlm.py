"""Test client for VLM-based inspection tasks."""

import argparse
import json
import sys
import time
from pathlib import Path

import requests


def upload_vlm_task(
    server_url: str,
    image_path: str,
    object_type: str,
    inspection_prompt: str = None,
    metadata: dict = None
) -> dict:
    """
    Upload ROI image for VLM-based inspection.
    
    Args:
        server_url: Base URL of the server
        image_path: Path to ROI image file
        object_type: Object type (door/fire_extinguisher/emergency_exit/main_cylinder/unknown)
        inspection_prompt: Custom inspection prompt (for unknown tasks)
        metadata: Optional additional metadata
        
    Returns:
        Response JSON with job_id
    """
    url = f"{server_url}/api/v1/jobs"
    
    # Prepare metadata
    meta = metadata or {}
    if inspection_prompt:
        meta["inspection_prompt"] = inspection_prompt
    
    # Prepare files and data
    with open(image_path, "rb") as f:
        files = {"file": (Path(image_path).name, f, "image/jpeg")}
        
        data = {"object_type": object_type}
        if meta:
            data["metadata_json"] = json.dumps(meta)
        
        # Upload
        response = requests.post(url, files=files, data=data)
        response.raise_for_status()
        
    return response.json()


def get_job_status(server_url: str, job_id: str) -> dict:
    """Get job status and results."""
    url = f"{server_url}/api/v1/jobs/{job_id}"
    response = requests.get(url)
    response.raise_for_status()
    return response.json()


def poll_until_complete(
    server_url: str,
    job_id: str,
    poll_interval: float = 1.0,
    timeout: float = 300.0
) -> dict:
    """Poll job status until completion or timeout."""
    start_time = time.time()
    
    while True:
        elapsed = time.time() - start_time
        if elapsed > timeout:
            raise TimeoutError(f"Job did not complete within {timeout} seconds")
        
        job = get_job_status(server_url, job_id)
        status = job["status"]
        
        print(f"[{elapsed:.1f}s] Status: {status}")
        
        if status in ["DONE", "FAILED"]:
            return job
        
        time.sleep(poll_interval)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Upload ROI image for VLM-based inspection"
    )
    parser.add_argument(
        "--image",
        required=True,
        help="Path to ROI image file"
    )
    parser.add_argument(
        "--object-type",
        required=True,
        choices=["door", "fire_extinguisher", "emergency_exit", "main_cylinder", "unknown", "person"],
        help="Object type for VLM inspection"
    )
    parser.add_argument(
        "--inspection-prompt",
        help="Custom inspection prompt (for unknown tasks)"
    )
    parser.add_argument(
        "--server-url",
        default="http://localhost:8000",
        help="Server URL (default: http://localhost:8000)"
    )
    parser.add_argument(
        "--no-poll",
        action="store_true",
        help="Don't poll for results, just upload and exit"
    )
    
    args = parser.parse_args()
    
    # Validate image file exists
    if not Path(args.image).exists():
        print(f"Error: Image file not found: {args.image}", file=sys.stderr)
        sys.exit(1)
    
    # Validate inspection_prompt for unknown tasks
    if args.object_type == "unknown" and not args.inspection_prompt:
        print(
            "Warning: 'unknown' object_type without --inspection-prompt will use generic inspection",
            file=sys.stderr
        )
    
    try:
        # Upload image
        print(f"Uploading {args.image} for VLM inspection...")
        print(f"Object Type: {args.object_type}")
        if args.inspection_prompt:
            print(f"Custom Prompt: {args.inspection_prompt}")
        print()
        
        result = upload_vlm_task(
            args.server_url,
            args.image,
            args.object_type,
            args.inspection_prompt
        )
        
        job_id = result["job_id"]
        print(f"✓ Job created successfully!")
        print(f"  Job ID: {job_id}")
        print(f"  Status: {result['status']}")
        
        if args.no_poll:
            print(f"\nTo check status later, run:")
            print(f"  curl {args.server_url}/api/v1/jobs/{job_id}")
            return
        
        # Poll for results
        print(f"\nPolling for VLM results...")
        final_job = poll_until_complete(args.server_url, job_id)
        
        print(f"\n{'='*70}")
        print(f"VLM INSPECTION RESULT")
        print(f"{'='*70}")
        print(f"Status: {final_job['status']}")
        
        if final_job['status'] == "DONE":
            print(f"\n✓ Inspection completed!")
            if final_job.get('result_json'):
                result_data = json.loads(final_job['result_json'])
                
                print(f"\n📋 VLM Analysis:")
                print(f"  Task: {result_data.get('task', 'N/A')}")
                print(f"  Decision: {result_data.get('decision', 'N/A')}")
                print(f"  Confidence: {result_data.get('confidence', 0):.2%}")
                print(f"  Summary: {result_data.get('summary', 'N/A')}")
                
                if result_data.get('findings'):
                    print(f"\n🔍 Findings:")
                    for i, finding in enumerate(result_data['findings'], 1):
                        print(f"  {i}. {finding}")
                
                if result_data.get('evidence'):
                    print(f"\n📊 Evidence:")
                    for key, value in result_data['evidence'].items():
                        print(f"  {key}: {value}")
                
                if result_data.get('extracted_objects'):
                    print(f"\n🏷️  Detected Objects:")
                    print(f"  {', '.join(result_data['extracted_objects'])}")
                
                print(f"\n📄 Full Result (JSON):")
                print(json.dumps(result_data, indent=2))
        else:
            print(f"\n✗ Inspection failed!")
            if final_job.get('error_message'):
                print(f"Error: {final_job['error_message']}")
        
        print(f"\n🖼️  ROI Image URL: {args.server_url}/api/v1/jobs/{job_id}/roi")
        
    except requests.exceptions.ConnectionError:
        print(f"Error: Could not connect to server at {args.server_url}", file=sys.stderr)
        print(f"Make sure the server is running.", file=sys.stderr)
        sys.exit(1)
    except requests.exceptions.HTTPError as e:
        print(f"Error: HTTP {e.response.status_code}", file=sys.stderr)
        print(f"Response: {e.response.text}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
