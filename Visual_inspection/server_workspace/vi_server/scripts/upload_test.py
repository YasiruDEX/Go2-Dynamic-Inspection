"""Test client for uploading ROI images and polling results."""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional

import requests


def upload_image(
    server_url: str,
    image_path: str,
    object_type: str,
    metadata: Optional[dict] = None
) -> dict:
    """
    Upload ROI image to server.
    
    Args:
        server_url: Base URL of the server
        image_path: Path to ROI image file
        object_type: Object type (gauge/door/fire_extinguisher/unknown)
        metadata: Optional metadata dictionary
        
    Returns:
        Response JSON with job_id
    """
    url = f"{server_url}/api/v1/jobs"
    
    # Prepare files
    with open(image_path, "rb") as f:
        files = {"file": (Path(image_path).name, f, "image/jpeg")}
        
        # Prepare form data
        data = {"object_type": object_type}
        if metadata:
            data["metadata_json"] = json.dumps(metadata)
        
        # Upload
        response = requests.post(url, files=files, data=data)
        response.raise_for_status()
        
    return response.json()


def get_job_status(server_url: str, job_id: str) -> dict:
    """
    Get job status and results.
    
    Args:
        server_url: Base URL of the server
        job_id: Job identifier
        
    Returns:
        Job details JSON
    """
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
    """
    Poll job status until completion or timeout.
    
    Args:
        server_url: Base URL of the server
        job_id: Job identifier
        poll_interval: Seconds between polls
        timeout: Maximum time to wait
        
    Returns:
        Final job details
    """
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
    parser = argparse.ArgumentParser(description="Upload ROI image for inspection")
    parser.add_argument(
        "--image",
        required=True,
        help="Path to ROI image file"
    )
    parser.add_argument(
        "--object-type",
        required=True,
        choices=["gauge", "door", "fire_extinguisher", "unknown"],
        help="Object type"
    )
    parser.add_argument(
        "--server-url",
        default="http://localhost:8000",
        help="Server URL (default: http://localhost:8000)"
    )
    parser.add_argument(
        "--metadata",
        help="Additional metadata as JSON string"
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
    
    # Parse metadata if provided
    metadata = None
    if args.metadata:
        try:
            metadata = json.loads(args.metadata)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid metadata JSON: {e}", file=sys.stderr)
            sys.exit(1)
    
    try:
        # Upload image
        print(f"Uploading {args.image} as {args.object_type}...")
        result = upload_image(
            args.server_url,
            args.image,
            args.object_type,
            metadata
        )
        
        job_id = result["job_id"]
        print(f"\n✓ Job created successfully!")
        print(f"  Job ID: {job_id}")
        print(f"  Status: {result['status']}")
        
        if args.no_poll:
            print(f"\nTo check status later, run:")
            print(f"  curl {args.server_url}/api/v1/jobs/{job_id}")
            return
        
        # Poll for results
        print(f"\nPolling for results...")
        final_job = poll_until_complete(args.server_url, job_id)
        
        print(f"\n{'='*60}")
        print(f"FINAL RESULT")
        print(f"{'='*60}")
        print(f"Status: {final_job['status']}")
        
        if final_job['status'] == "DONE":
            print(f"\n✓ Job completed successfully!")
            if final_job.get('result_json'):
                result_data = json.loads(final_job['result_json'])
                print(f"\nResult:")
                print(json.dumps(result_data, indent=2))
        else:
            print(f"\n✗ Job failed!")
            if final_job.get('error_message'):
                print(f"Error: {final_job['error_message']}")
        
        print(f"\nROI Image URL: {args.server_url}/api/v1/jobs/{job_id}/roi")
        
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
