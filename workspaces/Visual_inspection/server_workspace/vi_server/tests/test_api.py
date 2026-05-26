"""Basic API tests for the visual inspection server."""

import json
from io import BytesIO

import pytest
from fastapi.testclient import TestClient
from PIL import Image

from app.main import app

client = TestClient(app)


def create_test_image() -> BytesIO:
    """Create a simple test image in memory."""
    img = Image.new("RGB", (100, 100), color="red")
    img_bytes = BytesIO()
    img.save(img_bytes, format="JPEG")
    img_bytes.seek(0)
    return img_bytes


def test_health_check():
    """Test health check endpoint."""
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "ok"
    assert "timestamp" in data
    assert data["version"] == "0.1.0"


def test_create_job_gauge():
    """Test creating a gauge inspection job."""
    img_bytes = create_test_image()
    
    response = client.post(
        "/api/v1/jobs",
        files={"file": ("test_gauge.jpg", img_bytes, "image/jpeg")},
        data={"object_type": "gauge"}
    )
    
    assert response.status_code == 200
    data = response.json()
    assert data["accepted"] is True
    assert "job_id" in data
    assert data["status"] == "QUEUED"


def test_create_job_with_metadata():
    """Test creating a job with metadata."""
    img_bytes = create_test_image()
    metadata = {"location": "Room A", "camera_id": "cam_01"}
    
    response = client.post(
        "/api/v1/jobs",
        files={"file": ("test.jpg", img_bytes, "image/jpeg")},
        data={
            "object_type": "door",
            "metadata_json": json.dumps(metadata)
        }
    )
    
    assert response.status_code == 200
    data = response.json()
    assert data["accepted"] is True


def test_create_job_invalid_object_type():
    """Test creating a job with invalid object type."""
    img_bytes = create_test_image()
    
    response = client.post(
        "/api/v1/jobs",
        files={"file": ("test.jpg", img_bytes, "image/jpeg")},
        data={"object_type": "invalid_type"}
    )
    
    assert response.status_code == 400
    assert "Invalid object_type" in response.json()["detail"]


def test_create_job_invalid_file_type():
    """Test creating a job with invalid file type."""
    response = client.post(
        "/api/v1/jobs",
        files={"file": ("test.txt", BytesIO(b"not an image"), "text/plain")},
        data={"object_type": "gauge"}
    )
    
    assert response.status_code == 400


def test_list_jobs():
    """Test listing jobs."""
    response = client.get("/api/v1/jobs")
    assert response.status_code == 200
    data = response.json()
    assert "total" in data
    assert "jobs" in data
    assert isinstance(data["jobs"], list)


def test_list_jobs_with_filters():
    """Test listing jobs with filters."""
    response = client.get("/api/v1/jobs?status=QUEUED&limit=10")
    assert response.status_code == 200
    data = response.json()
    assert data["limit"] == 10


def test_get_job_not_found():
    """Test getting a non-existent job."""
    response = client.get("/api/v1/jobs/nonexistent-job-id")
    assert response.status_code == 404


def test_get_roi_not_found():
    """Test getting ROI for non-existent job."""
    response = client.get("/api/v1/jobs/nonexistent-job-id/roi")
    assert response.status_code == 404


@pytest.mark.asyncio
async def test_job_processing_flow():
    """Test complete job processing flow (integration test)."""
    import asyncio
    
    # Create job
    img_bytes = create_test_image()
    response = client.post(
        "/api/v1/jobs",
        files={"file": ("test_gauge.jpg", img_bytes, "image/jpeg")},
        data={"object_type": "gauge"}
    )
    
    assert response.status_code == 200
    job_id = response.json()["job_id"]
    
    # Wait for processing (with timeout)
    max_wait = 10  # seconds
    wait_interval = 0.5
    elapsed = 0
    
    while elapsed < max_wait:
        response = client.get(f"/api/v1/jobs/{job_id}")
        assert response.status_code == 200
        job_data = response.json()
        
        if job_data["status"] in ["DONE", "FAILED"]:
            # Job completed
            assert job_data["status"] == "DONE"
            assert job_data["result_json"] is not None
            
            # Parse result
            result = json.loads(job_data["result_json"])
            assert "reading" in result or "message" in result
            break
        
        await asyncio.sleep(wait_interval)
        elapsed += wait_interval
    else:
        pytest.fail(f"Job did not complete within {max_wait} seconds")
