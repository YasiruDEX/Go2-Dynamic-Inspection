"""Seed sample jobs into the database for testing."""

import asyncio
import json
import uuid
from datetime import datetime

from sqlalchemy import select

from app.db import AsyncSessionLocal, init_db
from app.models import Job


async def seed_sample_jobs():
    """Create sample jobs in the database."""
    await init_db()
    
    async with AsyncSessionLocal() as db:
        # Check if we already have jobs
        result = await db.execute(select(Job))
        existing_jobs = result.scalars().all()
        
        if existing_jobs:
            print(f"Database already has {len(existing_jobs)} jobs. Skipping seed.")
            return
        
        # Create sample jobs
        sample_jobs = [
            Job(
                job_id=str(uuid.uuid4()),
                object_type="gauge",
                status="DONE",
                roi_filename="sample/gauge.jpg",
                result_json=json.dumps({
                    "reading": "3.5",
                    "confidence": 0.95,
                    "method": "gauge_pipeline"
                }),
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            ),
            Job(
                job_id=str(uuid.uuid4()),
                object_type="door",
                status="DONE",
                roi_filename="sample/door.jpg",
                result_json=json.dumps({
                    "message": "VLM not implemented",
                    "object_type": "door",
                    "method": "vlm_stub"
                }),
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            ),
            Job(
                job_id=str(uuid.uuid4()),
                object_type="gauge",
                status="FAILED",
                roi_filename="sample/bad_gauge.jpg",
                error_message="Image quality too low",
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            ),
        ]
        
        for job in sample_jobs:
            db.add(job)
        
        await db.commit()
        print(f"✓ Seeded {len(sample_jobs)} sample jobs")


if __name__ == "__main__":
    asyncio.run(seed_sample_jobs())
