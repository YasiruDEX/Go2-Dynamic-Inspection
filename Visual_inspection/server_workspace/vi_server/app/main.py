"""Main FastAPI application — Visual Inspection Server."""

from datetime import datetime

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.watcher import start_watcher
from app.processor import enqueue, start_worker, stop_worker
from app.utils.logging import setup_logging, get_logger

setup_logging()
logger = get_logger(__name__)

app = FastAPI(
    title="Visual Inspection Server",
    description="Watches Local_database and runs gauge/VLM pipelines automatically.",
    version="2.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup():
    logger.info("Starting Visual Inspection Server...")
    await start_worker()
    start_watcher(enqueue)
    logger.info("Server ready — watching Local_database for new inspections")


@app.on_event("shutdown")
async def shutdown():
    await stop_worker()
    logger.info("Server stopped")


@app.get("/api/v1/health")
async def health():
    return {"status": "ok", "timestamp": datetime.utcnow().isoformat()}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
