"""
cloud_uploader.py — Uploads inspection results to the cloud PostgreSQL database.

image_url column : base64-encoded JPEG string (data:image/jpeg;base64,...)
analysis column  : full result.json serialised as a JSON string

Table: mission_results
Schema:
    id                  uint PK (auto-increment)
    created_at          timestamp
    updated_at          timestamp
    deleted_at          timestamp (nullable)
    mission_id          uint NOT NULL — FK → missions.id
    mission_waypoint_id uint NOT NULL — FK → mission_waypoints.id
    date                varchar NOT NULL (YYYY-MM-DD)
    image_url           varchar  (captured frame URL)
    success             varchar  ('yes' / 'no')
    analysis            text     (AI-generated inspection description)
    confidence          float64  (0.0 – 1.0)

The event_id sent by the robot IS the mission_id integer (e.g. "1" → 1).
The waypoint_id sent by the robot IS the mission_waypoint_id integer (e.g. "1" → 1).

Requirements:
    pip install psycopg2-binary

Connection string is read from .env → CLOUD_DB_URL
"""

import base64
import json
import logging
import os
from datetime import datetime, timezone

logger = logging.getLogger(__name__)

CLOUD_DB_URL = os.getenv(
    "CLOUD_DB_URL",
    "postgresql://go2_user:FWkKdFsYfFNIDmVxMHF9GRSsz78DjqY6"
    "@dpg-d7ogpcapmmbs73b5p7s0-a.singapore-postgres.render.com/go2_db_0hr5"
)


def _get_conn():
    try:
        import psycopg2
        return psycopg2.connect(CLOUD_DB_URL)
    except ImportError:
        raise RuntimeError(
            "psycopg2 not installed. Run:  pip install psycopg2-binary"
        )


def upload_result(
    event_id: str,
    waypoint_id: str,
    object_class: str,
    result: dict,
    image_path: str,
):
    """
    Upload one inspection result to the cloud mission_results table.

    Args:
        event_id:     numeric string → used directly as mission_id (e.g. "1")
        waypoint_id:  numeric string → used directly as mission_waypoint_id (e.g. "1")
        object_class: e.g. "gauge", "door", "fire_extinguisher"
        result:       the result dict from result.json
        image_path:   absolute local path to inspection_image.jpg
    """
    if not CLOUD_DB_URL:
        logger.info("CLOUD_DB_URL not set — skipping cloud upload")
        return

    # ── Resolve mission_id and mission_waypoint_id ──────────────────────────
    try:
        mission_id = int(event_id)
    except (ValueError, TypeError):
        logger.warning(
            f"[CLOUD] event_id '{event_id}' is not a numeric mission_id — skipping upload."
        )
        return

    try:
        mission_waypoint_id = int(waypoint_id)
    except (ValueError, TypeError):
        logger.warning(
            f"[CLOUD] waypoint_id '{waypoint_id}' is not a numeric mission_waypoint_id — skipping upload."
        )
        return

    try:
        conn = _get_conn()
        cur  = conn.cursor()

        # ── Map result fields to schema ─────────────────────────────────────
        decision    = str(result.get("decision", "UNKNOWN")).upper()
        success     = "yes" if decision == "PASS" else "no"
        confidence  = float(result.get("confidence", 0.0))
        date_str    = datetime.now().strftime("%Y-%m-%d")
        now         = datetime.now(timezone.utc)

        # Encode image as base64
        image_url = ""
        if image_path:
            try:
                with open(image_path, "rb") as img_f:
                    b64 = base64.b64encode(img_f.read()).decode("utf-8")
                image_url = f"data:image/jpeg;base64,{b64}"
            except Exception as img_err:
                logger.warning(f"[CLOUD] Could not encode image: {img_err}")

        # Full result JSON as the analysis string
        analysis = json.dumps(result, ensure_ascii=False)

        # ── UPSERT — update if (mission_id, mission_waypoint_id, date) already exists ─
        cur.execute(
            """
            INSERT INTO mission_results
                (created_at, updated_at, deleted_at,
                 mission_id, mission_waypoint_id,
                 date, image_url, success, analysis, confidence)
            VALUES
                (%s, %s, %s,
                 %s, %s,
                 %s, %s, %s, %s, %s)
            ON CONFLICT (mission_id, mission_waypoint_id, date)
            DO UPDATE SET
                updated_at   = EXCLUDED.updated_at,
                image_url    = EXCLUDED.image_url,
                success      = EXCLUDED.success,
                analysis     = EXCLUDED.analysis,
                confidence   = EXCLUDED.confidence
            """,
            (
                now, now, None,
                mission_id, mission_waypoint_id,
                date_str, image_url, success, analysis, confidence,
            ),
        )

        conn.commit()
        cur.close()
        conn.close()

        logger.info(
            f"[CLOUD] ✓ Uploaded: mission_id={mission_id} waypoint_id={mission_waypoint_id} "
            f"class={object_class} decision={decision} confidence={confidence:.2f}"
        )

    except Exception as e:
        logger.error(
            f"[CLOUD] Upload failed for mission_id={mission_id} waypoint_id={mission_waypoint_id}: {e}",
            exc_info=True
        )
