"""
Gauge reading pipeline wrapper.

Integrates the analog_gauge_reader pipeline for gauge reading tasks.
Uses the gauge pipeline's own virtual environment which has all dependencies.
"""

import logging
import os
import subprocess
import sys
import json
from pathlib import Path
from typing import Dict

logger = logging.getLogger(__name__)

# Paths
GAUGE_DIR = Path(__file__).parent / "gauge"
GAUGE_VENV_PYTHON = GAUGE_DIR / ".venv" / "Scripts" / "python.exe"


def run_gauge_pipeline(roi_path: str) -> Dict:
    """
    Run analog gauge reading pipeline on ROI image.
    
    Uses the gauge pipeline's own virtual environment to avoid dependency conflicts.
    
    Args:
        roi_path: Absolute path to ROI image file
        
    Returns:
        Dictionary with gauge reading result:
        {
            "reading": float,      # Gauge reading value
            "unit": str,           # Unit of measurement (e.g., "bar", "psi")
            "confidence": float,   # Confidence score 0-1 (estimated)
            "method": str          # Pipeline identifier
        }
    """
    logger.info(f"Running analog gauge reader on: {roi_path}")
    
    try:
        # Verify gauge venv exists
        if not GAUGE_VENV_PYTHON.exists():
            raise FileNotFoundError(
                f"Gauge virtual environment not found: {GAUGE_VENV_PYTHON}\n"
                f"Please ensure the gauge pipeline's .venv is set up."
            )
        
        # Verify ROI image exists
        roi_path_abs = Path(roi_path).absolute()
        if not roi_path_abs.exists():
            raise FileNotFoundError(f"ROI image not found: {roi_path}")
        
        # Set up model paths
        detection_model = str(GAUGE_DIR / "models" / "gauge_detection_model.pt")
        key_point_model = str(GAUGE_DIR / "models" / "key_point_model.pt")
        segmentation_model = str(GAUGE_DIR / "models" / "segmentation_model.pt")
        
        # Verify model files exist
        for model_path in [detection_model, key_point_model, segmentation_model]:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # Create unique run path (Plotter will create the final directory)
        import uuid
        run_id = str(uuid.uuid4())[:8]
        runs_dir = GAUGE_DIR / "runs"
        runs_dir.mkdir(exist_ok=True)  # Create parent runs directory
        run_path = runs_dir / f"server_run_{run_id}"
        
        # Create a wrapper script to call the pipeline
        # Important: Add gauge dir to sys.path before importing
        wrapper_script = f"""
import sys
import os
import json

# Add gauge directory to Python path
gauge_dir = r"{GAUGE_DIR.absolute()}"
sys.path.insert(0, gauge_dir)

# Change to gauge directory (for relative imports)
os.chdir(gauge_dir)

# Import pipeline
from pipeline import process_image

# Run pipeline
try:
    result = process_image(
        image=r"{roi_path_abs}",
        detection_model_path=r"{detection_model}",
        key_point_model_path=r"{key_point_model}",
        segmentation_model_path=r"{segmentation_model}",
        run_path=r"{run_path}",
        debug=False,
        eval_mode=False,
        image_is_raw=False
    )
    
    # Output result as JSON
    print(json.dumps(result))
except Exception as e:
    import traceback
    error_info = {{
        "error": str(e),
        "traceback": traceback.format_exc()
    }}
    print(json.dumps(error_info), file=sys.stderr)
    sys.exit(1)
"""
        
        # Create temp directory for wrapper script (separate from run_path)
        import tempfile
        temp_dir = Path(tempfile.mkdtemp(prefix="gauge_wrapper_"))
        
        # Write wrapper script to temp file
        wrapper_file = temp_dir / "run_pipeline.py"
        with open(wrapper_file, "w") as f:
            f.write(wrapper_script)
        
        # Run the pipeline using gauge's venv
        logger.info(f"Executing gauge pipeline with venv: {GAUGE_VENV_PYTHON}")
        
        result = subprocess.run(
            [str(GAUGE_VENV_PYTHON), str(wrapper_file)],
            capture_output=True,
            text=True,
            timeout=180  # 180 second timeout (first run takes longer)
        )
        
        # Check for errors
        if result.returncode != 0:
            error_msg = result.stderr or "Unknown error"
            logger.error(f"Gauge pipeline failed: {error_msg}")
            
            # Try to parse error JSON
            try:
                error_data = json.loads(error_msg)
                error_text = error_data.get("error", error_msg)
            except:
                error_text = error_msg
            
            return {
                "reading": None,
                "unit": None,
                "confidence": 0.0,
                "method": "analog_gauge_reader",
                "error": error_text
            }
        
        # Parse result - extract JSON from last line (there may be logging output before it)
        try:
            # Try to find JSON in the output
            output_lines = result.stdout.strip().split('\n')
            json_output = None
            
            # Search from the end for a line that looks like JSON
            for line in reversed(output_lines):
                line = line.strip()
                if line.startswith('{') and line.endswith('}'):
                    json_output = line
                    break
            
            if json_output is None:
                raise ValueError("No JSON output found in pipeline output")
            
            pipeline_result = json.loads(json_output)
        except (json.JSONDecodeError, ValueError) as e:
            logger.error(f"Failed to parse pipeline output: {e}")
            logger.error(f"Output was: {result.stdout}")
            return {
                "reading": None,
                "unit": None,
                "confidence": 0.0,
                "method": "analog_gauge_reader",
                "error": f"Invalid JSON output: {str(e)}"
            }
        
        # Check for error in result
        if "error" in pipeline_result:
            logger.error(f"Pipeline returned error: {pipeline_result['error']}")
            return {
                "reading": None,
                "unit": None,
                "confidence": 0.0,
                "method": "analog_gauge_reader",
                "error": pipeline_result["error"]
            }
        
        # Extract reading and unit
        # pipeline.py line 604 returns: {"value": reading, "unit": unit}
        reading = pipeline_result.get("value")   # ← "value" not "reading"
        unit = pipeline_result.get("unit")
        
        # Estimate confidence (default for successful reading)
        confidence = 0.85
        
        # Format response
        response = {
            "reading": float(reading) if reading is not None else None,
            "unit": str(unit) if unit is not None else "unknown",
            "confidence": confidence,
            "method": "analog_gauge_reader"
        }
        
        logger.info(f"Gauge reading successful: {response}")
        return response
        
    except subprocess.TimeoutExpired:
        logger.error("Gauge pipeline timed out after 180 seconds")
        return {
            "reading": None,
            "unit": None,
            "confidence": 0.0,
            "method": "analog_gauge_reader",
            "error": "Pipeline execution timed out after 180 seconds"
        }
    
    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
        return {
            "reading": None,
            "unit": None,
            "confidence": 0.0,
            "method": "analog_gauge_reader",
            "error": str(e)
        }
    
    except Exception as e:
        logger.error(f"Gauge pipeline failed: {e}", exc_info=True)
        return {
            "reading": None,
            "unit": None,
            "confidence": 0.0,
            "method": "analog_gauge_reader",
            "error": str(e)
        }
