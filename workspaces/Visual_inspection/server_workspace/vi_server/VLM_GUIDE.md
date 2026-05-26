# VLM-Based Reasoning Pipeline - Complete Guide

## Overview

The VLM (Vision Language Model) pipeline enables intelligent visual inspection using AI models. It supports multiple inspection tasks with customizable prompts and structured outputs.

---

## Supported Object Types

| Object Type | Description | Evidence Fields |
|------------|-------------|-----------------|
| **fire_extinguisher** | Check fire extinguisher presence and accessibility | `present`, `blocked` |
| **door** | Determine door status (open/closed) | `open` |
| **emergency_exit** | Check if emergency exit is blocked | `blocked` |
| **main_cylinder** | Detect oil leaks near main cylinder/splicer | `oil_leak` |
| **unknown** | Custom inspection with user-defined prompt | Flexible |
| **gauge** | Gauge reading (uses separate gauge pipeline) | N/A |

---

## Configuration

### 1. VLM Provider Settings

Edit `.env` file:

```bash
# VLM Configuration
VLM_PROVIDER=stub          # Options: "stub" or "openai"
VLM_API_KEY=               # Your OpenAI API key (required for openai provider)
VLM_MODEL=gpt-4o           # Model name (e.g., gpt-4o, gpt-4-vision-preview)
```

### 2. Provider Options

#### **Stub Provider** (Default - Offline Testing)
- No API key required
- Returns deterministic placeholder results
- Perfect for development and testing
- No cost

```bash
VLM_PROVIDER=stub
```

#### **OpenAI Provider** (Production)
- Requires OpenAI API key
- Uses GPT-4 Vision models
- Real AI-powered analysis
- Costs apply per API call

```bash
VLM_PROVIDER=openai
VLM_API_KEY=sk-your-api-key-here
VLM_MODEL=gpt-4o
```

---

## File Structure

```
app/pipelines/vlm/
├── __init__.py              # Package exports
├── vlm_router.py            # Main entry point
├── vlm_client.py            # Provider abstraction (stub/OpenAI)
├── schemas.py               # Pydantic models for outputs
├── postprocess.py           # Response validation & normalization
└── prompts/                 # Task-specific prompts
    ├── base_rules.md        # Common inspection guidelines
    ├── door.yaml            # Door inspection prompt
    ├── fire_extinguisher.yaml
    ├── emergency_exit.yaml
    ├── main_cylinder.yaml
    └── unknown.yaml         # Generic/custom inspection
```

---

## Usage Examples

### 1. Fire Extinguisher Inspection

```bash
python scripts/upload_test_vlm.py \
  --image path/to/fire_extinguisher.jpg \
  --object-type fire_extinguisher
```

**Expected Output:**
```json
{
  "task": "fire_extinguisher",
  "decision": "PASS",
  "confidence": 0.85,
  "summary": "Fire extinguisher is present and accessible",
  "findings": [
    "Red cylindrical extinguisher visible on wall",
    "No obstructions blocking access"
  ],
  "evidence": {
    "present": true,
    "blocked": false
  },
  "extracted_objects": ["fire extinguisher", "wall mount"]
}
```

### 2. Door Status Check

```bash
python scripts/upload_test_vlm.py \
  --image path/to/door.jpg \
  --object-type door
```

**Expected Output:**
```json
{
  "task": "door",
  "decision": "PASS",
  "confidence": 0.78,
  "summary": "Door appears closed",
  "findings": [
    "Door panel aligned with frame",
    "No visible gap between door and frame"
  ],
  "evidence": {
    "open": false
  },
  "extracted_objects": ["door", "frame"]
}
```

### 3. Emergency Exit Blockage

```bash
python scripts/upload_test_vlm.py \
  --image path/to/emergency_exit.jpg \
  --object-type emergency_exit
```

**Expected Output:**
```json
{
  "task": "emergency_exit",
  "decision": "FAIL",
  "confidence": 0.84,
  "summary": "Emergency exit appears blocked by equipment",
  "findings": [
    "Trolley positioned in front of exit door",
    "Exit path is obstructed"
  ],
  "evidence": {
    "blocked": true
  },
  "extracted_objects": ["emergency exit sign", "trolley", "door"]
}
```

### 4. Oil Leak Detection

```bash
python scripts/upload_test_vlm.py \
  --image path/to/cylinder.jpg \
  --object-type main_cylinder
```

**Expected Output:**
```json
{
  "task": "main_cylinder",
  "decision": "PASS",
  "confidence": 0.73,
  "summary": "No oil leak detected near main cylinder",
  "findings": [
    "Floor area appears dry",
    "No reflective puddles visible"
  ],
  "evidence": {
    "oil_leak": false
  },
  "extracted_objects": ["cylinder", "floor"]
}
```

### 5. Custom Unknown Task

```bash
python scripts/upload_test_vlm.py \
  --image path/to/inspection_area.jpg \
  --object-type unknown \
  --inspection-prompt "Check if emergency exits are blocked or not"
```

**Expected Output:**
```json
{
  "task": "unknown",
  "decision": "PASS",
  "confidence": 0.80,
  "summary": "Emergency exits appear clear",
  "findings": [
    "No visible obstructions near exit pathways",
    "Exit signs are visible and unobstructed"
  ],
  "evidence": {
    "blocked": false
  },
  "extracted_objects": ["exit sign", "doorway"]
}
```

---

## Decision Logic

### Decision Values

- **PASS**: Inspection criteria met, no issues detected
- **FAIL**: Clear violation or issue detected
- **UNKNOWN**: Cannot determine with confidence

### Task-Specific Rules

#### Fire Extinguisher
- **PASS**: Present AND accessible
- **FAIL**: Present BUT blocked/occluded, OR missing when expected
- **UNKNOWN**: Cannot determine presence or accessibility

#### Door
- **PASS**: Door state confidently identified
- **UNKNOWN**: Cannot determine state

#### Emergency Exit
- **PASS**: Exit is clear and accessible
- **FAIL**: Exit is blocked or obstructed
- **UNKNOWN**: Cannot determine blockage status

#### Main Cylinder
- **PASS**: No oil leak detected
- **FAIL**: Oil leak or spill visible
- **UNKNOWN**: Cannot determine with confidence

---

## Customizing Prompts

### Editing Existing Prompts

1. Navigate to `app/pipelines/vlm/prompts/`
2. Edit the relevant `.yaml` file
3. Modify any of these sections:
   - `system`: System instructions
   - `user_template`: User prompt (supports placeholders)
   - `decision_rules`: Decision criteria
   - `output_schema`: Expected output format

### Adding New Task Types

1. Create new prompt file: `app/pipelines/vlm/prompts/new_task.yaml`

```yaml
system: |
  You are an inspector analyzing images for [task description].
  Follow the base inspection rules and return only valid JSON.

user_template: |
  Inspect this image for [specific criteria].
  
  TASK: [Clear task description]
  
  DECISION RULES:
  - PASS: [criteria for pass]
  - FAIL: [criteria for fail]
  - UNKNOWN: [criteria for unknown]
  
  EVIDENCE TO PROVIDE:
  - "key_name": true/false/null
  
  Return ONLY valid JSON matching this schema:
  {
    "task": "new_task",
    "decision": "PASS|FAIL|UNKNOWN",
    "confidence": 0.0-1.0,
    "summary": "Brief assessment",
    "findings": ["Finding 1", "Finding 2"],
    "evidence": {"key_name": true/false},
    "extracted_objects": ["object1", "object2"]
  }

decision_rules: |
  PASS: [pass criteria]
  FAIL: [fail criteria]
  UNKNOWN: [unknown criteria]

output_schema: |
  {
    "task": "new_task",
    "decision": "PASS|FAIL|UNKNOWN",
    "confidence": float (0-1),
    "summary": string,
    "findings": [string],
    "evidence": {flexible fields},
    "extracted_objects": [string]
  }
```

2. Add to allowed object types in `.env`:
```bash
ALLOWED_OBJECT_TYPES=gauge,door,fire_extinguisher,unknown,emergency_exit,main_cylinder,new_task
```

3. Restart server

---

## API Integration

### Upload with Metadata

```python
import requests
import json

url = "http://localhost:8000/api/v1/jobs"

# Prepare metadata
metadata = {
    "inspection_prompt": "Check if emergency exits are blocked",
    "site": "Building A",
    "camera_id": "cam_01"
}

# Upload
files = {"file": open("roi.jpg", "rb")}
data = {
    "object_type": "unknown",
    "metadata_json": json.dumps(metadata)
}

response = requests.post(url, files=files, data=data)
job_id = response.json()["job_id"]
```

### Poll for Results

```python
import time

while True:
    result = requests.get(f"{url}/{job_id}").json()
    if result["status"] in ["DONE", "FAILED"]:
        vlm_result = json.loads(result["result_json"])
        print(f"Decision: {vlm_result['decision']}")
        print(f"Summary: {vlm_result['summary']}")
        break
    time.sleep(1)
```

---

## Troubleshooting

### Issue: "VLM_API_KEY is required for OpenAI provider"

**Solution**: Set your OpenAI API key in `.env`:
```bash
VLM_API_KEY=sk-your-actual-api-key-here
```

### Issue: "Invalid JSON response"

**Causes**:
- Model returned markdown code blocks
- Model didn't follow schema
- API error

**Solutions**:
1. Check VLM_MODEL is vision-capable (e.g., `gpt-4o`)
2. Review prompt clarity in `.yaml` files
3. Check server logs for raw response
4. Try with `VLM_PROVIDER=stub` first

### Issue: Low Confidence Scores

**Solutions**:
1. Improve image quality (lighting, focus, resolution)
2. Ensure object is clearly visible in ROI
3. Adjust prompt to be more specific
4. Use higher-capability model

### Issue: Wrong Decision

**Solutions**:
1. Review prompt decision rules
2. Check if evidence fields match expectations
3. Adjust confidence thresholds in your application logic
4. Refine prompt with more specific criteria

---

## Performance & Costs

### Stub Provider
- **Speed**: Instant (< 100ms)
- **Cost**: Free
- **Use Case**: Development, testing, demos

### OpenAI Provider
- **Speed**: 2-5 seconds per image
- **Cost**: ~$0.01-0.03 per image (GPT-4o)
- **Use Case**: Production deployments

### Optimization Tips
1. Use stub provider for development
2. Batch process images when possible
3. Cache results for identical images
4. Use lower-cost models for simple tasks
5. Implement rate limiting

---

## Security Best Practices

### API Key Management

**❌ DON'T:**
- Hardcode API keys in code
- Commit `.env` to git
- Share API keys in logs

**✅ DO:**
- Use `.env` file (gitignored)
- Rotate keys regularly
- Use environment variables in production
- Monitor API usage

### Production Deployment

```bash
# Use environment variables
export VLM_API_KEY="sk-..."
export VLM_PROVIDER="openai"
export VLM_MODEL="gpt-4o"

# Start server
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

---

## Advanced Features

### Custom Evidence Fields

Prompts can define any evidence fields:

```yaml
evidence: {
  "blocked": true/false,
  "damage_detected": true/false,
  "compliance_level": "high|medium|low",
  "item_count": integer
}
```

### Multi-Step Reasoning

For complex tasks, structure prompts with:
1. Identification step
2. Assessment step
3. Decision step

### Confidence Calibration

Adjust confidence interpretation:
- 0.9-1.0: Very high confidence
- 0.7-0.9: High confidence
- 0.5-0.7: Medium confidence
- < 0.5: Low confidence (consider UNKNOWN)

---

## Testing

### Unit Tests

```bash
pytest tests/test_vlm.py -v
```

### Integration Tests

```bash
# Test with stub provider
VLM_PROVIDER=stub python scripts/upload_test_vlm.py \
  --image sample_images/sample_door.jpg \
  --object-type door

# Test with OpenAI (requires API key)
VLM_PROVIDER=openai python scripts/upload_test_vlm.py \
  --image sample_images/sample_door.jpg \
  --object-type door
```

---

## Future Enhancements

- [ ] Support for additional VLM providers (Anthropic Claude, local models)
- [ ] Multi-image analysis
- [ ] Video frame analysis
- [ ] Result caching
- [ ] Confidence threshold configuration
- [ ] Automated prompt optimization
- [ ] A/B testing for prompts

---

## Support

For issues or questions:
1. Check server logs for detailed error messages
2. Test with `VLM_PROVIDER=stub` first
3. Verify prompt YAML syntax
4. Review API key permissions
5. Check OpenAI API status

---

**The VLM pipeline is now fully integrated and ready to use!** 🎉
