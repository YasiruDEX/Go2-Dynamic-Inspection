# VLM Reasoning Pipeline - Prompt Structure

```
VLM Inspection System
│
├── Base Rules (Common to All Tasks)
│   ├── Role: Visual inspection AI assistant
│   ├── Decision Framework: PASS / FAIL / UNKNOWN
│   └── Output: Structured JSON with validation
│
├── Task-Specific Prompts
│   │
│   ├── Fire Extinguisher
│   │   ├── Check: Presence & Accessibility
│   │   └── Evidence: present (true/false), blocked (true/false)
│   │
│   ├── Emergency Exit
│   │   ├── Check: Sign visibility & Path clearance
│   │   └── Evidence: sign_visible (true/false), blocked (true/false)
│   │
│   ├── Door
│   │   ├── Check: Door status (open/closed)
│   │   └── Evidence: open (true/false)
│   │
│   ├── Main Cylinder
│   │   ├── Check: Oil leaks
│   │   └── Evidence: oil_leak (true/false)
│   │
│   └── Unknown (Smart Detection)
│       ├── Step 1: Identify object type
│       ├── Step 2: Apply appropriate criteria
│       └── Evidence: identified_object + type-specific fields
│
└── Output Schema
    ├── task: object type
    ├── decision: PASS/FAIL/UNKNOWN
    ├── confidence: 0.0-1.0
    ├── summary: brief description
    ├── findings: array of observations
    ├── evidence: task-specific fields
    └── vlm_raw_response: full VLM output
```

## Flow

```
Image → Identify Type → Load Prompt → VLM API → Validate → Store → Return
```
