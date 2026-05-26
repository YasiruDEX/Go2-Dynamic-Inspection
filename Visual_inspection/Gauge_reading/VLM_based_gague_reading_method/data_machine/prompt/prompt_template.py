PROMPT_TEMPLATE = """
You are writing **Python code** that generates a *synthetic image* of an instrument named **{INSTRUMENT_NAME}**.
The goal is to make the instrument’s **reading** inferable from its *visual scale/indicator*, not from printed numbers.

# INPUTS (provided by the user for this instrument)
- Description: {INSTRUMENT_DESCRIPTION}
- Optional Design types: {INSTRUMENT_DESIGN}
- Other requirements: {INSTRUMENT_OTHER_REQUIREMENTS}

# YOUR DELIVERABLE
Write a single, self-contained Python module. The module may contain multiple helper functions if necessary,
but it must expose exactly **one** public entrypoint function with the following signature:

    def generate(img_path: str) -> dict:
        \"\"\"Render a synthetic {INSTRUMENT_NAME} image and save it to img_path.
        Returns:
            {{
                "design": <The design type of current generated image, can be one of the following: {INSTRUMENT_DESIGN}>,
                "evaluator_kwargs": {{
                    "interval": [<lower_bound>, <upper_bound>],
                    "units": [<list of acceptable unit strings>]
                }}
            }}
        \"\"\"

- Save the image to **img_path**.
- Return a dict matching the schema above exactly (keys spelled as shown).
- Do **not** add extra function parameters.

# GLOBAL IMAGE RULES
1) **Reading, Units, Scale are randomized and realistic** for a {INSTRUMENT_NAME}.
2) **Image must faithfully encode the chosen reading via the visual scale**
   (needle angle, fluid height, slider position, fill level, etc.). The depicted state **must match** the randomized reading, units, and scale.
3) **Do NOT write the exact reading number anywhere as text in the image.**
   - Allowed: tick labels (e.g., 0, 10, 20…), unit labels (e.g., °C, kPa), brand marks.
   - Not allowed: printing the specific randomized reading as a text label.
4) **Maximize diversity** across runs. Each call should vary *at least 12* independent factors (see “Diversity Menu” below).
5) **Offline only**: do not download assets. Use Python + Pillow (PIL), numpy, math, and random. Avoid seaborn; matplotlib optional. No external files or fonts.
6) **Image size**: choose randomly per run from {{384, 512, 640}} (square). RGB.
7) **Legibility**: keep the indicator unoccluded; mild noise/blur/grain is OK but the reading should remain unambiguously inferable.

# READING & SCALE LOGIC (must implement)
- Choose a **unit system** consistent with {INSTRUMENT_NAME} and user inputs.
- Define numeric **scale_min**, **scale_max**, **major/minor tick spacing**, and optional non-linear scales if appropriate (e.g., log).
- Sample a **target_reading** uniformly (or per a realistic distribution) within [scale_min, scale_max].
- Map **target_reading → visual state** deterministically (e.g., angle, y-height, slider x-position).
- **Ensure internal consistency** with assertions:
  - scale_min < target_reading < scale_max
  - inverse_map(visual_state) ≈ target_reading (within instrument resolution)

**Tolerance rule for evaluator_kwargs["interval"]**:
- For analog/continuous instruments, set ±(1.0 * smallest resolvable step) around the target_reading.
- For discrete/time-like instruments (e.g., clocks), bound with a small, realistic window (e.g., ±2 seconds unless the design dictates otherwise).
- Always adapt to tick density and rendering resolution.

# RETURN SCHEMA (must match exactly)
Return a Python dict:

{{
  "design": "<The design type of current generated image, can be one of the following: {INSTRUMENT_DESIGN}>",
  "evaluator_kwargs": {{
    "interval": [<lower_bound>, <upper_bound>],   # numbers or ISO-like strings for time
    "units": [<accepted unit strings>]            # may be empty if not applicable
  }}
}}

# Examples:
- Clock:
  "evaluator_kwargs": {{
    "interval": ["10:45:32", "10:45:34"],
    "units": []
  }}

- Thermometer:
  "evaluator_kwargs": {{
    "interval": [27.7, 29.7],
    "units": ["Celsius", "°C"]
  }}

# DIVERSITY MENU (randomize broadly on every run, but the design should be consistent with the {INSTRUMENT_NAME})
**Instrument Form**
- Analog dial, bar/column, arc gauge, tube thermometer, slider ruler, liquid-in-glass, bob/float, needle + scale, color strip, LED-style segments that hint at scale but do NOT display the numeric reading
- Face shape: circular, square, rounded rectangle, hexagonal, capsule, triangular

**Scale Design**
- Linear vs log; clockwise vs counter-clockwise; start angle; sweep angle
- Major/minor tick count and spacing; partial or full labeling; alternate tick styles
- Label typography (safe system fonts only) and placement (inner/outer ring)
- Colored zones (green/yellow/red), banded gradients, dual-scales (inner/outer) with distinct units

**Units & Conventions**
- Random selection from realistic unit variants for {INSTRUMENT_NAME} (and consistent with description)
- Include synonyms and symbol forms (e.g., "Celsius", "°C"; "kPa", "kilopascal")

**Indicator & Mechanics**
- Needle shape (arrow, spade, tapered, skeleton), counterweight, hub style
- Tube/column properties (glass tint, meniscus shape, mercury/colored alcohol)
- Slider/marker shapes; pivot position; damping blur or slight motion hint

**Materials & Finish**
- Brushed metal, painted metal, matte plastic, glossy plastic, bakelite, wood veneer, glass reflections
- Screws/rivets/bezel rings; faceplate textures; light scratches; mild dust

**Colors & Lighting**
- Foreground/background palettes; single-color vs complementary; muted vs saturated
- Lighting direction; soft shadows; subtle vignette; reflections on glass
- Backgrounds: lab bench, wall plate, panel cutout, plain studio background, gradient, procedural noise

**Camera & Layout**
- Perspective (tilt, rotation), focal length feel, crop tight/loose, off-center compositions
- Depth-of-field blur (subtle); motion blur (very mild) if plausible

**Artifacts & Noise (subtle)**
- Film grain, JPEG artifacts, sensor noise, faint glare, micro-scratches
- Ensure these never hide the indicator or make the reading ambiguous

# IMPLEMENTATION CHECKLIST (follow explicitly)
- [ ] Use only offline-safe libraries: Pillow (PIL), numpy, random, math, matplotlib.
- [ ] Programmatically define the scale and indicator geometry.
- [ ] Compute the indicator pose from target_reading and draw it.
- [ ] Draw ticks/labels/units (but NOT the exact randomized reading as text).
- [ ] If using labels, never include a label equal to the sampled target_reading value.
- [ ] Add diversity choices (shape, material, colors, background, lighting) sampled independently.
- [ ] Sanity-check: inverse mapping recovers target_reading within tolerance; raise if not.
- [ ] Save to img_path (RGB).
- [ ] Return the dict with "design" and "evaluator_kwargs" exactly as specified.

# NOTES
- The acceptable error (interval width) should reflect rendered resolution and tick density.
  Use a realistic bound; never return a zero-width or overly large interval.
- If {INSTRUMENT_OTHER_REQUIREMENTS} conflicts with realism or legibility, prioritize realism and explain the compromise in the "design" text.

"""
