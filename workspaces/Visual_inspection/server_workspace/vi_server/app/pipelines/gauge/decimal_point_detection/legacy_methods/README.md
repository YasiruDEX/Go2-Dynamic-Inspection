# Legacy Decimal Point Detection Methods

This folder contains **archived/failed** approaches for detecting decimal points on analog gauges. These methods were attempted but did not produce reliable results.

## Why These Methods Failed

The decimal point detection problem is challenging because:
- Decimal points are very small (1-5 pixels) and easily lost in noise
- Lighting conditions vary significantly across gauge images
- Gauge face textures and backgrounds interfere with detection
- OCR bounding boxes may not include the decimal point area

## Archived Methods

### 1. DecimalPointDetector (`detect_decimal.py`)
**Approach:** Adaptive thresholding + contour detection for small circular dots

**Why it failed:**
- Too sensitive to noise and artifacts
- Decimal points often too small to detect reliably
- False positives from gauge texture and scratches
- Adaptive thresholding inconsistent across different lighting

### 2. ContourBasedDecimalDetector (`contour_based_detector.py`)
**Approach:** Perspective transform of number regions + contour analysis

**Why it failed:**
- Requires `imutils` library for perspective transforms
- Complex preprocessing pipeline prone to errors
- Expanded bounding boxes still missed decimal points
- Contour filtering too restrictive or too permissive

### 3. FilterBasedDecimalDetector (`filter_based_detector.py`)
**Approach:** Morphological top-hat filtering + smart region search

**Why it failed:**
- Highly sensitive to parameter tuning
- Top-hat filter amplified noise in some images
- Region search heuristics (left of numbers, bottom portion) too rigid
- Circularity scoring unreliable for very small features

## Current Approach

The **new ROI extraction approach** (in parent folder) takes a different strategy:
- Extract clean ROIs containing numbers
- Group adjacent digits together (e.g., "1" and "5" in "15")
- Provide cropped images for further analysis
- Enables future deep learning or template matching approaches

This approach is more flexible and provides a foundation for various decimal detection strategies without committing to a specific detection algorithm.

## Test Files

The test files for each legacy method are also archived here:
- `test_decimal_detection.py` - Tests for original detector
- `test_contour_detector.py` - Tests for contour-based detector  
- `test_filter_detector.py` - Tests for filter-based detector

## Usage (Not Recommended)

If you want to experiment with these legacy methods:

```python
import sys
sys.path.insert(0, 'decimal_point_detection/legacy_methods')

from detect_decimal import DecimalPointDetector
# ... use at your own risk
```

However, we recommend focusing on the current ROI extraction approach instead.

## Lessons Learned

1. **Start with data preparation**: Clean ROIs are more valuable than complex detection algorithms
2. **Avoid over-tuning**: Methods with many parameters are fragile
3. **Consider ML approaches**: Hand-crafted features struggle with gauge variability
4. **Validate on diverse data**: Methods that work on test images often fail on real gauges
