# COMPLETE PRESENTATION GUIDE - ANALOG GAUGE READING

---

# SLIDE 1: LAST WEEK - OVERVIEW

## WHAT TO PUT ON SLIDE:

**Title:** Last Week

• **Reviewed and evaluated a 2024 learning-based analog gauge reading pipeline - work for decimal numbers**

• **Developed and evaluated three computer vision approaches for decimal detection**
  ■ Adaptive thresholding with contour detection - failed due to noise sensitivity and false positives
  ■ Contour-based detection with perspective transform - failed due to preprocessing complexity
  ■ Morphological filtering with top-hat transform - failed due to rigid heuristics and parameter sensitivity

• **Implemented successful ROI-based decimal point detection solution**

• **Integrated decimal detection into production pipeline with automatic correction**

---

## SCRIPT FOR SLIDE 1:

"Good morning everyone. Let me share what I've been working on this past week.

I've been reviewing and evaluating a learning-based analog gauge reading pipeline that was published in 2024. This pipeline uses deep learning models to automatically read pressure gauges from images. The system works really well for standard gauges, but I discovered it has a critical weakness when it comes to gauges that use decimal notation - gauges that display values like 0.5, 1.5, 2.5, and so on.

So I spent time developing and testing different approaches to solve this decimal point detection problem. I tried three different computer vision methods. The first was adaptive thresholding with contour detection - basically trying to find small circular dots in the image. This failed because it was too sensitive to noise and gave us lots of false positives from scratches and reflections on the gauge face.

The second approach was contour-based detection with perspective transforms. This was more sophisticated but it failed because the preprocessing pipeline became too complex and error-prone. Even with all that complexity, we still missed decimal points in many cases.

The third approach used morphological filtering with top-hat transforms to enhance small features. This also failed because it required too much manual parameter tuning for each gauge type, and the heuristics I used were too rigid.

After these failures, I developed a successful solution using an ROI-based approach. Instead of trying to detect tiny decimal points in the full image, I extract small regions around each number and analyze them individually. This approach works much better and I've now integrated it into the production pipeline so it runs automatically and corrects the values when decimal points are detected.

Let me walk you through the details of how this all works."

---

# SLIDE 2: GAUGE READING PIPELINE - METHOD & RESULTS

## WHAT TO PUT ON SLIDE:

**Title:** Analog Gauge Reading Pipeline - Architecture & Method

**Show selected key images** (you choose which 12 from the list below)

**Complete pipeline has 17 stages - script explains all of them:**

---

## SCRIPT FOR SLIDE 2:

"Now let me explain how the gauge reading pipeline works. I'm going to walk through the complete process step by step. On the slide, I'm showing you selected key images from the pipeline, but let me explain the entire flow so you understand what's happening at each stage.

**[original_image.jpg]**

This is where we start - a raw image of a pressure gauge. This could be from an industrial setting, a factory floor, a laboratory, anywhere you have analog gauges that need to be monitored. The challenge is to automatically extract the reading from this image without any human intervention. The gauge might be at any angle, the lighting might vary, there might be reflections or shadows - the system has to handle all of that.

**[bbox_results.jpg]**

The first stage uses YOLOv8, which is a state-of-the-art object detection model. YOLOv8 scans the entire image and identifies where the gauge face is located. You can see the red bounding box it draws around the gauge. This detection happens very quickly - in about 100 milliseconds. The model has been trained on thousands of gauge images, so it can reliably find gauges even in cluttered environments, with varying lighting conditions, or when the gauge is partially occluded. The bounding box gives us the coordinates we need to extract just the gauge from the image.

**[image_cropped.jpg]**

Once we know where the gauge is, we crop the image to focus just on the gauge face. We also add some padding around the edges and make it square. This gives us a clean, standardized image to work with for all the subsequent processing stages. Everything from here on operates on this cropped image, not the original full image. This cropping step is important because it eliminates background clutter and normalizes the input size for the deep learning models.

**[key_point_results.jpg]**

The next stage is keypoint detection using DINOv2, which is a vision transformer model from Meta AI. This is a really interesting part of the pipeline. What you're seeing here are three heatmaps side by side. The left heatmap shows where the model thinks the start point of the scale is - that's the beginning of the measurement range. The middle heatmap shows all the tick marks or notches around the gauge - these are the scale markers. And the right heatmap shows the end point of the scale - where the measurement range ends.

The model doesn't just give us single points - it gives us probability distributions, which is why you see these heatmaps with bright spots indicating high confidence regions. We take the brightest point in each heatmap as our detected keypoint. This stage takes about 200 milliseconds. These keypoints are absolutely crucial because they define the geometry of the gauge - they tell us the shape and position of the scale, which we need for all the subsequent calculations.

**[ellipse_results_key_points.jpg]**

Now we take all those detected keypoints - the start point, all the tick marks, and the end point - and fit an ellipse to them. The blue curve you see is the fitted ellipse, and the green points are the keypoints we detected. This ellipse is a mathematical model of the gauge's circular scale. We use least-squares fitting to find the ellipse that best fits through all the keypoints.

Notice how well the ellipse fits - it goes right through most of the keypoints. This ellipse gives us important parameters: the center point coordinates, the major and minor axes lengths, and the rotation angle. We'll use this ellipse model throughout the rest of the pipeline to map positions on the gauge to angular measurements. Even if the gauge is slightly oval-shaped or viewed at an angle, the ellipse fitting handles that automatically.

**[ellipse_zero_point.jpg]**

Now we calculate a zero point for angle reference. The red point you see at the bottom of the gauge is the zero point. This is calculated as the midpoint between the start point and the end point of the scale. This zero point serves as our angle reference - we measure all angles relative to this point. So when we later calculate that the needle is at, say, 45 degrees, that's 45 degrees measured from this zero point. This standardizes our angle measurements regardless of how the gauge is oriented in the image.

**[image_non_rotated_zero_point.jpg]**

This shows the image before rotation alignment. You can see the zero point marked on the gauge, but it's not aligned vertically. The gauge might have been captured at any angle by the camera. Before we proceed with OCR and other processing, we want to standardize the orientation.

**[image_rotated_zero_point.jpg]**

So we rotate the image to align the zero point vertically at the bottom. Now the gauge is in a standard orientation - zero point at the bottom, scale going clockwise from there. This standardization is important for two reasons. First, it makes the OCR more reliable because the numbers are now in a more predictable orientation. Second, it simplifies our angle calculations because we have a consistent reference frame.

**[image_warped.jpg]**

Before we do OCR, we apply a polar warp transformation to the image. This is a clever preprocessing step. What we're doing is unwrapping the circular gauge into a more rectangular form. If you look carefully at this image, you can see how the curved scale has been straightened out. The circular arrangement of numbers has been transformed into a more linear arrangement.

This makes it much easier for the OCR model to read the numbers because OCR models are typically trained on horizontal text - text in books, documents, signs - not text arranged in a circle. This warping is done using the ellipse parameters we calculated earlier. We're essentially transforming from Cartesian coordinates to polar coordinates centered on the ellipse. It's like unwrapping a label from a can - we're flattening the curved surface into a flat image.

**[ocr_visualization_results_chosen.jpg]**

Now we run OCR - Optical Character Recognition. This visualization shows the text detection stage. The system has identified regions in the image that contain text. You can see boxes or highlights around the detected text regions. This is the output from DBNet, which is the text detection component of MMOCR. DBNet scans the image and finds all the regions that look like they contain text, whether that's numbers on the scale or unit labels or manufacturer information.

**[ocr_results_full.jpg]**

This shows all the OCR detections - both numbers and units and any other text. We use MMOCR, which is a comprehensive OCR toolkit that combines two models: DBNet for text detection and ABINet for text recognition. DBNet first finds all the regions in the image that contain text, then ABINet reads what each text region says.

In this image, you can see bounding boxes around everything the OCR detected - the numbers around the scale like 0, 20, 40, 60, 80, 100, and also any text labels like units. This stage is the slowest part of the entire pipeline, taking about 2 seconds, because OCR is computationally intensive. It has to detect text regions, crop them out, normalize them, and then run a recognition model on each one to identify the characters.

**[ocr_results_numbers.jpg]**

From all those OCR detections, we filter to keep only the numbers. The system uses several heuristics to identify which detections are scale numbers and which are other text. For example, we filter by confidence score - if the OCR isn't confident about a detection, we discard it. We check if the text is numeric - does it parse as a number? We filter out things that look like serial numbers or model numbers based on their position on the gauge and their value range.

What you see here are just the scale markings - the numbers that tell us the measurement values at different positions around the gauge, like 0, 20, 40, 60, 80, 100. These are the numbers we'll use to calibrate our reading. We need at least two numbers to establish the scale, but more numbers give us better accuracy.

**[ocr_results_unit.jpg]**

Separately, we identify the unit of measurement. This shows the detected unit label - it might be 'bar', 'psi', 'MPa', 'kPa', or whatever unit the gauge is measuring. This is important because the final output needs to include not just a number but also what that number represents. A reading of 50 means something very different if it's 50 psi versus 50 bar. The system extracts this unit information and includes it in the final output.

**[segmentation_results.jpg]**

While the OCR is running, we also segment the gauge needle using another YOLOv8 model, but this time configured for segmentation rather than object detection. The blue region you see is the segmentation mask - it shows exactly which pixels in the image belong to the needle. This is a pixel-level classification - every pixel is classified as either needle or not-needle.

We then fit a straight line to this needle mask using classical line fitting techniques like RANSAC or least-squares. The red line you see is that fitted line. This gives us the precise angle of the needle. Even if the needle is slightly bent or has an irregular shape, the line fitting gives us a good estimate of its overall direction. This segmentation stage is quite fast - about 60 milliseconds - because it's running in parallel with the OCR.

**[ellipse_results_needle_point.jpg]**

Now we find where the needle line intersects with the ellipse. The red point you see is this intersection point. This is the exact point on the scale that the needle is indicating. We calculate this by finding the intersection between the fitted needle line and the ellipse we calculated earlier. This gives us the angular position of the needle on the scale. We can now say, for example, that the needle is at 127 degrees relative to our zero point.

**[ellipse_results_projected.jpg]**

Now we bring everything together. We take each number that OCR detected and project it onto the ellipse we fitted earlier. What this means is we find the point on the ellipse that's closest to each number's position in the image. This gives us the angular position of each number on the scale.

You can see the numbers positioned around the ellipse. Each number now has an associated angle - we know that, for example, the number 0 might be at 180 degrees, the number 20 at 144 degrees, the number 40 at 108 degrees, and so on. These angles are measured relative to the zero point we calculated earlier. This projection step is important because the OCR gives us numbers at pixel coordinates, but we need to know where those numbers are in angular space on the gauge.

**[reading_line_fit.jpg]**

This is the crucial step that converts angles to actual reading values. What we're doing here is fitting a linear relationship between the angular positions and the reading values. This is the heart of the calibration process.

Look at the graph carefully. The x-axis shows the angle in our coordinate system - these are the angles we calculated for each number. The y-axis shows the reading value - these are the actual numbers that OCR detected. Each blue point represents one of the numbers we detected - its x-coordinate is its angle on the gauge, and its y-coordinate is its numerical value. So if OCR detected the number 40 at an angle of 108 degrees, you'd see a blue point at coordinates 108, 40.

The blue line is a linear fit through these points. We use linear regression to find the line that best fits the data. The red point shows where the needle falls on this line. We know the needle's angle from the segmentation - let's say it's 127 degrees. We find 127 degrees on the x-axis, go up to the blue line, and read off the corresponding y-value. That y-value is our final reading.

The reason this works is that most analog gauges have a linear relationship between angle and reading. If the scale goes from 0 to 100 over 180 degrees, then every degree corresponds to 100 divided by 180 units, which is about 0.56 units per degree. By fitting a line to the detected numbers, we can handle any linear scale automatically without needing to know the gauge's specifications in advance. The slope of the line tells us the units per degree, and the intercept tells us the offset.

This approach is very robust. Even if we miss some numbers in the OCR, as long as we have at least two numbers, we can fit a line. And if we have more numbers, the line fit becomes more accurate because we can average out any errors in individual detections.

**[ellipse_results_final.jpg]**

And this is the final result. This image shows everything together - the blue ellipse model of the scale, the green keypoints we detected, the red needle line, the projected numbers around the ellipse, and the calculated reading value displayed on the image. This is a complete visualization of what the system has done. It's like a summary image that shows all the components working together.

The reading value you see displayed is what gets saved to the output file. This is the final answer - the measurement that the gauge is showing. According to the paper that published this method, the pipeline achieves strong performance on benchmark datasets of industrial gauges. The authors tested it on hundreds of gauge images with different designs, different measurement ranges, different lighting conditions, and different camera angles. They reported that the system works reliably across all these variations.

The key innovation in this pipeline is the combination of modern deep learning models - YOLOv8 for detection and segmentation, DINOv2 for keypoint detection, MMOCR for text recognition - with classical computer vision techniques like ellipse fitting and linear regression. This hybrid approach is robust and generalizes well to different gauge designs. The deep learning models handle the hard perception problems like finding the gauge and reading the numbers, while the classical techniques handle the geometric calculations.

However, as I mentioned earlier, there's one critical limitation that the original paper didn't address - and that's decimal point detection. The OCR component works great for reading digits, but it completely fails when gauges use decimal notation. Let me show you what happens when we encounter gauges with decimal points and how I solved this problem."

---

# SLIDE 3: DECIMAL POINT DETECTION - PROBLEM, METHOD & RESULTS

## WHAT TO PUT ON SLIDE:

**Title:** Decimal Point Detection Extension

**Part A - The Problem:**
• Show example gauge with decimal notation
• OCR failure: "3.5" → "35" (missing decimal point)
• Impact: 10× systematic error

**Part B - The Solution:**
• ROI-based approach
• 3-step method: Extract → Detect → Correct

**Part C - Visual Results:**

**Show these images:**

1. **Gauge image showing decimal numbers** (from test_images)
2. **decimal_rois_visualization.png** - ROI extraction visualization
3. **debug_000_35.png** - Decimal detection in "3.5"
4. **debug_001_05.png** - Decimal detection in "0.5"
5. **Before/After comparison** - Show OCR values vs corrected values
6. **Final result image** - Corrected gauge reading

---

## SCRIPT FOR SLIDE 3:

"Now let me show you the problem I discovered and how I solved it.

**[Show gauge image with decimal notation]**

Look at this pressure gauge. This is a real industrial gauge, and if you look at the scale markings, you can see it displays 0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, and 3.5. These are decimal values - the gauge measures pressure in increments of 0.5 bar.

Now here's the problem. When we run the OCR stage of the pipeline on this gauge, the OCR model reads these numbers as 0, 5, 10, 15, 20, 25, 30, and 35. It completely misses all the decimal points! The OCR sees '3.5' and reads it as '35'. It sees '0.5' and reads it as '5'.

Why does this happen? Decimal points are extremely small - typically just 1 to 5 pixels in diameter in these images. OCR models like DBNet and ABINet are trained primarily on alphanumeric text. They're optimized for recognizing letters and digits, not tiny punctuation marks. During the image preprocessing that OCR models do - resizing, normalization, thresholding - these tiny decimal points often get lost completely. They look very similar to noise, dust particles, or image artifacts, so they get filtered out.

**[Point to the numbers on the gauge]**

The impact of this is severe. If the gauge is actually reading 1.5 bar, but our OCR reads it as 15, and we use those numbers to calibrate our angle-to-reading conversion, we'll calculate a final reading that's 10 times too large. This is a systematic error - it happens every single time we encounter a gauge with decimal notation. In an industrial setting, this kind of error could be catastrophic. Imagine a pressure vessel that's actually at 1.5 bar being reported as 15 bar - you might think there's a dangerous over-pressure situation when everything is actually fine, or worse, you might miss an actual problem because the readings are completely wrong.

So I needed to solve this. The challenge is that we can't just use a better OCR model - even state-of-the-art OCR models fail on decimal points because they're so small. And we can't just assume all gauges use decimals and divide by 10, because many gauges don't use decimal notation at all.

**[Transition to solution]**

After trying three different approaches that all failed - which I mentioned in the first slide - I developed a solution that works. The key insight was to stop trying to detect decimal points in the full gauge image. That's too hard because the image is large and there's too much noise. Instead, I extract small regions around each number first, then analyze those regions individually.

Let me show you how this works.

**[Show Image: decimal_rois_visualization.png]**

This is the ROI extraction visualization. ROI stands for Region of Interest. What you're seeing here is the gauge image with colored bounding boxes drawn around each detected number. Each box represents one ROI - one small region that we extract and analyze separately.

The system uses the OCR bounding boxes from the pipeline to know where each number is located. Then it extracts that region with some padding around it. So instead of analyzing a 1000 by 1000 pixel image looking for tiny decimal points, we're analyzing maybe ten different 50 by 50 pixel regions. This is a much easier problem.

Each of these ROIs contains just one number - like '3.5' or '0.5' or '15'. Now we can look for decimal points in a focused, controlled way.

**[Show Image: debug_000_35.png]**

This is a debug visualization showing what happens when we analyze one of these ROIs. This particular ROI contains the number that OCR read as '35' - but it's actually '3.5'.

You can see two images side by side. On the left is the original ROI - the small region we extracted around this number. On the right is the binary image after thresholding. In the binary image, we're looking for small circular contours that could be decimal points.

The detection algorithm works like this: First, we convert the ROI to grayscale. Then we apply Otsu's automatic binary thresholding to convert it to black and white. Then we find all the contours - the outlines of shapes - in the binary image. We filter these contours by three criteria: size, circularity, and position.

For size, we only consider contours with an area between 3 and 100 pixels. Anything smaller is probably noise, anything larger is probably not a decimal point. For circularity, we calculate a score that's 1.0 for a perfect circle and lower for elongated shapes. We require a score above 0.4. Decimal points are round, so this filters out scratches and lines. For position, we only consider contours in the lower portion of the ROI, because decimal points are bottom-aligned with numbers.

After filtering, we select the best candidate based on the highest circularity score. In this case, the system detected a decimal point with high confidence. So we know that '35' should actually be '3.5'.

**[Show Image: debug_001_05.png]**

Here's another example. This ROI contains what OCR read as '5' - but it's actually '0.5'. Again, you can see the original ROI on the left and the binary image on the right. The detection algorithm found a small circular contour in the lower portion of the image and correctly identified it as a decimal point.

The beauty of this approach is that it's robust. Because we're analyzing small, focused regions instead of the full image, we have much less noise to deal with. Background scratches, reflections, gauge texture - most of that is outside the ROIs. And because decimal points always appear with numbers, we have good spatial context. We can use relative position - like 'bottom of the number' - instead of trying to guess absolute positions in the full image.

**[Show before/after comparison]**

Let me show you the impact of this correction. Here's what the system does:

Before decimal detection:
- OCR reads: 35, 5, 25, 15
- These values are used to calibrate the angle-to-reading conversion
- The final reading would be calculated incorrectly

After decimal detection:
- Decimal points detected in all four numbers
- Values corrected to: 3.5, 0.5, 2.5, 1.5
- These corrected values are used for calibration
- The final reading is now correct

The correction is simple - if a decimal point is detected in an ROI, we divide the OCR value by 10. So '35' becomes 3.5, '15' becomes 1.5, and so on. Then we use these corrected values in the angle-to-reading fit instead of the raw OCR values.

**[Show final result image]**

And here's the final result. This is the gauge reading calculated using the corrected decimal values. You can see the reading displayed on the image. Without the decimal detection, this reading would have been 10 times too large - a critical error. With the decimal detection, we get the correct reading.

The system now handles both decimal and non-decimal gauges automatically. When it encounters a gauge without decimal notation, the decimal detection runs but doesn't find any decimal points, so it uses the original OCR values. When it encounters a gauge with decimal notation, it detects the decimal points and applies the correction. No manual configuration is needed - it just works.

I've integrated this into the production pipeline, so it runs automatically between the OCR stage and the segmentation stage. It adds minimal computational overhead - about 50 milliseconds per image - which is negligible compared to the 2-second OCR stage. And it solves a critical problem that would otherwise make the system unusable for a significant portion of industrial gauges.

The key lesson here is that sometimes the best solution to a hard problem isn't to solve it directly, but to transform it into an easier problem. Instead of detecting 1-pixel dots in 1000-pixel images, we extract 50-pixel regions and detect dots there. Simple, but effective.

That's what I've been working on this week. Thank you."
