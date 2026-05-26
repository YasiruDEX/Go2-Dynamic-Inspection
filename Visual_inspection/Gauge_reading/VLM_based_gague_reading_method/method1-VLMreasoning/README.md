# Method 1: Direct VLM Reasoning for Gauge Reading

## 📚 Table of Contents

1.  [Project Overview](#1-project-overview)
2.  [Theoretical Framework](#2-theoretical-framework)
    *   [The Paradigm Shift](#the-paradigm-shift)
    *   [Chain-of-Thought Reasoning](#chain-of-thought-reasoning)
    *   [Architecture Diagram](#architecture-diagram)
3.  [Installation & Environment Setup](#3-installation--environment-setup)
    *   [Prerequisites](#prerequisites)
    *   [Dependency Installation](#dependency-installation)
    *   [API Key Configuration (.env)](#api-key-configuration-env)
4.  [File Structure & Component Analysis](#4-file-structure--component-analysis)
    *   [Core Logic Modules](#core-logic-modules)
    *   [Execution Scripts](#execution-scripts)
    *   [Analysis Tools](#analysis-tools)
5.  [Usage Guide: Step-by-Step](#5-usage-guide-step-by-step)
    *   [Scenario A: Testing a Single Image](#scenario-a-testing-a-single-image)
    *   [Scenario B: Running Full Dataset Evaluation](#scenario-b-running-full-dataset-evaluation)
    *   [Scenario C: Visualizing Results](#scenario-c-visualizing-results)
6.  [Data Formats & Schemas](#6-data-formats--schemas)
    *   [Input Dataset Structure](#input-dataset-structure)
    *   [Output Prediction Structure](#output-prediction-structure)
7.  [Advanced Configuration & Optimization](#7-advanced-configuration--optimization)
    *   [Rate Limiting Strategies](#rate-limiting-strategies)
    *   [Token Cycling (Multi-Key Rotation)](#token-cycling-multi-key-rotation)
    *   [Prompt Engineering Customization](#prompt-engineering-customization)
8.  [Evaluation Metrics & Logic](#8-evaluation-metrics--logic)
9.  [Troubleshooting & FAQ](#9-troubleshooting--faq)

---

## 1. Project Overview

This directory (`method1-VLMreasoning`) contains the implementation of **Method 1** for the MeasureBench Gauge Reading project. 

**The Core Objective:** To determine if modern Vision-Language Models (VLMs) can "read" analog and digital measurement instruments directly from images, without the need for traditional computer vision pipelines (like edge detection, circle hough transforms, or needle segmentation).

This method treats the gauge reading problem as a **Visual Question Answering (VQA)** task. Instead of calculating angles and mapping them to values mathematically, we ask the AI model to "look" at the image, understand the scale, identify the needle's position relative to the tick marks, and reason its way to a numerical answer.

**Supported Models:**
*   **Google Gemini 2.5 Flash:** (Primary) Fast, cost-effective, high reasoning capability.
*   **Google Gemini 1.5 Pro:** Higher reasoning capability, slower inference.
*   **OpenAI GPT-4o:** State-of-the-art performance, higher cost.

---

## 2. Theoretical Framework

### The Paradigm Shift
Traditional gauge reading algorithms typically follow a rigid pipeline:
1.  **Pre-processing:** Grayscale, blur, thresholding.
2.  **Detection:** Find the gauge face (Circle detection).
3.  **Scale Unrolling:** Polar to Cartesian transformation.
4.  **Needle Detection:** Line detection (Hough Transform).
5.  **Angle Calculation:** $\theta = \arctan(y/x)$.
6.  **Mapping:** Linear interpolation of angle to value.

**Method 1** bypasses this entirely. It relies on the **emergent capabilities** of Large Multimodal Models (LMMs). These models have been trained on vast amounts of internet data, including diagrams, textbooks, and photographs of instruments. They possess an inherent understanding of:
*   **Visual Semantics:** What a "needle" looks like, what "tick marks" represent.
*   **Numerical Literacy:** How to interpolate between numbers (e.g., "halfway between 10 and 20 is 15").
*   **Context:** Distinguishing between a voltmeter (V) and an ammeter (A) based on symbols.

### Chain-of-Thought Reasoning
To maximize accuracy, we do not simply ask "What is the value?". We employ **Chain-of-Thought (CoT)** prompting. We force the model to output its internal monologue before giving the final answer.

The prompt explicitly instructs the model to follow these steps:
1.  **Identify Instrument:** "This is a pressure gauge."
2.  **Analyze Scale:** "The scale goes from 0 to 100 psi. Major ticks are every 20 units. Minor ticks are every 5 units."
3.  **Locate Indicator:** "The red needle is past the 40 mark, specifically at the second minor tick after 40."
4.  **Calculate:** "40 + (2 * 5) = 50."
5.  **Final Output:** Return JSON `{ "value": 50, "unit": "psi" }`.

### Architecture Diagram

```mermaid
graph LR
    A[Input Image] --> B[VLM Client];
    C[System Prompt] --> B;
    B --> D[Google Gemini API];
    D --> E[Raw Text Response];
    E --> F[Method1 Inference Engine];
    F --> G{Parsing Logic};
    G --> H[Extracted Value (float)];
    G --> I[Extracted Unit (str)];
    H --> J[Evaluation Module];
    I --> J;
    K[Ground Truth] --> J;
    J --> L[Final Result JSON];
```

---

## 3. Installation & Environment Setup

To get started, ensure you have a compatible environment. This section covers the prerequisites, how to install the necessary dependencies, and how to configure your API keys.

### Prerequisites
*   Python 3.8 or higher.
*   Pip package manager.
*   Access to Google Cloud Platform (for Gemini API) and/or OpenAI (for GPT-4o).

### Dependency Installation
1.  Clone this repository.
2.  Navigate to the project directory.
3.  Install the required Python packages:
    ```bash
    pip install -r requirements.txt
    ```

### API Key Configuration (.env)
1.  Create a new file named `.env` in the project root.
2.  Add your API keys:
    ```env
    GOOGLE_API_KEY="your_google_api_key"
    OPENAI_API_KEY="your_openai_api_key"
    ```
3.  Save and close the file.

---

## 4. File Structure & Component Analysis

A brief overview of the important files and directories in this project.

### Core Logic Modules
*   `method1_inference.py`: Contains the `Method1SimpleVLM` class for prompt construction and response parsing.
*   `vlm_client.py`: The `VLMClient` class for API interactions (Google and OpenAI).

### Execution Scripts
*   `run_single_image.py`: Debugging tool for single image processing.
*   `run_full_evaluation.py`: Main script for batch processing and evaluation.

### Analysis Tools
*   `visualize_results.ipynb`: Jupyter Notebook for result visualization and analysis.

---

## 5. Usage Guide: Step-by-Step

Detailed instructions for using this method in different scenarios.

### Scenario A: Testing a Single Image
1.  Prepare your image file. Ensure it's clear and well-lit.
2.  Run the debug script:
    ```bash
    python run_single_image.py --image_path /path/to/your/image.jpg
    ```
3.  Review the output JSON and logs for details.

### Scenario B: Running Full Dataset Evaluation
1.  Ensure your dataset is in the correct format and located at `data/real_world.json`.
2.  Execute the batch processing script:
    ```bash
    python run_full_evaluation.py
    ```
3.  Results will be saved in the `results/` directory.

### Scenario C: Visualizing Results
1.  Open the Jupyter Notebook:
    ```bash
    jupyter notebook visualize_results.ipynb
    ```
2.  Follow the instructions in the notebook to generate visualizations.

---

## 6. Data Formats & Schemas

Understanding the data formats used in this project is crucial for proper usage and integration.

### Input Dataset Structure
The input dataset (`real_world.json`) should be a JSON file with the following structure:
```json
{
    "images": [
        {
            "file_name": "image1.jpg",
            "instrument_type": "pressure_gauge",
            "ground_truth": {
                "value": 50,
                "unit": "psi"
            }
        },
        {
            "file_name": "image2.jpg",
            "instrument_type": "thermometer",
            "ground_truth": {
                "value": 75,
                "unit": "celsius"
            }
        }
    ]
}
```

### Output Prediction Structure
The output predictions will be saved as JSON files in the `results/` directory. The structure will mirror the input dataset, with the addition of a `predicted` field:
```json
{
    "images": [
        {
            "file_name": "image1.jpg",
            "instrument_type": "pressure_gauge",
            "ground_truth": {
                "value": 50,
                "unit": "psi"
            },
            "predicted": {
                "value": 49.5,
                "unit": "psi"
            }
        }
    ]
}
```

---

## 7. Advanced Configuration & Optimization

For users who need more control or want to optimize their usage, several advanced options are available.

### Rate Limiting Strategies
To avoid hitting API rate limits, especially during batch processing, consider the following:
*   Spread out requests using longer sleep intervals.
*   Use a pool of API keys and rotate them using the Token Cycling feature.

### Token Cycling (Multi-Key Rotation)
This method allows you to bypass rate limits by cycling through multiple API keys. Configure this in the `.env` file:
```env
API_KEY_POOL="key1,key2,key3"
```

### Prompt Engineering Customization
Advanced users can experiment with custom prompts. Modify the `PROMPT_TEMPLATE` in `method1_inference.py`. Ensure you maintain the logical steps for accurate reasoning.

---

## 8. Evaluation Metrics & Logic

To objectively measure the performance of the gauge reading, several metrics are calculated:

1. **Mean Absolute Error (MAE):** Measures the average magnitude of the errors in a set of predictions, without considering their direction. It's the average over the test sample of the absolute differences between prediction and actual observation where all individual differences have equal weight.
    *   **Formula:** $MAE = \frac{1}{n} \sum_{i=1}^{n} |y_i - \hat{y}_i|$
    *   **Interpretation:** A lower MAE indicates a better performing model. It provides a linear score that averages the absolute differences between predicted values and actual values.

2. **Mean Squared Error (MSE):** Measures the average of the squares of the errors—that is, the average squared difference between the estimated values ($\hat{y}_i$) and the actual value ($y_i$).
    *   **Formula:** $MSE = \frac{1}{n} \sum_{i=1}^{n} (y_i - \hat{y}_i)^2$
    *   **Interpretation:** Like MAE, a lower MSE value indicates a better fit. However, MSE gives a higher weight to larger errors, meaning it's more sensitive to outliers than MAE.

3. **Coefficient of Determination (R² Score):** Represents the proportion of the variance for a dependent variable (in this case, the gauge reading) that's explained by an independent variable or variables (the features used for prediction) in a regression model.
    *   **Formula:** $R^2 = 1 - \frac{SS_{res}}{SS_{tot}}$
        *   Where $SS_{res}$ is the sum of squares of residuals and $SS_{tot}$ is the total sum of squares.
    *   **Interpretation:** An R² score of 1 indicates that the regression predictions perfectly fit the data. A lower R² score indicates a worse fit.

4. **Unit Consistency Check:** Ensures that the predicted unit of measurement matches the expected unit based on the instrument type and scale.
    *   **Logic:** If the ground truth unit is "psi" and the predicted unit is "bar", flag as inconsistent.

5. **Range Check:** Validates that the predicted value falls within a plausible range given the instrument type.
    *   **Logic:** For a pressure gauge with a range of 0-100 psi, any predicted value less than 0 or greater than 100 is flagged.

6. **Tick Mark Alignment:** For analog gauges, checks that the predicted needle position aligns with a tick mark or a known subdivision.
    *   **Logic:** If the major tick interval is 20 and the minor tick resolution is 5, then valid needle positions are multiples of 5.

---

## 9. Troubleshooting & FAQ

**Q1: Why is my API key not working?**
*   Ensure you've enabled the correct API (Google Gemini or OpenAI) in your cloud console.
*   Check that you've copied the API key correctly, without extra spaces.

**Q2: Why are the predictions inaccurate?**
*   Ensure the images are clear, well-lit, and properly centered.
*   Check that the instrument type is correctly labeled in the input dataset.

**Q3: How can I improve the model's accuracy?**
*   Provide higher quality images.
*   Use the advanced configuration options to fine-tune the model's settings.
*   Consider using a more powerful model (e.g., GPT-4o).

**Q4: What should I do if I encounter an error?**
*   Check the error logs in the `results/logs/` directory.
*   Common issues include API rate limits, invalid image formats, and JSON parsing errors.
*   For persistent issues, consider reaching out on the project forum or GitHub discussions page.

**Q5: How is the evaluation conducted?**
*   The evaluation module compares the predicted values against the ground truth using the metrics described in the Evaluation Metrics & Logic section.
*   Detailed logs are generated, highlighting any discrepancies and potential reasons.

**Q6: Can I use my own dataset?**
*   Yes, but ensure it follows the Input Dataset Structure outlined in the Data Formats & Schemas section.
*   For best results, use high-quality images and accurate ground truth labels.

**Q7: How do I contribute to the project?**
*   We welcome contributions! Please submit a pull request with a clear description of your changes.
*   For major changes, please open an issue first to discuss what you'd like to change.

**Q8: Where can I find additional resources or documentation?**
*   Check the `docs/` directory for additional documentation.
*   For specific questions, refer to the Troubleshooting & FAQ section or consider reaching out to the project maintainers.

---