# Simulation Data Generators

This directory contains Python scripts for generating golden data to verify the hardware implementation of the Gaussian Splatting. 

## 1. `gen_golden_demo.py`

**Purpose:** Generates golden data for the final demonstration scenes (ICLAB Logo and Cornell Box).

*   **Output Directory:** `golden_demo/`
*   **Resolution:** 128x128
*   **Scenes:**
    *   **Patterns 0-49:** "ICLAB" text animation.
    *   **Patterns 50-63:** Cornell Box scene.

**Usage:**
```bash
python gen_golden_demo.py
```

## 2. `gen_golden_128.py`

**Purpose:** Generates golden data for various 128x128 3D test shapes (Cube, Helix, Sphere) to validate the hardware at higher resolution.

*   **Output Directory:** `golden_128/`
*   **Resolution:** 128x128

**Usage:**
```bash
python gen_golden_128.py
```

## Output Structure

Both scripts generate subdirectories for each hardware stage in their respective output folders:
*   `input/`: Input Gaussian and View data.
*   `stage1_rot/` ... `stage8_render/`: Intermediate golden data for each stage.
*   `visual_debug/`: Reconstructed images (Python rendering) for visual verification.
