# Self-Propelled Arm System
___

## Install First

- Python 3
- Git
- A webcam or USB camera

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Train Paper Model

Run:

```bash
python train_paper.py
```

Stop:

```bash
Ctrl+C
```

## Run Detection Test

Run:

```bash
python test_detection.py
```

Example:

![Paper detection example](screenshots/detection.png)

Stop:

Press `q` in the camera window.

## Auto-Label New Images

Pre-label new images with the trained model, then review before training.

Add images to `new_data/images/`, then run:

```bash
python autolabel.py
```

Draft labels go to `new_data/labels/` and previews to `new_data/preview/`. 
Review and fix them, then merge and retrain:

```bash
python add_reviewed_data.py --val-frac 0.2 -- Keep in mind that you have to specify a validation fraction recommended value is 0.2 
python train_paper.py
```
