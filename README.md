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
