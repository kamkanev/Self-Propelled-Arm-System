# Can detector for jetson-inference detectNet

This package contains a one-class can detector exported through the official
`dusty-nv/jetson-inference` PyTorch-SSD interface.  It does not use the custom
TFOD/PyCUDA post-processing backend required by `wild2_can.onnx`.

## Files

- `can_ssd_mobilenet_v1.onnx`: fixed `1x3x300x300` FP32 ONNX, opset 11.
- `labels.txt`: generated model labels.  Keep `BACKGROUND` as line 1.

ONNX interface:

- input: `input_0`, float32 `[1, 3, 300, 300]`
- confidence output: `scores`, float32 `[1, 3000, 2]`
- box output: `boxes`, float32 `[1, 3000, 4]`

## Native detectNet smoke test

Run this from a jetson-inference installation:

```bash
python3 detectnet.py input.jpg output.jpg \
  --model=/absolute/path/can_ssd_mobilenet_v1.onnx \
  --labels=/absolute/path/labels.txt \
  --input-blob=input_0 \
  --output-cvg=scores \
  --output-bbox=boxes \
  --threshold=0.20 \
  --clustering=0.30
```

For application code, pass the same arguments through `sys.argv`, or use:

```python
net = jetson_inference.detectNet(
    model="/absolute/path/can_ssd_mobilenet_v1.onnx",
    labels="/absolute/path/labels.txt",
    input_blob="input_0",
    output_cvg="scores",
    output_bbox="boxes",
    threshold=0.20,
)
```

The local evaluation selected confidence `0.20` and clustering/NMS overlap
`0.30` as the initial operating point.  Confirm these values on the Jetson
camera because detectNet clustering is not byte-for-byte identical to the
offline evaluator.

## Compatibility and limitations

- Architecture: SSD MobileNet V1, 300x300.  MobileNet V1 was chosen because it
  follows the official jetson-inference training/export/deployment path.
- The model has been checked by the ONNX checker and ONNX Runtime.  PyTorch and
  ONNX outputs differed by at most `4.4e-7` on the export equivalence test.
- A final TensorRT build and camera smoke test must still be run on the Jetson
  Nano.  Keep the already working Wild2 FP32 model as the fallback until that
  test passes.
- This native model trades some accuracy for removal of the custom runtime:
  on the frozen robot sets it reached photo F1 `0.839` and video F1 `0.924` at
  the shared `0.20` threshold, versus Wild2's previously measured `0.899` and
  `0.945` at its own `0.50` threshold.

## Integrity

```text
42cd78aaec8315445d7d295bcf534c6b074c0c18f03cac40a7846b58a35d6fe1  can_ssd_mobilenet_v1.onnx
f6e88c02da68c0b945f1116455221cf12562d5bc008a5d40b2e7568a93fe535f  labels.txt
```
