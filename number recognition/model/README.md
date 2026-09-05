# Model provenance

- Model: ONNX Model Zoo MNIST, opset 8; pretrained in CNTK.
- Model card: https://github.com/onnx/models/tree/main/validated/vision/classification/mnist
- Download: https://media.githubusercontent.com/media/onnx/models/main/validated/vision/classification/mnist/model/mnist-8.onnx
- SHA256: `2f06e72de813a8635c9bc0397ac447a601bdbfa7df4bebc278723b958831c9bf`
- The upstream MNIST model card declares **MIT**. The ONNX models repository root
  license is Apache-2.0; its unmodified root license is included as `LICENSE`.
  Model and repository license declarations are recorded separately.
- Input: float32 NCHW `[1,1,28,28]`, white ink on black, values 0..1.
- Output: ten pre-softmax logits.
- The original `.onnx` is preserved. `tools/export_model.py` verifies its SHA256,
  exports the six learned parameter tensors without quantization, and generates
  ten boot self-test images with ONNX reference logits.

MNIST test inputs are the public test set from
https://storage.googleapis.com/cvdf-datasets/mnist/t10k-images-idx3-ubyte.gz
and corresponding `t10k-labels-idx1-ubyte.gz`. Downloaded files currently reside
in the workspace `tmp/` directory. A normal firmware build uses the generated
headers and does not need these files.

`validation.json` describes standard ONNX accuracy. `host-validation.json`
describes the C implementation and synthetic preprocessing checks. A future
`board-validation.json` will only be written after a real serial injection test.
None of these substitutes for labelled camera images from the actual car.
