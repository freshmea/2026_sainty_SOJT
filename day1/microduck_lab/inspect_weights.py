"""Report dense weight matrices actually stored in this specific ONNX file."""
import numpy as np
import onnx
from onnx import numpy_helper
from inspect_model import ROOT

graph = onnx.load(ROOT / "policies/alpha_walking.onnx").graph
arrays = {item.name: numpy_helper.to_array(item) for item in graph.initializer}
total = 0
for name, values in arrays.items():
    if name.endswith(".weight") and values.ndim == 2:
        total += values.size
        print(name, values.shape, "count", values.size, "min/max",
              float(values.min()), float(values.max()), "std", float(values.std()))
print("matrix elements (biases and other tensors excluded):", total)
