"""Check both LimX WF ONNX files against the trusted training checkpoint that produced them."""

import argparse
import json
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch.nn import functional as F


def verify(checkpoint, folder=None):
    folder = folder or checkpoint.parent / "exported"
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    torch.set_num_threads(1)
    errors = {}
    for filename, weights, prefix, inputs, outputs in (
        ("policy", state["model_state_dict"], "actor", 40, 10),
        ("encoder", state["encoder_state_dict"], "encoder", 340, 3),
    ):
        path = folder / (filename + ".onnx")
        onnx.checker.check_model(str(path))
        model = ort.InferenceSession(
            str(path), options, providers=["CPUExecutionProvider"]
        )
        key = model.get_inputs()[0]
        if key.shape != [inputs] or model.get_outputs()[0].shape != [outputs]:
            raise ValueError(f"Unexpected WF model dimensions in {path}")
        layers = sorted(
            int(k.split(".")[1])
            for k in weights
            if k.startswith(prefix + ".") and k.endswith(".weight")
        )
        if not layers:
            raise ValueError(f"No {prefix} weights found in {checkpoint}")
        max_error = 0.0
        for data in (
            np.random.default_rng(73).normal(size=(32, inputs)).astype(np.float32)
        ):
            expected = torch.from_numpy(data)
            for index in layers:
                expected = F.linear(
                    expected,
                    weights[f"{prefix}.{index}.weight"],
                    weights[f"{prefix}.{index}.bias"],
                )
                if index != layers[-1]:
                    expected = F.elu(expected)
            actual = model.run(None, {key.name: data})[0]
            np.testing.assert_allclose(actual, expected.numpy(), rtol=2e-5, atol=5e-5)
            max_error = max(max_error, float(np.max(np.abs(actual - expected.numpy()))))
        errors[filename] = max_error
    return errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument(
        "--folder", type=Path, help="Defaults to exported/ beside the checkpoint."
    )
    args = parser.parse_args()
    print(json.dumps(verify(args.checkpoint, args.folder), indent=2))
