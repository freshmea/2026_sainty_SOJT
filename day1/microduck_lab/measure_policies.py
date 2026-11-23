"""Count the deployed actor and time a warmed batch-one policy call on the CPU."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
from time import perf_counter_ns

import numpy as np
import onnx
from onnx import numpy_helper
import onnxruntime as ort
from stable_baselines3 import PPO
import stable_baselines3
import torch

ROOT = Path(__file__).resolve().parents[2]


def describe_onnx(path):
    graph = onnx.load(path).graph
    arrays = {t.name: numpy_helper.to_array(t) for t in graph.initializer}
    linear = [n for n in graph.node if n.op_type == "Gemm"]
    weights = [arrays[n.input[1]] for n in linear]
    biases = [arrays[n.input[2]] for n in linear]
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    obs = np.zeros((1, inp.shape[-1]), dtype=np.float32)
    feed = {inp.name: obs}
    meta = {
        "backend": "ONNX Runtime session.run",
        "layers": [obs.size] + [int(b.size) for b in biases],
        "normalizer_elements": sum(a.size for a in arrays.values()) - sum(a.size for a in weights + biases),
    }
    return meta, weights, biases, lambda: session.run(None, feed)[0]


def describe_ppo(path):
    model = PPO.load(path, device="cpu")
    actor = list(model.policy.mlp_extractor.policy_net.modules()) + [model.policy.action_net]
    layers = [layer for layer in actor if isinstance(layer, torch.nn.Linear)]
    weights = [layer.weight.detach().numpy() for layer in layers]
    biases = [layer.bias.detach().numpy() for layer in layers]
    obs = np.zeros((1, *model.observation_space.shape), dtype=np.float32)
    meta = {
        "backend": "SB3 predict deterministic",
        "layers": [layers[0].in_features] + [layer.out_features for layer in layers],
        "policy_trainable": sum(p.numel() for p in model.policy.parameters() if p.requires_grad),
        "training_steps": model.num_timesteps,
    }
    return meta, weights, biases, lambda: model.predict(obs, deterministic=True)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("results/model_scale.json"))
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    patterns = ("day1/K1/sources/**/*.onnx", "day1/policies/*.onnx", "day4/policies/*.onnx",
                "day4/microduck-playground/**/*.onnx", "day1/microduck_lab/results/*/ppo.zip",
                "day3/results/*/policy.zip")
    paths = {p for pattern in patterns for p in ROOT.glob(pattern)}
    for project in (ROOT / "day1/microduck_lab", ROOT / "day3"):
        for sample in (project / "model_samples").glob("*/*.zip"):
            trained = project / "results" / sample.parent.name / sample.name
            if trained not in paths:
                paths.add(sample)
    rows = []
    for path in sorted(paths):
        reader = describe_onnx if path.suffix == ".onnx" else describe_ppo
        meta, weights, biases, predict = reader(path)
        for _ in range(100):
            result = predict()
        assert result.shape == (1, meta["layers"][-1]) and np.isfinite(result).all()
        times = []
        for _ in range(1000):
            start = perf_counter_ns()
            predict()
            times.append((perf_counter_ns() - start) / 1000)
        meta.update(path=str(path.relative_to(ROOT)), file_bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    weight_count=sum(a.size for a in weights), bias_count=sum(a.size for a in biases),
                    dtype=str(weights[0].dtype), median_us=float(np.median(times)),
                    p95_us=float(np.percentile(times, 95)))
        rows.append(meta)
        print(meta["path"], round(meta["median_us"], 2), round(meta["p95_us"], 2), flush=True)
    cpu = next(line.split(":", 1)[1].strip() for line in Path("/proc/cpuinfo").read_text().splitlines()
               if line.startswith("model name"))
    report = dict(cpu=cpu, platform=platform.platform(), python=platform.python_version(),
                  onnxruntime=ort.__version__, torch=torch.__version__, sb3=stable_baselines3.__version__,
                  threads=1, batch=1, warmup=100, repetitions=1000, input="fixed zeros, float32", models=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
