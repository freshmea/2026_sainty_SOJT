"""파일 경로와 결과 저장만 공유한다. 물리 계산은 각 예제에 있다."""
from pathlib import Path
import csv
import json
import mujoco

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)


def load_model(name):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / name))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def save_csv(name, fields, rows):
    path = RESULTS / name
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        writer.writerows(rows)
    return path


def save_json(name, value):
    path = RESULTS / name
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
    return path
