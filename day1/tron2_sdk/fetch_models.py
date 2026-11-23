"""Install the XML and meshes used by the four TRON2A simulation labs."""
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha1
import json
from pathlib import Path
from http.client import IncompleteRead
from tempfile import NamedTemporaryFile
from time import sleep
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

REVISION = "9939c22e69d27653ec0ba8a505859a2903dd1a71"
REPO = "limxdynamics/tron2-robot-description"
ROOT = Path(__file__).parent / "vendor/tron2_mujoco_sim/robot-description"


def fetch(row):
    name = row["path"]
    path = ROOT / name
    def matches(data):
        return sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest() == row["sha"]
    if path.is_file() and matches(path.read_bytes()):
        return
    url = f"https://raw.githubusercontent.com/{REPO}/{REVISION}/{name}"
    for attempt in range(4):
        try:
            with urlopen(url, timeout=120) as stream:
                data = stream.read()
            if not matches(data):
                raise ValueError(f"Git blob checksum mismatch: {name}")
            break
        except (OSError, URLError, IncompleteRead, ValueError) as exc:
            if isinstance(exc, HTTPError) and 400 <= exc.code < 500 and exc.code not in (408, 429):
                raise
            if attempt == 3:
                raise RuntimeError(f"Model download failed: {name}. Rerun to resume.") from exc
            sleep(2 ** attempt)
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(dir=path.parent, delete=False) as temp:
        temp.write(data)
    Path(temp.name).replace(path)


if __name__ == "__main__":
    selected = json.loads(Path(__file__).with_name("model_files.json").read_text())
    with ThreadPoolExecutor(max_workers=6) as pool:
        list(pool.map(fetch, selected))
    print("Verified upstream model files:", len(selected))
