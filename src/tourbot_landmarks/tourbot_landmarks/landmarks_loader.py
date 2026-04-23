from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def load_landmarks(map_name: str):
    share_dir = Path(get_package_share_directory("tourbot_landmarks"))
    yaml_path = share_dir / "config" / map_name / "landmarks.yaml"

    if not yaml_path.exists():
        raise FileNotFoundError(
            f"Landmark file not found for map '{map_name}': {yaml_path}"
        )

    with open(yaml_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)