from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def load_landmarks(map_name: str):
    """Loads the landmark information for a given map from a YAML file to shared memory.
    The YAML file should be located at `tourbot_landmarks/config/<map_name>/landmarks.yaml` 
    and contain a dictionary of landmark names to their properties 
    """
    
    share_dir = Path(get_package_share_directory("tourbot_landmarks"))
    yaml_path = share_dir / "config" / map_name / "landmarks.yaml"

    if not yaml_path.exists():
        raise FileNotFoundError(
            f"Landmark file not found for map '{map_name}': {yaml_path}"
        )

    with open(yaml_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)