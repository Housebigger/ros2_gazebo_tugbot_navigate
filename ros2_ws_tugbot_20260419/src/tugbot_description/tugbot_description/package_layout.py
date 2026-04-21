from pathlib import Path
from typing import List, Tuple


def collect_data_files(package_root: Path, package_name: str) -> List[Tuple[str, list[str]]]:
    data_files: List[Tuple[str, list[str]]] = [
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ]

    model_root = package_root / 'models'
    for subdir in sorted(path for path in model_root.rglob('*') if path.is_dir()):
        files = sorted(str(path.relative_to(package_root)) for path in subdir.iterdir() if path.is_file())
        if files:
            relative_subdir = subdir.relative_to(package_root)
            data_files.append((f'share/{package_name}/{relative_subdir.as_posix()}', files))

    return data_files
