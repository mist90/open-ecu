#!/usr/bin/env python3
"""Recreate the pinned, minimal STM32F4 vendor archive (normally already shipped)."""
import hashlib
import io
from pathlib import Path
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parent
PACKAGES = (
    ("STMicroelectronics/stm32f4xx-hal-driver", "c2e1406d7ea4b73aa42b98ddeb75a8670b1a5a16",
     "a78844548d362a8548935f877201b3deafad9aac93fc4815bc3c2fa20f99dc02", "STM32F4xx_HAL_Driver"),
    ("STMicroelectronics/cmsis-device-f4", "5f41fb29d22773896c780052bf61e47fc924d524",
     "7b70f5e3613eac5472426b206765620aa4192d96df1f21a3a5d84ea999f70c41", "CMSIS/Device/ST/STM32F4xx"),
)


def download(url):
    with urllib.request.urlopen(url, timeout=120) as response:
        return response.read()


def main():
    files = {}
    for repo, revision, checksum, destination in PACKAGES:
        data = download(f"https://codeload.github.com/{repo}/tar.gz/{revision}")
        if hashlib.sha256(data).hexdigest() != checksum:
            raise RuntimeError(f"Checksum mismatch: {repo}")
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
            for entry in archive:
                if not entry.isfile():
                    continue
                relative = entry.name.split("/", 1)[1]
                if relative.startswith(("Inc/", "Include/", "Src/", "Source/Templates/")) or relative == "LICENSE.md":
                    files[f"Drivers/{destination}/{relative}"] = archive.extractfile(entry).read()

    # CMSIS core headers are taken from the same STM32CubeF4 release family.
    revision = "2b7495b8535bdcb306dac29b9ded4cfb679d7e5c"  # CMSIS_5 5.9.0
    for name in ("cmsis_compiler.h", "cmsis_gcc.h", "cmsis_version.h", "core_cm4.h", "mpu_armv7.h"):
        files[f"Drivers/CMSIS/Include/{name}"] = download(
            f"https://raw.githubusercontent.com/ARM-software/CMSIS_5/{revision}/CMSIS/Core/Include/{name}")
    files["Drivers/CMSIS/LICENSE.txt"] = download(
        f"https://raw.githubusercontent.com/ARM-software/CMSIS_5/{revision}/LICENSE.txt")

    with tarfile.open(ROOT / "Drivers.tar.gz", "w:gz") as archive:
        for name, data in sorted(files.items()):
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            entry.mode = 0o644
            archive.addfile(entry, io.BytesIO(data))
    print(f"Created {ROOT / 'Drivers.tar.gz'} ({len(files)} files)")


if __name__ == "__main__":
    main()
