Import("env")
import os, shutil

def copy_firmware(source, target, env):
    # Extract VERSION and VERSION_BUILD from SCons CPPDEFINES
    defines = {}
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, (list, tuple)) and len(item) == 2:
            defines[str(item[0])] = str(item[1])

    version = defines.get("VERSION", '"0.0"').strip('"').replace(".", "")
    version_build = defines.get("VERSION_BUILD", "'a'").strip("'")

    if not version or not version_build:
        print("WARNING: Could not determine VERSION or VERSION_BUILD")
        return

    output_dir = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "build", "firmware")
    os.makedirs(output_dir, exist_ok=True)

    src = str(target[0])
    dst = os.path.join(output_dir, "LoRaTracker_V{}{}.bin".format(version, version_build))

    shutil.copy2(src, dst)
    print("\n*** Firmware saved: {} ***\n".format(dst))

env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware)
