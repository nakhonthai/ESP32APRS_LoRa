Import("env")
import os, re, shutil

FIRMWARE_NAMES = {
    "ht-ct62":            "HTCT62",
    "esp32c3-mini":       "ESP32C3mini",
    "esp32-sh1106_8MB":   "ESP32SH1106-8B",
    "ttgo-lora32-v1":     "TTGO_LoRa32V1",
    "ttgo-lora32-v1_6":   "TTGO_LoRa32-V16",
    "ttgo-t-beam-v1_2":   "TTGO_TBeam-V2",
    "heltec_htit_tracker":"HTITTracker",
    "t_beam_s3_supreme":  "TBEAM-S3-SUPREME",
    "heltec_V3_GPS":      "HELTEC-LoRa32-V3",
    "aprs-lora-dongle":   "LoRaDongle",
    "t_beam_s3_bpf":      "TBEAM-S3-BPF",
    "t_beam_s3_1w":       "TBEAM-S3-1W",
    "aprs-lora-ht":       "LoRaHT",
    "NAWS4-ESP32S3":       "NAWS4",
}

def copy_firmware(source, target, env):
    # Extract VERSION and VERSION_BUILD from SCons CPPDEFINES
    defines = {}
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, (list, tuple)) and len(item) == 2:
            defines[str(item[0])] = str(item[1])

    version = re.sub(r'[\\"\']', '', defines.get("VERSION", "0.0")).replace(".", "")
    version_build = re.sub(r'[\\"\']', '', defines.get("VERSION_BUILD", "a"))

    env_name = env["PIOENV"]
    prefix = FIRMWARE_NAMES.get(env_name, env_name)

    output_dir = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "build", "firmware")
    os.makedirs(output_dir, exist_ok=True)

    src = str(target[0])
    dst = os.path.join(output_dir, "{}_V{}{}.bin".format(prefix, version, version_build))

    shutil.copy2(src, dst)
    print("\n*** Firmware saved: {} ***\n".format(dst))

env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware)
