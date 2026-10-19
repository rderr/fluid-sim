# PlatformIO pre-upload guard: refuse to flash a board whose MAC is not in
# the environment's `custom_expected_macs` list. Two ESP32-S3 boards on one
# USB hub can share a port name, and flashing the wrong one is not fun.
#
# platformio.ini:
#   extra_scripts = pre:tools/check_mac.py
#   custom_expected_macs = 24:ec:4a:2a:f6:44   ; comma-separated, or empty to skip
Import("env")
import subprocess, re, sys

def check_mac(source, target, env):
    expected = [m.strip().lower() for m in env.GetProjectOption("custom_expected_macs", "").split(",") if m.strip()]
    if not expected:
        print("check_mac: no custom_expected_macs set for this env, skipping the guard")
        return
    port = env.subst("$UPLOAD_PORT")
    if not port:
        env.AutodetectUploadPort()
        port = env.subst("$UPLOAD_PORT")
    esptool = env.subst("$PYTHONEXE") , env.subst("$UPLOADER")
    cmd = [esptool[0], esptool[1], "--chip", env.BoardConfig().get("build.mcu"), "--port", port,
           "--before", "default_reset", "--after", "hard_reset", "read_mac"]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    macs = set(m.lower() for m in re.findall(r"MAC: ([0-9a-fA-F:]{17})", out))
    if not macs:
        sys.stderr.write("check_mac: could not read a MAC from %s; refusing to upload\n%s\n" % (port, out[-400:]))
        env.Exit(1)
    if not (macs & set(expected)):
        sys.stderr.write("check_mac: board on %s has MAC %s, expected one of %s. Refusing to upload.\n"
                         % (port, ", ".join(sorted(macs)), ", ".join(expected)))
        env.Exit(1)
    print("check_mac: %s matches an expected MAC, uploading" % ", ".join(sorted(macs)))

env.AddPreAction("upload", check_mac)
