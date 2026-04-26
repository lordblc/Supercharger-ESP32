#!/usr/bin/env python3
"""
sign_ota.py — Append an HMAC-SHA256 trailer to a Supercharger .bin file.

Called automatically by the Arduino IDE post-build hook (see platform.local.txt
snippet in the project README), so you should never need to run this by hand.
Manual invocation:

    python sign_ota.py path/to/Supercharger_ESP32S3.ino.esp32s3.bin

The script reads the .bin, computes HMAC-SHA256(image, OTA_HMAC_SECRET), and
appends the resulting 32 bytes to the file in place. The firmware's OTA upload
handler strips and verifies these 32 bytes before flashing -- a mismatch (or
a missing trailer) aborts the update.

WARNING: the secret below MUST stay byte-identical to OTA_HMAC_SECRET in
ota_secret.h. If you rotate the key, rotate it in BOTH files at once. Like
ota_secret.h, this script is gitignored -- it must never leave the build host.
"""

import sys
import hmac
import hashlib

OTA_HMAC_SECRET = bytes.fromhex(
    "youneedtocreateyourownkeyhere"
)


def sign(path: str) -> int:
    with open(path, "rb") as f:
        data = f.read()
    sig = hmac.new(OTA_HMAC_SECRET, data, hashlib.sha256).digest()
    with open(path, "ab") as f:
        f.write(sig)
    print(
        "[sign_ota] {}: appended {} byte HMAC "
        "(image was {} bytes, now {})".format(
            path, len(sig), len(data), len(data) + len(sig)
        )
    )
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: sign_ota.py <path/to/firmware.bin>", file=sys.stderr)
        sys.exit(2)
    sys.exit(sign(sys.argv[1]))
