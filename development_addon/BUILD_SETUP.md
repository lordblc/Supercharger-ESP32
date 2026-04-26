# Build setup — secrets, OTA signing, gitignore

This sketch ships with two firmware-side secrets that must NEVER be
committed to a public repository:

| File              | Contents                                       |
|-------------------|------------------------------------------------|
| `arduino_secrets.h` | WiFi / MQTT / dashboard-OTA credentials      |
| `ota_secret.h`      | 32-byte HMAC-SHA256 key for signed OTA       |

The `sign_ota.py` script in this folder also contains the HMAC key in plain
text and must be treated identically.

Each of these files is generated locally on the build host and intentionally
varies per host — your HMAC key should not be the same as anyone else's.

## 1. Rotating the OTA key

Generate a new 32-byte hex string with `openssl rand -hex 32`, then update
**both** of:

- `OTA_HMAC_SECRET[32]` in `ota_secret.h` (32 comma-separated `0xNN` bytes)
- `OTA_HMAC_SECRET = bytes.fromhex("...")` in `sign_ota.py`

If they get out of sync, the firmware will reject every OTA upload — including
the one you're trying to deploy *to fix the mismatch*. Test locally before
flashing remotely.

## 2. Auto-sign every build (Arduino IDE)

You don't want to remember to run `python sign_ota.py` after each compile.
Set up a post-build hook so it runs automatically.

1. Find your installed ESP32 core's `platform.txt`. On Windows it's typically
   `%USERPROFILE%\AppData\Local\Arduino15\packages\esp32\hardware\esp32\<version>\platform.txt`.

2. In the **same directory** as that `platform.txt`, create or append to
   `platform.local.txt`:

   ```
   recipe.hooks.objcopy.postobjcopy.99.pattern=python "{build.source.path}/sign_ota.py" "{build.path}/{build.project_name}.bin"
   ```

   Notes:
   - The `99.` is the hook ordering — pick any number that doesn't collide
     with existing hooks. 99 is high enough to run after all the core's own
     post-objcopy steps.
   - `{build.source.path}` resolves to the sketch directory (where
     `sign_ota.py` lives), `{build.path}` is the temporary build output dir,
     `{build.project_name}` is `Supercharger`.

3. Restart the Arduino IDE so it re-reads `platform.local.txt`.

4. Compile any sketch. You should see a line like
   `[sign_ota] .../Supercharger.ino.esp32s3.bin: appended 32 byte HMAC ...`
   in the build output. The `.bin` on disk is now signed.

5. Upload via `/update` as normal. The firmware will accept the signed image
   and reject any unsigned (or wrong-signed) `.bin`.

## 3. Verifying the hook works

Easy sanity check: build, then try uploading an *unsigned* `.bin` via `/update`.  
The dashboard should report `Update rejected: signature invalid or upload incomplete.` and the device  
should keep running the previous firmware unchanged.
