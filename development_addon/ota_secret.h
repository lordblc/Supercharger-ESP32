// =============================================================================
// OTA HMAC secret — YOU NEED TO CREATE YOUR OWN
// =============================================================================
//
// This 32-byte (256-bit) random secret is the shared key for the application-
// level signed-OTA scheme. The same bytes must live in:
//   - this file (compiled into firmware → used to verify uploaded .bin files)
//   - sign_ota.py next to the sketch (used by the post-build hook to append
//     an HMAC-SHA256 trailer to every freshly-built .bin)
//
// If you regenerate the secret, regenerate it in BOTH places at the same time.
// Otherwise the firmware will reject every OTA upload, including ones it
// produced itself.
//
// Threat model:
//   - Protects against an attacker with leaked OTA credentials uploading
//     arbitrary firmware. Without the matching HMAC trailer, the firmware
//     refuses to flash.
//   - Does NOT protect against an attacker with physical access to a flashed
//     device — they can extract the secret from flash. But physical access
//     already lets them reflash via USB, so this isn't widening the attack
//     surface.
//
// 
// =============================================================================

#pragma once
#include <stdint.h>

static const uint8_t OTA_HMAC_SECRET[32] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
