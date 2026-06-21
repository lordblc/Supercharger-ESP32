// Auth types — kept in a separate header to dodge the Arduino IDE 2.x
// auto-prototype generator. The IDE prepends forward declarations of all
// top-level functions to the .ino BEFORE the rest of the body, so any type
// referenced by a function signature must be visible to the preprocessor at
// that early position. Defining LoginOutcome inline in the .ino causes
// `static LoginOutcome tryLogin(...)` to be auto-prototyped before the
// struct exists → "does not name a type" build error.
//
// Same workaround already used for CycleRecord (cycle_record.h) and
// HttpCtx (https_ctx.h).
//
// Used by Supercharger_ESP32S3.ino — tryLogin() returns LoginOutcome.

#pragma once

#include <stdint.h>

enum LoginResult : uint8_t {
  LOGIN_OK,
  LOGIN_BAD_CREDS,
  LOGIN_RATE_LIMITED,
  LOGIN_HARD_LOCKED,
};

struct LoginOutcome {
  LoginResult   result;
  unsigned long retryAfterSec;
  char          token[33];   // 32 hex chars + NUL — valid only when result == LOGIN_OK
};

// Per-client-IP login lockout slot (finding #9). Defined here, not in the .ino,
// for the SAME Arduino auto-prototype reason as LoginOutcome: the helpers
// authIpSlot_nolock()/hardLockMaybeAutoExpire_nolock()/peekLockStatus() take or
// return AuthIpSlot, and their auto-generated prototypes are emitted near the
// top of the .ino — before any struct defined inline there would be visible.
struct AuthIpSlot {
  bool          inUse                = false;
  uint32_t      ip                   = 0;   // client IPv4 key (0 = unknown client)
  uint8_t       failCount            = 0;
  unsigned long nextAllowedAttemptMs = 0;
  bool          hardLocked           = false;
  unsigned long hardLockSinceMs      = 0;
  unsigned long lastSeenMs           = 0;   // for LRU eviction
};

