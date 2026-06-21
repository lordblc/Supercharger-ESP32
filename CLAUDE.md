# Project Memory — Supercharger-ESP32

ESP32-S3 (LilyGo T-2CAN) charge controller for Zero motorcycles driving up to
4 Elcon TC HK-J 3300W chargers. Firmware lives in `development/Supercharger.ino`
plus headers; build/signing addon in `development_addon/`.

This file is the persistent decision/audit memory for AI-assisted sessions.
Append, don't rewrite history.

---

## Established design decisions (extracted from code comments / history)

These were deliberate choices — do not "fix" them without owner sign-off:

- **Auth model** (Supercharger.ino ~line 2117): form login + random 128-bit
  session cookie (`scs=`, HttpOnly, SameSite=Lax), replacing an earlier HTTP
  Digest design. Plaintext credentials over HTTP on the LAN are an accepted
  trade-off so password managers can autofill. Persistent ("keep me signed
  in") sessions are mirrored to NVS to survive the cold boot that happens at
  every charger power-on.
- **Hard lock honours existing sessions**: after 5 failed logins only NEW
  logins are blocked (423, 15 min auto-clear, BOOT-button 3–5 s manual clear).
  Rationale documented inline: an attacker can't hold a valid session.
- **Rate limiting is immediate-reject** (429 + Retry-After), never
  vTaskDelay, so the WebServer task is never blocked.
- **MQTT TLS refuses to connect without a pinned CA cert** rather than
  falling back to `setInsecure()` — "pretending to be secure is worse than
  plaintext".
- **OTA is HMAC-SHA256 signed** (32-byte trailer over the image, shared
  secret in `ota_secret.h` + `sign_ota.py`, both gitignored on real build
  hosts; repo copies are placeholders). Threat model excludes physical
  access (USB reflash is already possible).
- **When HTTPS is enabled, /update, /log, /api/log/stream, /save stay
  HTTP-only** (disabled, port 80 redirects) — IDF path lacks a
  multipart/SSE story. Documented escape hatch: `/api/tls` stays on HTTP.
- **MCP2515 init happens inside chargerBusTask on Core 0** — moving it to
  setup() (Core 1) crashes in SPI HAL (LoadProhibited). Don't move it.
- **Charger command defaults are STOP / 0 V / 0 A**; chargers cut out 5 s
  after heartbeat loss, so reboot/OTA mid-charge is fail-safe.
- **Session energy accumulates from MEASURED charger amps, not commanded
  power** (fixed earlier "Bug 3"); CC→CV has three triggers (voltage,
  current-taper, VCB plateau) added to fix 100 %-preset oscillation
  (v202605171800). External decision docs referenced in comments
  (`project_charging_bugs_2026-04-26.md`) are NOT in this repo.

---

## Audit 2026-06-09 (security / safety / stability)

Scope: all of `development/` and `development_addon/` at commit 442af9c.
No prior audit memory existed; this section is the baseline.

### Safety — open findings

1. **[HIGH] Signed/unsigned bug in `find_cutback()`**
   (`battery_tables.h:21`): `entries[i].threshold <= measurement` compares
   `uint32_t` with `int`; a negative temperature converts to a huge unsigned
   value, so ANY sub-zero `monolithMaxTemp` matches the 75 °C / 0-C-rate
   entry → charge power clamps to 0 W and the UI shows "thermal throttling"
   while the pack is freezing. Accidentally conservative, but wrong, and it
   masks finding 2. Fix: make threshold comparison signed.
2. **[HIGH] `COLD_CUTBACK` table is defined but never used.** rampTask
   (~line 5416) applies only `VOLTAGE_CUTBACK` and `HOT_CUTBACK`. Between
   0–10 °C the firmware allows full power (up to 13.2 kW ≈ >1 C) — lithium
   plating risk. `live.monolithMinTemp` is decoded and available but unused
   for protection.
3. **[HIGH] No staleness check on bike BMS data.** If the bike CAN drops
   mid-charge, `live.*` retains the last values forever (`dataFresh` never
   reverts); rampTask keeps commanding chargers from stale voltage/temps
   indefinitely. Chargers have CHARGER_TIMEOUT_MS; BMS data has no
   equivalent. Suggest: timestamp last BMS frame, stop charging when stale.
4. **[MED] Thermal protection ignores the PowerTank** — only
   `monolithMaxTemp` feeds `HOT_CUTBACK`; PowerTank temps are displayed but
   never limit power.
5. **[MED — verify hardware topology] PowerTank voltage is SUMMED into
   `rawPackDv`** (`rawPackDv += ptDv`, ~line 5339). On production Zeros the
   Power Tank is in parallel at the same pack voltage; summing would double
   the measured voltage → instant skip-to-DONE / max cutback, charging with
   a PowerTank fitted could never run. If the author's pack is genuinely in
   series this is fine — needs owner confirmation.
6. **[MED] No dead-man link between rampTask and chargerBusTask.** The
   heartbeat task re-sends the LAST command at 1 Hz forever; if rampTask
   stalls (mutex wedge, stack overflow hook parks a task but others keep
   running), the charger keeps charging unsupervised. Suggest a tick counter
   from rampTask that chargerBusTask checks, sending STOP if it stops
   advancing.

### Security — open findings

7. **[MED] PubSubClient / mqttCaCert cross-task race**:
   `applyApiSettingsBody()` (WebServer task, and IDF HTTPS task) calls
   `mqttClient.disconnect()` and reassigns the `mqttCaCert` String while
   mqttTask concurrently drives `mqttClient` and may be inside a TLS
   handshake holding `mqttCaCert.c_str()`. PubSubClient is not thread-safe;
   potential use-after-free. (Contradicts the "Owns PubSubClient
   exclusively" comment.) Suggest a "reconnect requested" flag consumed by
   mqttTask instead.
8. **[MED] Settings globals unsynchronised across servers**: when HTTPS is
   on, the WebServer task and the IDF httpd task can mutate
   `mqttHost/apSSID/...` and call `preferences.put*` concurrently (NVS
   itself is thread-safe; the char arrays / Strings are not).
9. **[LOW] Global login lockout = LAN DoS**: any LAN device can trip and
   re-trip the hard lock forever, blocking new logins (sessions survive).
   Accepted by design; per-IP tracking would mitigate.
10. **[LOW] Non-constant-time compares** for credentials
    (`String.equals`) and session tokens. The OTA HMAC compare IS
    constant-time. 128-bit tokens + rate limit make this academic; cheap fix.
11. **[LOW] `nextAllowedAttemptMs` uses absolute `now <` comparison** —
    not millis()-rollover-safe (49.7-day uptime edge); other timers use
    subtraction correctly.
12. **[LOW] `handleHTTPSRedirect` reflects the Host header** into the
    Location URL — open-redirect class, negligible on a LAN device.
13. **[LOW] Placeholder OTA key compiles silently**: `ota_secret.h` ships
    all-zeros; firmware built without generating a real key "verifies"
    against a public key, nullifying signed OTA. `sign_ota.py` placeholder
    fails loudly (non-hex), the firmware side doesn't. Suggest a
    compile-time guard (e.g. `#error` until edited).

### Stability — open findings

14. **[LOW] `buildApiStatusJson` stack buffer `char buf[1024]`** is close to
    worst-case output; silent snprintf truncation would emit invalid JSON.
15. **[LOW] SSE `sseClient.print` from loop()** can block the whole web
    server for the TCP timeout if the single log viewer stalls.
16. **[LOW] Charger `present` / `chargerCount` never decay** after
    CHARGER_TIMEOUT_MS (only per-frame checks use lastSeenMs), so
    `cmdStart` gating uses a count that can be stale-high.
17. **[LOW] `/cycles.csv` grows unbounded** until FFat is full; append then
    fails silently.
18. **[LOW] OTA streams through HMAC + `Update.write()` one byte at a
    time** — correct but CPU-heavy; emit in blocks for faster uploads.
19. **[LOW] HTTPS bodies > 8 KB are silently dropped** by
    `initFromIDFReq`, so a large cert+key upload over HTTPS yields a
    confusing "Invalid JSON" instead of 413 (HTTP path returns 413
    properly).

### Verified-good (don't re-flag)

- Every route, including OTA upload chunks (silent variant) and the legacy
  `/save`, is auth-gated; `/api/status` and settings JSON never return
  passwords or cert blobs.
- OTA HMAC design is sound: sliding 32-byte window, constant-time compare,
  abort-before-commit, sticky failure flag checked before `Update.hasError()`.
- CSRF: SameSite=Lax + POST-only logout/control; JSON APIs.
- CAN decoders length-check frames; charger status frames <4 bytes dropped;
  cell mV sanity-gated 1000–6000.
- Mutex discipline (live/charger/control/session/auth/sysStats) is broadly
  correct, with timeouts everywhere; auth state fails closed on mutex
  timeout.
- JSON parsing uses ArduinoJson with size caps (replaced indexOf parsers).
- WiFi fallback state machine (STA→secrets→AP+STA retry→AP) is sound;
  AP teardown only after 90 s stable STA.

---

## Fixes 2026-06-15 (safety findings #1, #2)

Both done in the local working tree (owner handles the git commit/push).

- **#1 RESOLVED — `find_cutback()` signedness** (`battery_tables.h`): both
  threshold comparisons now cast `entries[i].threshold` to `int`, so a
  negative `measurement` (sub-zero temp) compares as signed instead of
  wrapping to a huge unsigned value. All thresholds are small positive
  values, so the cast is lossless. This also makes COLD_CUTBACK behave
  correctly for negative temps (see #2).
- **#2 RESOLVED — COLD_CUTBACK now applied** (`Supercharger.ino` rampTask):
  reads `live.monolithMinTemp` into a local `minTemp`, computes
  `coldCbK`/`coldPwrW` via `find_cutback(minTemp, CUTBACK_AT_OR_BELOW,
  COLD_CUTBACK)`, and folds `coldPwrW` into `powerLimit` alongside the
  voltage and hot limits (CC phase only). Keyed off the coldest thermocouple
  (conservative for a plating limit). Ramp log gains a " COLD" annotation,
  shown only when the cold limit actually constrains power below target
  (the table returns a value at nearly any temp, so a bare non-MAX check
  would flag COLD at room temperature).
  - Notes for future sessions: the `g_thermalThrottle` dashboard banner is
    deliberately left HOT-only (its text says "temperature high"), so cold
    limiting is currently visible only in the ramp log, not the UI. At boot
    before the first 0x408 frame, `monolithMinTemp` defaults to 0 → cold
    table allows 0.2 C (~2.5 kW); with the default 100 W/s ramp, real temp
    data arrives (~3 s) well before commanded power reaches that limit, so
    the boot-time ambiguity is benign. Cold cutback still uses ONLY the
    monolith min temp — PowerTank temps remain unconsidered (finding #4,
    still open).  [SUPERSEDED 2026-06-16: #4 now folds PowerTank temps into
    BOTH hot and cold cutbacks — see below.]

---

## Fixes 2026-06-16 (safety findings #3, #4)

Done in the local working tree; pushed to `claude/jolly-ptolemy-xnajyd`.

- **#3 RESOLVED — bike BMS staleness guard** (`Supercharger.ino`):
  - `LiveData` gains `bms0LastMs`; `processBikeFrame()` stamps it `= now` on
    every monolith voltage frame (0x388, ~10 Hz) — the safety-critical
    control signal.
  - rampTask snapshots it as `bmsLastMs` and, right after the charger
    snapshot (before any phase/cutback/command logic), checks
    `(now0 - bmsLastMs) > BMS_STALE_TIMEOUT_MS` (5 s). If stale — or never
    received (bmsLastMs == 0, the boot state) — it commands STOP
    (cmdVolt/Amps=0, cmdStart=false), zeroes currentPowerW, clears throttle/
    ETA/taper/plateau state, rate-logs once per 5 s, and `continue`s. Uses
    rollover-safe millis() subtraction (unlike finding #11). Charging resumes
    automatically when fresh frames return. `phase`/`lastPhase` are untouched
    during a stall, so no spurious phase-entry hooks fire on resume.
  - Note: only the monolith voltage frame refreshes the timestamp. If 0x388
    keeps flowing but temp/amps frames (0x408) stop, staleness won't trip —
    judged acceptable since the whole bike bus drops together in practice and
    voltage is the essential control input.
- **#4 RESOLVED — PowerTank temps now limit power** (`Supercharger.ino`
  rampTask): snapshots `powerTankMaxTemp`/`powerTankMinTemp`; computes
  `hotTemp`/`coldTemp` as the worst case across both packs, folding the
  PowerTank in only when `ptPresent && ptDv > 0` (the same "PT is really
  here/live" gate already used for the voltage sum). HOT_CUTBACK now keys off
  `hotTemp`, COLD_CUTBACK off `coldTemp`. Because the fold uses max()/min(),
  a PowerTank reading 0 (no data / disconnected sensor) can never create a
  false HOT trigger; for COLD a momentary 0 is benign at boot for the same
  slow-ramp reason as the monolith. `g_thermalThrottle` now also reflects a
  hot PowerTank (banner text "pack temperature high" stays accurate).
  - Cold-side UI visibility (finding #2 note) is unchanged: still log-only.
