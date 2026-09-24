# Dragon

Arduino sketches driving the servos for an animatronic dragon head: eyes, eyelids, jaw, and a three-segment neck.

## Repository layout

- [`dragon_servos_15/`](dragon_servos_15/dragon_servos_15.ino) — current sketch, the latest version.
- [`archive/`](archive/) — superseded versions 1–13, kept for history.
- [`experiments/`](experiments/) — one-off test sketches not part of the main dragon build.
- [`notes/`](notes/) — earlier code drafts and scratch notes saved as `.txt`.
- [`sounds/`](sounds/) — dog-voiced audio takes referenced by sound-synced animations in the sketch.

## Building

The current sketch is set up as a [PlatformIO](https://platformio.org/) project (`platformio.ini` at the repo root, pointing at `dragon_servos_15/` as the source). PlatformIO IDE is installed as a VS Code extension for day-to-day editing, and the `pio` CLI works from this directory for scripted builds:

```
pio run              # compile
pio run -t upload    # compile and flash to the ESP32
pio device monitor    # open the serial monitor at 9600 baud
```

The sketch originally ran on an Arduino Uno; it was migrated to an ESP32 (see the version history below) for far more flash/RAM headroom, more GPIO for the planned physical buttons, and a free hardware UART for the DFPlayer instead of `SoftwareSerial`. The Uno version (and its `platformio.ini`) is still available in git history (commit `fe84b30` and earlier) if ever needed again.

Older sketches in `archive/` and `experiments/` are plain `.ino` files kept for reference — they aren't part of the PlatformIO build and would need their own environment if compiled again.

## Wiring

8 servos, driven through a **PCA9685 PWM driver board over I2C** rather than directly from the microcontroller. Each servo plugs into its own channel on the PCA9685; the microcontroller just sends it angle commands over I2C.

**Why not the `Servo` library directly:** that was the original design (back on the Uno), but `Servo` relies on constant timer interrupts to hold every servo's position, which intermittently collided with `SoftwareSerial`'s interrupts for the DFPlayer — causing a glitch on a random servo after any sound-triggering command, and occasionally dropping the jaw (least torque margin) out entirely. Moving pulse generation to the PCA9685 removes the microcontroller's timers from the picture, so there's nothing left for serial communication to collide with.

**PCA9685 wiring (ESP32):**
- `VCC` → ESP32 `3.3V`, `GND` → ESP32 `GND`, `SCL` → ESP32 `GPIO22`, `SDA` → ESP32 `GPIO21` (logic/I2C side)
- `V+` (a separate screw terminal from the logic `VCC` pin) → the LM2596/battery rail, its `GND` → shared common ground — servos draw real current, so they're powered the same way the DFPlayer is, not from the microcontroller
- Check for a jumper linking `VCC` and `V+` on your specific board and remove it if present — logic and servo power should stay electrically separate
- **`VCC` must actually be 3.3V, not left on a 5V/battery source.** The PCA9685's I2C pins get pulled up to whatever powers its `VCC`, and ESP32 GPIOs are 3.3V-only (not 5V-tolerant) — feeding them a 5V-referenced I2C bus risks damaging those pins over time even if communication appears to work. This exact mistake (VCC wired to the battery rail alongside V+ instead of to the ESP32's 3.3V pin) caused a real "servos won't move" debugging session — worth double-checking directly with a meter if servos ever go completely unresponsive after rewiring.

| Servo | PCA9685 channel | Home angle | Safe range | Trim | Notes |
|---|---|---|---|---|---|
| `eyeLeft` | 2 | 90 | 30–150 | — | |
| `eyeRight` | 3 | 90 | 30–150 | — | |
| `eyelidLeft` | 4 | 90 | 60–120 | — | |
| `eyelidRight` | 5 | 90 | 60–120 | — | |
| `jaw` | 6 | 90 | 15–120 | — | |
| `neck1` | 7 | 90 | 30–150 | — | up/down |
| `neck2` | 8 | 90 | 20–165 | +15° | side-to-side sway |
| `neck3` | 9 | 90 | 30–150 | +10° | side-to-side sway |

Channels, home angles, safe ranges, and trims are all defined in one place at the top of the sketch (`servoConfigs[]`), so rewiring a servo to a different channel or changing its limits doesn't require touching the animation code below it. `trim` corrects small per-servo mechanical differences (e.g. a horn seated slightly off) by nudging the actual PWM output while every animation still just thinks in terms of "90 = centered" — nothing outside `moveServo()` needs to know a trim exists.

If your board's channels aren't numbered, type `scan` into the Serial Monitor: it cycles through channels 0–15, wiggling each one briefly and printing its number, so you can watch which physical connector moves and match it up.

### Controlling it

Open the Serial Monitor at 9600 baud (line ending set to "Newline" or "Both NL & CR"). Commands:

- `<servoName> <angle>` — move one servo directly, e.g. `jaw 60`
- `blink` — synchronized eyelid blink
- `ror` — roar animation (neck drops, jaw opens, eyelids blink, neck sways), paired with [`sounds/clip_11.mp3`](sounds/clip_11.mp3)
- `ror two` — alternate roar: neck/jaw bob through two cycles with a mid-blink, paired with [`sounds/ror_two.mp3`](sounds/ror_two.mp3) (a single bark from a longer recording, sped up 1.5x and duplicated so its two peaks land exactly on this animation's two mouth-fully-open instants)
- `look right` / `look left` / `front` — eyes and lower neck turn together, or reset to center
- `eyes closed` / `eyes open` — eyelids to fully closed/open
- `tilt` — curious head tilt to a random side (neck2/neck3 lean, eyes glance the same way), no sound
- `yawn` — slow silent yawn/stretch: jaw opens wide, eyelids squint, neck leans back
- `look around` — scanning sweep: eyes/neck to one side, across to the other, back to center, no sound
- `look hold` — turns to a random side and lingers there for 5–12s before returning, unlike `look around`'s quick sweep, no sound
- `chomp` — three quick, shallow jaw snaps, like stretching the jaw, no sound
- `big tilt` — neck2 leans almost to its full safe range (with only a little follow-through on neck3) with a longer hold, no sound
- `look up` — neck cranes upward and holds, like sniffing the air, no sound
- `sleepy` — eyelids ease to a heavy half-closed squint with a slight downward droop, no sound
- `shake` — neck2 swings between its two safe extremes a few times, like a head shake, paced to stay under the speed cap, no sound
- `shake chomp` — eyes close, then neck2 shakes and the jaw does quick little chomps at the same time, like shaking something in its mouth, no sound
- `clip5` — servo motion generated from [`sounds/clip_05.mp3`](sounds/clip_05.mp3)'s volume envelope (see below)
- `idle` — ambient idle mode: continuous neck sway, regular blinking, occasional bigger animations and "freeze" pauses, all randomized, no sound (see below); type anything to stop it
- `scan` — diagnostic: cycles PCA9685 channels 0–15, wiggling and announcing each one, for figuring out physical wiring on an unlabeled board
- `stress test` — diagnostic: all 8 servos twitch ±12° around home in sync, back and forth, for as long as it runs -- the worst case for the shared servo power rail (every servo accelerates at once on every direction change), meant for reproducing/metering a power brownout rather than looking natural; type anything to stop it
- `test1` — runs every animation above in sequence, for a quick smoke test after rewiring

### Sound module (DFPlayer Mini)

- DFPlayer TX → ESP32 `GPIO32`, DFPlayer RX → ESP32 `GPIO33` — this is UART2, a real hardware serial port (the ESP32 has three independent UARTs, so unlike the Uno the DFPlayer gets its own dedicated port instead of `SoftwareSerial`). GPIO32/33 were picked over the more "conventional" default RX2/TX2 pins (16/17) because 16/17 double as the PSRAM interface on some ESP32 module variants, which would make them unusable as a UART regardless of external wiring.
- DFPlayer VCC → the LM2596/battery rail (**not** the microcontroller's own power pin — its amp draws more current than a microcontroller's own regulator can reliably supply, and starving it can brown out the whole board)
- DFPlayer GND → shared with the microcontroller and servo ground (all one common ground)
- SD card: FAT32, with an `mp3` folder in the root containing `0001.mp3`, `0002.mp3`, etc. — `playMp3Folder(N)` plays `000N.mp3`
- `play <N>` in the Serial Monitor tests a track directly, independent of any animation
- **Currently unresolved on the ESP32 build**: the DFPlayer isn't being detected (`dfPlayerReady` stays `false`, so sound-triggering animations run their servo motion silently) even with TX/RX crossed correctly and both wiring and power double-checked. Doesn't block any servo/animation work — every sound-triggering animation already checks `dfPlayerReady` before attempting playback.

**Speaker:** the stock/bundled speaker that ships with most DFPlayer kits is quiet even at max software volume (`dfPlayer.volume(30)`, already set in the sketch). For a louder upgrade, look for:

- **4Ω impedance** (8Ω also works, but 4Ω lets the onboard amp deliver its full rated output)
- **3W power rating** — matches what the DFPlayer Mini's amp is built to drive
- **40–57mm diameter or larger** — noticeably louder/fuller than the tiny 20–28mm speakers commonly bundled with these kits, at the same wattage
- **Enclosed/housed**, not a bare driver — an enclosure stops the front and back sound waves from canceling out, which matters more for perceived volume than most spec differences
- Mounting it to fire into an enclosed cavity in the head (e.g. behind the mouth) with only a small opening to the outside can meaningfully boost volume for free, similar to an instrument body

If a speaker swap still isn't loud enough, the next step up is a small external amplifier (e.g. a PAM8403-based board) between the DFPlayer's line-level output and the speaker, bypassing the onboard amp's power ceiling entirely.

### Idle mode

`idleAnimation()` (the `idle` command) is meant to make the dragon look alive with no operator input and no sound — the long-term plan is a physical button wired to trigger this, alongside separate "off" and "full random including sound" buttons. It layers several independent, randomly-timed behaviors:

- **Ambient sway** — `neck1`/`neck2`/`neck3` sway continuously, each on a different period so the combined motion doesn't look like a robotic uniform wobble.
- **Regular blinking** — every 3–6 seconds, independent of everything else.
- **Bigger animations** — every 10–20 seconds, one of `tilt` / `yawn` / `look around` / `look hold` / `chomp` / `big tilt` / `look up` / `sleepy` / `shake` / `shake chomp` fires, weighted so small/cheap gestures (`tilt`, `look around`, `chomp`) come up far more often than the dramatic ones (`big tilt`, `shake`, `shake chomp`), and never the same one twice in a row. Deliberately excludes "stateful" animations (`eyes closed`, `look right`/`look left`) that move somewhere and stay, since a random pick landing on one of those and not revisiting it for a while would look broken rather than alive. Also excludes `flinch`, which was removed after the servos would occasionally seize up on its fast startle snap.
- **Freezes** — every 6–10 seconds (between the blink and big-animation cadence), the sway eases down to a dead stop for 2–4 seconds, then eases back up — just a moment of stillness before it keeps moving.

The sway doesn't actually stop while a blink or bigger animation plays — `moveServosTogether()` drives whichever neck/eye axis a given call isn't already using itself (at full amplitude under a blink, since blinking never touches the neck; at a lessened amplitude under a bigger animation, since that one *is* actively steering some of those axes). This only activates while idle mode has set a global flag, so it's a no-op for any animation triggered directly from the Serial Monitor.

A few of the smaller animations (`tilt`, `yawn`, `look around`, `big tilt`, `look up`, `sleepy`, `chomp`) also jitter their own hold durations, rep counts, or depth slightly on every call, so the exact same animation doesn't look and time out identically every time it fires.

A jaw "breathing" micro-motion (a subtle idle wobble when nothing else was using the jaw) was tried and then removed -- it made the jaw move almost continuously instead of only during animations, and that was enough extra cycling to expose a marginal connection at the jaw servo's connector (it would intermittently lose all holding torque, fixed by reseating the connector). Worth revisiting once that wiring is confirmed solid.

Getting the freeze to not look jerky took a couple of real fixes worth knowing about if this code gets touched again: the sway's amplitude fades smoothly using the same exponential ease as everything else, but the sine wave's *phase* has to be explicitly frozen too (snapshotted at freeze start, resumed from exactly that point at freeze end) — otherwise the phase keeps advancing invisibly underneath a purely amplitude-based fade, and resuming can land on a fast-moving part of the cycle that fights the ramp-up. There was also a sign error where the fade-*out of* a freeze was computing time-remaining instead of progress-into-the-ramp, so it counted the wrong direction and re-zeroed itself right as the freeze ended.

### Sound-synced animations

`clip5Animation()` isn't hand-timed like the others — it's generated from the actual volume envelope of a recording. The audio is sampled in 40ms slices; the jaw and neck1 angles for each slice are derived directly from how loud that slice is, so the mouth snaps open and the head dips on every bark and eases back on the quiet stretches between them. The eyelids get one quick blink at the recording's single loudest instant. The per-frame angle tables live in the sketch itself (`clip5Jaw[]` / `clip5Neck1[]`); the source audio is kept in [`sounds/`](sounds/) for reference and for future resyncing if a sound module gets added.

To generate a new one from another clip: run the file through a high-pass + FFT denoise pass, sample RMS loudness in fixed time slices (e.g. via `ffmpeg`'s `astats`/`ametadata` filters), normalize and smooth the envelope, then map it to servo angles the same way `clip5Jaw`/`clip5Neck1` do.

## Version history

| Version | Added |
|---|---|
| 1 | Initial sketch — 8 servos, `blink` command |
| 2–8 | `ror` (roar) animation added; iterated on timing/feel |
| 9 | `look right` / `look left` / `front` |
| 10 | Refinements to the roar animation |
| 11 | `ror two` (alternate roar) |
| 12 | `eyes closed` / `eyes open`, `test1` smoke-test command |
| 13 | Per-servo `reversed` flag for backwards-mounted servos (`jaw` was mounted reversed at the time); logical angle tracked in code instead of read back from the servo |
| 15 | Jaw remounted normally, so the `reversed` flag is dropped again; roar animation (`ror`) and its jaw wobble/return phases run a bit quicker than in v13; later given a DFPlayer Mini for sound (`clip5`, `play <N>`), then migrated from the `Servo` library to a PCA9685 driver board to fix an interrupt conflict between `Servo` and the DFPlayer's `SoftwareSerial` connection; added sound-free animations (`tilt`, `yawn`, `look around`, `flinch`, `look hold`) and an `idle` mode that layers ambient sway, blinking, freezes, and those animations together randomly; added a hard per-servo speed cap; `flinch` removed after its fast startle snap kept making the servos seize up, replaced in the idle pool by `chomp`, `big tilt`, `look up`, and `sleepy`; added `shake` and `shake chomp`; migrated from an Arduino Uno to an ESP32 (`stress test` diagnostic command added along the way to help track down a servo power brownout); DFPlayer detection still unresolved on the new board |

All prior versions are archived in [`archive/`](archive/) rather than deleted, so earlier animation timings/approaches stay available for reference.
