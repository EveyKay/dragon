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
pio run -t upload    # compile and flash to the Uno
pio device monitor    # open the serial monitor at 9600 baud
```

Older sketches in `archive/` and `experiments/` are plain `.ino` files kept for reference — they aren't part of the PlatformIO build and would need their own environment if compiled again.

## Wiring

8 servos, driven through a **PCA9685 PWM driver board over I2C** rather than directly from the Arduino. Each servo plugs into its own channel on the PCA9685; the Arduino just sends it angle commands over I2C.

**Why not the `Servo` library directly:** that was the original design, but `Servo` relies on constant timer interrupts to hold every servo's position, which intermittently collided with `SoftwareSerial`'s interrupts for the DFPlayer — causing a glitch on a random servo after any sound-triggering command, and occasionally dropping the jaw (least torque margin) out entirely. Moving pulse generation to the PCA9685 removes the Arduino's timers from the picture, so there's nothing left for `SoftwareSerial` to collide with.

**PCA9685 wiring:**
- `VCC` → Arduino `5V`, `GND` → Arduino `GND`, `SCL` → Arduino `A5`, `SDA` → Arduino `A4` (logic/I2C side)
- `V+` (a separate screw terminal from the logic `VCC` pin) → the LM2596/battery rail, its `GND` → shared common ground — servos draw real current, so they're powered the same way the DFPlayer is, not from the Arduino
- Check for a jumper linking `VCC` and `V+` on your specific board and remove it if present — logic and servo power should stay electrically separate

| Servo | PCA9685 channel | Home angle | Safe range | Trim | Notes |
|---|---|---|---|---|---|
| `eyeLeft` | 2 | 90 | 30–150 | — | |
| `eyeRight` | 3 | 90 | 30–150 | — | |
| `eyelidLeft` | 4 | 90 | 60–120 | — | |
| `eyelidRight` | 5 | 90 | 60–120 | — | |
| `jaw` | 6 | 90 | 15–120 | — | |
| `neck1` | 7 | 90 | 30–150 | — | up/down |
| `neck2` | 8 | 90 | 30–150 | +15° | side-to-side sway |
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
- `clip5` — servo motion generated from [`sounds/clip_05.mp3`](sounds/clip_05.mp3)'s volume envelope (see below)
- `scan` — diagnostic: cycles PCA9685 channels 0–15, wiggling and announcing each one, for figuring out physical wiring on an unlabeled board
- `test1` — runs every animation above in sequence, for a quick smoke test after rewiring

### Sound module (DFPlayer Mini)

- DFPlayer TX → Arduino pin 10, DFPlayer RX → Arduino pin 11 (`SoftwareSerial`)
- DFPlayer VCC → the LM2596/battery rail (**not** the Arduino's 5V pin — its amp draws more current than the Arduino's own regulator can reliably supply, and starving it can brown out the whole board)
- DFPlayer GND → shared with the Arduino and servo ground (all one common ground)
- SD card: FAT32, with an `mp3` folder in the root containing `0001.mp3`, `0002.mp3`, etc. — `playMp3Folder(N)` plays `000N.mp3`
- `play <N>` in the Serial Monitor tests a track directly, independent of any animation

**Speaker:** the stock/bundled speaker that ships with most DFPlayer kits is quiet even at max software volume (`dfPlayer.volume(30)`, already set in the sketch). For a louder upgrade, look for:

- **4Ω impedance** (8Ω also works, but 4Ω lets the onboard amp deliver its full rated output)
- **3W power rating** — matches what the DFPlayer Mini's amp is built to drive
- **40–57mm diameter or larger** — noticeably louder/fuller than the tiny 20–28mm speakers commonly bundled with these kits, at the same wattage
- **Enclosed/housed**, not a bare driver — an enclosure stops the front and back sound waves from canceling out, which matters more for perceived volume than most spec differences
- Mounting it to fire into an enclosed cavity in the head (e.g. behind the mouth) with only a small opening to the outside can meaningfully boost volume for free, similar to an instrument body

If a speaker swap still isn't loud enough, the next step up is a small external amplifier (e.g. a PAM8403-based board) between the DFPlayer's line-level output and the speaker, bypassing the onboard amp's power ceiling entirely.

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
| 15 | Jaw remounted normally, so the `reversed` flag is dropped again; roar animation (`ror`) and its jaw wobble/return phases run a bit quicker than in v13; later given a DFPlayer Mini for sound (`clip5`, `play <N>`), then migrated from the `Servo` library to a PCA9685 driver board to fix an interrupt conflict between `Servo` and the DFPlayer's `SoftwareSerial` connection |

All prior versions are archived in [`archive/`](archive/) rather than deleted, so earlier animation timings/approaches stay available for reference.
