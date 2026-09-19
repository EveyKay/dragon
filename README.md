# Dragon

Arduino sketches driving the servos for an animatronic dragon head: eyes, eyelids, jaw, and a three-segment neck.

## Repository layout

- [`dragon_servos_15/`](dragon_servos_15/dragon_servos_15.ino) — current sketch, the latest version.
- [`archive/`](archive/) — superseded versions 1–13, kept for history.
- [`experiments/`](experiments/) — one-off test sketches not part of the main dragon build.
- [`notes/`](notes/) — earlier code drafts and scratch notes saved as `.txt`.

## Wiring

8 servos, one Arduino, each servo's signal wire on its own digital pin. Power and ground for the servos should come from a supply rated for all 8 running at once — not the Arduino's 5V pin.

| Servo | Pin | Home angle | Safe range | Notes |
|---|---|---|---|---|
| `eyeLeft` | 2 | 90 | 30–150 | |
| `eyeRight` | 3 | 90 | 30–150 | |
| `eyelidLeft` | 4 | 90 | 60–120 | |
| `eyelidRight` | 5 | 90 | 60–120 | |
| `jaw` | 6 | 90 | 15–120 | |
| `neck1` | 7 | 90 | 30–150 | up/down |
| `neck2` | 8 | 90 | 30–150 | side-to-side sway |
| `neck3` | 9 | 90 | 30–150 | side-to-side sway |

Pins, home angles, and safe ranges are all defined in one place at the top of the sketch (`servoConfigs[]`), so rewiring a servo to a different pin or changing its limits doesn't require touching the animation code below it.

### Controlling it

Open the Serial Monitor at 9600 baud (line ending set to "Newline" or "Both NL & CR"). Commands:

- `<servoName> <angle>` — move one servo directly, e.g. `jaw 60`
- `blink` — synchronized eyelid blink
- `ror` — roar animation (neck drops, jaw opens, eyelids blink, neck sways)
- `ror two` — alternate roar: neck/jaw bob through two cycles with a mid-blink
- `look right` / `look left` / `front` — eyes and lower neck turn together, or reset to center
- `eyes closed` / `eyes open` — eyelids to fully closed/open
- `test1` — runs every animation above in sequence, for a quick smoke test after rewiring

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
| 15 | Jaw remounted normally, so the `reversed` flag is dropped again; roar animation (`ror`) and its jaw wobble/return phases run a bit quicker than in v13 |

All prior versions are archived in [`archive/`](archive/) rather than deleted, so earlier animation timings/approaches stay available for reference.
