# R503 WAKE pin: finger presence

How the firmware decides whether a finger is on the R503. In short: the WAKE pin
is reliable only while the sensor is powered off. While it is powered, the sensor
itself answers through `GenImg`.

## WAKE pin basics

- WAKE (PA2) is active low: idle high, pulled low by the touch IC on contact.
  The touch IC runs from the always-on 3.3VT supply, so it works with the main
  sensor powered off. That is what wakes the system.
- **Use `Pull::None`.** WAKE is actively driven both ways. With the internal
  ~40 kΩ pull-up the low level reached only ~0.8 V, with steps at ~1.2 V. That
  is the undefined input region, so it gave spurious edges. Without the pull-up
  the low is 0 V and idle is a solid 3.3 V. If idle ever floats, add an external
  weak (~100 kΩ) pull-up, not the internal one.

## The problem

Symptom: on a successful match, WAKE rose ~300–350 ms after touch while the
finger was still on the sensor. The scope showed it doesn't rise once: it
toggles for as long as the sensor stays powered, several seconds.

The old design treated every WAKE edge as presence. A WAKE task published a
`bool` watch, debounced HIGH edges and ignored them during "command windows".
The toggling still leaked through:

- Every new LOW edge looked like a fresh press. After `unlock()` finished,
  `unlock_task` saw one and started a second recognition, though the finger was
  never lifted (confirmed: finger held → two full rounds).
- Between command windows, a false HIGH looked like a lift (enroll's
  "wait for finger removal").

### Ruled out

- **Lock actuator / supply sag:** it happens with the actuator disconnected.
- **Firmware:** the scope shows the physical pin moving.
- **LED aura effect:** the early theory, since the slow toggles seemed to follow
  Breathing and the fast bursts Flashing. A test with no effect on match (steady
  purple, sensor kept powered just as long) toggled the same way.

The toggling is the sensor's own behaviour while powered. It isn't documented:
the datasheet only says "WAKEUP – Finger Detection Signal", and the manual says
"standby high level, finger present output low level". No public reports were
found (ESPHome `fingerprint_grow` has unexplained touch oddities:
esphome/esphome#9719, esphome/issues#6128).

## The solution

**Trust WAKE only for the press, and only while the sensor is off. While it is
powered, ask the sensor:** `GenImg` returns `NoFinger` when there is no finger.

`fingerprint_manager_task` owns both the sensor and the WAKE pin:

```
loop:
    wait for (WAKE high → WAKE low) or a command on FINGERPRINT_CHANNEL
    power up the sensor
    run the flow (validate or enroll), which ends with the finger confirmed off
    power down
```

- While a flow runs, nobody listens to WAKE, so its toggling can't matter.
- Each flow ends with `wait_lift` (poll `GenImg` every 100 ms until `NoFinger`)
  before power-down. The next WAKE low is therefore a real new press.
- Waiting for high before low means a WAKE stuck low (wet glass, a flow that
  bailed on a bus error) can't re-run validate back to back and keep the
  sensor awake.
- Enroll uses `capture` (poll `GenImg` until an image is taken) to wait for the
  finger, and `wait_lift` to wait for removal.
- A match signals `UNLOCK_SIGNAL`. `unlock_task` only drives the solenoid and
  never touches the sensor, so there is no contention.
- `show_until_lift` starts the result LED effect, waits for the lift, then for
  the rest of the animation. Powering down neither cuts it short nor happens
  with a finger still on.
- `NoFinger` on the very first scan (a stray WAKE low, or too short a touch)
  ends silently, logging `wake without finger`.

This removed the WAKE task, the `SENSOR_ACTIVE` flag, `SensorGuard` command
windows, the resync signal, the debounce/settle delays and the presence watch.

### Costs

- While a finger stays on, the sensor stays powered (~20 mA) and is polled.
- A press during a running flow isn't seen until that flow ends. Since every
  flow ends on a lift, the next touch starts a new round normally.
- A lift and re-press within one 100 ms poll is seen as one placement.

## If it regresses

- `wake without finger` after every power-down → WAKE glitches low when the
  sensor powers down.
- A second recognition round with the finger held → something powers down or
  re-arms without `wait_lift`.
- If the issue ever needs vendor input: Grow support (hzgrow.com), with scope
  captures of WAKE alongside MCU TX.
