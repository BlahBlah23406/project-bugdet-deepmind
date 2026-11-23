# Robot Arm Dashboard

Control the arm (joints 2 + 3 and the claw) from your keyboard.

## Run it

1. Plug the arm's Arduino Mega into your computer over USB, and turn on the 12V.
2. Open `dashboard/index.html` in **Chrome** or **Edge** (not Safari or Firefox).
3. Click **Connect USB Serial** and pick the Arduino.
4. Click the page once, then use the keys below.

Only one program can use the Arduino at a time, so close the Arduino Serial Monitor first.

No arm? Click **Start Demo / Simulator** to try it.

## Keys

| Key | Does |
|---|---|
| W / S | Pitch up / down |
| A / D | Roll left / right |
| → / ← | Claw open / close |
| ↑ / ↓ | Faster / slower |
| Shift | Slow, precise moves (hold) |
| Space | Stop everything |
| E | Motors on / off |
| Z | Set this pose as zero |
| H | Home the claw |

Hold a key to move. Let go to stop.

## First time on a new arm

1. Move the arm to its home pose and press **Z**.
2. Hold **→** to open the claw fully. This also resets the claw's count.

The motors have no position sensors, so the arm counts its steps. If the page says the position is
not verified, or the claw angle shows a `~`, do the two steps above again.

## Upload the Arduino code (only once, or after changes)

Open `firmware/ArmDashboard/ArmDashboard.ino` in the Arduino IDE, install the **AccelStepper**
library, pick **Arduino Mega 2560** and click Upload.
