# Robot Arm Dashboard

Control the arm (joints 2 + 3 and the servo claw) from your keyboard.

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
| H | Open the claw |

Hold a key to move. Let go to stop.

## Claw servo wiring

goBILDA 300° Torque servo:
- Signal (yellow/white) → **D11** (J27 IN4)
- Power (red) → a separate **5–7.4 V** supply (it can pull over 2 A, too much for the Arduino)
- Ground (black) → that supply's ground **and** the Arduino's GND

## Board health

The **Board health** panel shows how hard the Arduino is working:
- **5V rail**: should stay near 5 V. Below 4.6 V the board can reset or drop USB (usually the servo or motors pulling it down).
- **CPU load** and **worst pause**: high values mean the motor steps get delayed.
- **Free RAM**.
- **Current**: needs an INA219 sensor (about $3). Wire it in the 12V line, with SDA → D20, SCL → D21, VCC → 5V, GND → GND.
  It's found automatically at power-up.

## First time on a new arm

1. Move the arm to its home pose and press **Z**.
2. Move the claw to fully open and click **Set open point here**. Then close it as far as it
   should go and click **Set close limit here**. If → closes it, click **Flip claw direction**.

The arm motors have no position sensors, so the arm counts its steps. If the page says the position
is not verified, do step 1 again. The servo always knows its own position.

## Upload the Arduino code (only once, or after changes)

Open `firmware/ArmDashboard/ArmDashboard.ino` in the Arduino IDE, install the **AccelStepper**
and **Servo** libraries, pick **Arduino Mega 2560** and click Upload.
