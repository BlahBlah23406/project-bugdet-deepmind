"""Hardware check for the ArmDashboard firmware (PROTOCOL.md).

Usage:  python hw_test.py [port]            safe checks only, no motion
        python hw_test.py [port] --move     also tiny jogs (a few degrees) to check mixing, watchdog, G, limits

Needs pyserial. Close the dashboard first: only one program can hold the port.
"""
import json, sys, time
import serial

port = next((a for a in sys.argv[1:] if not a.startswith("-")), "/dev/cu.usbmodem101")
MOVE = "--move" in sys.argv
s = serial.Serial(port, 115200, timeout=0.05)
buf = b""
last = {}
fails = []


def send(line):
    s.write((line + "\n").encode())


def pump(secs, until=None):
    """Read lines for up to secs; return the first message that matches until(msg)."""
    global buf
    end = time.time() + secs
    while time.time() < end:
        buf += s.read(512)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            raw = raw.strip().decode(errors="replace")
            if not raw:
                continue
            try:
                m = json.loads(raw)
            except ValueError:
                print("  NON-JSON:", raw)
                fails.append("non-JSON output: " + raw[:60])
                continue
            last[m.get("t")] = m
            if m.get("t") in ("msg", "err", "hello"):
                print("  <", raw)
            if until and until(m):
                return m
    return None


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (" - " + detail if detail else ""))
    if not ok:
        fails.append(name)


def st():
    pump(0.25)                      # let earlier commands land, then take a fresh status
    return pump(0.5, lambda m: m.get("t") == "st") or {}


def jog(cmd, secs):
    """Send a jog line every 50 ms for secs, then stop and wait until nothing moves."""
    end = time.time() + secs
    while time.time() < end:
        send(cmd)
        pump(0.05)
    send("V 0 0 0")
    pump(3, lambda m: m.get("t") == "st" and m.get("mv") == 0)


print("Opening", port, "(the Mega resets, ~2 s)")
check("hello", pump(4, lambda m: m.get("t") == "hello") is not None)
send("?")
cfg = pump(1, lambda m: m.get("t") == "cfg")
check("cfg reply", cfg is not None, json.dumps(cfg))
a = st()
check("status line", "j2" in a, json.dumps(a))
t0 = time.time(); n = 0
while time.time() - t0 < 1.0:
    if pump(0.2, lambda m: m.get("t") == "st"):
        n += 1
check("status rate ~10 Hz", 6 <= n <= 14, f"{n}/s")
send("FOO")
check("unknown command -> err", pump(1, lambda m: m.get("t") == "err") is not None)
send("SPD 30"); check("SPD", pump(1, lambda m: m.get("t") == "st" and m.get("spd") == 30) is not None)

if MOVE:
    send("E 1"); send("SPD 30"); send("Z")
    a = st()
    check("Z -> trusted and 0", a.get("tr") == 1 and abs(a.get("p", 9)) < 0.05 and abs(a.get("r", 9)) < 0.05, json.dumps(a))

    jog("V 100 0 0", 0.4)          # pitch only: both motors the same way
    a = st()
    check("pitch jog moves both motors equally", a["j2"] != 0 and a["j2"] == a["j3"], f"j2={a['j2']} j3={a['j3']}")
    check("pitch jog leaves roll at 0", abs(a["r"]) < 0.05, f"r={a['r']}")

    jog("V 0 100 0", 0.4)          # roll only: opposite ways
    b = st()
    check("roll jog moves motors oppositely", (b["j2"] - a["j2"]) == -(b["j3"] - a["j3"]) != 0,
          f"d2={b['j2']-a['j2']} d3={b['j3']-a['j3']}")
    check("roll jog leaves pitch unchanged", abs(b["p"] - a["p"]) < 0.05, f"p {a['p']} -> {b['p']}")

    jog("V 70 40 0", 0.5)          # mixed: ramps must stay coordinated (pitch/roll ratio holds)
    c = st()
    dp, dr = c["p"] - b["p"], c["r"] - b["r"]
    check("mixed jog keeps pitch:roll ratio", dr != 0 and abs(dp / dr - 70 / 40) < 0.15, f"dp={dp:.2f} dr={dr:.2f}")

    send("V 100 0 0")              # one command, then silence: the watchdog must stop it
    t = time.time()
    m = pump(2, lambda m: m.get("t") == "st" and m.get("mv") == 0 and time.time() - t > 0.3)
    check("watchdog stops after ~400 ms", m is not None and m.get("wd") == 1, f"{time.time()-t:.2f}s")

    send("G 0 0")
    m = pump(6, lambda m: m.get("t") == "st" and m.get("mv") == 0 and time.time() - t > 0.5)
    a = st()
    check("G 0 0 returns to zero", a["j2"] == 0 and a["j3"] == 0, f"j2={a['j2']} j3={a['j3']}")

    send("LIM -2 2 -2 2")
    pump(0.3)
    jog("V 100 0 0", 1.5)
    a = st()
    check("soft limit holds pitch <= 2 deg", a["p"] <= 2.05, f"p={a['p']}")
    send("LIM OFF"); send("G 0 0"); pump(4, lambda m: m.get("t") == "st" and m.get("mv") == 0)

    send("E 0"); a = st()
    check("drivers off -> untrusted", a.get("en") == 0 and a.get("tr") == 0)
    send("G 1 1")
    check("G refused while untrusted", pump(1, lambda m: m.get("t") == "err") is not None)
    send("E 1"); send("Z"); pump(0.3)

send("S")
pump(0.3)
s.close()
print("\nALL PASSED" if not fails else "\nFAILED: " + ", ".join(fails))
sys.exit(1 if fails else 0)
