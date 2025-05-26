# UAV Autopilot Demo

> **Mission Planner SITL ✦ DroneKit ✦ ALT‑HOLD waypoint flight**

This project contains a minimal proof‑of‑concept that takes off in **ALT\_HOLD**, flies the simulated copter from **Point A (Home)** to **Point B**, yaws **350 °**, and lands.
Everything happens inside a local **Mission Planner SITL** instance; no real hardware is required.

---

## Contents

* [Architecture](#architecture)
* [Prerequisites](#prerequisites)
* [Local quick‑start](#local-quick-start)
* [Docker quick‑start](#docker-quick-start)

---

## Architecture

```
.
├── app/               Python package with the flight logic
│   ├── driver.py      High‑level DroneKit wrapper with PID helpers
│   ├── flying.py      Mission entry‑point (`flying()`)
│   ├── telemetry.py  Low‑level helpers (mode, arm, position)
│   ├── schemas.py    Typed DTOs & settings (Pydantic)
│   └── logger.py     Global logging config
├── run.py            CLI wrapper – runs `app.flying.flying()`
├── Dockerfile        Multi‑stage image (Poetry + runtime)
├── docker‑compose.yml One‑shot service mapped to host `.env`
└── pyproject.toml    Poetry project + dependencies
```

The code keeps **Mission Planner** clean: all stick‑input replacements are sent over MAVLink *only while needed*, then neutralised.

---

## Prerequisites

| Tool                                     | Purpose                      | Notes                                                            |
| ---------------------------------------- | ---------------------------- | ---------------------------------------------------------------- |
| **Mission Planner**                      | Ground station + SITL binary | [https://ardupilot.org/planner/](https://ardupilot.org/planner/) |
| **Python ≥ 3.9**                         | Runs the control script      | Poetry manages deps                                              |
| **DroneKit‑Python ≈ 2.9**                | High‑level MAVLink API       | Installed by Poetry                                              |
| **Pymavlink**                            | Low‑level MAVLink packets    | Installed by Poetry                                              |
| **Git**                                  | Clone + push repo            |                                                                  |
| *(optional)* **Docker & Docker Compose** | Reproducible sandbox         | If you prefer containers                                         |

### Ports

* **127.0.0.1:5760** – Mission Planner GCS (default)
* **127.0.0.1:5762** – **Recommended**: dedicate to the script
* **127.0.0.1:5763** – Extra, free

---

## Local quick‑start

```bash
# 1 – Clone & enter
$ git clone https://github.com/<your‑account>/uav‑demo.git
$ cd uav‑demo

# 2 – Launch Mission Planner SITL (QuadPlane or Copter) ↩︎
#    In the MP toolbar: *Simulation ▶︎ X‑Plane SITL* (choose any quad‑frame)
#    ✔ A UDP endpoint on 127.0.0.1:5760 appears.

# 3 – Install Poetry & deps
$ curl -sSL https://install.python-poetry.org | python3 -
$ poetry install

# 4 – Create .env
$ echo "UAV_CONNECTION_STRING=127.0.0.1:5762" > .env

# 5 – Run script (new terminal)
$ poetry run python run.py
```

You should see logs like:

```
00:00:01 | INFO     | Connecting to vehicle @ 127.0.0.1:5762…
00:00:05 | INFO     | Vehicle connected – FW: …
…
00:00:12 | INFO     | Take‑off – target_altitude=100.0 m
…
```

Open the *Flight Data* tab in Mission Planner – the green vehicle icon will climb to 100 m, fly south‑west to Point B, yaw almost‑north (350 ° absolute), then land.

---

## Docker quick‑start

> **Tip:** the container is mapped to `./app`, so live‑code reload is instant – stop/re‑run to test changes.

```bash
# 1 – Build image (first time)
$ docker compose build

# 2 – Run flight
$ docker compose up
```

Environment variables are injected from **.env** the same way as local.


