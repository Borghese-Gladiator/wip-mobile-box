Great idea. Let’s turn **Mobile Box** into something structured and buildable.

---

# 📄 Product Requirements Document (PRD)

**Project Name:** Mobile Box
**Version:** v1.0
**Owner:** Tim

## 1. Product Overview

Mobile Box is a small autonomous robot that:

* Connects to a smartphone via Bluetooth
* Displays both phone and robot location on a map
* Follows the phone when “Follow Me” is pressed
* Uses tank-style differential drive for movement

Primary use case: a robot that follows the user and carries items.

---

## 2. Functional Requirements

## 2.1 Mobile App

### Map Screen

* Display map (Google Maps / Apple Maps)
* Show:

  * Phone position (live GPS)
  * Robot position (live estimate)
* Auto-refresh at 1 Hz minimum

### Controls

* Button: **“Follow Me”**

  * Sends follow command to robot
* Button: **“Stop”**

  * Immediately halts movement
* Bluetooth connection status indicator

---

## 2.2 Robot

### Connectivity

* Bluetooth Low Energy (BLE)
* Reconnect automatically
* Send telemetry:

  * Battery level
  * Position estimate
  * IMU data
  * Motor state

### Movement

* Differential drive (tank style)

  * Left motor
  * Right motor
* Speed control via PWM
* Turning via differential velocity

### Sensors

Minimum viable sensor set:

| Sensor              | Purpose                       |
| ------------------- | ----------------------------- |
| Wheel encoders      | Odometry (distance + heading) |
| IMU (gyro + accel)  | Orientation + smoothing       |
| Ultrasonic or LiDAR | Obstacle detection            |
| BLE RSSI            | Rough distance to phone       |

---

## 3. Non-Functional Requirements

* Max speed: 1.5 m/s
* Follow distance: 1–2 meters
* Latency: <200ms control loop
* Battery life: 1–2 hours
* Outdoor and indoor capable

---

## 4. System Architecture

### High-Level

```
[Phone App]
    |
 Bluetooth BLE
    |
[Robot MCU]
    |
Sensors + Motors
```

Phone does:

* UI
* GPS tracking
* High-level control

Robot does:

* Motion control
* Sensor fusion
* Obstacle avoidance
* Low-level autonomy

---

# 🔧 Implementation Plan

Now let’s make it real.

---

# 🧠 Hardware Selection

## Robot Brain

**Option A (Recommended):**

* Raspberry Pi 5 (high-level logic)
* Arduino Nano (motor control)

OR

**Option B (Simpler):**

* ESP32 (does BLE + motor control)

  * Cheap
  * Integrated Bluetooth
  * Real-time capable

I recommend **ESP32** for v1.

---

## Motors

* 2x 12V DC gear motors
* Tank tread or two large wheels
* Motor driver:

  * L298N (cheap)
  * Or better: TB6612FNG

---

## Sensors

### 1. Wheel Encoders

* Hall effect encoders or optical encoders
* Needed for odometry

### 2. IMU

* MPU6050 (cheap)
* Or BNO055 (better — does onboard fusion)

### 3. Obstacle Detection

* HC-SR04 ultrasonic (cheap)
* OR
* RPLidar A1 (real robot-level)

---

# 📱 Mobile App Stack

## Option A: Flutter (Cross Platform)

* Google Maps SDK
* BLE plugin

## Option B: React Native

## Option C: Native Swift / Kotlin

I’d recommend:

> Flutter + BLE + Google Maps SDK

---

# 🧠 Core Algorithms

Now the interesting part.

---

# 📍 How Does the Robot Know Its Position?

### 1. Outdoor:

Use phone GPS as truth.
Robot estimates relative position.

### 2. Indoor:

Use:

* Wheel odometry
* IMU
* BLE signal strength
* Possibly UWB for precision

---

# 🤖 Follow Algorithm

You have two main approaches:

---

## Option 1: BLE RSSI Following (Simple)

Robot measures signal strength:

```
if RSSI < threshold_far:
    move forward
if RSSI > threshold_close:
    stop
```

Problem:

* No direction.
* Only distance approximation.
* Very noisy.

Not great.

---

## Option 2 (Better): Phone Sends GPS + Heading

Phone calculates:

```
phone_position = (lat, lon)
robot_position = estimate_from_odometry()

vector = phone_position - robot_position
desired_heading = atan2(dy, dx)
```

Robot turns toward heading and moves forward.

---

## Core Control Loop (Pseudo Code)

```
loop every 100ms:

    read encoders
    read IMU
    update position estimate

    if follow_mode:
        target = phone_position
        error_vector = target - current_position

        desired_angle = atan2(error_y, error_x)
        angle_error = desired_angle - current_heading

        left_speed  = base_speed - Kp * angle_error
        right_speed = base_speed + Kp * angle_error

        set_motors(left_speed, right_speed)

    check obstacles
    if obstacle_detected:
        stop or reroute
```

---

# 🗺️ Map Implementation

Phone:

* Displays:

  * Blue dot = phone GPS
  * Red dot = robot position (from robot telemetry)
* Robot sends position updates every 500ms via BLE

---

# ❓ Your Z-Position Question

You asked:

> How does a Roomba do Z position?

Great question.

### Short answer:

It doesn’t track absolute Z like GPS.

### Example: iRobot Roomba

Modern Roombas use:

1. Wheel encoders
2. IMU
3. Optical flow sensors (looking at floor)
4. SLAM (Simultaneous Localization and Mapping)
5. Camera (in higher-end models)

They estimate position relative to a starting point.

No GPS.
No global coordinates.
Pure sensor fusion.

---

# 🧠 What is SLAM?

SLAM = build a map while tracking your location inside it.

Uses:

* LiDAR or camera
* IMU
* Wheel encoders

If you want Mobile Box v2:
→ Add LiDAR
→ Run SLAM on Raspberry Pi
→ Use ROS2

---

# 🚀 Phase-Based Development Plan

---

## Phase 1 — Remote Control Only

* BLE connection
* Manual joystick control
* Motor driver working

Goal: Robot moves reliably.

---

## Phase 2 — Odometry

* Add encoders
* Track distance traveled
* Display robot position relative to start

Goal: Position estimate on map.

---

## Phase 3 — Follow Me (Outdoor GPS)

* Phone sends GPS
* Robot moves toward GPS target
* Add basic obstacle avoidance

Goal: Works in open areas.

---

## Phase 4 — Indoor Following

* Add LiDAR
* Implement SLAM
* Add obstacle navigation

Goal: Works in house.

---

# ⚡ If You Want “Real Robotics Level”

Use:

* Raspberry Pi 5
* RPLidar
* ROS2
* Navigation stack

That puts you in Roomba-tier architecture.

---

# 🧠 Hard Problems You’ll Hit

1. BLE latency
2. RSSI instability
3. Wheel slip
4. GPS drift
5. Indoor localization
6. Power management
7. Obstacle avoidance edge cases

---

# 🔥 If I Were Building This

I’d do:

ESP32 + encoders + IMU + ultrasonic
Flutter app
Outdoor GPS following first
Then upgrade to SLAM later

---

If you want, I can next:

* Design full hardware wiring diagram
* Write ESP32 firmware skeleton
* Write Flutter BLE + map starter code
* Or design the full SLAM-based version

Where do you want to go next?


---
Perfect. This is exactly how real robotics teams do it: **simulate first, break it in software, then touch hardware**.

We’ll build:

* A physics simulation
* Differential drive robot
* Phone moving target
* Follow algorithm
* Obstacle avoidance
* Real-time visualization

All in pure Python.

---

# 🧠 What We’ll Simulate

* 2D world (meters)
* Robot with:

  * (x, y)
  * heading θ
  * left/right wheel speeds
* Phone moving along path
* Follow controller
* Optional obstacle
* Real-time animation

---

# 🧰 Stack

* Python 3
* numpy
* matplotlib (animation)
* no external physics engine required

---

# 🚗 Differential Drive Physics

Robot state:

```
x, y        # position
theta       # heading (radians)
v_l, v_r    # wheel velocities
```

Physics:

```
v = (v_r + v_l) / 2
omega = (v_r - v_l) / wheel_base
```

Update:

```
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt
```

---

# 🎯 Follow Algorithm

```
error_vector = phone_pos - robot_pos
desired_heading = atan2(dy, dx)
angle_error = wrap(desired_heading - theta)

v_l = base_speed - Kp * angle_error
v_r = base_speed + Kp * angle_error
```

If close → stop.

---

# 🧪 Full Working Simulation Code

Copy this into a file: `mobile_box_sim.py`

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =============================
# Simulation Parameters
# =============================
dt = 0.05
wheel_base = 0.4
base_speed = 1.0
Kp = 2.0
follow_distance = 0.8

world_size = 10

# =============================
# Robot State
# =============================
robot = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,
    "v_l": 0.0,
    "v_r": 0.0
}

# =============================
# Phone Path (Target)
# =============================
def phone_position(t):
    # Circular motion
    radius = 3
    speed = 0.4
    angle = speed * t
    return np.array([radius * np.cos(angle),
                     radius * np.sin(angle)])

# =============================
# Helper Functions
# =============================
def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def update_robot():
    v_l = robot["v_l"]
    v_r = robot["v_r"]

    v = (v_r + v_l) / 2
    omega = (v_r - v_l) / wheel_base

    robot["x"] += v * np.cos(robot["theta"]) * dt
    robot["y"] += v * np.sin(robot["theta"]) * dt
    robot["theta"] += omega * dt

# =============================
# Control Logic
# =============================
def follow_controller(target):
    dx = target[0] - robot["x"]
    dy = target[1] - robot["y"]

    distance = np.sqrt(dx**2 + dy**2)
    desired_heading = np.arctan2(dy, dx)
    angle_error = wrap_angle(desired_heading - robot["theta"])

    if distance < follow_distance:
        robot["v_l"] = 0
        robot["v_r"] = 0
        return

    robot["v_l"] = base_speed - Kp * angle_error
    robot["v_r"] = base_speed + Kp * angle_error

# =============================
# Visualization Setup
# =============================
fig, ax = plt.subplots()
ax.set_xlim(-world_size, world_size)
ax.set_ylim(-world_size, world_size)

robot_dot, = ax.plot([], [], 'ro')
phone_dot, = ax.plot([], [], 'bo')
heading_line, = ax.plot([], [], 'r-')

trail_x = []
trail_y = []

time = 0

# =============================
# Animation Loop
# =============================
def animate(frame):
    global time
    time += dt

    target = phone_position(time)

    follow_controller(target)
    update_robot()

    trail_x.append(robot["x"])
    trail_y.append(robot["y"])

    # Update visuals
    robot_dot.set_data(robot["x"], robot["y"])
    phone_dot.set_data(target[0], target[1])
    heading_line.set_data(
        [robot["x"], robot["x"] + np.cos(robot["theta"])],
        [robot["y"], robot["y"] + np.sin(robot["theta"])]
    )

    ax.plot(trail_x, trail_y, 'r--', linewidth=0.5)

    return robot_dot, phone_dot, heading_line

ani = FuncAnimation(fig, animate, interval=dt*1000)
plt.title("Mobile Box Follow Simulation")
plt.show()
```

---

# 🧪 What You’ll See

* Blue dot = Phone
* Red dot = Robot
* Red arrow = heading
* Dashed trail = robot path

Robot will:

* Turn toward phone
* Follow circular path
* Maintain distance

---

# 🔥 Want To Make It More Real?

Add noise:

Replace:

```python
desired_heading = np.arctan2(dy, dx)
```

With:

```python
noise = np.random.normal(0, 0.02)
desired_heading = np.arctan2(dy, dx) + noise
```

Now it behaves like real sensors.

---

# 🧠 Add Obstacle Simulation

Add this inside controller:

```python
obstacle = np.array([1.5, 1.5])
obs_dist = np.linalg.norm(
    obstacle - np.array([robot["x"], robot["y"]])
)

if obs_dist < 1.0:
    robot["v_l"] = -0.5
    robot["v_r"] = 0.5
    return
```

Now it avoids a fixed obstacle.

---

# 🚀 Next Level (If You Want)

We can extend this into:

* SLAM-like mapping simulation
* Kalman filter for sensor fusion
* BLE latency simulation
* Full ROS-style architecture
* Web-based UI dashboard with live map

---

# 🧠 Why This Is Powerful

Once this simulation behaves correctly:

* You already validated:

  * Controller math
  * Physics model
  * Follow logic
* Hardware becomes “just sensors + motors”

You’ve eliminated 70% of risk.

---

If you want, next we can:

* Turn this into a real UI dashboard with a clickable “Follow Me” button
* Add virtual BLE latency
* Convert this into a browser-based simulation
* Add real path planning (A*)

What direction do you want to evolve it?


----
run gpt-oss-20b on my server for some passive data processing. It handles things like data extraction and summarization throughout the day, and the same server powers my home voice assistant. When I ask it to turn off a light or check tomorrow's weather, the GPU wakes up, processes the request through the LLM, and goes right back to sleep. 



- washing machine gasket - wipe with paper towel
- rice puff 
- ask Dad to bring cutter for security lock
  - tell SW how it goes 
- clean up glove box

We realized that we no longer needed such complex orchestration to build agents, because models now have native tool-use and reasoning capabilities.” What used to take months to make production-ready now takes days with a model-driven approach. Today, with Claude Sonnet 4.6, Gemini 3 Pro, and Llama 4 Scout with its 10-million-token context window, models handle planning, tool selection, and self-correction on their own. The need for heavy framework orchestration shrinks with every model generation.
- context engineering - Google adk
- evaluation and observability 
- agent security 
- Interoperability (protocols)

Interoperability protocols might be the most important shift of all. MCP is becoming the universal standard for tool integration. A2A handles agent-to-agent communication. Google’s ADK and Amazon’s Strands ship with support for both. If MCP and A2A harden into industry standards, the framework above them becomes a thinner commodity layer.

First, bet on protocols, not frameworks. Design your tool integrations around MCP from day one. Adopt A2A for inter-agent communication. This is your hedge against framework churn, and there will be churn.

Second, invest in evaluation and observability independently of your framework choice. This layer outlasts any framework. Pick tooling that works across frameworks, not tooling that locks you into one.

Third, if you are already deep in a hyperscaler, use their SDK. The co-design between Strands and Bedrock, or ADK and Vertex, is a real engineering advantage. Do not fight that gravity. It is not worth the friction.

https://thenewstack.io/agent-framework-container-wars/
(Smart dude)

ガラケー


---
https://youtube.com/shorts/zMa5tsJvDg4?si=O1ON5COVyDm6FUXv

https://youtu.be/mOptJl8Xkx0?si=inykqgu3TD-5Amw8
Summarize 


Ajin
Violet Evergarden

https://youtube.com/shorts/8oG6wooF1C8?si=vj-19ttrRTrne_w8
Kawaii
https://youtube.com/shorts/XsZZ9nUS8RE?si=qFIsWKnoUGHJdAh6



Who 
https://youtube.com/shorts/puATxO6s52I?si=aCHZE6d3ev8vPjgK

Rewatch transformers 

即張り倒す
片付け
口調
姉妹
限度
飽きた
Inugami-san and Nekoyama-san 
ANIME








-> Agent Platform team

Follow-up Comments for Gillian
- `src/learning/app/custom_analytics/data_services/types/dashboards.py`
  - `set_thread_count`
- weird Bucket param removal
  - https://github.com/klaviyo/app/pull/108267/files#r2823793774
- weird `filter_empty_objects` investigation
  - https://github.com/klaviyo/app/pull/108267/files#r2824264105

Look up Japanese culture events -> speaking proficiency Japanese (and cuter background)
Look up orange juice vs soda
Look up query difficulty in MS, why does it take so long to query data

Zed - file infer language and preview 

To Read
- https://mangadex.org/title/fe9ab339-acc7-4b28-a25f-a5022d6a05db/flowglow-kessei-hiwa
- https://mangadex.org/title/2b22a9ca-7620-4826-aff4-c8c91129dc61
- https://www.instagram.com/toge0toge1/related_profiles/
- https://x.com/ane_shinyu
  - https://comic-walker.com/detail/KC_004345_S?episodeType=first
- https://kirapo.jp/meteor/titles/jyashin
- https://yurinavi.com/2025/12/05/akita-sale/
- https://yurinavi.com/2017/02/28/web_yuri/#15

To Watch (YT Shorts)
- https://www.youtube.com/@chris_waddle/shorts
- https://www.youtube.com/@momozono_omu/shorts


How to Evaluate HEALTH
- Out-of-pocket max
- Employer premium contribution
- Network coverage
- Preventive care
- HSA/FSA availability
- Drug coverage

Beads alternatives
**DeepAgents CLI**
**agent-cli**
**memo-brain**

Chinese learning 
https://youtu.be/UNgmdGJGoTE?si=qbt-E0GBJiFrdQPn

用事




TO DO THU
- [ ] Agentic Development Level 3 - multi-agent workflows
- [ ] read: MS Latency - https://app.glean.com/chat/43423dc5664848afbb13501f75937984?qe=https%3A%2F%2Fklaviyo-be.glean.com


- write message to Clara
  Hey Clara 
- clean phone with alcohol patch
- grab folders from office
- [ ] Experian - get credit report
  - https://usa.experian.com/registration/mobile-match/get-started

- plan trip to Taiwan
  - 小姑姑 may book stuff for me? Though I have to decide earlier rather than later for it to be possible to book!
  - Flights - April or September based on costs for direct flight
  - [ ] message Clara -> Taiwan or Singapore or Other?

- Mobile Water Cooler (Martino~)
  - check out code from Claude Code + Beads 
  - invite myself on wip-mobile-box repo
  - write a MD requirements doc
  - write a MD requirements doc specific to software portion (mobile app to control AND logic to move towards target)

  - [ ] investigate how arduino can figure out location of another sensor AND follow it
  - [ ] write a requirements doc
  - [ ] write a GitHub repo
  - [ ] write some pseudocode of how it should work
  - [ ] test out Python 2D physics environment like how a robotics team does it
  - [ ] write Python code to do basic following
  - [ ] write Python code to do advanced following w/ object avoidance (and other interesting stuff)
  - [ ] write message to Martino

- [ ] markdown-notes - aggregate to_fix and maintenance log (and any other)
- [ ] learn: Chinese
  - https://chatgpt.com/c/6994b841-fdc4-8329-8193-b4bcb74324c0
- [ ] 2025 Full Financial
  - [ ] create template (and maybe just enter totals from last payslip from that year?)
  - https://gemini.google.com/app/6b7a2f5d0876c1e9
  - https://docs.google.com/spreadsheets/d/1NDt7ZsvsJxdEuE6FPW2DwJVoeMHH0dsKERzWcHmaaf8/edit?gid=815824232#gid=81582423

TO DO FRI
- watch Tech Talk on Assets Domain - how to image search




青[あお]あざ



Great notes 👍 You’re doing this the right way — pulling vocabulary out of real passages is one of the best study methods.

I’ll:

1. Fix / refine a few of your vocab glosses
2. Break down key grammar
3. Translate the whole passage naturally
4. Clarify the garbled line near the end

---

## Vocabulary Refinements

### 改めて本当にありがとうございました！

* **改めて（あらためて）**

  * “once again”
  * “anew”
  * “formally (again)”

In this context it’s:
👉 **“Once again, thank you very much.”**

It has a slightly formal, heartfelt tone.

---

### 進行が押してしまったり

* **進行（しんこう）**

  * progress
  * proceedings (of an event)
  * schedule progression

Here it likely means:
👉 **the event schedule running long**

* **押す（おす）** (in event context)

  * “to run late”
  * “to go over time”

So:

> 進行が押してしまったり
> = “the schedule ended up running late”

The **〜てしまう** adds:

* regret
* unintended result

---

### 回線トラブル

* **回線（かいせん）**

  * communication line
  * internet line
  * connection

So:
👉 “connection trouble” / “internet issues”

---

### お待たせしてしまう

* **待たせる** = to make someone wait
* **お〜する** = humble form
* **〜てしまう** = regret / unintended

So:
👉 “ending up making you wait”

Very apologetic tone.

---

### 場面（ばめん）

Your gloss is fine, but here it’s:

👉 “situations” or “moments”

---

### それでも急なお願いにも対応してくれたり

* **急なお願い**

  * a sudden request
  * last-minute request

* **対応してくれる**

  * to handle (for someone)
  * to respond (for someone)
  * to accommodate

* **〜てくれる**

  * someone does something *for me/us*

* **〜たり**

  * listing examples (non-exhaustive)

So:
👉 “even so, handling sudden requests for me…”

---

### MCを助けてくれたり

* **MC**

  * Master of Ceremonies
  * host

* **助けてくれる**

  * help (me/us)

---

### 温かい言葉をかけてくれた

* **言葉をかける**

  * to say words to someone
  * to offer words (often kind ones)

👉 “offering warm words”

---

### おかげで

* because of (positive)
* thanks to

Important nuance:
It implies gratitude.

---

### 最後まで走り切ることができました

* **走り切る（はしりきる）**

  * literally: run all the way through
  * figuratively: see something through to the end

Very common in event / performance context.

👉 “We were able to make it all the way to the end.”

---

## Full Natural Translation

Here’s a smooth translation:

> Once again, thank you so very much.
>
> There were moments when the schedule ran late or connection troubles caused us to keep you waiting, and I’m very sorry about that.
>
> Even so, thanks to all the members who handled sudden requests, helped the MC, and offered warm words of support,
>
> we were able to make it all the way to the end.
>
> To all the members who participated, and to all the listeners who watched — thank you so very much ♡♡♡♡
>
> I truly love playing with everyone,
>
> so from here on as well…
>
> This is a small gift (a cheki photo present) for everyone who supported us!
>
> #HoloNewYearGameFestival2026

---

## The Garbled Line

This part:

> これから土、国铨朋伛た!
> +-1に遊びナーいハ。

This contains corrupted characters (likely encoding issues).

Given context, it probably was something like:

* これからもいっぱい遊びたいな！
* これからもたくさん遊びたいな！

Which would mean:

👉 “I want to keep playing a lot with everyone from now on!”

That fits the tone perfectly.

---

## Key Grammar Patterns in This Passage

### 〜てしまう

Regret / unintended action

* 押してしまった
* お待たせしてしまう

---

### 〜たり、〜たり

Listing examples

* 対応してくれたり
* 助けてくれたり

---

### 〜てくれる

Someone does something *for me/us*

Very important emotional grammar marker.

---

### おかげで

Positive “because of”

---

If you’d like, I can:

* Turn this into a structured Anki deck
* Break down the grammar at JLPT level
* Rewrite it in simpler Japanese
* Or quiz you on it

What level are you roughly at right now?



---
Great — from an **employee perspective**, here’s what actually matters.

---

# 📦 Employee RSUs at a Company Like Klaviyo

## 1️⃣ What You’re Actually Receiving

When you’re granted RSUs (Restricted Stock Units), you’re typically granted:

> **Class A common stock** (the publicly traded shares)

That means:

* You participate in **stock price appreciation**
* You receive **dividends (if any are issued)**
* You typically get **1 vote per share**

You do **not** receive high-vote Class B shares.

So economically, you’re aligned with public shareholders — but you don’t influence control.

---

## 2️⃣ Do Voting Rights Matter for You?

In practice? Not much.

Even though your shares technically have voting rights:

* Founders and insiders holding super-voting shares usually control the outcome
* Your individual vote will not materially affect board elections or major decisions

So from an employee standpoint:

| What matters       | What doesn’t matter much |
| ------------------ | ------------------------ |
| Stock price growth | Voting influence         |
| Vesting schedule   | Governance control       |
| Tax timing         | Founder voting power     |

---

## 3️⃣ How RSUs Actually Work (Mechanically)

### 🗓 Vesting

You’re granted units that vest over time (e.g., 4 years with a 1-year cliff).

Once vested:

* The units convert into real Class A shares
* They become taxable income

---

### 💰 Taxes (The Important Part)

At vest:

* The fair market value (FMV) of the shares = **ordinary income**
* Your company typically withholds shares to cover taxes

Example:

* 1,000 shares vest at $40
* $40,000 counts as W-2 income
* Shares are automatically sold/withheld for taxes

After vesting:

* Any further gain = capital gains
* Holding > 1 year = long-term capital gains rates

This tax structure matters far more than voting rights.

---

## 4️⃣ Why Companies Don’t Give Employees Super-Voting Shares

Super-voting shares are typically reserved for:

* Founders
* Early executives
* Pre-IPO insiders

The purpose is governance stability, not employee compensation.

Employees are compensated for:

* Talent contribution
* Performance
* Retention

Not governance control.

---

## 5️⃣ What Actually Impacts You as an Employee

Much more important than voting power:

* Company growth rate
* Revenue quality
* Profitability trajectory
* Dilution from future grants
* Stock price volatility
* Liquidity timing (lockups, trading windows)

Those drive your financial upside.

---

# 🔎 Practical Takeaway

As an employee holding RSUs in a dual-class company like Klaviyo:

* You benefit from **economic upside**
* You do not meaningfully influence corporate control
* Your biggest lever is **career + performance + holding strategy**, not voting

---

If you'd like, I can next explain:

* Smart strategies for managing RSU concentration risk
* When to sell vs hold from a risk perspective
* How to think about equity as part of total comp

---

### What I need from you

1. Are you early in your vesting schedule or already sitting on vested shares?
2. Is your goal wealth building, risk reduction, or liquidity planning?

### Suggested next step

Share your vesting stage (new grant vs multi-year holder), and I’ll outline a practical RSU strategy framework tailored to that stage.


Consultant VS Product Team
- Just because they complain doesn't make them mean they get special treatment
- They should submit a feature request and we weigh it against other feature requests
- In the past, we've caved to requests; I would also like that too


3 Engineers from Analytics are now working on Composer
3 new engineers joining Analytics
- Chris Fox
- James
- Steven

still hiring


"AI Adoption" for Level 3 by end of H1
- "AI Champion" -> 1 per team
  - meet regularly
  - connect with enablements teams to understand best practices
- AJ will be pushing Catalog Insights to reach Level 3 first





---
Excellent. Let’s turn this into an actual build plan you could execute.

I’ll assume:

* Outdoor + pavement use
* Walking speed (~1.4 m/s max)
* 2–3 hour runtime
* UWB-based following
* ESP32 control brain

This is a **concrete V1 → V2 roadmap with real algorithms and pseudocode.**

---

# 🧊 SYSTEM ARCHITECTURE (V1)

**Phone (UWB tag + app)**
⬇
**UWB Anchor (on cooler)**
⬇
**ESP32**
⬇
**Motor Driver**
⬇
**Left + Right Motors**

Optional:

* 2x ultrasonic sensors (front)
* Emergency stop button

---

# PHASE 1 — Build a Driveable Robot (No Following Yet)

## Step 1: Mechanical Assembly

1. Buy a 4-wheel differential drive chassis rated for 50–80 lbs.
2. Mount:

   * 2 geared DC motors (left/right)
   * Motor driver (dual H-bridge)
   * 12V–24V battery
3. Secure cooler to chassis (low center of gravity).

---

## Step 2: Wire Electronics

### Wiring Layout

Battery → Motor Driver
Battery → Buck Converter → ESP32
ESP32 GPIO → Motor Driver inputs

Motor driver typical pins:

* IN1 / IN2 = Left motor
* IN3 / IN4 = Right motor
* PWM pins for speed

---

## Step 3: Basic Motor Control Code

Goal: Make it drive forward, backward, left, right.

### Pseudocode

```pseudo
function set_motor(left_speed, right_speed):
    set_pwm(LEFT_PWM, abs(left_speed))
    set_pwm(RIGHT_PWM, abs(right_speed))

    if left_speed > 0:
        set_pin(IN1, HIGH)
        set_pin(IN2, LOW)
    else:
        set_pin(IN1, LOW)
        set_pin(IN2, HIGH)

    if right_speed > 0:
        set_pin(IN3, HIGH)
        set_pin(IN4, LOW)
    else:
        set_pin(IN3, LOW)
        set_pin(IN4, HIGH)
```

Test:

* set_motor(100, 100) → forward
* set_motor(-100, -100) → backward
* set_motor(100, -100) → rotate

---

# PHASE 2 — Add UWB Tracking

## Step 4: UWB Setup

You need:

* 1 UWB module on cooler (anchor)
* 1 UWB module carried by you (tag)

Modules give:

* Distance (meters)
* Sometimes angle (if using multi-antenna)

If angle not available:
You must compute direction using motion.

---

# FOLLOWING ALGORITHM (CORE LOGIC)

We define:

* `target_distance = 1.5m`
* `deadband = 0.2m`
* `max_speed = 150 PWM`

---

## Simple Distance Control (Proportional Control)

This keeps the cooler 1.5m behind you.

### Pseudocode

```pseudo
loop:
    distance = uwb_get_distance()

    error = distance - target_distance

    if abs(error) < deadband:
        stop()
    else:
        speed = Kp_distance * error
        speed = clamp(speed, -max_speed, max_speed)
        set_motor(speed, speed)
```

Where:

```
Kp_distance ≈ 100
```

This is a proportional controller (P-controller).

---

# DIRECTION CONTROL (TURNING TOWARD USER)

## If UWB Gives Angle

Assume:

* angle = 0° means straight ahead
* positive = target to right
* negative = target to left

### Combined Control

```pseudo
loop:
    distance = uwb_get_distance()
    angle = uwb_get_angle()

    distance_error = distance - target_distance
    angle_error = angle

    forward_speed = Kp_distance * distance_error
    turn_speed = Kp_angle * angle_error

    left_speed = forward_speed - turn_speed
    right_speed = forward_speed + turn_speed

    left_speed = clamp(left_speed, -max_speed, max_speed)
    right_speed = clamp(right_speed, -max_speed, max_speed)

    set_motor(left_speed, right_speed)
```

Typical values:

```
Kp_distance = 80
Kp_angle = 3–5
```

---

# If UWB Does NOT Provide Angle

You must estimate direction.

## Direction Estimation Algorithm

1. Measure distance.
2. Rotate slightly left.
3. Measure distance.
4. If distance decreases → target is left.
5. Else → target is right.

### Pseudocode

```pseudo
function find_direction():
    d1 = uwb_get_distance()

    rotate_left_small()
    wait(200ms)

    d2 = uwb_get_distance()

    if d2 < d1:
        return LEFT
    else:
        return RIGHT
```

Then steer in that direction until distance decreases.

This is slower but works.

---

# PHASE 3 — Add Obstacle Avoidance

Add 2 ultrasonic sensors (left + right front).

## Obstacle Logic

```pseudo
front_left = ultrasonic_left()
front_right = ultrasonic_right()

if front_left < 0.5m OR front_right < 0.5m:
    stop()
    rotate_away_from_closest_obstacle()
```

More advanced version blends obstacle force with follow force:

```pseudo
avoidance = K_avoid / obstacle_distance
turn_speed += avoidance
```

---

# PHASE 4 — BLE App Control

## App Commands

Define protocol:

```
CMD_START
CMD_STOP
CMD_SET_DISTANCE:1.5
CMD_MAX_SPEED:120
```

---

### ESP32 BLE Loop

```pseudo
if ble_received("CMD_START"):
    follow_enabled = true

if ble_received("CMD_STOP"):
    follow_enabled = false
    stop()

if follow_enabled:
    run_follow_algorithm()
```

---

# PHASE 5 — Safety Logic (Critical)

## Signal Loss Protection

```pseudo
if uwb_signal_lost_for > 1 second:
    stop()
```

## Speed Ramp (Prevent Jerky Motion)

```pseudo
current_speed += (target_speed - current_speed) * 0.1
```

This smooths acceleration.

---

# CONTROL ARCHITECTURE SUMMARY

Main Loop:

```pseudo
loop:
    read_ble()
    read_uwb()
    read_obstacles()

    if emergency_pressed:
        stop()
        continue

    if follow_enabled:
        compute_follow_control()
        compute_obstacle_adjustment()
        apply_motor_output()
    else:
        stop()
```

---

# PHASE 6 — Tuning Process

1. Tune distance Kp first.
2. Then tune angle Kp.
3. Then tune obstacle gain.
4. Then tune acceleration smoothing.

Test with:

* No load
* Half load
* Full beer load

---

# REALISTIC BUILD TIMELINE

Week 1:

* Assemble chassis
* Basic motor control

Week 2:

* UWB distance reading

Week 3:

* Basic follow logic

Week 4:

* App + BLE control

Week 5:

* Obstacle avoidance

Week 6:

* Testing + tuning

---

# MVP CHECKLIST

✔ Drives manually
✔ Maintains 1.5m distance
✔ Turns toward you
✔ Stops if too close
✔ Stops if signal lost
✔ Avoids obstacles
✔ App start/stop

---

If you'd like, I can next:

* Write actual ESP32-style C++ code
* Draw a block diagram with real signal flow
* Design a V2 with full PID control
* Or design a “commercializable” version

How serious is this project — fun hack or something you’d actually invest $1–2k into?


---
Excellent. This is exactly how real robotics teams work.

You want a **software-in-the-loop (SIL) simulation** so you can:

* Validate the follow algorithm
* Tune gains (Kp values)
* Test edge cases (signal loss, obstacles)
* Confirm stability
* Avoid smashing a $1,000 beer robot into a wall

Below is a concrete, realistic way to do this.

---

# ✅ Best Option: Python 2D Physics Simulation

We simulate:

* You (target)
* Icebox robot
* Distance sensor (simulated UWB)
* Optional angle
* Obstacles
* Noise
* Latency

Then run your exact control algorithm against it.

---

# 🧠 System Model

We model:

### State Variables

Robot:

```
x, y
heading (theta)
left_speed
right_speed
```

User:

```
x, y
```

---

# 📐 Robot Motion Model (Differential Drive)

Standard equations:

```
v = (left + right) / 2
omega = (right - left) / wheel_base
```

Update:

```
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt
```

This mirrors real robot physics.

---

# 🧪 Step 1 — Minimal Simulation Framework (Python)

Install:

```
pip install pygame numpy
```

---

## Basic Simulation Code (Concrete)

This simulates following with proportional control.

```python
import pygame
import numpy as np
import math
import random

# --- Constants ---
WIDTH, HEIGHT = 800, 600
DT = 0.05
WHEEL_BASE = 40
TARGET_DISTANCE = 80  # pixels
KP_DISTANCE = 1.5
KP_ANGLE = 2.0
MAX_SPEED = 150

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Robot state
robot_x, robot_y = 200, 300
robot_theta = 0

# User state
user_x, user_y = 500, 300

def clamp(val, minv, maxv):
    return max(min(val, maxv), minv)

running = True
while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Move user with arrow keys
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]: user_y -= 3
    if keys[pygame.K_DOWN]: user_y += 3
    if keys[pygame.K_LEFT]: user_x -= 3
    if keys[pygame.K_RIGHT]: user_x += 3

    # --- Simulated UWB ---
    dx = user_x - robot_x
    dy = user_y - robot_y

    distance = math.sqrt(dx**2 + dy**2)
    angle_to_user = math.atan2(dy, dx)

    angle_error = angle_to_user - robot_theta
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    # Add simulated noise
    distance += random.uniform(-2, 2)
    angle_error += random.uniform(-0.02, 0.02)

    # --- Control Algorithm ---
    distance_error = distance - TARGET_DISTANCE

    forward_speed = KP_DISTANCE * distance_error
    turn_speed = KP_ANGLE * angle_error

    left_speed = forward_speed - turn_speed
    right_speed = forward_speed + turn_speed

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    # --- Physics Update ---
    v = (left_speed + right_speed) / 2
    omega = (right_speed - left_speed) / WHEEL_BASE

    robot_x += v * math.cos(robot_theta) * DT
    robot_y += v * math.sin(robot_theta) * DT
    robot_theta += omega * DT

    # --- Draw ---
    screen.fill((30, 30, 30))

    pygame.draw.circle(screen, (0, 255, 0), (int(user_x), int(user_y)), 10)
    pygame.draw.circle(screen, (0, 0, 255), (int(robot_x), int(robot_y)), 15)

    pygame.display.flip()

pygame.quit()
```

---

# 🧪 What This Lets You Test

✔ Does it oscillate?
✔ Does it overshoot?
✔ Does it spiral?
✔ Does it lose control at high speed?
✔ Is KP_DISTANCE too high?

You tune in simulation until it behaves cleanly.

---

# 🧠 Add Realistic Failure Modes

To make it useful:

## 1️⃣ Add Signal Drop

```python
if random.random() < 0.01:
    distance = None
```

Then test your stop logic.

---

## 2️⃣ Add Latency

Simulate UWB delay:

```python
measurement_queue.append((distance, angle_error))
if len(measurement_queue) > 5:
    distance, angle_error = measurement_queue.pop(0)
```

This simulates 250ms delay.

---

## 3️⃣ Add Obstacles

Define circular obstacles and simulate ultrasonic distance:

```python
obstacles = [(400, 300, 40)]

for ox, oy, r in obstacles:
    pygame.draw.circle(screen, (255, 0, 0), (ox, oy), r)
```

Then add avoidance force.

---

# 🔥 Even Better: Test Arduino Code Directly

Instead of rewriting control logic twice:

### Option A (Best Practice)

Write control logic as:

```
control.py
```

Then write the same logic in C++.

Or…

---

### Option B (Professional Method)

Write control logic in pure C (no Arduino functions).

Then:

* Compile for PC (for simulation)
* Compile for Arduino

This is how real robotics stacks work.

---

# 🧪 Advanced Option — Use a Real Robotics Simulator

If you want professional-grade testing:

Use:

* Gazebo
* Webots
* ROS

But that’s much heavier.

For your project, Python 2D simulation is perfect.

---

# 🎯 What You Should Validate Before Uploading to Arduino

1. Stable distance hold
2. No oscillation
3. No spin-in-place bug
4. Smooth acceleration
5. Signal-loss stop
6. Obstacle avoidance does not conflict with follow logic
7. Gain values stable under noise + latency

---

# 🧊 Final Recommendation

Build this progression:

1. Python sim
2. Hardware with wheels lifted off ground
3. Low-speed real-world test
4. Gradually increase speed

---

If you want next, I can:

* Upgrade this to full PID simulation
* Add obstacle force blending math
* Design a tuning workflow
* Or simulate a full “park party” scenario

How realistic do you want this demo to feel — quick validation or near-real physics?

https://chatgpt.com/c/6993727c-17d8-8326-8480-e9a3c99d3012



- System Design topics
  - Scalability
  - Availability
  - Reliability
  - Latency vs Throughput vs Bandwidth
  - Client-Server Architecture
  - Databases
  - SQL vs NoSQL
  - Load Balancing
  - Load Balancing Algorithms
  - Caching
  - Cache Invalidation
  - CDN
  - DNS
  - API Design
  - REST APIs
  - Authentication vs Authorization
  - Session-based vs Token-based Authentication
  - OAuth/OAuth2/OpenID Connect
  - JWT
  - Rate Limiting
  - Single Point of Failure
  - High Availability vs Fault Tolerance
  - CAP Theorem
  - Consistency Models
  - Data Replication
  - Read Replicas
  - Sharding
  - Data Partitioning
  - Consistent Hashing
  - Denormalization
  - Indexing
  - Microservices Architecture
  - Monolithic Architecture
  - Serverless Architecture
  - Event-Driven Architecture
  - Message Queues
  - Pub/Sub
  - Synchronous vs Asynchronous Communicatio


- Badminton Training for Tournaments
  - defense training
    - [ ] shadow defense
      - straight block
      - cross block
      - lift
    - [ ] squats
    - [ ] wall sits
  - offense training (racket swinging)
    - [ ] net kill
    - [ ] smash
