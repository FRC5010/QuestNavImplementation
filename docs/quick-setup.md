# Quick setup

Do these steps in order before a test session. Each step has a check. Do not go to the next step
until the check passes.

Team 5010. roboRIO: `10.50.10.2`. Headset: a `10.50.10.2xx` address that can change (it was
`.203`, then `.200`). Find it with the scan in [Problems](#problems).

## 1. The night before

- [ ] Charge the headset to 100%.
- [ ] Charge the robot batteries.
- [ ] Deploy the robot code from the `questnav-tag-distance` branch.

## 2. Hub tags

The hub is on the outside corner of the T junction.

| Tag | Face | Center from the corner | Center height |
|---|---|---|---|
| 24 | Face toward the 16 ft hallway (the test area) | 9.8 in | 44.25 in |
| 21 | Same face as 24 | 23.8 in | 44.25 in |
| 25 | Face toward the 11 ft hallway (the mural) | 9.8 in | 44.25 in |
| 26 | Same face as 25 | 23.7 in | 44.25 in |

- [ ] Facing tag 24, tag 21 is on your left and the corner is on your right.
- [ ] The black square of each tag is 6.5 in.
- [ ] Each sheet is flat, with tape on all four edges.
- [ ] No other tags with IDs 1 to 32 are in view anywhere in the hallway.

## 3. Robot

- [ ] Robot tags 1 to 4 are on the bumper corners.
- [ ] The headset is on its mount. It faces the right side of the robot.
- [ ] The headset is plugged in to power and to the robot network.

## 4. Laptop network

The Mac uses the robot Wi-Fi for the robot and the iPhone cable for the internet.

- [ ] Plug in the iPhone with a cable. On the phone, turn on **Personal Hotspot**.
- [ ] Join the robot Wi-Fi.
- [ ] In **System Settings → Network → ⋯ → Set Service Order**, **iPhone USB** is above
      **Wi-Fi**. (This stays set. Check it only the first time.)

> **Note:** With the iPhone connected, always use `curl -4` for the robot network. Without
> `-4`, the request goes to the phone and does not get an answer.

## 5. Headset

- [ ] Put on the headset. In **Quick Settings**, **Airplane mode** is **on**. This keeps Wi-Fi
      off. The Ethernet adapter still works. (If Wi-Fi is on, the headset is on two networks and
      does not connect to the robot.)
- [ ] If QuestNav is not open, open it: **App Library → Unknown Sources → QuestNav**.
- [ ] Take off the headset and put it on the robot.
- [ ] Find the headset address on the Mac:

```bash
for i in $(seq 200 220); do curl -4 -s -m 0.5 "http://10.50.10.$i:5801/api/status" >/dev/null && echo "QuestNav at 10.50.10.$i"; done
```

- [ ] Get its status. Use the address from the scan in place of `10.50.10.203` here and below:

```bash
curl -4 -s http://10.50.10.203:5801/api/status
```

Check these values in the answer:

| Value | Must be |
|---|---|
| `ipAddress` | The same `10.50.10.2xx` address that the scan found |
| `teamNumber` | `5010` |
| `robotIpAddress` | `10.50.10.2` |
| `networkConnected` | `true` |
| `isTracking` | `true` |
| `batteryPercent` | 50 or more |

If `teamNumber` is not 5010, set it:

```bash
curl -4 -X POST "http://10.50.10.203:5801/api/config" -H "Content-Type: application/json" -d '{"teamNumber":5010}'
```

If the command gets no answer, see [Problems](#problems).

## 6. Robot code and dashboard

- [ ] Turn on the robot. Connect the driver station.
- [ ] On the dashboard, `QuestNav/Connected` is true.
- [ ] `Test/Settings/Tag ID` is 24 and `questnav/Target Tag ID` is 24. (24 is the default.)
- [ ] Set `Test/Settings/Bumper Length (m)` and `Test/Settings/Bumper Width (m)` to the measured
      robot size.
- [ ] Set `Test/Settings/Sync Wiggle` to true if a camera records video. Else set it to false.

## 7. Field alignment

The headset must see 2 or more tags to align to the field.

1. Put the robot about 2 m from tag 24, a little to the right of the tag 24 line (toward the
   corner).
2. Turn the robot so that its **right side** faces the corner. The headset then sees tags 24, 21,
   25 and 26.
3. Wait 5 s.
4. Measure the straight floor distance from the point on the floor under tag 24 to the robot
   center.

- [ ] `questnav/Distance To Target Tag` agrees with the measurement to about 5 cm.

If it does not agree, the headset is not aligned. Turn the robot so that the headset sees more
tags, and wait again.

## Problems

| Problem | Fix |
|---|---|
| `curl` gets no answer | Add `-4`. Make sure that the Mac is on the robot Wi-Fi |
| `curl -4` gets no answer | Restart QuestNav: `curl -4 -X POST http://10.50.10.203:5801/api/restart`. If that gets no answer too, close and open QuestNav on the headset |
| `teamNumber` stays 9999 after the POST | The body must be `{"teamNumber":5010}`. The `path`/`value` example in the QuestNav docs does not work in this version |
| QuestNav is not in the app list | **App Library → filter (top right) → Unknown Sources** |
| The team number field in the QuestNav web page cannot be clicked | Use the `curl -4 -X POST` command in step 5 |
| The QuestNav menu shows two IP addresses that change back and forth, or `ipAddress` is not `10.50.10.x` | The headset Wi-Fi is on. Turn on **Airplane mode** in the headset Quick Settings (turning off only Wi-Fi did not stay off), then restart QuestNav |
| `networkConnected` is false | First make sure that the headset Wi-Fi is off. Then check the headset Ethernet adapter, its power and its cable. Make sure that the robot is on |
| `isTracking` is false | Turn on the lights. Make sure that nothing covers the headset cameras |
| `fps` is about 50 | Known with AprilTag detection on in this beta. Not a blocker |
