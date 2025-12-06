# Navigation robustified for DuckieTown 🚗♾️
> Achieve infinite drive in DuckieTown

<p align="center">
<a href="https://duckietown.com"><img src="/assets/images/dtlogo.png" alt="Duckietown Logo" width="50%"></a>
</p>

[![GitHub issues](https://img.shields.io/github/issues/mhoudeib/dt-core)](https://github.com/username/repo-name/issues)
[![GitHub stars](https://img.shields.io/github/stars/mhoudeib/dt-core)](https://github.com/username/repo-name/stargazers)
[![Top Language](https://img.shields.io/github/languages/top/mhoudeib/dt-core)](https://github.com/username/repo-name/)

**TODO [ Insert a high-quality GIF or screenshot of your project in action here]**

## Table of Contents

* [💡 Overview](#-overview)
* [✨ Features](#-features)
* [⚙️ Usage](#-usage)
* [🤝 Contributing](#-contributing)

## 💡 Overview

**Infinite drive** is the feature that allows a duckiest to drive indefinitely on the map by crossing intersections and statying in its way.

This project aims to solve intersection navigation by implementing **adaptive speed regulation, trajectory generation, and short range message protocol**. Our main goal is to provide a robust drive behavior to run indefinitely the duckiebot.

---
*Released as part of UDEM course IFT6757 autonomous vehicles*

## ✨ Features
- [x] **Adaptive stop breaking:** The duckiebot gradually decreased its speed as it approached a stop line.
- [x] **Intersection navigation:** The controller gives angular and velocity commands to navigate the intersection.
- [ ] **Generating trajectory:** Mention the unique selling point (e.g., *Blazing fast performance due to Caching*).
- [ ] **Rescue behavior:** Mention the unique selling point (e.g., *Blazing fast performance due to Caching*).
- [ ] **Short range communication:** Mention the unique selling point (e.g., *Blazing fast performance due to Caching*).

*Minor fix: add fsm mode to unicorn navigation node, Apriltag selection includes normal condition from the camera plan*
## ⚙️ Usage

Once installed, you can start using **inifinit navigation** with the following commands/steps:

```bash
dts matrix run --standalone -m assets/duckiematrix/maps/intersections
dts matrix attach [YOURBOTNAME] map_0/vehicle_0
dts devel build -H [YOURBOTNAME]
dts devel run -H [YOURBOTNAME] -M -L single_robot_indefinite_navigation
dts duckiebot keyboard_control [YOURBOTNAME]
```
After this, click the autopilot toggle in the keyboard_control window; this will launch the autonomous drive.

### 🤝 Contributing
- Firmin Chapoulie
- Guillaume Genois
- Mohamad Houdeib
