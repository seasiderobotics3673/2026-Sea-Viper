# 2026 Sea Viper — FRC Team 3673

Robot code for the 2026 bot. Written in Java using WPILib, YAGSL for swerve drive, and PhotonVision for vision processing.

---

## What's in here

- Command-based robot code
- YAGSL swerve drive configuration
- PathPlanner autonomous routines

---

## Getting Started

You'll need WPILib and Java 17 installed. Add the YAGSL vendor library in VS Code using this URL:

```
https://broncbotz3481.github.io/YAGSL-Lib/yagsl/yagsl.json
```

Then clone the repo and build:

```bash
git clone https://github.com/seasiderobotics3673/2026-Sea-Viper.git
cd 2026-Sea-Viper
./gradlew build
```

To deploy to the robot:

```bash
./gradlew deploy
```

---

## Swerve Config

Config files live in `src/main/deploy/swerve/`. If you need to regenerate them use the [YAGSL config generator](https://broncbotz3481.github.io/YAGSL-Example/).

```
deploy/swerve/
├── controllerproperties.json
├── swervedrive.json
└── modules/
    ├── frontleft.json
    ├── frontright.json
    ├── backleft.json
    ├── backright.json
    ├── physicalproperties.json
    └── pidfproperties.json
```

---

## Useful Links

- [YAGSL Wiki](https://github.com/BroncBotz3481/YAGSL/wiki)
- [WPILib Docs](https://docs.wpilib.org)
- [PathPlanner](https://pathplanner.dev)
- [Vision Processing](https://docs.photonvision.org/en/v2026.2.2/)

---

Licensed under Apache 2.0
