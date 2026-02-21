# 2026-Robot

FRC Team 6908's robot code for the 2026 FRC season, **REBUILT presented by Haas**. This robot uses a **swerve drivetrain** (all 4 wheels can independently drive and steer) and an **algae intake/outtake mechanism** with a pivoting arm.

> **Note:** Some code and command names still reference the 2025 season (e.g., "algae"). The mechanisms are being carried forward and adapted for the 2026 game.

## Table of Contents

- [The Game: REBUILT](#the-game-rebuilt)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Cloning the Project](#cloning-the-project)
  - [Common Commands](#common-commands)
- [Project Structure](#project-structure)
- [How the Code Works](#how-the-code-works)
  - [The Big Picture](#the-big-picture)
  - [Subsystems](#subsystems)
  - [Commands](#commands)
  - [How Buttons Connect to Commands](#how-buttons-connect-to-commands)
  - [Key Concepts Explained](#key-concepts-explained)
    - [Field-Relative vs Robot-Relative Driving](#field-relative-vs-robot-relative-driving)
    - [PID Control](#pid-control)
    - [Odometry](#odometry)
  - [How to Create a New Command](#how-to-create-a-new-command)
  - [How to Create a New Subsystem](#how-to-create-a-new-subsystem)
- [Controller Layout](#controller-layout)
- [Hardware Overview](#hardware-overview)
- [Common Modifications](#common-modifications)
- [Key Libraries](#key-libraries)
- [Troubleshooting / FAQ](#troubleshooting--faq)
- [Glossary](#glossary)
- [Contributing Guidelines](#contributing-guidelines)
- [Git & GitHub Setup](#git--github-setup)
  - [Step 1: Install Git](#step-1-install-git)
  - [Step 2: Create a GitHub Account](#step-2-create-a-github-account)
  - [Step 3: Authenticate with GitHub](#step-3-authenticate-with-github)
  - [Step 4: Making Changes and Pushing Code](#step-4-making-changes-and-pushing-code)
  - [Git Tips](#git-tips)
- [Team Number](#team-number)

## The Game: REBUILT

The 2026 FRC game is **REBUILT presented by Haas**. Two alliances of up to 4 teams each compete in 2-minute-40-second matches to score **FUEL** (foam balls), navigate field obstacles, and **climb a TOWER**.

**How scoring works:**
- **FUEL** (1 point each) -- Collect foam balls from around the field and shoot them into your alliance's **HUB** (a raised goal). FUEL exits the HUB after scoring and returns to the field, so the same balls can be scored multiple times.
- **TOWER climbing** (15-30 points) -- A vertical structure with three rungs (LOW at 27", MID at 45", HIGH at 63"). Robots can climb to higher rungs for more points.
- **Ranking Points** -- Bonus RP awarded for reaching FUEL thresholds (100+ for ENERGIZED, 360+ for SUPERCHARGED) and TOWER point thresholds (50+ for TRAVERSAL).

**Match structure:**
- **Autonomous** (20 seconds) -- Robots run pre-programmed routines with no driver input. Both HUBs are active.
- **Teleop** (2 minutes 20 seconds) -- Drivers take control. HUBs alternate between active and inactive in 25-second "shifts," so teams must time their scoring. In the final 30 seconds (END GAME), both HUBs activate for a last scoring push.

The alliance that scores the most total points wins the match. For full rules, see the [official game manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf).

## Getting Started

### Prerequisites

- [WPILib 2025](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (includes VS Code, Java 17, and the WPILib extension)
- A [GitHub account](https://github.com) with access to the team's organization
- Git installed on your computer (see [Git & GitHub Setup](#git--github-setup) below for details)
- A configured RoboRIO (only needed for deploying to the real robot)

### Cloning the Project

Before you can work on the code, you need to download (clone) the repository to your computer. You only do this once.

**Using SSH** (if you've set up an SSH key -- see [Git & GitHub Setup](#git--github-setup)):
```bash
git clone git@github.com:frc6908/2026-Robot.git
cd 2026-Robot
```

**Using HTTPS** (if you're using a Personal Access Token):
```bash
git clone https://github.com/frc6908/2026-Robot.git
cd 2026-Robot
# When prompted, enter your GitHub username and paste your PAT as the password
```

**Using VS Code**:
1. Open VS Code
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac) to open the command palette
3. Type **"Git: Clone"** and press Enter
4. Paste the URL: `https://github.com/frc6908/2026-Robot.git`
5. If prompted, enter your GitHub username and paste your PAT as the password
6. Pick a folder on your computer to save it to
7. Click **"Open"** when it asks if you want to open the cloned repository

After cloning, verify everything works by building the project:
```bash
./gradlew build
```

If the build succeeds with no errors, you're ready to start working on the code.

### Common Commands

| Command | What it does |
|---|---|
| `./gradlew build` | Compile the project and check for errors |
| `./gradlew deploy` | Deploy code to the robot (must be connected to the RoboRIO) |
| `./gradlew simulateJava` | Run the robot in desktop simulation (no real robot needed) |
| `./gradlew test` | Run unit tests |
| `./gradlew clean` | Delete build artifacts and start fresh |

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                    # Entry point -- boots up the robot
├── Robot.java                   # Lifecycle methods (auto, teleop, disabled, etc.)
├── RobotContainer.java          # Wires everything together (subsystems + button bindings)
├── Constants.java               # All configuration values (motor IDs, speeds, PID, etc.)
│
├── subsystems/                  # Hardware groups that the robot is made of
│   ├── SwerveSubsystem.java     # The full swerve drivetrain (manages all 4 modules)
│   ├── SwerveModule.java        # One swerve module (drive motor + steering motor + encoder)
│   ├── AlgaeMechanism.java      # Algae arm + intake/outtake rollers
│   └── ExampleSubsystem.java    # WPILib template -- not connected to real hardware (safe to ignore)
│
└── commands/                    # Actions the robot can perform
    ├── SwerveJoystickCmd.java   # Teleop driving (joystick → swerve drive)
    ├── MoveArm.java             # Move the algae arm up or down
    ├── IntakeAlgae.java         # Spin rollers to suck in algae
    ├── OuttakeAlgae.java        # Spin rollers to spit out algae
    ├── FlipFieldRelativity.java # Enable field-relative driving
    ├── FlipFieldRelativity2.java# Enable robot-relative driving
    ├── ResetNavX.java           # Reset the gyro heading to 0
    ├── ResetArmEncoder.java     # Reset the arm encoder to 0
    ├── MobilityAuton.java       # Simple auto: just drive forward
    └── ExampleCommand.java      # WPILib template -- not connected to real hardware (safe to ignore)

src/main/deploy/pathplanner/     # PathPlanner autonomous paths and routines
vendordeps/                      # Third-party library configurations
```

## How the Code Works

This project uses **WPILib's command-based framework**. If you understand three concepts -- subsystems, commands, and how they're wired together -- you can understand the entire codebase.

### The Big Picture

Imagine a restaurant kitchen. The **CommandScheduler** is the head chef calling out orders. Every 20 milliseconds (50 times per second) -- the robot's heartbeat -- the head chef:

1. Checks if any new orders came in (buttons pressed or released)
2. Assigns new orders to the right station (starts new commands)
3. Tells every station to do one step of their current order (runs `execute()`)
4. Clears finished orders off the board (removes completed commands)
5. Has each station do a quick status check (calls `periodic()` on every subsystem)

This means you don't write one giant loop yourself. Instead you write small, focused pieces (commands), and the scheduler calls them for you at the right time. It's like writing recipe cards instead of personally cooking every dish.

### Subsystems

A **subsystem** is like a station in that restaurant kitchen -- the grill station, the prep station, the fryer station. Each station "owns" its own equipment and nobody else touches it.

On our robot, each subsystem owns a physical group of hardware (motors and sensors) and provides methods to control them.

| Subsystem | What it controls | Real-world analogy |
|---|---|---|
| `SwerveSubsystem` | The entire drivetrain (all 4 swerve modules + gyro) | The wheels and steering wheel of a car |
| `SwerveModule` | One swerve module (1 drive motor + 1 steering motor + 1 encoder) | One individual wheel that can spin and pivot |
| `AlgaeMechanism` | The algae arm + intake/outtake rollers | A hand that can reach out and grab things |

Key rules about subsystems:

- **Each subsystem reports in automatically.** Every subsystem has a `periodic()` method that runs every 20ms. Think of it as the station yelling "status update!" to the head chef. This is where we read sensors and send data to the dashboard.

- **Each subsystem has a "default task."** A **default command** runs whenever nothing else needs that subsystem. For the drivetrain, the default command is joystick driving (`SwerveJoystickCmd`). It's like the grill station always cooking burgers unless a special order comes in.

- **Only one command can use a subsystem at a time.** Just like only one person should be driving a car at once -- if you press a button that triggers a drivetrain command, the joystick driving command gets paused until the new command finishes. This prevents two pieces of code from fighting over the same motors.

### Commands

A **command** is a single task for the robot to do -- like a recipe card in the kitchen. "Spin the intake rollers," "move the arm up," "drive forward for 3 seconds."

Every command follows the same lifecycle. Think of it like washing dishes:

```
initialize()  →  execute()  →  execute()  →  execute()  → ... →  end()
```

- **`initialize()`** -- "Fill the sink with water." Runs **once** when the command first starts. Use it for one-time setup.

- **`execute()`** -- "Scrub a dish." Runs **every 20ms** while the command is active. Each call does one small step of work (set a motor speed, check a sensor). The command doesn't need to do everything at once -- it just does a tiny bit of work each time, 50 times per second.

- **`isFinished()`** -- "Are all the dishes clean?" Checked every 20ms. If it returns `true`, the command stops and `end()` is called. If it always returns `false`, the command runs forever -- like a sink that never runs out of dishes (useful for things like "keep the intake spinning as long as the button is held").

- **`end(boolean interrupted)`** -- "Drain the sink and put everything away." Runs **once** when the command stops. Use it to clean up (like stopping motors so they don't keep spinning). The `interrupted` parameter tells you if you finished naturally (`false`) or someone pulled you away mid-task (`true`).

Every command also calls `addRequirements()` in its constructor to "claim" a subsystem. This is like putting your name on a kitchen station -- it tells the scheduler "I need this station, nobody else can use it right now."

Here are all the commands in this project:

| Command | What it does | Subsystem | How it's triggered |
|---|---|---|---|
| `SwerveJoystickCmd` | Drives the robot with joystick input | `SwerveSubsystem` | Default command (always running) |
| `IntakeAlgae` | Spins intake rollers to suck in algae | `AlgaeMechanism` | Operator B button (hold) |
| `OuttakeAlgae` | Spins rollers in reverse to spit out algae | `AlgaeMechanism` | Operator X button (hold) |
| `MoveArm` | Moves the arm up or down | `AlgaeMechanism` | Operator Y (up) / A (down) buttons (hold) |
| `FlipFieldRelativity` | Enables field-relative driving | `SwerveSubsystem` | Driver X button (hold) |
| `FlipFieldRelativity2` | Enables robot-relative driving | `SwerveSubsystem` | Driver A button (hold) |
| `ResetNavX` | Resets the gyro heading to 0 degrees | `SwerveSubsystem` | Driver Y button (hold) |
| `ResetArmEncoder` | Resets the arm encoder to 0 | `AlgaeMechanism` | Operator Right Bumper (hold) |
| `MobilityAuton` | Simple auto: drives forward for 10 seconds | `SwerveSubsystem` | Autonomous mode (if selected) |

### How Buttons Connect to Commands

The connections between buttons and commands are set up in `RobotContainer.java`, inside the `configureBindings()` method. Think of it like programming a TV remote -- "when I press this button, do this thing."

There are two main ways to bind a button:

- **`whileTrue(command)`** -- like holding down the trigger on a power drill. The command runs as long as you hold the button, and stops the moment you let go. Most of our commands use this (intake, arm movement, etc.). When you release the button, the command's `end()` method is called to clean up (stop the motors).

- **`onTrue(command)`** -- like flipping a light switch. One press starts the command, and it keeps going even after you release the button. The command runs until `isFinished()` returns `true` or another command interrupts it. This is useful for things like "drive forward for 5 seconds" where you want to press once and let it run.

### Key Concepts Explained

#### Field-Relative vs Robot-Relative Driving

Imagine you're playing a video game where you're looking down at your character from above:

- **Field-relative** (default): Pushing the joystick "up" always moves the robot toward the far end of the field, no matter which way the robot is facing. If the robot is turned sideways, pushing "up" still goes forward on the field. This is usually the most intuitive mode -- it works like controlling a character in a top-down video game.

- **Robot-relative**: Pushing "up" moves the robot wherever its nose is pointing. If the robot is turned sideways, pushing "up" moves it sideways across the field. This can be useful in specific situations but is generally harder for drivers.

Field-relative driving needs the **gyroscope (NavX)** to know which way the robot is facing. If the gyro drifts or gets confused, press Y to reset it.

#### PID Control

PID stands for **Proportional-Integral-Derivative**, but the concept is simple. It's like **cruise control in a car**.

Say you set cruise control to 60 mph and you're currently going 45 mph. The system needs to figure out how hard to press the gas pedal:
- **P (Proportional)**: "I'm 15 mph too slow, so press the gas proportionally hard." The further from the target, the harder it pushes. This is the most important term.
- **I (Integral)**: "I've been a little bit slow for a while now, let me push a tiny bit harder to make up for it." Corrects persistent small errors that P alone can't fix. (We usually keep this at 0.)
- **D (Derivative)**: "I'm getting close to 60 mph fast -- better ease off the gas so I don't overshoot." Prevents the system from slamming past the target and oscillating back and forth.

On our robot, we use PID to:
- Point the swerve wheels at the right angle (rotation PID)
- Drive the wheels at the right speed (drive PID)
- Follow autonomous paths accurately (PathPlanner PID)

If you see the wheels wobbling back and forth, the P value is probably too high. If they're slow to reach their target angle, P is too low. You can tweak these in `Constants.java`.

#### Odometry

Odometry is how the robot tracks its own position on the field **without a GPS or camera**. It's like walking through a dark room while counting your steps -- you can't see where you are, but if you know where you started, how many steps you took, and which direction you walked, you can estimate your position.

The robot combines two things:
- **Wheel encoders**: "How far has each wheel rolled?" (the steps)
- **Gyroscope**: "Which direction am I facing?" (the compass)

By combining these, the robot maintains an estimated (x, y, angle) position on the field. This is critical for autonomous routines -- the robot needs to know where it is to follow a path.

The downside: odometry drifts over time (like counting steps in the dark -- small errors add up). That's why some teams add cameras or AprilTag detection to correct the drift.

### How to Create a New Command

If you want to add a new action to the robot (like "spin a shooter wheel" or "extend a climber"):

1. **Create a new file** in `src/main/java/frc/robot/commands/` (copy an existing simple command like `IntakeAlgae.java` as a starting point -- it's the simplest one).
2. **Extend `Command`** and fill in the lifecycle methods:
   - `initialize()` -- any one-time setup
   - `execute()` -- what to do every 20ms (usually just set a motor speed)
   - `end()` -- clean up (stop motors)
   - `isFinished()` -- return `false` if it should run until the button is released
3. **Call `addRequirements()`** in the constructor with the subsystem(s) your command needs. This is how you "claim" your kitchen station.
4. **Bind it to a button** in `RobotContainer.configureBindings()`:
   ```java
   m_operatorController.leftBumper().whileTrue(new YourNewCommand(m_yourSubsystem));
   ```

### How to Create a New Subsystem

If you add new hardware to the robot (like a climber, a shooter, or a vision system):

1. **Add constants** (CAN IDs, speeds, etc.) to `Constants.java` in a new inner class. This keeps all the "settings" in one place.
   ```java
   public static class ClimberConstants {
       public static final int climbMotorPort = 50;
       public static final double climbSpeed = 0.5;
   }
   ```
2. **Create a new file** in `src/main/java/frc/robot/subsystems/` (copy `AlgaeMechanism.java` as a starting template -- it's the simplest subsystem).
3. **Create motor/sensor objects** in the constructor, configure them, and add methods to control them (like `setSpeed()`, `stop()`, etc.).
4. **Create command(s)** for the new subsystem in the `commands/` folder.
5. **Instantiate the subsystem** in `RobotContainer.java` (create it as a field at the top of the class) and bind commands to buttons in `configureBindings()`.

## Controller Layout

### Driver Controller (Xbox, Port 0)
- **Left Stick**: Move the robot (forward/backward + strafe left/right)
- **Right Stick X**: Rotate the robot
- **Left Trigger**: Speed slider (pull to slow down for precise movements)
- **X Button**: Enable field-relative driving
- **A Button**: Enable robot-relative driving
- **Y Button**: Reset gyro heading

### Operator Controller (Xbox, Port 1)
- **B Button**: Run intake (hold to suck in algae)
- **X Button**: Run outtake (hold to spit out algae)
- **Y Button**: Move arm up (hold)
- **A Button**: Move arm down (hold)
- **Right Bumper**: Reset arm encoder

## Hardware Overview

### Drivetrain
- **4 swerve modules**, each with:
  - 1 NEO drive motor (SparkMax controller)
  - 1 NEO steering motor (SparkMax controller)
  - 1 CANcoder absolute encoder (tracks wheel angle)
- **NavX gyroscope** for heading/orientation
- Wheelbase: 21" x 21"

### Algae Mechanism
- 1 NEO motor for intake/outtake rollers (CAN ID 40)
- 1 NEO motor for arm pivot (CAN ID 41)
- Quadrature encoder for arm position tracking

## Common Modifications

Here's where to look when you want to change specific robot behavior:

| I want to... | Where to look |
|---|---|
| Change the robot's max speed | `Constants.java` → `DrivetrainConstants.maxVelocity` |
| Change how fast it accelerates | `Constants.java` → `DrivetrainConstants.maxAcceleration` |
| Adjust joystick sensitivity/dead zones | `Constants.java` → `OperatorConstants` deadband values |
| Change intake/outtake speed | `Constants.java` → `AlgaeConstants.intakeSpeed` / `outtakeSpeed` |
| Change arm speed | `Constants.java` → `AlgaeConstants.algaeArmSpeed` |
| Change which button does what | `RobotContainer.java` → `configureBindings()` |
| Switch joystick axes | `RobotContainer.java` → the `setDefaultCommand` lambdas |
| Tune swerve module PID | `Constants.java` → `kPDrive`, `kPRotation`, `kDRotation` |
| Tune auto path following PID | `SwerveSubsystem.java` → `PPHolonomicDriveController` PID values |
| Recalibrate swerve module angles | `Constants.java` → `kFLOffsetRad`, `kFROffsetRad`, etc. |
| Change a motor CAN ID | `Constants.java` → the module/mechanism CAN ID constants |
| Switch between brake and coast mode | `SwerveModule.java` → `IdleMode.kBrake` in the constructor |
| Start in robot-relative mode | `SwerveSubsystem.java` → `fieldRelativeStatus = false` |
| Change the speed slider range | `SwerveJoystickCmd.java` → the `0.8` cap in `execute()` |
| Add a new subsystem | Create a class in `subsystems/`, add it to `RobotContainer` |
| Add a new command | Create a class in `commands/`, bind it in `RobotContainer.configureBindings()` |
| Add a new auto routine | Create it in PathPlanner GUI, register commands in `RobotContainer` |
| Enable arm soft stops | Uncomment the soft stop code in `MoveArm.java` and fix the encoder in `AlgaeMechanism.java` |

## Key Libraries

- **[WPILib](https://docs.wpilib.org/)** -- FRC framework (command-based)
- **[REVLib](https://docs.revrobotics.com/revlib)** -- REV Robotics SparkMax motor controllers
- **[Phoenix6](https://v6.docs.ctr-electronics.com/)** -- CTRE CANcoder absolute encoders
- **[PathPlanner](https://pathplanner.dev/)** -- Autonomous path planning and execution
- **[AdvantageKit](https://docs.advantagekit.org/)** -- Logging and data replay for post-match analysis
- **[Studica (NavX)](https://docs.studica.com/)** -- NavX AHRS gyroscope

## Troubleshooting / FAQ

| Problem | Solution |
|---|---|
| `./gradlew build` fails with "could not find vendor dependency" | Open VS Code, press `Ctrl+Shift+P`, run **"WPILib: Manage Vendor Libraries"** → **"Install new libraries (online)"** and re-add the missing library URL from `vendordeps/`. |
| `./gradlew deploy` fails with "no RoboRIO found" | Make sure you're connected to the robot's network (WiFi or USB). The RoboRIO must be on and configured with team number 6908. |
| Robot doesn't move during teleop | Check that the correct joystick/controller is plugged in and assigned to the right port in the Driver Station. Port 0 = driver, port 1 = operator. |
| Field-relative driving feels wrong / robot drifts | The gyro may have drifted. Press **Y** on the driver controller to reset the NavX heading. Make sure the robot is facing away from you (toward the field) when you press it. |
| Swerve wheels jitter or oscillate back and forth | The rotation PID's P value is too high. Lower `kPRotation` in `Constants.java` by small increments (e.g., 0.57 → 0.5). |
| Swerve wheels are slow to reach target angle | The rotation PID's P value is too low. Increase `kPRotation` in `Constants.java` by small increments. |
| Arm doesn't move or moves the wrong way | Check the CAN ID in `Constants.java` matches the physical motor controller. Verify the motor direction isn't inverted. |
| "Unresolved dependency" or Gradle sync issues | Run `./gradlew clean` then `./gradlew build`. If it persists, check your internet connection -- Gradle needs to download dependencies the first time. |
| Code deploys but nothing happens on the robot | Open the Driver Station and check for errors in the console. Make sure you're in the right mode (Teleop, not Disabled). Check CAN wiring in Phoenix Tuner / REV Hardware Client. |
| `git push` rejected | Someone else pushed changes. Run `git pull` first to merge their changes with yours, then push again. |
| Merge conflict after `git pull` | Don't panic. VS Code highlights conflicts with `<<<<<<<` markers. Pick which version to keep (or combine them), save the file, then `git add` and `git commit`. Ask a teammate if unsure. |

## Glossary

| Term | What it means |
|---|---|
| **CAN bus** | A wiring network that connects the RoboRIO to motor controllers, encoders, and other devices. Each device gets a unique ID number (like a mailing address). Think of it as a shared highway where all the robot's electronics send messages to each other. |
| **CANcoder** | An absolute encoder made by CTRE that connects over CAN bus. It always knows the exact angle of the wheel, even after a reboot -- unlike a relative encoder that starts at zero. |
| **Command** | A task for the robot to do (e.g., "spin the intake"). Commands have a lifecycle: `initialize()` → `execute()` (repeats) → `end()`. Like a recipe card that the scheduler follows step by step. |
| **CommandScheduler** | The "brain" that runs all commands and subsystems every 20ms. You never call it directly -- it runs automatically in the background. |
| **Deadband** | A small zone around the joystick center where input is ignored. Prevents the robot from drifting when you let go of the stick (since joysticks rarely return to exactly 0). |
| **Deploy** | Uploading your compiled code from your laptop to the RoboRIO (the robot's onboard computer). |
| **Encoder** | A sensor that measures rotation. **Relative encoders** count rotations from when they were last reset. **Absolute encoders** always know their exact angle. |
| **Field-relative** | Driving mode where "forward" on the joystick always means toward the far end of the field, regardless of which way the robot is facing. Requires the gyroscope to work. |
| **FUEL** | The 2026 game piece -- a ~6-inch foam ball that robots collect and score into the HUB. |
| **Gyroscope (NavX)** | A sensor that tracks which direction the robot is facing. Used for field-relative driving and odometry. Can drift over time and may need to be reset (driver Y button). |
| **HUB** | The 2026 scoring goal -- a raised structure where robots shoot FUEL into. HUBs alternate between active and inactive during teleop. |
| **Lambda** | A shorthand way to pass a function as a value in Java. Written as `() -> someMethod()`. Used so the robot reads the joystick fresh every 20ms instead of reading it once and using a stale value. |
| **Odometry** | The robot's system for tracking its own position on the field using wheel encoders and the gyroscope. Like counting your steps in a dark room to estimate where you are. |
| **PID** | Proportional-Integral-Derivative controller. A math formula that smoothly moves a motor to a target position or speed. Like cruise control -- it adjusts power based on how far off you are from the target. |
| **RoboRIO** | The main computer on the robot. All your code runs here. Made by National Instruments for FRC. |
| **Robot-relative** | Driving mode where "forward" on the joystick means wherever the robot's nose is pointing. Simpler but harder for drivers when the robot is turned. |
| **Shift** | In the 2026 game, a 25-second window during teleop where one alliance's HUB is active and the other's is inactive. Teams must time their scoring around these shifts. |
| **Slew rate** | How quickly a value is allowed to change. A slew rate limiter prevents sudden jumps in motor speed, making the robot accelerate smoothly instead of jerking. Like easing onto the gas pedal instead of flooring it. |
| **SparkMax** | A motor controller made by REV Robotics that drives NEO brushless motors. Connects over CAN bus. Each one has a unique CAN ID set in `Constants.java`. |
| **Subsystem** | A group of related hardware (motors + sensors) that works together. Only one command can use a subsystem at a time. Like a station in a kitchen -- only one cook works there. |
| **Swerve drive** | A drivetrain where each of the 4 wheels can independently spin (drive) and pivot (steer). This lets the robot move in any direction without turning its body. |
| **TOWER** | The 2026 climbing structure with three rungs at different heights. Robots climb it during the match for bonus points. |
| **Vendor dependency** | A third-party library (like REVLib or Phoenix6) that adds support for specific hardware. Configured via JSON files in the `vendordeps/` folder. |

## Contributing Guidelines

### Branch Naming

Use short, descriptive branch names that explain what you're working on:
- `tune-arm-speed` -- good
- `add-climber-subsystem` -- good
- `my-changes` -- too vague
- `fix` -- too vague

### Pull Request (PR) Workflow

1. **Create a branch** for your change (don't work directly on `main`).
2. **Make your changes** and test them (at least `./gradlew build`, and on the robot if possible).
3. **Push your branch** and open a Pull Request on GitHub.
4. **Get at least one review** from a teammate before merging. Describe what you changed and why in the PR description.
5. **Merge into `main`** once approved. Delete your branch after merging to keep things tidy.

### Commit Messages

Write clear, short commit messages that describe *what* you changed:
- `Reduce max drive speed to 80%` -- good
- `Fix arm overshoot by lowering PID P value` -- good
- `stuff` -- not helpful
- `asdfasdf` -- definitely not helpful

### Code Style

- Follow the existing patterns in the codebase. If you're adding a new command, look at `IntakeAlgae.java` for the simplest example.
- Keep constants in `Constants.java`, not hardcoded in commands or subsystems.
- Always call `addRequirements()` in command constructors so the scheduler knows which subsystem your command needs.
- Test your code with `./gradlew build` before pushing. If it doesn't compile, it shouldn't be pushed.

## Git & GitHub Setup

This project uses Git for version control and GitHub to store the code online. You'll need to set up authentication so you can push (upload) your code changes.

### Step 1: Install Git

- **Windows**: Git comes bundled with WPILib. You can also install it separately from [git-scm.com](https://git-scm.com/downloads).
- **macOS**: Run `git --version` in Terminal. If it's not installed, it will prompt you to install it.
- **Linux**: Run `sudo apt install git` (Debian/Ubuntu) or `sudo dnf install git` (Fedora).

After installing, tell Git who you are (use your real name and the email tied to your GitHub account):

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Step 2: Create a GitHub Account

If you don't have one yet, sign up at [github.com](https://github.com). Ask a team lead to add you to the team's GitHub organization so you can push to this repository.

### Step 3: Authenticate with GitHub

You need to prove to GitHub that you're allowed to push code. There are two ways to do this -- pick whichever one you prefer.

#### Option A: SSH Key (Recommended)

SSH keys let you push/pull without typing your password every time.

1. **Generate a key** -- open a terminal and run:
   ```bash
   ssh-keygen -t ed25519 -C "your.email@example.com"
   ```
   Press Enter to accept the default file location. It will then ask you to set a passphrase -- this is like a password that protects your key. You can press Enter for no passphrase (easier but less secure), or type one in. **If you set a passphrase, remember it!** You'll need to type it every time you push or pull. There's no way to recover a forgotten passphrase -- you'd have to generate a new key and add it to GitHub again.

2. **Copy the public key** to your clipboard:
   ```bash
   # macOS
   cat ~/.ssh/id_ed25519.pub | pbcopy

   # Windows (Git Bash)
   cat ~/.ssh/id_ed25519.pub | clip

   # Linux
   cat ~/.ssh/id_ed25519.pub
   # (then manually copy the output)
   ```

3. **Add the key to GitHub**:
   - Go to [github.com/settings/keys](https://github.com/settings/keys)
   - Click **"New SSH key"**
   - Give it a title (like "My Laptop") and paste your key
   - Click **"Add SSH key"**

4. **Test the connection**:
   ```bash
   ssh -T git@github.com
   ```
   You should see: `Hi username! You've successfully authenticated...`

5. **Clone the repo using SSH** (if you haven't already):
   ```bash
   git clone git@github.com:frc6908/2026-Robot.git
   ```

#### Option B: Personal Access Token (PAT) for HTTPS

If SSH doesn't work on your network (some school WiFi blocks it), use a PAT instead.

1. Go to [github.com/settings/tokens](https://github.com/settings/tokens?type=beta) (Fine-grained tokens)
2. Click **"Generate new token"**
3. Give it a name (like "Robotics Laptop"), set an expiration date, and select the repository
4. Under **Permissions**, grant **"Contents"** read/write access
5. Click **"Generate token"** and **copy the token immediately** (you won't see it again!)
6. When you clone or push, use the token as your password:
   ```bash
   git clone https://github.com/frc6908/2026-Robot.git
   # When prompted for a password, paste your token (not your GitHub password)
   ```

To avoid re-entering the token every time, you can cache it:
```bash
# Store credentials for 8 hours (28800 seconds)
git config --global credential.helper 'cache --timeout=28800'
```

### Step 4: Making Changes and Pushing Code

#### Using the Terminal (Git CLI)

Here's the basic workflow for contributing code:

```bash
# 1. Make sure you have the latest code before starting work
git pull

# 2. Create a new branch for your changes (don't work directly on main!)
#    Name it something descriptive like "tune-arm-speed" or "add-climber"
git checkout -b your-branch-name

# 3. Make your code changes in VS Code or your editor...

# 4. See what files you changed
git status

# 5. Stage the files you want to commit (add them to the "ready to save" pile)
git add src/main/java/frc/robot/Constants.java
#   Or stage all changed files:
git add .

# 6. Commit (save a snapshot of your changes with a message)
git commit -m "Describe what you changed and why"

# 7. Push your branch to GitHub
git push -u origin your-branch-name

# 8. Go to GitHub and create a Pull Request (PR) to merge your branch into main.
#    This lets teammates review your code before it goes into the main codebase.
```

Some other useful commands:

```bash
# See what you changed (before staging)
git diff

# See the commit history
git log --oneline

# Switch to an existing branch
git checkout branch-name

# Undo changes to a file (before staging)
git checkout -- path/to/file.java
```

#### Using VS Code

VS Code has Git built in, so you can do everything without the terminal if you prefer:

1. **See changes**: Click the **Source Control** icon in the left sidebar (it looks like a branch). You'll see all your modified files listed.
2. **Stage files**: Hover over a file and click the **+** button to stage it, or click the **+** next to "Changes" to stage everything.
3. **Commit**: Type a message in the text box at the top and click the **checkmark** button (or press `Ctrl+Enter`).
4. **Push**: Click the **"..."** menu in Source Control and select **"Push"**, or click the sync icon in the bottom status bar.
5. **Pull**: Click **"..."** → **"Pull"** to get the latest code from GitHub.
6. **Create a branch**: Click the branch name in the bottom-left corner of VS Code, then select **"Create new branch"**.

### Git Tips

- **Always pull before you start working** (`git pull`) so you don't get out of sync with the team.
- **Work on branches, not on main.** This keeps the main branch clean and working.
- **Commit often with clear messages.** "Fixed arm speed" is better than "stuff".
- **If you get a merge conflict**, don't panic. VS Code highlights the conflicts and lets you pick which version to keep. Ask a teammate for help if you're unsure.

## Team Number

**6908** (configured in `.wpilib/wpilib_preferences.json`)
