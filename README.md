# FRC Team 6908 -- 2026 Robot Code

> **Swerve drive robot** with **intake**, **dual-motor shooter with kicker**, **climbing mechanism**, and **Limelight vision** for AprilTag tracking. Built on the WPILib 2026 command-based framework with AdvantageKit logging and PathPlanner autonomous.

---

## Table of Contents

- [The Game: REBUILT](#the-game-rebuilt)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Cloning the Project](#cloning-the-project)
  - [Common Commands](#common-commands)
  - [Git & GitHub Setup](#git--github-setup)
- [Project Structure](#project-structure)
- [How the Code Works](#how-the-code-works)
  - [The Big Picture](#the-big-picture)
  - [Subsystems](#subsystems)
  - [Commands](#commands)
  - [How Buttons Connect to Commands](#how-buttons-connect-to-commands)
  - [Key Concepts Explained](#key-concepts-explained)
    - [Field-Relative vs Robot-Relative Driving](#field-relative-vs-robot-relative-driving)
    - [PID Control](#pid-control)
    - [Odometry and Pose Estimation](#odometry-and-pose-estimation)
  - [How to Create a New Command](#how-to-create-a-new-command)
  - [How to Create a New Subsystem](#how-to-create-a-new-subsystem)
- [Controller Layout](#controller-layout)
- [Hardware Overview](#hardware-overview)
- [Common Modifications](#common-modifications)
- [Key Libraries](#key-libraries)
- [Troubleshooting / FAQ](#troubleshooting--faq)
- [Glossary](#glossary)
- [Contributing Guidelines](#contributing-guidelines)
- [Team Number](#team-number)

## The Game: REBUILT

> **Note:** The 2026 FRC game details below reflect our best understanding at the time of writing. Some scoring mechanics may be placeholder or speculative since the full 2026 game may not have been completely revealed yet. The robot's mechanisms (intake, shooter, climb) are designed based on available information.

The 2026 FRC game is **REBUILT presented by Haas**. Two alliances of up to 4 teams each compete in 2-minute-40-second matches to score **FUEL** (foam balls), navigate field obstacles, and **climb a TOWER**.

**How scoring works:**
- **FUEL** (1 point each) -- Collect foam balls from around the field and shoot them into your alliance's **HUB** (a raised goal). FUEL exits the HUB after scoring and returns to the field, so the same balls can be scored multiple times.
- **TOWER climbing** (15-30 points) -- A vertical structure with three rungs (LOW at 27", MID at 45", HIGH at 63"). Robots can climb to higher rungs for more points.
- **Ranking Points** -- Bonus RP awarded for reaching FUEL thresholds (100+ for ENERGIZED, 360+ for SUPERCHARGED) and TOWER point thresholds (50+ for TRAVERSAL).

**Match structure:**
- **Autonomous** (20 seconds) -- Robots run pre-programmed routines with no driver input. Both HUBs are active.
- **Teleop** (2 minutes 20 seconds) -- Drivers take control. HUBs alternate between active and inactive in 25-second "shifts," so teams must time their scoring. In the final 30 seconds (END GAME), both HUBs activate for a last scoring push.

The alliance that scores the most total points wins the match. For full rules, see the [official game manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf).

**How our robot plays:**
1. **Intake** FUEL from the ground using our roller mechanism
2. **Shoot** FUEL into the HUB using our dual-motor flywheel shooter (with Limelight auto-aiming and distance-based speed control)
3. **Climb** the TOWER during endgame for bonus points

## Getting Started

### Prerequisites

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (includes VS Code, Java 17, and the WPILib extension)
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

### Git & GitHub Setup

This project uses Git for version control and GitHub to store the code online. You'll need to set up authentication so you can push (upload) your code changes.

#### Step 1: Install Git

- **Windows**: Git comes bundled with WPILib. You can also install it separately from [git-scm.com](https://git-scm.com/downloads).
- **macOS**: Run `git --version` in Terminal. If it's not installed, it will prompt you to install it.
- **Linux**: Run `sudo apt install git` (Debian/Ubuntu) or `sudo dnf install git` (Fedora).

After installing, tell Git who you are (use your real name and the email tied to your GitHub account):

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

#### Step 2: Create a GitHub Account

If you don't have one yet, sign up at [github.com](https://github.com). Ask a team lead to add you to the team's GitHub organization so you can push to this repository.

#### Step 3: Authenticate with GitHub

You need to prove to GitHub that you're allowed to push code. There are two ways to do this -- pick whichever one you prefer.

<details>
<summary><strong>Option A: SSH Key (Recommended)</strong></summary>

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

</details>

<details>
<summary><strong>Option B: Personal Access Token (PAT) for HTTPS</strong></summary>

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

</details>

#### Step 4: Making Changes and Pushing Code

<details>
<summary><strong>Using the Terminal (Git CLI)</strong></summary>

Here's the basic workflow for contributing code:

```bash
# 1. Make sure you have the latest code before starting work
git pull

# 2. Create a new branch for your changes (don't work directly on main!)
#    Name it something descriptive like "tune-shooter-speed" or "add-climber"
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

**Example -- lowering the shooter speed:**

```bash
# Start from the latest main
git checkout main
git pull

# Create a branch for your change
git checkout -b slow-down-shooter

# (Open Constants.java in VS Code and change shooterSpeed from 0.75 to 0.5, then save)

# Check what changed
git status
#   modified: src/main/java/frc/robot/Constants.java

# Stage and commit
git add src/main/java/frc/robot/Constants.java
git commit -m "Reduce shooter speed from 0.75 to 0.5 to prevent overshoot"

# Push your branch to GitHub
git push -u origin slow-down-shooter

# Now go to GitHub and open a Pull Request to merge "slow-down-shooter" into main
```

**Working with branches:**

```bash
# See all branches (the * marks which one you're on)
git branch

# Create a new branch and switch to it
git checkout -b my-new-branch

# Switch to an existing branch (like one a teammate made)
git checkout branch-name

# Switch back to main
git checkout main

# Pull the latest changes from GitHub into your current branch
git pull

# Delete a branch after it's been merged (cleanup)
git branch -d branch-name
```

**Other useful commands:**

```bash
# See what you changed (before staging)
git diff

# See the commit history
git log --oneline

# Undo changes to a file you haven't staged yet
git checkout -- path/to/file.java

# Unstage a file (remove from the "ready to commit" pile, but keep your changes)
git reset path/to/file.java
```

</details>

<details>
<summary><strong>Using VS Code</strong></summary>

VS Code has Git built in, so you can do everything without the terminal if you prefer:

1. **See changes**: Click the **Source Control** icon in the left sidebar (it looks like a branch). You'll see all your modified files listed.
2. **Stage files**: Hover over a file and click the **+** button to stage it, or click the **+** next to "Changes" to stage everything.
3. **Commit**: Type a message in the text box at the top and click the **checkmark** button (or press `Ctrl+Enter`).
4. **Push**: Click the **"..."** menu in Source Control and select **"Push"**, or click the sync icon in the bottom status bar.
5. **Pull**: Click **"..."** > **"Pull"** to get the latest code from GitHub.
6. **Create a branch**: Click the branch name in the bottom-left corner of VS Code, then select **"Create new branch"**.

</details>

<details>
<summary><strong>Using a Git GUI App</strong></summary>

If you prefer a dedicated app over the terminal or VS Code's built-in Git, these are popular options that let you do everything (clone, commit, push, pull, branching, merge conflicts) through a visual interface:

- **[GitHub Desktop](https://desktop.github.com/)** -- Made by GitHub. The simplest option -- great if you're new to Git. Handles cloning, commits, pushes, and PRs with minimal setup. Free.
- **[GitKraken](https://www.gitkraken.com/)** -- Visually shows your branch history as a graph, making it easier to understand what's happening. Free for public repos (like ours).
- **[Sourcetree](https://www.sourcetreeapp.com/)** -- Similar to GitKraken with a detailed visual interface. Free, but requires an Atlassian account. Windows and macOS only.

All three do the same core things -- pick whichever feels most comfortable. The terminal commands and concepts (commit, push, pull, branch) are the same regardless of which tool you use.

</details>

#### Git Tips

- **Always pull before you start working** (`git pull`) so you don't get out of sync with the team.
- **Work on branches, not on main.** This keeps the main branch clean and working.
- **Commit often with clear messages.** "Fixed shooter speed" is better than "stuff".
- **If you get a merge conflict**, don't panic. VS Code highlights the conflicts and lets you pick which version to keep. Ask a teammate for help if you're unsure.

## Project Structure

```
2026-Robot/
├── build.gradle                          # Build configuration (dependencies, plugins)
├── settings.gradle                       # Gradle project settings
├── gradlew / gradlew.bat                 # Gradle wrapper scripts (Linux/Mac and Windows)
├── vendordeps/                           # Vendor library JSON files (REVLib, Phoenix6, etc.)
│   ├── AdvantageKit.json
│   ├── PathplannerLib-2026.1.2.json
│   ├── Phoenix6-26.1.0.json
│   ├── REVLib.json
│   ├── Studica.json
│   └── ... (other vendor deps)
├── src/
│   └── main/
│       ├── java/frc/robot/
│       │   ├── Main.java                 # Entry point -- starts the robot
│       │   ├── Robot.java                # TimedRobot -- mode switching (auto/teleop/etc.)
│       │   ├── RobotContainer.java       # Wires everything together (subsystems + buttons)
│       │   ├── Constants.java            # All robot settings in one place (motor IDs, speeds, PID)
│       │   │
│       │   ├── commands/                 # Actions the robot performs
│       │   │   ├── SwerveJoystickCmd.java    # Default teleop driving (joystick -> swerve)
│       │   │   ├── Intake.java               # Suck in game pieces
│       │   │   ├── Outtake.java              # Spit out game pieces
│       │   │   ├── Shooter.java              # Constant-speed shooting
│       │   │   ├── AutoShooter.java          # Distance-based auto-speed shooting
│       │   │   ├── AlignToTag.java           # Auto-rotate to face AprilTag
│       │   │   ├── AlignAndShoot.java        # Align + shoot simultaneously
│       │   │   ├── Climb.java                # Climb up
│       │   │   ├── ClimbDown.java            # Lower climb mechanism
│       │   │   ├── FlipFieldRelativity.java  # Enable field-relative driving
│       │   │   ├── FlipFieldRelativity2.java # Enable robot-relative driving
│       │   │   ├── ResetNavX.java            # Reset gyro heading to 0
│       │   │   └── ExampleCommand.java       # WPILib template (safe to ignore)
│       │   │
│       │   └── subsystems/               # Hardware control layers
│       │       ├── SwerveSubsystem.java      # Swerve drivetrain + vision + odometry
│       │       ├── SwerveModule.java         # Single swerve module (drive + steer + encoder)
│       │       ├── IntakeMechanism.java      # Intake roller motor
│       │       ├── ShooterMechanism.java     # Dual shooter motors + kicker
│       │       ├── ClimbMechanism.java       # Climb motor
│       │       └── ExampleSubsystem.java     # WPILib template (safe to ignore)
│       │
│       └── deploy/
│           └── pathplanner/              # PathPlanner autonomous files
│               ├── autos/                # Autonomous routines (.auto files)
│               │   ├── MobilityAuto.auto     # Default: just drive forward
│               │   ├── FarIntake.auto
│               │   ├── SweepFar.auto
│               │   ├── SweepNear.auto
│               │   └── ... (other autos)
│               └── paths/                # Individual path segments (.path files)
│                   ├── FarIntake.path
│                   ├── FarShoot.path
│                   └── ... (other paths)
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
| `SwerveSubsystem` | The entire drivetrain (all 4 swerve modules + gyro + Limelight vision) | The wheels, steering wheel, and GPS of a car |
| `SwerveModule` | One swerve module (1 drive motor + 1 steering motor + 1 encoder) | One individual wheel that can spin and pivot |
| `IntakeMechanism` | A single roller motor for sucking in/spitting out game pieces | A vacuum cleaner nozzle |
| `ShooterMechanism` | Two flywheel motors + one kicker motor for launching game pieces | A pitching machine |
| `ClimbMechanism` | A single motor for climbing the TOWER | A winch |

Key rules about subsystems:

- **Each subsystem reports in automatically.** Every subsystem has a `periodic()` method that runs every 20ms. Think of it as the station yelling "status update!" to the head chef. This is where we read sensors and send data to the dashboard.

- **Each subsystem has a "default task."** A **default command** runs whenever nothing else needs that subsystem. For the drivetrain, the default command is joystick driving (`SwerveJoystickCmd`). It's like the grill station always cooking burgers unless a special order comes in.

- **Only one command can use a subsystem at a time.** Just like only one person should be driving a car at once -- if you press a button that triggers a drivetrain command, the joystick driving command gets paused until the new command finishes. This prevents two pieces of code from fighting over the same motors.

### Commands

A **command** is a single task for the robot to do -- like a recipe card in the kitchen. "Spin the intake rollers," "shoot the ball," "align to the target."

Every command follows the same lifecycle. Think of it like washing dishes:

```
initialize()  ->  execute()  ->  execute()  ->  execute()  -> ... ->  end()
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
| `Intake` | Spins intake rollers to suck in game pieces | `IntakeMechanism` | Operator B button (hold) |
| `Outtake` | Spins rollers in reverse to spit out game pieces | `IntakeMechanism` | Operator X button (hold) |
| `Shooter` | Runs shooter flywheel at constant speed | `ShooterMechanism` | Driver B button (hold) |
| `AutoShooter` | Adjusts shooter speed based on distance to target | `ShooterMechanism` | Driver Left Bumper (hold) |
| `AlignToTag` | Auto-rotates to face AprilTag (manual translation still works) | `SwerveSubsystem` | Driver Right Bumper (hold) |
| `AlignAndShoot` | Aligns to tag AND shoots simultaneously | `SwerveSubsystem` + `ShooterMechanism` | Autonomous only |
| `Climb` | Runs climb motor upward | `ClimbMechanism` | Operator Left Bumper (hold) |
| `ClimbDown` | Runs climb motor downward | `ClimbMechanism` | Operator Right Bumper (hold) |
| `FlipFieldRelativity` | Enables field-relative driving | `SwerveSubsystem` | Driver X button (hold) |
| `FlipFieldRelativity2` | Enables robot-relative driving | `SwerveSubsystem` | Driver A button (hold) |
| `ResetNavX` | Resets the gyro heading to 0 degrees | `SwerveSubsystem` | Driver Y button (hold) |

### How Buttons Connect to Commands

The connections between buttons and commands are set up in `RobotContainer.java`, inside the `configureBindings()` method. Think of it like programming a TV remote -- "when I press this button, do this thing."

There are two main ways to bind a button:

- **`whileTrue(command)`** -- like holding down the trigger on a power drill. The command runs as long as you hold the button, and stops the moment you let go. Most of our commands use this. When you release the button, the command's `end()` method is called to clean up (stop the motors).

- **`onTrue(command)`** -- like flipping a light switch. One press starts the command, and it keeps going even after you release the button. The command runs until `isFinished()` returns `true` or another command interrupts it.

The connection flow looks like this:
```
Button Press  -->  Command Created  -->  Command Uses Subsystem  -->  Motors Move
Driver B      -->  new Shooter()    -->  ShooterMechanism         -->  Flywheel spins
Operator B    -->  new Intake()     -->  IntakeMechanism           -->  Rollers spin in
Driver RB     -->  new AlignToTag() -->  SwerveSubsystem           -->  Robot rotates to tag
```

### Key Concepts Explained

<details>
<summary><strong>Field-Relative vs Robot-Relative Driving</strong></summary>

Imagine you're playing a video game where you're looking down at your character from above:

- **Field-relative** (default): Pushing the joystick "up" always moves the robot toward the far end of the field, no matter which way the robot is facing. If the robot is turned sideways, pushing "up" still goes forward on the field. This is usually the most intuitive mode -- it works like controlling a character in a top-down video game.

- **Robot-relative**: Pushing "up" moves the robot wherever its nose is pointing. If the robot is turned sideways, pushing "up" moves it sideways across the field. This can be useful in specific situations but is generally harder for drivers.

Field-relative driving needs the **gyroscope (NavX)** to know which way the robot is facing. If the gyro drifts or gets confused, press **Y** on the driver controller to reset it.

The driver can toggle between modes: **X button = field-relative ON**, **A button = robot-relative**.

</details>

<details>
<summary><strong>PID Control</strong></summary>

PID stands for **Proportional-Integral-Derivative**, but the concept is simple. It's like **cruise control in a car**.

Say you set cruise control to 60 mph and you're currently going 45 mph. The system needs to figure out how hard to press the gas pedal:
- **P (Proportional)**: "I'm 15 mph too slow, so press the gas proportionally hard." The further from the target, the harder it pushes. This is the most important term.
- **I (Integral)**: "I've been a little bit slow for a while now, let me push a tiny bit harder to make up for it." Corrects persistent small errors that P alone can't fix. (We usually keep this at 0.)
- **D (Derivative)**: "I'm getting close to 60 mph fast -- better ease off the gas so I don't overshoot." Prevents the system from slamming past the target and oscillating back and forth.

On our robot, we use PID to:
- Point the swerve wheels at the right angle (rotation PID -- `kPRotation`)
- Drive the wheels at the right speed (drive PID -- `kPDrive`)
- Auto-rotate to face AprilTags (alignment PID -- in `AlignToTag.java`)
- Follow autonomous paths accurately (PathPlanner PID)

If you see the wheels wobbling back and forth, the P value is probably too high. If they're slow to reach their target angle, P is too low. You can tweak these in `Constants.java`.

</details>

<details>
<summary><strong>Odometry and Pose Estimation</strong></summary>

Odometry is how the robot tracks its own position on the field. It's like walking through a dark room while counting your steps -- you can't see where you are, but if you know where you started, how many steps you took, and which direction you walked, you can estimate your position.

The robot combines two things for basic odometry:
- **Wheel encoders**: "How far has each wheel rolled?" (the steps)
- **Gyroscope**: "Which direction am I facing?" (the compass)

**Pose estimation** (what we actually use) goes one step further. It combines encoder + gyro data with **Limelight vision measurements**. When the Limelight camera sees an AprilTag on the field, it can calculate the robot's exact position. This corrects the odometry drift -- like occasionally opening your eyes while counting steps in that dark room.

Our `SwerveDrivePoseEstimator` fuses both data sources for maximum accuracy. Every 20ms, it:
1. Updates from wheel encoders + gyro (always available, but drifts over time)
2. Adds vision corrections from the Limelight (only when AprilTags are visible, but very accurate)

This estimated position is critical for autonomous routines (the robot needs to know where it is to follow a path) and for distance-based auto-shooting.

</details>

<details>
<summary><strong>How to Create a New Command</strong></summary>

If you want to add a new action to the robot (like "spin a new motor" or "activate a mechanism"):

1. **Create a new file** in `src/main/java/frc/robot/commands/` (copy an existing simple command like `Intake.java` as a starting point -- it's the simplest one).
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

</details>

<details>
<summary><strong>How to Create a New Subsystem</strong></summary>

If you add new hardware to the robot (like a new mechanism):

1. **Add constants** (CAN IDs, speeds, etc.) to `Constants.java` in a new inner class. This keeps all the "settings" in one place.
   ```java
   public static class NewMechanismConstants {
       public static final int motorPort = 50;
       public static final double motorSpeed = 0.5;
       public static final int currentLimit = 35;
   }
   ```
2. **Create a new file** in `src/main/java/frc/robot/subsystems/` (copy `IntakeMechanism.java` as a starting template -- it's the simplest subsystem).
3. **Create motor/sensor objects** in the constructor, configure them, and add methods to control them (like `setSpeed()`, `stop()`, etc.).
4. **Create command(s)** for the new subsystem in the `commands/` folder.
5. **Instantiate the subsystem** in `RobotContainer.java` (create it as a field at the top of the class) and bind commands to buttons in `configureBindings()`.

</details>

## Controller Layout

### Driver Controller (Xbox, Port 0)

```
              [LB: Auto-Shooter]  [RB: Align to Tag]
              [LT: Speed Slider]  [RT: --]
                    ___                ___
                   /   \              /   \
Left Stick:       | L   |            | R   |   Right Stick:
 Forward/Back     | Y/X |            | X   |    Rotation
 & Strafe          \___/              \___/

        [X: Field-Relative ON]    [Y: Reset NavX]
        [A: Robot-Relative ON]    [B: Shooter]
```

| Button | Action |
|---|---|
| Left Stick | Move the robot (forward/backward + strafe left/right) |
| Right Stick X | Rotate the robot |
| Left Trigger | Speed slider (pull to slow down for precise movements) |
| X Button | Enable field-relative driving |
| A Button | Enable robot-relative driving |
| Y Button | Reset gyro heading |
| B Button | Run shooter (constant speed) |
| Left Bumper | Auto-shooter (distance-based speed via Limelight) |
| Right Bumper | Align to AprilTag (auto-rotation while allowing manual driving) |

### Operator Controller (Xbox, Port 1)

| Button | Action |
|---|---|
| B Button | Run intake (hold to suck in game piece) |
| X Button | Run outtake (hold to spit out game piece) |
| Left Bumper | Climb up (hold) |
| Right Bumper | Climb down (hold) |

## Hardware Overview

### Drivetrain: Swerve Drive (4 Modules)

Each swerve module has:
- 1 **NEO drive motor** (SparkMax controller) -- spins the wheel
- 1 **NEO steering motor** (SparkMax controller) -- points the wheel
- 1 **CANcoder absolute encoder** -- always knows the wheel angle

| Module | Drive CAN ID | Steering CAN ID | CANcoder CAN ID |
|---|---|---|---|
| Front Left | 4 | 7 | 14 |
| Front Right | 5 | 3 | 13 |
| Back Left | 6 | 1 | 12 |
| Back Right | 8 | 2 | 11 |

**Gyroscope:** Studica NavX (AHRS) via MXP SPI -- measures robot heading
**Wheelbase:** 21" x 21" (0.5334m x 0.5334m)

### Intake Mechanism

- **Motor:** REV SparkMax + NEO (CAN ID 40)
- **Purpose:** Roller that sucks in or spits out FUEL (game pieces)
- **Speeds:** 75% intake, 75% outtake (reverse)
- **Current Limit:** 35A

### Shooter Mechanism

- **Flywheel Motor 1:** REV SparkMax + NEO (CAN ID 42)
- **Flywheel Motor 2:** REV SparkMax + NEO (CAN ID 43)
- **Kicker Motor:** REV SparkMax + NEO (CAN ID 44)
- **Purpose:** Dual flywheel launches FUEL into the HUB. Kicker feeds the game piece into the spinning flywheel at 50% speed.
- **Speed:** 75% constant mode, or auto-adjusted 35%-95% based on Limelight distance
- **Current Limit:** 35A per motor

### Climb Mechanism

- **Motor:** REV SparkMax + NEO (CAN ID 45)
- **Purpose:** Lifts the robot onto the chain during endgame
- **Speed:** 75% up/down
- **Current Limit:** 35A

### Vision: Limelight Camera

- **Camera:** Limelight (accessed via NetworkTables)
- **Purpose:** Detects AprilTags on the field for:
  - **Auto-alignment** -- `AlignToTag` command auto-rotates the robot to face the target
  - **Distance measurement** -- `AutoShooter` command reads distance to set flywheel speed
  - **Pose estimation** -- `updateVisionPose()` feeds vision data into the pose estimator to correct odometry drift
- **Key NetworkTable Values:**
  - `tv` -- has valid target (0 or 1)
  - `tx` -- horizontal angle to target (degrees)
  - `tid` -- AprilTag ID of best target
  - `botpose_wpiblue` -- robot pose [x, y, z, roll, pitch, yaw, latency]
  - `targetpose_cameraspace` -- target position relative to camera [x, y, z, ...]

## Common Modifications

Here's where to look when you want to change specific robot behavior:

<details>
<summary><strong>Click to expand modification guide</strong></summary>

| I want to... | Where to look |
|---|---|
| Change the robot's max speed | `Constants.java` -> `DrivetrainConstants.maxVelocity` |
| Change how fast it accelerates | `Constants.java` -> `DrivetrainConstants.maxAcceleration` |
| Change teleop driving speed | `Constants.java` -> `DrivetrainConstants.kTeleDriveMaxSpeed` |
| Adjust joystick sensitivity/dead zones | `Constants.java` -> `OperatorConstants` deadband values |
| Change intake/outtake speed | `Constants.java` -> `IntakeConstants.intakeSpeed` / `outtakeSpeed` |
| Change constant shooter speed | `Constants.java` -> `ShooterConstants.shooterSpeed` |
| Change auto-shooter distance-to-speed curve | `Constants.java` -> `ShooterConstants.kDistanceToSpeedMap` |
| Change climb speed | `Climb.java` and `ClimbDown.java` -> `execute()` methods |
| Change which button does what | `RobotContainer.java` -> `configureBindings()` |
| Switch joystick axes | `RobotContainer.java` -> the `setDefaultCommand` lambdas |
| Tune swerve steering PID | `Constants.java` -> `kPRotation`, `kDRotation` |
| Tune swerve drive PID | `Constants.java` -> `kPDrive` |
| Tune auto path following PID | `SwerveSubsystem.java` -> `PPHolonomicDriveController` PID values |
| Tune AprilTag alignment PID | `AlignToTag.java` -> `turnPID` values in the constructor |
| Recalibrate swerve module angles | `Constants.java` -> `kFLOffsetRad`, `kFROffsetRad`, etc. |
| Change a motor CAN ID | `Constants.java` -> the appropriate mechanism Constants class |
| Switch between brake and coast mode | The subsystem's `configureMotor()` call -> `IdleMode.kBrake`/`kCoast` |
| Start in robot-relative mode | `SwerveSubsystem.java` -> `fieldRelativeStatus = false` |
| Change the speed slider range | `SwerveJoystickCmd.java` -> the `0.8` cap in `execute()` |
| Change the kicker speed | `ShooterMechanism.java` -> the `0.5` value in `setIOSpark()` |
| Change which AprilTags are HUB tags | `Constants.java` -> `ShooterConstants.kRedHubTags` / `kBlueHubTags` |
| Change the Limelight name | `Constants.java` -> `VisionConstants.kLimelightName` |
| Change the default auto routine | `RobotContainer.java` -> `AutoBuilder.buildAutoChooser("MobilityAuto")` |
| Add a new subsystem | Create class in `subsystems/`, add constants, instantiate in `RobotContainer` |
| Add a new command | Create class in `commands/`, bind it in `RobotContainer.configureBindings()` |
| Add a new auto routine | Create it in PathPlanner GUI, register named commands in `RobotContainer` |
| Add a new PathPlanner named command | `RobotContainer.java` -> `NamedCommands.registerCommand(...)` |

</details>

## Key Libraries

| Library | Version | Purpose |
|---|---|---|
| [WPILib](https://docs.wpilib.org/) | 2026.1.1 | Core FRC framework (TimedRobot, Commands, kinematics) |
| [AdvantageKit](https://docs.advantagekit.org/) | -- | Telemetry logging and replay for post-match analysis |
| [PathPlanner](https://pathplanner.dev/) | 2026.1.2 | Autonomous path planning and following |
| [REVLib](https://docs.revrobotics.com/revlib) | -- | SparkMax motor controller API |
| [Phoenix6](https://v6.docs.ctr-electronics.com/) | 26.1.0 | CTRE CANcoder absolute encoder API |
| [Studica (NavX)](https://docs.studica.com/) | -- | NavX AHRS gyroscope API |

Vendor dependency JSON files are in the `vendordeps/` directory.

## Troubleshooting / FAQ

<details>
<summary><strong>Click to expand common problems and solutions</strong></summary>

| Problem | Solution |
|---|---|
| `./gradlew build` fails with "could not find vendor dependency" | Open VS Code, press `Ctrl+Shift+P`, run **"WPILib: Manage Vendor Libraries"** -> **"Install new libraries (online)"** and re-add the missing library URL from `vendordeps/`. |
| `./gradlew deploy` fails with "no RoboRIO found" | Make sure you're connected to the robot's network (WiFi or USB). The RoboRIO must be on and configured with team number 6908. |
| Robot doesn't move during teleop | Check that the correct joystick/controller is plugged in and assigned to the right port in the Driver Station. Port 0 = driver, port 1 = operator. |
| Field-relative driving feels wrong / robot drifts | The gyro may have drifted. Press **Y** on the driver controller to reset the NavX heading. Make sure the robot is facing away from you when you press it. |
| Swerve wheels jitter or oscillate back and forth | The rotation PID's P value is too high. Lower `kPRotation` in `Constants.java` by small increments (e.g., 0.57 -> 0.5). |
| Swerve wheels are slow to reach target angle | The rotation PID's P value is too low. Increase `kPRotation` in `Constants.java` by small increments. |
| Shooter isn't firing far enough | Increase `shooterSpeed` in `Constants.java`, or if using auto-shooter, adjust the distance-to-speed lookup table (`kDistanceToSpeedMap`). |
| Auto-shooter fires at wrong speed | Calibrate the `kDistanceToSpeedMap` lookup table by testing at known distances and adjusting the speed entries. |
| AlignToTag oscillates / can't lock on | Adjust the `turnPID` P value in `AlignToTag.java` (decrease if oscillating, increase if too slow to center). |
| Limelight isn't showing data | Check power/network, verify `VisionConstants.kLimelightName` matches the Limelight's web interface name, and check `http://limelight.local:5801`. |
| Simulation mode won't start | Make sure `includeDesktopSupport = true` in `build.gradle`. Run `./gradlew simulateJava`. |
| "Unresolved dependency" or Gradle sync issues | Run `./gradlew clean` then `./gradlew build`. If it persists, check your internet connection. |
| Code deploys but nothing happens | Open the Driver Station and check for errors. Make sure you're in the right mode (Teleop, not Disabled). Check CAN wiring with Phoenix Tuner / REV Hardware Client. |
| Motors aren't responding | Check CAN wiring, verify the CAN ID matches `Constants.java`, use REV Hardware Client to confirm the SparkMax is detected. |
| `git push` rejected | Someone else pushed changes. Run `git pull` first to merge their changes, then push again. |
| Merge conflict after `git pull` | Don't panic. VS Code highlights conflicts with `<<<<<<<` markers. Pick which version to keep, save, then `git add` and `git commit`. |

</details>

## Glossary

<details>
<summary><strong>Click to expand glossary of terms</strong></summary>

| Term | What it means |
|---|---|
| **AprilTag** | A black-and-white visual marker (like a QR code) placed on the field. The Limelight camera detects these to determine the robot's position, angle, and distance to targets. |
| **Autonomous (Auto)** | The period at the start of a match where the robot runs pre-programmed routines without human input. |
| **CAN bus** | A wiring network that connects the RoboRIO to motor controllers, encoders, and other devices. Each device gets a unique ID number (like a mailing address). |
| **CANcoder** | An absolute encoder made by CTRE that connects over CAN bus. It always knows the exact angle of the wheel, even after a reboot -- unlike a relative encoder that starts at zero. |
| **ChassisSpeeds** | A WPILib class representing the robot's translational and rotational velocities (forward, strafe, rotation). |
| **Command** | A task for the robot to do (e.g., "spin the intake", "shoot"). Commands have a lifecycle: `initialize()` -> `execute()` (repeats) -> `end()`. Like a recipe card. |
| **CommandScheduler** | The "brain" that runs all commands and subsystems every 20ms. It starts/stops commands, checks buttons, and calls periodic methods. |
| **Deadband** | A small zone around the joystick center where input is ignored. Prevents the robot from drifting when you let go of the stick. |
| **Deploy** | Uploading your compiled code from your laptop to the RoboRIO (the robot's onboard computer). |
| **Desaturate** | Scaling wheel speeds proportionally so no wheel exceeds the maximum velocity while maintaining the desired motion. |
| **Encoder** | A sensor that measures rotation. **Relative encoders** count rotations from when they were last reset. **Absolute encoders** always know their exact angle. |
| **Field-relative** | Driving mode where "forward" on the joystick always means toward the far end of the field, regardless of which way the robot is facing. Requires the gyroscope. |
| **FUEL** | The 2026 game piece -- foam balls that robots collect and score into the HUB. |
| **Gyroscope (NavX)** | A sensor that tracks which direction the robot is facing. Used for field-relative driving, odometry, and pose estimation. Can drift over time. |
| **HUB** | The central scoring structure on the 2026 field. Has AprilTags for vision targeting. Also called the TOWER. |
| **Interpolating Map** | A lookup table that calculates values between known data points by drawing straight lines between them. Used for distance-to-shooter-speed conversion. |
| **Kinematics** | The math that converts desired robot motion (forward, strafe, rotate) into individual wheel speeds and angles (and vice versa). |
| **Lambda** | A shorthand way to pass a function as a value in Java. Written as `() -> someMethod()`. Used so the robot reads joystick values fresh every 20ms. |
| **Limelight** | A smart vision camera that detects AprilTags and provides target data via NetworkTables. Used for auto-alignment, distance measurement, and pose estimation. |
| **NetworkTables** | A shared data table that allows different parts of the robot system (code, dashboard, cameras) to communicate in real time. |
| **Odometry** | Tracking the robot's position on the field using wheel encoders and the gyroscope. Accurate short-term but drifts over time. |
| **PathPlanner** | A tool for creating and following autonomous paths. Paths are designed in a GUI and stored as JSON files in the deploy directory. |
| **PID Controller** | A control algorithm (Proportional-Integral-Derivative) that smoothly drives a measured value toward a target. Like cruise control in a car. |
| **Pose** | The robot's position AND heading on the field, represented as (x, y, theta). |
| **Pose Estimator** | An advanced odometry system that fuses wheel encoder data with vision measurements from the Limelight for higher accuracy. |
| **RoboRIO** | The main computer on the robot. Runs our Java code and communicates with all hardware. Made by National Instruments for FRC. |
| **Robot-relative** | Driving mode where "forward" on the joystick means wherever the robot's nose is pointing. Simpler but harder for drivers when the robot is turned. |
| **Shift** | In the 2026 game, a 25-second window during teleop where one alliance's HUB is active and the other's is inactive. |
| **Slew rate** | How quickly a value is allowed to change per second. A slew rate limiter prevents sudden jumps in motor speed, making the robot accelerate smoothly. |
| **SparkMax** | A motor controller made by REV Robotics that drives NEO brushless motors. Connects over CAN bus. Each one has a unique CAN ID. |
| **Subsystem** | A class that controls a physical mechanism on the robot. Only one command can use a subsystem at a time. Like a station in a kitchen. |
| **Swerve drive** | A drivetrain where each of the 4 wheels can independently spin (drive) and pivot (steer). This lets the robot move in any direction. |
| **Swerve Module** | One corner of the swerve drivetrain: a drive motor + steering motor + absolute encoder working together. |
| **Teleop** | The driver-controlled period of a match (typically ~2 minutes 20 seconds). |
| **TimedRobot** | WPILib's robot base class. Calls periodic methods every 20ms (50 times per second). The robot's heartbeat. |
| **TOWER** | The 2026 climbing structure and scoring goal. Robots climb it during endgame for bonus points. Also called the HUB. |
| **Vendor dependency** | A third-party library (like REVLib or Phoenix6) that adds support for specific hardware. Configured via JSON files in `vendordeps/`. |

</details>

## Contributing Guidelines

### Branch Naming

Use short, descriptive branch names that explain what you're working on:
- `tune-shooter-speed` -- good
- `add-vision-alignment` -- good
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
- `Fix shooter overshoot by lowering flywheel speed` -- good
- `stuff` -- not helpful
- `asdfasdf` -- definitely not helpful

### Code Style

- Follow the existing patterns in the codebase. If you're adding a new command, look at `Intake.java` for the simplest example.
- Keep constants in `Constants.java`, not hardcoded in commands or subsystems.
- Always call `addRequirements()` in command constructors so the scheduler knows which subsystem your command needs.
- Add documentation comments explaining WHY things are done, not just what.
- Test your code with `./gradlew build` before pushing. If it doesn't compile, it shouldn't be pushed.

## Team Number

**6908** (configured in `.wpilib/wpilib_preferences.json`)
