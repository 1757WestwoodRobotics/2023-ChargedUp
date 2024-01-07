# 2023-ChargedUp

Code for the FRC 1757 2023 Competition Bot

[Chief delphi form](https://www.chiefdelphi.com/t/frc-1757-wolverines-2022-2023-build-thread/416564)

[The Blue Alliance](https://www.thebluealliance.com/team/1757)

[Website](https://whsrobotics.org)

## Installation (but for landon specifically)

### Visual Studio 2019 redistributable

[vc_redist.x64](https://aka.ms/vs/16/release/vc_redist.x64.exe)

### Python

[3.9.6 amd64](https://www.python.org/ftp/python/3.9.6/python-3.9.6-amd64.exe)

### VS Code

[VS Code](https://code.visualstudio.com)

### FRC Game Tools

[FRC Game Tools](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#369633)

### FRC Radio Configuration Utility

[FRC Configuration Utility](https://firstfrc.blob.core.windows.net/frc2020/Radio/FRC_Radio_Configuration_20_0_0.zip)

### CTRE Phoenix

[Phoenix Tuner](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases)

## Setup

### roboRIO

1. Image the roboRIO
   [Imaging instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html)
1. Configure the roboRio
   | Item | Value |
   | - | - |
   | Team number | `1757` |
   | Firmware | `6.0.0f1` |
   | Image | `FRC_roboRIO_2021_v3.0` |
   | Static IP | `10.17.57.2` |
   | Subnet Mask | `255.255.255.0` |

### Run Phoenix Tuner

#### Update device firmware

- PDH
- FalconFX
- CANCoder
- Pneumatics Hub

#### Configure CAN devices

| Device              | Class   | Range   | ID             |
| ------------------- | ------- | ------- | -------------- |
| robo_rio            | core    | 0 - 9   | master (no ID) |
| pdh                 | core    | 0 - 9   | 0              |
| pneumatic_hub       | core    | 0 - 9   | 1              |
| front_left_drive    | motors  | 10 - 29 | 10             |
| front_left_steer    | motors  | 10 - 29 | 11             |
| front_right_drive   | motors  | 10 - 29 | 12             |
| front_right_steer   | motors  | 10 - 29 | 13             |
| back_left_drive     | motors  | 10 - 29 | 14             |
| back_left_steer     | motors  | 10 - 29 | 15             |
| back_right_drive    | motors  | 10 - 29 | 16             |
| back_right_steer    | motors  | 10 - 29 | 17             |
| left_climb_motor    | motors  | 10 - 29 | 24             |
| right_climb_motor   | motors  | 10 - 29 | 25             |
| front_left_encoder  | sensors | 40 - 59 | 40             |
| front_right_encoder | sensors | 40 - 59 | 41             |
| back_left_encoder   | sensors | 40 - 59 | 42             |
| back_right_encoder  | sensors | 40 - 59 | 43             |

#### Configure network devices

| Device                  | IP Address   | Subnet Mask       |
| ----------------------- | ------------ | ----------------- |
| OpenMesh radio          | `10.17.57.1` | `???.???.???.???` |
| roboRIO                 | `10.17.57.2` | `255.255.255.000` |
| Driver Station (laptop) | `10.17.57.5` | `255.000.000.000` |

### Install robotpy

- **IMPORTANT: Perform ALL operations in a python virtualenv**

#### Create virtualenv (if not previously done)

Recommend placing the virtualenv in the `mentorbot` repo folder under `.venv` (to keep everything together) however the virtualenv is local to your system and should not be uploaded (ignored in `.gitignore`)

```bash
cd <path-to-mentorbot-repo>
py -3 -m venv ./.venv
```

#### Workflow

1. **Activate virtualenv**
   (Virtualenv activation may differ depending on your operating system and terminal)
   - Git Bash (Windows)
     ```bash
     source <path-to-mentorbot-repo>/.venv/Scripts/activate
     ```
   - normal bash (linux, macOS)
     ```bash
     source <path-to-mentorbot-repo>/.venv/bin/activate
     ```
1. **Update pip and wheel. Old versions of pip can prevent binary wheels from being installed. Installing wheel makes other installs faster**
   (must have internet connection)
   ```bash
   python -m pip install --upgrade pip wheel
   ```
1. **Install / update robotpy**
   (must have internet connection)
   ```bash
   python -m pip install -r requirements.txt
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Download python for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download-python
   ```
1. **Download robotpy modules for roboRIO**
   (must have internet connection)
   ```bash
   python -m robotpy_installer download robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Install python on roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install-python
   ```
1. **Upload robotpy modules to roboRIO**
   (must be connected to roboRIO)
   ```bash
   python -m robotpy_installer install robotpy
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))
1. **Deploy robotpy program**

   - To robot
     (must be connected to roboRIO)
     ```bash
     python robot.py deploy
     ```
   - To simulator
     ```bash
     python robot.py sim
     ```

### Steps to take when commiting

1.  **Make sure Pylint, Black and Prettier are installed**

- Run Pylint in terminal and check for a help output. It should be autoinstalled with python.

```bash
pylint
```

- Run Black in terminal empty and look for "No path provided". If nothing happens, run the second command

```bash
black

python -m pip install -U black
```

1.  **Make Pylint happy**

- Run Pylint on your files

```bash
pylint $(git ls-files "*.py")
```

- With the error "Method could be a function" on an isFinished in a command, add this command above it. If necessary, swap out "no-self-use" with whatever error it gives you at the end.

```bash
# pylint: disable-next=no-self-use
```

1.  **Formatting with Black and Prettier**

- Run black on your files (autoformats). Also note that it can be configured to run on save of a file.

```bash
black .
```

-You also may need to format json files using Prettier. When opening a json file in VSCode, it should prompt you to download Prettier in a small window in the bottom right. If not, go to the extensions tab on the left and search "Prettier". The top result is it (about 19 million downloads)

1.  **Make sure it starts in sim and works as expected**

- Should be obvious but make sure to do it

1. **CODE FORMATTING PRACTICES**
   - for python we are using [black](https://github.com/psf/black)
   - for json we are using [prettier](https://prettier.io)
   - all JSON files must be alphabetized, you can use the [following extension for prettier](https://www.npmjs.com/package/prettier-plugin-sort-json)
   - If you're using VSCode, use json-sorter to sort the json without dealing with npm above [json-sorter](https://marketplace.visualstudio.com/items?itemName=msyesyan.json-sorter). Open command pallete in the file with ctrl+shift+p and type "alpha". "Sort JSON keys by alpha order" should show up, and hit enter to run it. Afterward, save to autoformat with prettier.
