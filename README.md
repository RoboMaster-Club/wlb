# Wheeled Bipedal Robot
This repository contains the control code for wheeled bipedal robot (WBR).

## Demonstration
<div align="center">
  <img src="asset/demo_1_ramp.gif" alt="GIF 1" width="60%">
</div>

<div align="center">
  <img src="asset/demo_2_disturbance_rejection.gif" alt="GIF 2" width="45%">
  <img src="asset/demo_6_failure_case.gif" alt="GIF 3" width="45%">
</div>

<div align="center">
  <img src="asset/demo_3_go_down_stairs.gif" alt="GIF 4" width="45%">
  <img src="asset/demo_4_leg_adaptation.gif" alt="GIF 5" width="45%">
</div>

<div align="center">
  <img src="asset/demo_5_slippage_resistance.gif" alt="GIF 6" width="45%">
  <img src="asset/demo_7_failure_case_slipping.gif" alt="GIF 7" width="45%">
</div>



# Repository Initialization Guide
``` bash
git clone https://github.com/RoboMaster-Club/wlb.git
cd wlb
git submodule update --init --recursive
```

## VSCode MAKEFILE environment setup guide
### Install tools
Download VSCode from [here](https://code.visualstudio.com/download)

**Linux**
- Install tools
```bash
sudo apt update
sudo apt upgrade
sudo apt install openocd
sudo apt install gcc-arm-none-eabi gdb-arm-none-eabi
```
- Check tool path with `which`
```
which arm-none-eabi-gcc
which arm-none-eabi-gdb //will be used when debug, so remember this path
which openocd
```
- Install extensions
   - Install Cortex Debug vscode extension
   - Edit the extension setting .json file
```json
"cortex-debug.gdbPath": "/usr/bin/gdb-multiarch" // this is the output from last step
```

**Windows**
- Download MSYS2 from [here](https://www.msys2.org/)
- The default installation path is `C:\msys64`, run `C:\msys64\msys2.exe`.
- Install OpenOCD, arm-none-eabi-gcc, and gdb-multiarch by running these commands in MSYS2 terminal.
```powershell
pacman -S mingw-w64-x86_64-make
pacman -S mingw-w64-x86_64-openocd
pacman -S mingw-w64-x86_64-arm-none-eabi-gcc
pacman -S mingw-w64-x86_64-gdb-multiarch
```
- Modify Environmental Variables
Add `C:\msys64\mingw64\bin` to `PATH`

> Refer to [Common Issues](#common-issues) section for local pointer to openocd and GNU toolchain

**MacOS - Apple Silicon**
 - Install Arm embedded toolchain and OpenOCD and arm-none-eabi-gdb using [homebrew](https://docs.brew.sh/Installation).
```zsh
brew install gcc-arm-embedded  
brew install openocd
```

**Make sure to add necessary tools to VSCode settings.json** Alternatively, you can add them to PATH variable to allow them to be accessed globally.

### Set up VSCode
- Add the tool path for OpenOCD and make tools.

- Install the VSCode extension [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) to enable ARM microcontroller debugging.

- Add GDB path by opening your VSCode settings.json in VSCode and add the following to the end of the file.
   - **Windows:** ```"cortex-debug.gdbPath": "c:/msys64/mingw64/bin/gdb-multiarch.exe"```.
   - **MacOS:** ```"cortex-debug.gdbPath": "/opt/homebrew/bin/arm-none-eabi-gdb"```.

## How to use
### Building the Project
Open the Command Palatte in VSCode: [Ctrl+Shift+P].
Then, select **Tasks: Run Build Tasks** and pick the appropriate build/flash task.
- **Windows:** build/flash (Windows). 
- **Unix-Like (Linux and MacOS):** build/flash.

> You can use the shortcut [Ctrl+Shift+B] or [Cmd+Shift+B].


### Debugging the Project
Navigate to [Run and Debug] in VSCode or press [Ctrl+Shift+D].
Select the appropriate launch configuration, depending on if you are using stlink or cmsis-dap debugger.
- **Windows:** dap/stlink (Windows)
- **Unix-Like (Linux and MacOS):** dap/stlink (Darwin)

> Click on the green play button or press [F5] to start debugging.

## Common Issues
### 1. Windows fails to initializing cmsis-dap debugger. 
Solution: Go to device manager and uninstall the usb device (probably having some error message in the list). Unplug and plug in the debugger again.

### 2. Tools (OpenOCD, make tools) not found
```
Failed to launch OpenOCD GDB Server:...
```
or
```
mingw32-make: The term 'mingw32-make' is not recognized as a name of a cmdlet, function, script file, or executable program.
Check the spelling of the name, or if a path was included, verify that the path is correct and try again.
```
**Solution1:**
Add openocd.exe to system environmental variable. If you followed the installation instruction in this README file, then OpenOCD should be install at default location `C:\msys64\mingw64\bin\openocd.exe`, for windows user. Add `C:\msys64\mingw64\bin` to system executable path.

**Solution2:**
If you don't want to mess with the system path, you could also add local openocd path in `.vscode/launch.json`. Add attribute `serverpath` by adding `"serverpath": "C:\\msys64\\mingw64\\bin\\openocd.exe"` in configuration.


> restarting terminal is liekly needed for new environment variable to take effect.

## Tips
### VSCode IntelliSense Configuration
```
"C_Cpp.default.compilerPath": "C:/msys64/mingw64/bin/arm-none-eabi-gcc.exe"
```
adding this would link the standard library header files, such as `stdint.h`, `stdlib.h`, `math.h`.
