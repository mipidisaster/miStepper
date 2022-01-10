# miStepper
This library contains the files and support for creating and installed code for the miStepper Device.e with this device).

# Linked libraries:
## milibrary
This library makes use of an extra repository - ['milibrary'](https://github.com/mipidisaster/miLibrary), which is not included within this repository; due to it being a seperate one. So this will need to be included...
> It doesn't matter were this is stored, however it should be at a high enough level that any lower level scripts can make reference to it.

```
<project root>
  ├─ milibrary
  │      └─ <contents>
  └─ < other folders >
```

# Stack overview
> To be added

# Build Tree
![build tree](/_image/Build_tree.png)

v1.1.0
* Changes made to the software to incorporate the motor HAL - Fan/Stepper, as well as changes to the USART interface (making it common with the 'miStepper_ROS' repository
* Issues [#4](https://github.com/mipidisaster/miStepper/issues/4) and [#5](https://github.com/mipidisaster/miStepper/issues/5) fixed
* Issue [#7](https://github.com/mipidisaster/miStepper/issues/7), partial progress has been made on working this - mainly that a single task now exists for the motors
* Submodules introduced for the 'miStepper_ROS' and 'miLibrary' repositories so as to manage which version(s) are appliable to this

v1.0.0
* Major update of build, changing the electronics (PCB):
  1. PCB/Electronics updated to include new Stepper IC - (issue [#1](https://github.com/mipidisaster/miStepper/issues/1))
  2. Hardware filtering on the voltage/current monitors
  3. Extra temperature sensors added - 2 onboard sensors, and 2 external sensors
  4. CAN interface improved, such that it not is compliant to the protocol/hardware
  5. 4 layer PCB, to aid thermal cooling

* Software code updated:
  1. Supports latest release of 'milibrary' - [v0.2.1](https://github.com/mipidisaster/miLibrary/tree/v0.2.1)
     * Closing issue [#3](https://github.com/mipidisaster/miStepper/issues/3)
  2. Update to support latest hardware additions
  > Note, external sensor support not provided currently, to be added in next release
  
  3. Re-structing of the code, such that it follows the ['Layered Software Architecure'](https://github.com/mipidisaster/miLibrary/wiki/Software-Architecture) as per milibrary wiki - (issue [#2](https://github.com/mipidisaster/miStepper/issues/2))


v0.1.0
* Initial release (to be used with [milibrary v0.1.0](https://github.com/mipidisaster/miLibrary/tree/v0.1.0))

# Wiki
The wiki for this can be found within the Github repository for this - > to be added

See https://github.com/mipidisaster/miLibrary/wiki for basic information on Eclipse setup/remote ROS downloading...

# Issues
If you find any issues with this library, please capture them within the Github repository issue area - https://github.com/mipidisaster/miStepper/issues.