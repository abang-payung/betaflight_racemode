## Welcome NotFastEnuf's RACEMODE in Betaflight project
## RACEMODE is a complete rewrite and replacement of the Horizon Flight Mode in betaflight intended for whoop racers.  The basic recipe for racemode is leveling assistance on the roll axis and pure acro (rate mode) behavior on the pitch axis. To activate racemode, flash a RACEMODE_Betaflight build and activate Horizon flight mode via switch as configured on the modes tab.

*There are 2 types of RACEMODE flight behavior selectable via CLI*

**RACEMODEangle** is a flight mode that will limit your roll axis to the max angle limit value set on the pids tab.  The leveling strength below max angle limit is adjustable by changing the horizon strength variable. Within angles below max angle limit, roll axis (only) will feel & behave exactly like angle mode does.  Because it will be possible to reach angles beyond max angle limit by pitching to inverted orientations - inverted behavior on the roll axis consists of 2 zones. In between 180 degrees inverted and max angle limit, there is a user set angle via CLI called racemode_tilt_effect.  Between 180 degrees inverted roll and racemode_tilt_effect value - behavior is pure acro (rate mode).  Passing racemode_tilt_effect angle will start to fade in leveling behaviour gradually till the craft reaches max angle limit and pops back into restricted angle mode type behavior. Default value for racemode_tilt_effect is 85 degrees and is suggested to be at least 20 degrees above max angle limit so a decent transition zone exists between acro and leveled zones.  RACEMODEangle is the default type of racemode and corresponds to CLI variable set racemode_horizon = OFF

**RACEMODEhorizon** is an unrestricted flight mode on roll axis and can be activated to replace RACEMODEangle on CLI with the command set racemode_horizon = ON.  There are 3 configurable zones of flight behavior for the roll axis.  Zone 1 has full leveled strength from a neutral orientation up to an angle set by the horizon transition value on the pids tab.  Zone 2 spans from the angle set by the horizon transition value (on pids tab) to the angle set on CLI with the racemode_tilt_effect value. Within zone 2 as the craft's roll angle of orientation increases, leveling strength is gradually reduced and is mixed in with a gradually increasing amount of acro (rate mode) behavior.   Once the racemode_tilt_effect angle is reached and beyond, behavior is pure acro with no leveling for zone 3.  Default value for horizon transition is set to 0 degrees effectively eliminating zone 1 for a more familiar but improved upon "horizon type feel".  Increase horizon transition to open zone 1's more aggressive leveling range if desired.  Default value for racemode_tilt_effect is set to 85 degrees on CLI - this is where all leveling stops.  Setting this variable to 0 or below horizon transition will turn off all leveling and setting it above 180 degrees will be constrained to a value of 180 degrees in firmware.


Join discussion of RACEMODE on betaflight, or reach out to request your fc target be compiled and posted for download here:  insert mmc link here

**Download your RACEMODE target here:** https://github.com/NotFastEnuf/betaflight/releases/tag/v3.2.4

*-NotFastEnuf*

![Betaflight](https://raw.githubusercontent.com/wiki/betaflight/betaflight/images/betaflight/bf_logo.png)
![Important Notice: Betaflight 4.0 will be the last release to include support for STM32F3 based flight controllers. (This includes all boards with 'F3' in the name.)](docs/assets/images/stm32f3_retirement_notice.svg)

(Please see the [note](https://github.com/betaflight/betaflight#end-of-active-development-for-stm32f3-based-flight-controllers) below.)


![Betaflight](docs/assets/images/bf_logo.png)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## News

### Betaflight 4.1

For Betaflight 4.0 we announced the advent of Unified Targets, and with them the capability to change all of the hardware resources on all (F4 and F7) targets. Unified Targets have been used successfully by testers since Betaflight 4.0 has been released, and users using the new bidirectional Dshot protocol have been using configurable resources to make this protocol work on their targets since then.

But we were still not happy with the way that users had to use Unified Targets: The target specific hardware configuration had to be applied after the firmware was flashed, and re-applied every time the configuration was reset to defaults - we wanted to make the Unified Targets as easy to use as the existing 'legacy' targets are.

We have figured out a way to make this work, and are currently working on implementing the required changes in the firmware and in Betaflight configurator, but it is taking longer to complete than we had anticipated. To allow us to get the integration of Unified Targets completed, **we have shifted the release date for Betaflight 4.1 to 01 October 2019**. Until then we will keep doing monthly releases of Betaflight 4.0 with bugfixes and new / updated targets.

To get the latest update from us, you can now also visit our webpage at https://betaflight.com/.


### New requirements for the submission of new and updated targets

As [announced earlier](https://github.com/betaflight/betaflight#betaflight-40), Betaflight 4.0 is introducing a radically new way to define targets, the so-called 'Unified Targets'.

This new approach makes it possible to use the same firmware binary (the so called 'Unified Target firmware') for all boards that share the same MCU type (only supported on F4 and F7). Manufacturers will be able to add support for new boards by simply publishing a new configuration (the so called 'Unified Target configuration') for their new board. Users can then simply load the already published Unified Target firmware and the new Unified Target configuration onto their new board to get it to work.

Work to give users a simple way to flash unified targets in Betaflight configurator still needs to be done, so Betaflight 4.0 will be released with targets done in the 'legacy' way. But the plan is to add support for seamless use of Unified Targets into Betaflight configurator after Betaflight 4.0 has been released, and convert all of the existing F4 and F7 targets to the new format after the release of Betaflight 4.1.

In order to be prepared for this move, the following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. No new F3 based targets will be accepted;

2. For any new target that is to be added, both a 'legacy' format target definition into `src/main/target/` and a new Unified Target config into https://github.com/betaflight/unified-targets/tree/master/configs/default need to be submitted. See the [instructions](https://github.com/betaflight/betaflight/blob/master/unified_targets/docs/CreatingAUnifiedTarget.md) for how to create a Unified Target configuration;

3. For changes to existing targets, the change needs to be applied to both the 'legacy' format target definition in `src/main/target/` and a new Unified Target config in https://github.com/betaflight/unified-targets/tree/master/configs/default. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted alongside the proposed change.


### End of active development for STM32F3 based flight controllers

For a while now, development of Betaflight for flight controllers based on the STM32F3 chip has been hampered by a severe limitation that this chip has: Unlike the STM32F4 and STM32F7 models, the STM32F3 versions that are used on flight controllers have only a very limited amount of flash space available to fit the firmware into. This has meant that, starting from around version 3.3, the majority of the new features that were developed for Betaflight could not be added to STM32F3 based boards. Even worse, due to improvement in basic features, other more and more of the less commonly used features had to be removed from these flight controllers, and a number of them are at a point where they only support the bare minimum of functionality required to make them fly.

This means that, even if we kept supporting STM32F3 based boards in future releases, there would only be little advantage in this, as there simply is no space left on STM32F3 to add any of the new features that these releases will contain.

For this reason, and because the effort required to remove features from STM32F3 based flight controllers on a weekly basis is cutting into the time that we have to actually develop new features, we have decided to drop support for STM32F3 based flight controllers after the last release of 4.0.

This does not mean that it won't be possible to use these flight controllers after this point in time - they will still work fine when used with the last release of 4.0, just as there are thousands of users who are still enjoying their STM32F1 based flight controllers with Betaflight 3.2.5. We will also strive to keep these versions supported in new releases of configurator, so that users still using these flight controllers will be able to configure them with the same configurator that they use to configure their STM32F4 and STM32F7 based boards.


## Events

| Date  | Event |
| - | - |
| 01 September 2019 | Start of feature freeze / Release Candidate window for Betaflight 4.1 |
| 01 October 2019 | Planned [release](https://github.com/betaflight/betaflight/milestone/30) date for Betaflight 4.1 |

## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300, 600 and 1200), Multishot, and Oneshot (125 and 42) motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F7, F4 and F3 processors
* PWM, PPM, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CSRF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://github.com/betaflight/betaflight/wiki

## Support and Developers Channel

There's a dedicated Slack chat channel here:

https://slack.betaflight.com/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/betaflight/betaflight-configurator/releases/latest

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help? Help others!
* Bug reporting & fixes.
* New feature ideas & suggestions.

The best place to start is the Betaflight Slack (registration [here](https://slack.betaflight.com/). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](docs/development/Development.md ) and [coding style definition](docs/development/CodingStyle.md).

TravisCI is used to run automatic builds

https://travis-ci.com/betaflight/betaflight

[![Build Status](https://travis-ci.com/betaflight/betaflight.svg?branch=master)](https://travis-ci.com/betaflight/betaflight)

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those whom have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
