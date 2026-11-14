# STM32FOC-HIL

A second board that stands in for the motor, so the drive can be tested
against something that behaves like hardware without any hardware being at
risk.

The drive firmware ([STM32FOC](https://github.com/JediJade/STM32FOC)) carries
its own motor model and closes the loop inside one chip. That proves the
arithmetic and proves nothing about the pins. This project moves the model
onto a second microcontroller, which:

1. **measures the three PWM duties** the drive puts out, on its real pins;
2. **runs the same motor model** - `src/vmotor.c` is the drive's file,
   copied without a single change;
3. **drives the sensor signals back** into the drive: Hall outputs, a
   throttle voltage, and the phase current as an analog voltage into the
   pin where a shunt amplifier would sit.

No power stage, no gate driver, no motor, no battery. Two microcontrollers
and a few wires.

The window for this board is `HIL.exe` on the
[releases page](https://github.com/JediJade/STM32FOC-HIL/releases): one file,
no Python, and `HIL.exe --model` runs all of it on made-up data.
[How to use both windows](https://github.com/JediJade/STM32FOC/blob/main/USAGE.th.md)
is written up in Thai on the drive's side, since the two are used together.

```
   STM32F103 (the drive)                 STM32F103 (this board)
   PA8   phase A high ─────────────>  PA0    TIM2 CH1, PWM input
   PA9   phase B high ─────────────>  PA6    TIM3 CH1, PWM input
   PA10  phase C high ─────────────>  PB6    TIM4 CH1, PWM input
   PB13  phase A low  ─────────────>  PB7    watched, not timed
   PB14  phase B low  ─────────────>  PB8    watched, not timed
   PB15  phase C low  ─────────────>  PB9    watched, not timed
   PB5   Hall A      <─────────────   PB12
   PB6   Hall B      <─────────────   PB13
   PB7   Hall C      <─────────────   PB14
   GND               ───────────────  GND    (both boards, always)
                                         PC13   LED: on while PWM is seen
                                         USB    the console, same CDC device
```

Two more wires and four passives add the other direction, once the bench
is to drive the current as well as the angle:

```
   HIL PA8 --[ 10k ]--+-- drive PA0     phase a current
                      |
                    [10nF]
                      |
                     GND

   HIL PA9 --[ 10k ]--+-- drive PA1     phase b current
                      |
                    [10nF]
                      |
                     GND
```

Those two pins carry a 300 kHz PWM whose duty is the model's phase
current; the RC turns it into a voltage, and the drive's ADC reads it
where a shunt amplifier would have been. Mid-rail is zero current. With
`isrc 1` the drive stops reading its own model and regulates what this
board sends it - which is what makes a load put on the shaft here reach
the controller over there.

Ten wires for the angle. The three high-side pins go to timers, which latch the period
and the high time in hardware and cost the model nothing per edge. The
three low-side pins are only read, once per model tick: a phase that ever
has both transistors on is a short across the supply on real hardware, and
catching that here instead is most of why a bench like this is worth
building. Nothing else is connected, and neither board has anything on it
that can be damaged by getting the wiring wrong.

## What it measures

With both halves crossing - the angle on three Hall pins, the current on
two ADC pins - a load applied to the rotor inside the bench board reaches
the controller in the other chip. Measured on the two boards, speed held
at 1200 units/ms:

```
  load   drive spd   drive iq   drive id
     0        1190       1331         42
     2        1199       1466        -35
     4        1272       1992          2
     8        1169       2316        -12
    15        1186       2968        -31
    27        1181       4151         80
    48        1201       6226          2
    85        1416      10240       -360
```

`iq` is the drive's own reading, off its own pins. It rises nearly five
times while the speed holds, because the controller is finding the torque
a brake somewhere else is asking for. `id` stays within a few tens of
zero throughout: the drive is putting that current on the q axis of a
rotor whose angle it also only knows from a wire. At a drag of 85 it runs
out - `iq` saturates and the speed is no longer held.

The same walk with only the angle crossing (stage one) left `iq` flat:
the drive was regulating the current of its own model, which nothing on
this bench could brake.

## Why this is worth building

Everything in the drive's own list of "never connected" is on the far side
of a pin: the Hall inputs, the throttle ADC, the current ADC, the break
input. A model that lives in the same chip can never exercise any of them.
This one has to, because the only way its numbers reach the drive is as
voltages on wires.

It also makes faults repeatable. A wire off, a Hall sensor that glitches, a
throttle that sticks, a break input pulled at the worst moment - all of them
become one line in a script, identical every run. That is hard to do with a
real motor and impossible to do with a model inside the firmware.

## What is here now

| | state |
| --- | --- |
| `unitymbed.json`, `Makefile`, `linker.ld`, `openocd.cfg` | done, STM32F103C8 |
| `src/vmotor.c`, `src/foc.c`, `src/foc_sin_table.c` | the drive's files, unchanged |
| `src/startup.c`, `src/usb_cdc.c` | the drive's files, unchanged |
| `src/pwmin.c` | the three duties, latched by hardware |
| `src/sensors.c` | the three Hall pins |
| `src/console.c`, `src/main.c` | the 20 kHz tick and a console in the drive's dialect |
| `studio/hil.py` | the window; runs today with `--model` |
| the current and throttle feedback | not yet: this part has no DAC |

**It builds**: 6,988 bytes of flash, no warnings. Five of the nine source
files were copied from the drive without a line changed - including the USB
device, which is why this board presents the same kind of console on the
same kind of cable.

## The window

```
python studio\hil.py --model      no board: see what it looks like
python studio\hil.py --port COM14 a board on that port
python studio\hil.py              pick a port in the window
```

Same look as `FOC.exe`, same console style - lines of `name value name
value` - so one terminal reads both boards and one pair of eyes reads both
windows.

It shows the three duties as they are measured off the drive's pins, the
model's rotor angle and speed, the phase currents it is reporting back, and
which Hall outputs are high. The shaft load and the motor preset are set
from the window.

## Why a second Blue Pill and not something better

Because the drive is a Blue Pill. The same part, the same 72 MHz crystal,
the same compiler flags: the model steps in a period of exactly the same
length, with the same arithmetic. Anything the two boards disagree about is
then a real disagreement and not a difference in what they are running on.

It also meant five of the nine files were a copy. A different part would
have needed a new startup file, a new clock, new headers and a different
console - a day of work that proves nothing about motor control.

The one thing this part has not got is a DAC, which stage two wants. An
MCP4921 on SPI1 settles in 8 us, comfortably inside a 50 us period, and
costs about a tenth of what a board with a built-in DAC does.

## Stages

1. **Hall only.** Capture one phase, run the model, drive three GPIO. The
   drive is set to `hsrc 1` and `sens 1`, and `hallcal` has to pass against
   real edges. Four pins.
2. **Current.** DAC into the drive's PA0. The drive's current loop starts
   running against a plant it reaches through its own ADC - the first time
   that path carries anything real.
3. **Throttle, brake, break input.** Then CAN and the dashboard, which can
   be tested on their own without the model running.
