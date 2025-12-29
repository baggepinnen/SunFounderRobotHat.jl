# SunFounderRobotHat.jl

[![Build Status](https://github.com/baggepinnen/SunFounderRobotHat.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/baggepinnen/SunFounderRobotHat.jl/actions/workflows/CI.yml?query=branch%3Amain)

Julia interface for [SunFounder Robot HAT](https://docs.sunfounder.com/projects/robot-hat-v4/) expansion boards (v4.x and v5.x) on Raspberry Pi.

Direct I2C/GPIO access via Linux kernel interfaces - no Python or external daemons required.

## Installation

```julia
using Pkg
Pkg.add(url="https://github.com/baggepinnen/SunFounderRobotHat.jl")
```

## Setup

Ensure I2C is enabled and accessible:

```bash
# Enable I2C (if not already)
sudo raspi-config  # Interface Options -> I2C -> Enable

# Grant I2C access (choose one):
sudo chmod 666 /dev/i2c-1              # Quick fix
sudo usermod -aG i2c $USER && logout   # Permanent (re-login required)
```

## Usage

```julia
using SunFounderRobotHat

# Open connection to Robot HAT
dev = opendevice()
println("Firmware: ", firmware_version(dev))

# Servo control
servo = Servo(dev, "P0")
angle!(servo, 15)
angle!(servo, -15)
angle!(servo, 0)

# PWM output
pwm = PWMChannel(dev, "P1")
freq!(pwm, 50)  # 1kHz
pulse_width_percent!(pwm, 50)

# ADC input
adc = ADCChannel(dev, "A0")
println("Raw: ", read(adc))
println("Voltage: ", read_voltage(adc), "V")

# Battery voltage
println("Battery: ", battery_voltage(dev), "V")

# Motor control (Mode 1: PWM + direction pin)
motor = Motor(dev, "P12", "D4")
speed!(motor, 50)   # Forward 50%
speed!(motor, -50)  # Backward 50%
speed!(motor, 0)    # Stop

# Cleanup
close(dev)
```

## API Reference

### Device
- `opendevice(; bus=1)` - Open I2C connection
- `firmware_version(dev)` - Get MCU firmware version
- `battery_voltage(dev)` - Read battery voltage (V)
- `reset_mcu()` - Reset the onboard MCU

### PWM (20 channels: P0-P19)
- `PWMChannel(dev, channel)` - Create PWM channel
- `freq!(pwm, hz)` - Set frequency
- `pulse_width!(pwm, value)` - Set pulse width (0-65535)
- `pulse_width_percent!(pwm, percent)` - Set duty cycle (0-100%)

### ADC (8 channels: A0-A7)
- `ADCChannel(dev, channel)` - Create ADC channel
- `read(adc)` - Read raw value (0-4095)
- `read_voltage(adc)` - Read voltage (0-3.3V)

### Servo
- `Servo(dev, channel; min_pulse=500, max_pulse=2500, offset=0.0)` - Create servo
- `angle!(servo, degrees)` - Set angle (-90 to +90)

### Motor
- `Motor(dev, pwm_channel, dir_pin; reversed=false)` - Mode 1 (TC1508S)
- `Motor(dev, pwm_a, pwm_b, Val(:mode2); reversed=false)` - Mode 2 (TC618S)
- `speed!(motor, speed)` - Set speed (-100 to +100)

### GPIO
- `Pin(pin; mode=:out)` - Create GPIO pin (BCM number or "D0"-"D16")
- `value!(pin, val)` - Set pin value (0 or 1)
- `value(pin)` - Read pin value
- `on!(pin)` / `off!(pin)` - Set pin high/low
