# servo_zeroing.jl - Set a servo to zero position for calibration
# Translated from https://github.com/sunfounder/piarm/blob/main/examples/servo_zeroing.py
#
# Usage: Change the channel number to match your servo, then run.
# This is useful for calibrating servos before assembly.

using SunFounderRobotHat

# Change this to the channel your servo is connected to (P0-P19)
channel = 11

dev = opendevice()
servo = Servo(dev, channel)

println("Setting servo on P$channel to 0 degrees...")
angle!(servo, 0)
println("Done. Servo is now at zero position.")

close(dev)
