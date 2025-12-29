# joystick_piarm_clip.jl - Control PiArm with dual joysticks (hanging clip version)
# Translated from https://github.com/sunfounder/piarm/blob/main/examples/joystick_module2.py

using SunFounderRobotHat

# Initialize device
try
    reset_mcu()
    sleep(0.01)
catch e
    @warn "Could not reset MCU (may require sudo): $e"
end

dev = opendevice()

# Create joysticks
left_joystick = Joystick(dev, "A0", "A1", "D0")
right_joystick = Joystick(dev, "A2", "A3", "D1")

# Create arm with servos on channels P0, P1, P2 (hardware ports)
# Hanging clip on P3
arm = PiArm(dev, [0, 1, 2])
init_hanging_clip!(arm, "P3")
set_offset!(arm, [-80, 0, 0])

# Track clip angle state
clip_angle = Ref(0.0)

function angles_control!(arm, left_js, right_js, clip_angle)
    set_speed!(arm, 100)
    arm_changed = false
    clip_changed = false

    alpha, beta, gamma = arm.servo_positions

    # Left joystick controls alpha (up/down) and gamma (left/right)
    status = read_status(left_js)
    if status === :up
        alpha += 1
        arm_changed = true
    elseif status === :down
        alpha -= 1
        arm_changed = true
    elseif status === :left
        gamma += 1
        arm_changed = true
    elseif status === :right
        gamma -= 1
        arm_changed = true
    end

    # Right joystick controls beta (up/down) and clip (left/right)
    status = read_status(right_js)
    if status === :up
        beta += 1
        arm_changed = true
    elseif status === :down
        beta -= 1
        arm_changed = true
    elseif status === :left
        clip_angle[] += 2
        clip_changed = true
    elseif status === :right
        clip_angle[] -= 2
        clip_changed = true
    end

    if arm_changed
        set_angle!(arm, [alpha, beta, gamma])
    end

    if clip_changed && arm.hanging_clip !== nothing
        set_hanging_clip!(arm, clip_angle[])
    end

    if arm_changed || clip_changed
        println("servo angles: $(arm.servo_positions), clip angle: $(clip_angle[])")
    end
end

# Main loop
println("Joystick PiArm control (hanging clip) started. Press Ctrl+C to exit.")
try
    while true
        angles_control!(arm, left_joystick, right_joystick, clip_angle)
        sleep(0.01)
    end
catch e
    if e isa InterruptException
        println("\nExiting...")
    else
        rethrow(e)
    end
finally
    close(dev)
end
