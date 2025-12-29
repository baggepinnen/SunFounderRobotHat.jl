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

# Create arm with servos on channels 1, 2, 3
arm = PiArm(dev, [1, 2, 3])
init_hanging_clip!(arm, "P3")
set_offset!(arm, [0, 0, 0])

# Track clip angle state
clip_angle = Ref(0.0)

function angles_control!(arm, left_js, right_js, clip_angle)
    set_speed!(arm, 100)
    changed = false

    alpha, beta, gamma = arm.servo_positions

    # Left joystick controls alpha and gamma
    status = read_status(left_js)
    if status === :up
        alpha += 1
        changed = true
    elseif status === :down
        alpha -= 1
        changed = true
    elseif status === :left
        gamma += 1
        changed = true
    elseif status === :right
        gamma -= 1
        changed = true
    elseif status === :pressed
        clip_angle[] += 2
        changed = true
    end

    # Right joystick controls beta and clip (alternate direction)
    status = read_status(right_js)
    if status === :up
        beta += 1
        changed = true
    elseif status === :down
        beta -= 1
        changed = true
    elseif status === :pressed
        clip_angle[] -= 2
        changed = true
    end

    if changed
        set_angle!(arm, [alpha, beta, gamma])
        if arm.hanging_clip !== nothing
            set_hanging_clip!(arm, clip_angle[])
        end
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
