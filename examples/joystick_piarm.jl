# joystick_piarm.jl - Control PiArm with dual joysticks
# Translated from https://github.com/sunfounder/piarm/blob/main/examples/joystick_module1.py

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
# Bucket on P3
arm = PiArm(dev, [0, 1, 2])
init_bucket!(arm, "P3")
set_offset!(arm, [0, 0, 0])

# Track bucket angle state
bucket_angle = Ref(0.0)

function angles_control!(arm, left_js, right_js, bucket_angle)
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
        bucket_angle[] += 2
        changed = true
    end

    # Right joystick controls beta and bucket (alternate direction)
    status = read_status(right_js)
    if status === :up
        beta += 1
        changed = true
    elseif status === :down
        beta -= 1
        changed = true
    elseif status === :pressed
        bucket_angle[] -= 2
        changed = true
    end

    if changed
        set_angle!(arm, [alpha, beta, gamma])
        if arm.bucket !== nothing
            set_bucket!(arm, bucket_angle[])
        end
        println("servo angles: $(arm.servo_positions), bucket angle: $(bucket_angle[])")
    end
end

# Main loop
println("Joystick PiArm control started. Press Ctrl+C to exit.")
try
    while true
        angles_control!(arm, left_joystick, right_joystick, bucket_angle)
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
