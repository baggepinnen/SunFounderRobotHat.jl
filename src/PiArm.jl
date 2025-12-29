# PiArm.jl - 3-DOF Robot Arm Control
# Translated from https://github.com/sunfounder/piarm

# =============================================================================
# Constants
# =============================================================================

const LINK_A = 80       # First link length
const LINK_B = 80       # Second link length
const MAX_REACH = 160   # Maximum arm reach
const MIN_REACH = 30    # Minimum arm reach
const MAX_DPS = 428     # Max degrees per second (servo limit)
const STEP_TIME_MS = 10 # Milliseconds per interpolation step

# =============================================================================
# PiArm Struct
# =============================================================================

mutable struct PiArm
    dev::I2CDevice
    servos::Vector{Servo}
    servo_positions::Vector{Float64}  # Current [α, β, γ]
    offset::Vector{Float64}           # Calibration offsets
    direction::Vector{Int}            # Direction multipliers (1 or -1)
    speed::Int                        # Movement speed 0-100
    current_coord::Vector{Float64}    # Current [x, y, z]
    # End-effectors (only one active at a time)
    bucket::Union{Servo, Nothing}
    hanging_clip::Union{Servo, Nothing}
    electromagnet::Union{PWMChannel, Nothing}
end

"""
    PiArm(dev::I2CDevice, channels::Vector{Int}; init_angles=[0,0,0])

Create a PiArm robot arm controller.

# Arguments
- `dev`: I2CDevice from `opendevice()`
- `channels`: Vector of 3 servo channel numbers (e.g., [1, 2, 3])
- `init_angles`: Initial servo angles (default [0, 0, 0])

# Example
```julia
dev = opendevice()
arm = PiArm(dev, [1, 2, 3])
```
"""
function PiArm(dev::I2CDevice, channels::Vector{Int}; init_angles::Vector{<:Real}=[0.0, 0.0, 0.0])
    @assert length(channels) == 3 "PiArm requires exactly 3 servo channels"
    @assert length(init_angles) == 3 "init_angles must have 3 elements"

    servos = [Servo(dev, ch) for ch in channels]
    servo_positions = Float64.(init_angles)
    offset = [0.0, 0.0, 0.0]
    direction = [1, 1, 1]
    current_coord = [0.0, 80.0, 80.0]  # Default position

    arm = PiArm(
        dev,
        servos,
        servo_positions,
        offset,
        direction,
        50,  # Default speed
        current_coord,
        nothing,  # bucket
        nothing,  # hanging_clip
        nothing   # electromagnet
    )

    # Move servos to initial position
    for (i, servo) in enumerate(servos)
        angle!(servo, offset[i] + servo_positions[i])
        sleep(0.15)
    end

    return arm
end

# =============================================================================
# Kinematics
# =============================================================================

"""
    coord2polar(x, y, z) -> (α, β, γ)

Inverse kinematics: Convert Cartesian coordinates to servo angles.

# Arguments
- `x`: Left-right position
- `y`: Forward-backward position (must be >= 0)
- `z`: Up-down position

# Returns
Tuple of (alpha, beta, gamma) angles in degrees.
"""
function coord2polar(x::Real, y::Real, z::Real)
    y = max(0, y)
    u = sqrt(x^2 + y^2 + z^2)

    if u == 0
        u = 0.1
    end

    # Clamp to valid reach range
    if u > MAX_REACH
        scale = MAX_REACH / u
        x *= scale
        y *= scale
        z *= scale
        u = MAX_REACH
    elseif u < MIN_REACH
        scale = MIN_REACH / u
        x *= scale
        y *= scale
        z *= scale
        u = MIN_REACH
    end

    # Law of cosines calculations
    angle1 = acos(clamp((LINK_A^2 + u^2 - LINK_B^2) / (2 * LINK_A * u), -1, 1))
    angle2 = asin(clamp(z / u, -1, 1))
    angle3 = acos(clamp((LINK_A^2 + LINK_B^2 - u^2) / (2 * LINK_A * LINK_B), -1, 1))
    angle4 = atan(x, y)

    # Convert to servo angles (degrees)
    alpha = 90 - (angle1 + angle2) * 180 / π
    beta = -180 + (angle1 + angle2 + angle3) * 180 / π
    gamma = -angle4 * 180 / π

    return (alpha, beta, gamma)
end

"""
    polar2coord(α, β, γ) -> (x, y, z)

Forward kinematics: Convert servo angles to Cartesian coordinates.

# Arguments
- `α`: Alpha angle (degrees)
- `β`: Beta angle (degrees)
- `γ`: Gamma angle (degrees)

# Returns
Tuple of (x, y, z) coordinates.
"""
function polar2coord(alpha::Real, beta::Real, gamma::Real)
    a1 = 90 + alpha + beta
    a2 = (180 - a1) / 2
    a3 = 90 - alpha - a2

    # Law of cosines: L = sqrt(A² + B² - 2AB·cos(a1))
    L = sqrt(LINK_A^2 + LINK_B^2 - 2 * LINK_A * LINK_B * cos(a1 * π / 180))
    L2 = L * cos(a3 * π / 180)

    x = L2 * sin(-gamma * π / 180)
    y = L2 * cos(abs(gamma) * π / 180)
    z = L * sin(a3 * π / 180)

    return (round(Int, x), round(Int, y), round(Int, z))
end

"""
    limit_angle(α, β, γ) -> (limited::Bool, (α, β, γ))

Enforce joint angle limits.

# Limits
- Alpha: [-30°, 60°]
- Beta: relative to alpha AND absolute [-90°, 40°]
- Gamma: [-90°, 90°]

# Returns
Tuple of (was_limited, (clamped_alpha, clamped_beta, clamped_gamma))
"""
function limit_angle(alpha::Real, beta::Real, gamma::Real)
    limited = false

    # Alpha limits
    new_alpha = clamp(alpha, -30, 60)
    if new_alpha != alpha
        alpha = new_alpha
        limited = true
    else
        # Beta relative limits (depends on alpha)
        beta_min = max(-90, -alpha - 60)
        beta_max = min(40, -alpha + 30)
        new_beta = clamp(beta, beta_min, beta_max)
        if new_beta != beta
            beta = new_beta
            limited = true
        end
    end

    # Gamma limits
    new_gamma = clamp(gamma, -90, 90)
    if new_gamma != gamma
        gamma = new_gamma
        limited = true
    end

    return (limited, (alpha, beta, gamma))
end

# =============================================================================
# Motion Control
# =============================================================================

"""
    servo_move!(arm::PiArm, targets, speed=arm.speed)

Smoothly move servos to target angles with interpolation.

# Arguments
- `arm`: PiArm instance
- `targets`: Vector of 3 target angles [α, β, γ]
- `speed`: Movement speed 0-100 (higher = faster)
"""
function servo_move!(arm::PiArm, targets::Vector{<:Real}, speed::Int=arm.speed)
    speed = clamp(speed, 0, 100)
    step_time = STEP_TIME_MS / 1000  # Convert to seconds

    # Calculate deltas
    delta = [targets[i] - arm.servo_positions[i] for i in 1:3]
    max_delta = maximum(abs.(delta))

    if max_delta == 0
        sleep(step_time)
        return
    end

    # Calculate total movement time
    total_time = -9.9 * speed + 1000  # ms

    # Check against max DPS limit
    current_max_dps = max_delta / total_time * 1000
    if current_max_dps > MAX_DPS
        total_time = max_delta / MAX_DPS * 1000
    end

    # Calculate number of steps
    max_step = max(1, round(Int, total_time / STEP_TIME_MS))

    # Calculate step sizes
    steps = delta ./ max_step

    # Execute interpolated movement
    for _ in 1:max_step
        start_time = time()

        # Update positions
        arm.servo_positions .+= steps

        # Write to servos
        for (i, servo) in enumerate(arm.servos)
            effective_angle = arm.direction[i] * (arm.servo_positions[i] + arm.offset[i])
            angle!(servo, effective_angle)
        end

        # Timing
        elapsed = time() - start_time
        delay = step_time - elapsed
        if delay > 0
            sleep(delay)
        end
    end
end

"""
    set_angle!(arm::PiArm, angles; warn=true)

Set servo angles with limit checking.

# Arguments
- `arm`: PiArm instance
- `angles`: Vector of 3 angles [α, β, γ]
- `warn`: Print warning if angles were limited (default true)
"""
function set_angle!(arm::PiArm, angles::Vector{<:Real}; warn::Bool=true)
    @assert length(angles) == 3 "angles must have 3 elements"

    limited, (alpha, beta, gamma) = limit_angle(angles[1], angles[2], angles[3])

    if limited && warn
        @warn "Coordinates out of controllable range, angles were clamped"
        # Update current_coord from clamped angles
        arm.current_coord = collect(polar2coord(alpha, beta, gamma))
    end

    servo_move!(arm, [alpha, beta, gamma])
end

"""
    move_to!(arm::PiArm, coord)

Move arm end-effector to Cartesian coordinate.

# Arguments
- `arm`: PiArm instance
- `coord`: Vector [x, y, z] or tuple (x, y, z)

See also [`set_angle!`](@ref) for setting angles directly.
"""
function move_to!(arm::PiArm, coord)
    x, y, z = coord
    alpha, beta, gamma = coord2polar(x, y, z)
    arm.current_coord = [Float64(x), Float64(y), Float64(z)]
    set_angle!(arm, [alpha, beta, gamma]; warn=false)
end

"""
    set_speed!(arm::PiArm, speed)

Set default movement speed (0-100).
"""
function set_speed!(arm::PiArm, speed::Int)
    arm.speed = clamp(speed, 0, 100)
end

"""
    set_offset!(arm::PiArm, offsets)

Set servo calibration offsets.

# Arguments
- `arm`: PiArm instance
- `offsets`: Vector of 3 offset values (clamped to ±20)
"""
function set_offset!(arm::PiArm, offsets::Vector{<:Real})
    @assert length(offsets) == 3 "offsets must have 3 elements"
    arm.offset = [clamp(o, -20, 20) for o in offsets]
end

"""
    calibration!(arm::PiArm)

Move all servos to home position (0, 0, 0).
"""
function calibration!(arm::PiArm)
    arm.servo_positions = [0.0, 0.0, 0.0]
    for (i, servo) in enumerate(arm.servos)
        effective_angle = arm.direction[i] * (arm.servo_positions[i] + arm.offset[i])
        angle!(servo, effective_angle)
    end
end

# =============================================================================
# End-Effector Control
# =============================================================================

"""
    init_bucket!(arm::PiArm, channel)

Initialize bucket/shovel end-effector on specified PWM channel.
"""
function init_bucket!(arm::PiArm, channel)
    arm.bucket = Servo(arm.dev, channel)
    arm.hanging_clip = nothing
    arm.electromagnet = nothing
end

"""
    init_hanging_clip!(arm::PiArm, channel)

Initialize hanging clip end-effector on specified PWM channel.
"""
function init_hanging_clip!(arm::PiArm, channel)
    arm.hanging_clip = Servo(arm.dev, channel)
    arm.bucket = nothing
    arm.electromagnet = nothing
end

"""
    init_electromagnet!(arm::PiArm, channel)

Initialize electromagnet end-effector on specified PWM channel.
"""
function init_electromagnet!(arm::PiArm, channel)
    arm.electromagnet = PWMChannel(arm.dev, channel)
    pulse_width_percent!(arm.electromagnet, 0)
    arm.bucket = nothing
    arm.hanging_clip = nothing
end

"""
    set_bucket!(arm::PiArm, angle)

Set bucket angle (-50 to 90 degrees).
"""
function set_bucket!(arm::PiArm, ang::Real)
    if arm.bucket === nothing
        error("Bucket not initialized. Call init_bucket! first.")
    end
    ang = clamp(ang, -50, 90)
    angle!(arm.bucket, ang)
end

"""
    set_hanging_clip!(arm::PiArm, angle)

Set hanging clip angle.
"""
function set_hanging_clip!(arm::PiArm, ang::Real)
    if arm.hanging_clip === nothing
        error("Hanging clip not initialized. Call init_hanging_clip! first.")
    end
    angle!(arm.hanging_clip, ang)
end

"""
    set_electromagnet!(arm::PiArm, on::Bool)

Turn electromagnet on or off.
"""
function set_electromagnet!(arm::PiArm, on::Bool)
    if arm.electromagnet === nothing
        error("Electromagnet not initialized. Call init_electromagnet! first.")
    end
    pulse_width_percent!(arm.electromagnet, on ? 100 : 0)
end
