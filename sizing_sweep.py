import numpy as np

#  CUBE PHYSICAL PARAMETERS - temporary parameters for simulation,

# Allow for rough estimates of the torques and inertia's needed for motor and wheel

CUBE = {
    "mass": 0.8,          # Assumed .8kg for mass
    "d": 0.08,            # m    - distance from pivot edge to center of mass
    "cubeInertia": 0.01,  # kg·m² - cube's rotational inertia about the edge
}

# I decided to air the side of heavier mass, since you can change the speed for lighter cube
# but not for heavier cube


#  CONTROL/SIM SETTINGS

Kp = 8.0
Ki = 0.05
Kd = 1.0
g = 9.81
dt = 0.001
t_end = 5.0

# "Did it survive?" if the final angle is below the threashold, then , Yes!

RECOVERED_ANGLE_RAD = np.radians(5)   #within 5° of upright = success


def run_sim(max_torque, wheelInertia, start_angle_deg,
            max_wheelSpeed=500.0):

    peak_wheel_speed = 0.0
    cube_angle = np.radians(start_angle_deg)
    cube_rate = 0.0
    wheel_speed = 0.0
    integral = 0.0
    previous_error = 0.0
    targetAngle = 0.0

    t = 0.0
    while t < t_end:

        #PID
        error = targetAngle - cube_angle
        integral += error * dt
        derivative = (error - previous_error) / dt
        previous_error = error

        reaction_torque = Kp * error + Ki * integral + Kd * derivative
        reaction_torque = np.clip(reaction_torque, -max_torque, max_torque)

        # Cube physics
        gravity_torque = CUBE["mass"] * g * CUBE["d"] * np.sin(cube_angle)
        net_torque = gravity_torque + reaction_torque
        cube_acceleration = net_torque / CUBE["cubeInertia"]
        cube_rate += cube_acceleration * dt
        cube_angle += cube_rate * dt

        #Wheel physics
        wheel_acceleration = -reaction_torque / wheelInertia
        wheel_speed += wheel_acceleration * dt
        wheel_speed = np.clip(wheel_speed, -max_wheelSpeed, max_wheelSpeed)

        peak_wheel_speed = max(peak_wheel_speed, abs(wheel_speed))
        if abs(cube_angle) > np.radians(90):
            return False, peak_wheel_speed

        t += dt

    # Recovered if it ended near upright AND isn't still whipping around fast
    ended_upright = abs(cube_angle) < RECOVERED_ANGLE_RAD
    settled = abs(cube_rate) < 1.0
    return (ended_upright and settled), peak_wheel_speed


def max_recoverable_angle(max_torque, wheelInertia, max_wheelSpeed=500.0):
    best = 0
    peak_at_best = 0.0
    for angle in range(1, 90):
        recovered, peak = run_sim(max_torque, wheelInertia, angle, max_wheelSpeed)
        if recovered:
            best = angle
            peak_at_best = peak   # records peak at highest angle survived
        else:
            break
    return best, peak_at_best


def run_sweep():

    # Candidate motor torques (N·m)
    torque_options = [0.05, 0.10, 0.15, 0.20, 0.30, 0.50]

    # Candidate wheel inertias (kg·m²), bigger wheel = larger number
    inertia_options = [0.0005, 0.0010, 0.0020, 0.0040]

    print("=" * 60)
    print(f"  CUBE: mass={CUBE['mass']} kg, d={CUBE['d']} m, "
          f"I={CUBE['cubeInertia']} kg·m²")
    print(f"  Success = ends within {np.degrees(RECOVERED_ANGLE_RAD):.0f}° "
          f"of upright")
    print("=" * 60)
    print()

    header = "  Torque \\ Inertia |" + "".join(
        f"{i:>10.4f}" for i in inertia_options)
    print(header)
    print("  " + "-" * (len(header) - 2))

    #One row per torque value
    for torque in torque_options:
        cells = []
        for inertia in inertia_options:
            deg, peak = max_recoverable_angle(torque, inertia)
            cells.append(f"{deg:>4}° ({peak:>3.0f})")
        row = f"  {torque:>8.2f} N·m   |" + "".join(cells)
        print(row)

    print()
    print("  Read: pick your target recovery angle, find a cell that meets")
    print("  or beats it, then buy a motor with that torque + design a wheel")
    print("  with that inertia. Re-run with heavier CUBE mass for safety margin.")
    print()


if __name__ == "__main__":
    run_sweep()
