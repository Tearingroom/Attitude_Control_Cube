import numpy as np
import matplotlib.pyplot as plt


def run_sim(Kp, Kd, Ki, start_angle_deg, max_torque=0.05):
    cubeInertia = 0.01        # kg·m²
    wheelInertia = 0.0005     # kg·m²
    mass = 0.5                 # cube mass, kg
    g = 9.81                # gravity, m/s²
    d = 0.08                # distance from pivot edge to center of mass, m

    max_wheelSpeed = 500    # rad/s

    # ---- Sim settings ----
    dt = 0.001
    t_end = 5

    # addition of friction
    friction_coef = 0.0001

    # has to stay upright
    targetAngle = 0.0

    # ---- Initial state: start slightly tipped so it has to recover ----


    cube_angle = start_angle_deg * np.pi / 180
    cube_rate = 0.0
    wheel_speed = 0.0
    integral = 0.0
    previous_error = 0.0

    # ---- Logs ----
    time_log, angle_log, wheel_log, torque_log = [], [], [], []

    t = 0.0
    while t < t_end:


        # PID Controls-
        error = targetAngle - cube_angle
        integral = integral + (error * dt)
        # changes over time with dt
        derivative = (error - previous_error) / dt
        previous_error = error

        # Friction

        viscousFrict = -friction_coef * wheel_speed
        #Torques

        gravity_torque = mass* g* d* np.sin(cube_angle)
        # clipped torque so it doesn't exceed physical constraints
        reaction_torque = Kp * error + Ki * integral + Kd * derivative
        reaction_torque = np.clip(reaction_torque, -max_torque, max_torque)
        netTorque = gravity_torque + reaction_torque


        # newtons second law T = Ia
        cube_acceleration = netTorque / cubeInertia

        # cube rate = change in velocity (through derivative)
        cube_rate += cube_acceleration * dt
        cube_angle += cube_rate * dt

        # reaction wheel speed from T = ia, and derivative of acceleration
        wheel_acceleration = (reaction_torque + viscousFrict) / wheelInertia
        wheel_speed += wheel_acceleration * dt





        # log everything
        time_log.append(t)
        angle_log.append(np.degrees(cube_angle))
        wheel_log.append(wheel_speed)
        torque_log.append(reaction_torque)


        t += dt

    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    axs[0].plot(time_log, angle_log, color='r', label='Cube Angle (deg)')
    axs[0].axhline(0, color='black', linestyle='--')
    axs[0].set_ylabel('Angle (degrees)')
    axs[0].legend()
    axs[0].set_title('Cubli Edge-Balancing Simulation')

    axs[1].plot(time_log, wheel_log, color='b', label='Wheel Speed (rad/s)')
    axs[1].set_ylabel('Speed (rad/s)')
    axs[1].legend()

    axs[2].plot(time_log, torque_log, color='g', label='Motor Torque (Nm)')
    axs[2].set_ylabel('Torque (Nm)')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

    return time_log, angle_log, wheel_log, torque_log

t, angle, wheel, torque = run_sim(Kp=8, Kd=1, Ki=0.05, start_angle_deg=3)