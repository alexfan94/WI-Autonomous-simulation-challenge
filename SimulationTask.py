import math
import matplotlib.pyplot as plt


# Vehicle parameters
mass = 2000
rolling_coefficient = 0.01
drag_coefficient = 0.30
frontal_area = 2.5
air_density = 1.2
gravity = 10
tire_radius = 0.4
drivetrain_efficiency = 0.90

# Simulation settings
time_step = 0.01
simulation_time = 120


# Find motor torque from motor RPM
def get_motor_torque(motor_rpm):

    if motor_rpm <= 4000:
        return 250

    elif motor_rpm <= 10000:
        return 250 * 4000 / motor_rpm

    else:
        return 0


# Run one simulation
def simulate(gear_ratio):

    time = 0
    speed = 0

    times = [0]
    speeds = [0]

    time_to_20 = None

    # Rolling resistance is constant in this model
    rolling_force = rolling_coefficient * mass * gravity

    steps = int(simulation_time / time_step)

    for i in range(steps):

        # 1. Vehicle speed -> wheel RPM -> motor RPM
        wheel_rpm = speed / (2 * math.pi * tire_radius) * 60
        motor_rpm = wheel_rpm * gear_ratio

        # 2. Find motor torque
        motor_torque = get_motor_torque(motor_rpm)

        # 3. Motor torque -> wheel torque -> driving force
        wheel_torque = motor_torque * gear_ratio * drivetrain_efficiency
        drive_force = wheel_torque / tire_radius

        # 4. Resistance forces
        drag_force = (
            0.5
            * air_density
            * drag_coefficient
            * frontal_area
            * speed**2
        )

        # 5. Net force -> acceleration
        net_force = drive_force - rolling_force - drag_force
        acceleration = net_force / mass

        # 6. Update speed and time
        speed = speed + acceleration * time_step

        # Do not allow the vehicle to move backward
        if speed < 0:
            speed = 0

        time = time + time_step

        times.append(time)
        speeds.append(speed)

        # Record the first time the car reaches 20 m/s
        if time_to_20 is None and speed >= 20:
            time_to_20 = time

    top_speed = max(speeds)

    return times, speeds, time_to_20, top_speed


# Run the required gear ratios
gear_ratios = [7, 9, 10.5]

plot_times = []
plot_speeds = []

for gear_ratio in gear_ratios:

    times, speeds, time_to_20, top_speed = simulate(gear_ratio)

    print("Gear ratio:", gear_ratio)
    print("Time to 20 m/s:", round(time_to_20, 2), "seconds")
    print("Top speed:", round(top_speed, 2), "m/s")

    # Checkpoint for gear ratio 9
    if gear_ratio == 9:
        speed_after_1_second = speeds[int(1 / time_step)]
        print(
            "Speed after 1 second:",
            round(speed_after_1_second, 2),
            "m/s"
        )

        plot_times = times
        plot_speeds = speeds

    print()


# Plot speed vs. time for gear ratio 9
plt.plot(plot_times, plot_speeds)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("EV Speed vs. Time - Gear Ratio 9")

plt.grid()

plt.savefig("speed_plot.png")
plt.show()