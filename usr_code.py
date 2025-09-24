import math
import struct
import time

def usr(robot):
    # robot.logger.info("Started Code")
    # log = open("experiment_log", "wb")
    # log.write(b"in user code")
    # log.flush()
    # Define the central point for rotation
    central_point = [0.0, 0.0]  # Fixed central point at the origin

    CIRCULAR_FORCE = 1.5  # Force driving the robots to circle the central point
    COHESION_FORCE = 0.5  # Cohesion toward the swarm's center of mass
    SEPARATION_FORCE = .50  # Separation to avoid collisions with neighbors

    # Start robot movement on startup
    robot.set_vel(30, 30)

    Neighbors = {}

    while True:
        # log.write(b"\nlooping\n")
        # log.flush()
        # Determine robot's current pose
        pose = robot.get_pose()
        if not pose:
            continue

        # Send robot's current position to neighbors
        robot.send_msg(struct.pack('fffi', pose[0], pose[1], pose[2], robot.id))

        # Receive messages from neighbors
        msgs = robot.recv_msg()
        for msg in msgs:
            data = struct.unpack('fffi', msg[:16])
            Neighbors[data[3]] = data[:3]  # Update neighbor data

        # Compute the circular force
        to_center = [central_point[0] - pose[0], central_point[1] - pose[1]]
        mag_to_center = math.sqrt(to_center[0]**2 + to_center[1]**2)
        if mag_to_center > 0:
            circular_force = [-to_center[1] / mag_to_center, to_center[0] / mag_to_center]
        else:
            circular_force = [0, 0]

        # Compute the cohesion force
        com = [0, 0]  # Center of mass
        for nID in Neighbors:
            neighbor_pose = Neighbors[nID]
            com[0] += neighbor_pose[0]
            com[1] += neighbor_pose[1]
        if Neighbors:
            com[0] /= len(Neighbors)
            com[1] /= len(Neighbors)
            cohesion = [com[0] - pose[0], com[1] - pose[1]]
            mag_cohesion = math.sqrt(cohesion[0]**2 + cohesion[1]**2)
            if mag_cohesion > 0:
                cohesion = [cohesion[0] / mag_cohesion, cohesion[1] / mag_cohesion]
        else:
            cohesion = [0, 0]

        # Compute the separation force
        repulsion = [0, 0]
        for nID in Neighbors:
            neighbor_pose = Neighbors[nID]
            dist = math.sqrt((pose[0] - neighbor_pose[0])**2 + (pose[1] - neighbor_pose[1])**2)
            if dist > 0 and dist < 0.25:  # Avoid neighbors within 25 cm
                repulsion[0] += (pose[0] - neighbor_pose[0]) / dist**2
                repulsion[1] += (pose[1] - neighbor_pose[1]) / dist**2

        # Combine forces
        x = (CIRCULAR_FORCE * circular_force[0] + 
             COHESION_FORCE * cohesion[0] + 
             SEPARATION_FORCE * repulsion[0])

        y = (CIRCULAR_FORCE * circular_force[1] + 
             COHESION_FORCE * cohesion[1] + 
             SEPARATION_FORCE * repulsion[1])

        # Calculate the desired heading
        heading = math.atan2(y, x)

        # Update robot's motion
        try:
            current_heading = pose[2]
            distR = math.fmod(abs(current_heading - heading + 2 * math.pi), 2 * math.pi)
            distL = math.fmod(abs(2 * math.pi - (current_heading - heading)), 2 * math.pi)

            fastWheel = 30
            slowWheel = max(15, -30 / math.pi * min(distL, distR) + 30)

            if distR <= distL:
                velR = slowWheel
                velL = fastWheel
            else:
                velR = fastWheel
                velL = slowWheel

            robot.set_vel(velL, velR)

        except Exception as e:
            # robot.logger.info(f"Error in force computation: {e}")
            pass


        robot.delay(500)
    # log.close()
