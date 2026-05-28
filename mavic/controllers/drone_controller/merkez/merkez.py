from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

receiver = robot.getDevice("receiver")
receiver.enable(timestep)

print("\n" + "="*40)
print("[MERKEZ] >>> GCS AKTIF")
print("="*40)

while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        data = receiver.getString()
        print(f"[MERKEZ] >>> {data}")
        receiver.nextPacket()