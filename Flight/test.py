import dronekit, PID, time
from dronekit import VehicleMode

vehicle = dronekit.connect("tcp:127.0.0.1:5762", wait_ready=True)


print("\n Connected")

def test_PWN(setpoint):
    print("Waiting for pre-arm checks")

    while not vehicle.is_armable:
        print("Waititng...\n")
        time.sleep(1.0)

    print("Arming motors\n")
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True

    time.sleep(5.0)

    pid = PID.PID(0.5, 0.1, 0.1)
    pid.SetPoint = setpoint

    PWM = getPWM(setpoint)
    print(PWM)

    vehicle.channels.overrides['3'] = PWM
    while True:
        try:
            time.sleep(1.0)
            currentAlt = vehicle.location.global_relative_frame.alt
            pid.update(currentAlt)
            updateThrot = getPWM(pid.output)
            vehicle.channels.overrides['3'] = updateThrot
            print("Update throt: %s" % updateThrot)
            print("Alt: %s" % currentAlt)
            
            
        except KeyboardInterrupt:
            break

    vehicle.mode = VehicleMode("LAND")
    
    time.sleep(10.0)

    vehicle.channels.overrides['3'] = 986.0
    vehicle.close()

    return

def getPWM(setpoint):
    return (0.6)*((512.0*setpoint)+1473.0)


test_PWN(3.0)