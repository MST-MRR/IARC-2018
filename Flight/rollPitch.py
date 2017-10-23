def getPitchPWM(angle):
    return (((512*angle)/5) + 1494)
while (True):

    angle = input("Value? ")
    print(getPitchPWM(angle))

