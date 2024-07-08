from controller.supervisor import Supervisor

# create the Supervisor instance
supervisor = Supervisor()

# get the timestep of the current world.
timestep = int(supervisor.getBasicTimeStep())

# get the car node using its DEF name
car_node = supervisor.getFromDef("car")

# main loop:
while supervisor.step(timestep) != -1:
    # get the field that contains the car position
    translation_field = car_node.getField("translation")

    # get the position of the car as an array [x, y, z]
    position = translation_field.getSFVec3f()
    print("Car position:", position)
