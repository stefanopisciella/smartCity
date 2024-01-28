import os.path

car_park_pos_file_path = './CarParkPos'


if os.path.isfile(car_park_pos_file_path):
    if input("Do you want to pick parking stalls (y/n): ") == "y":
        # user has to pick parking stall from scratch

        os.remove(car_park_pos_file_path)

        with open("parking_space_picker.py") as script:
            exec(script.read())
else:
    with open("parking_space_picker.py") as script:
        exec(script.read())
