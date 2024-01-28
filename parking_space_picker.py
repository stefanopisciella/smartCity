import cv2
import pickle

width, height = 36, 67  # in pixels
parking_row_corner_num = 0

# START importing the positions from CarParkPos
try:
    # CarParkPos already exists
    with open('CarParkPos', 'rb') as f:
        posList = pickle.load(f)
except:
    # CarParkPos doesn't exist yet
    posList = []  # creating new empty list
# END importing the positions from CarParkPos


def on_click(events, x, y, flags, params):
    global parking_row_corner_num

    if events == cv2.EVENT_LBUTTONDOWN:
        # user has left-clicked ==> add the clicked position to the list

        posList.append((x, y, False))

        # global parking_row_corner_num
        parking_row_corner_num += 1

        if parking_row_corner_num == 2:
            # it has a sufficient number of corners for splitting the parking row in more parking stalls
            current_parking_row_left_corner = (posList[-2][0], posList[-2][1])  # posList[-1] for retrieving the second-last inserted position

            current_stall_left_corner = (current_parking_row_left_corner[0], current_parking_row_left_corner[1])

            while current_stall_left_corner[0] < x:
                posList.append((current_stall_left_corner[0], current_stall_left_corner[1], True))
                current_stall_left_corner = (current_stall_left_corner[0] + width, current_stall_left_corner[1])

            parking_row_corner_num = 0  # so that can be added others parking rows
    if events == cv2.EVENT_RBUTTONDOWN:
        # user has right-clicked ==> remove the clicked position from the list

        for i, pos in enumerate(posList):  # looping the saved parking stalls
            x1, y1 = pos
            if x1 < x < x1 + width and y1 < y < y1 + height:
                # the clicked position belongs to the current parking stall area
                posList.pop(i)

    # START saving the positions in the file
    with open('CarParkPos', 'wb') as f:
        pickle.dump(posList, f)
    # END saving the positions in the file


while True:
    img = cv2.imread('camera.jpg')
    for pos in posList:
        if pos[2]:
            # it only considers corners relative to parking stalls and not those relative to parking rows
            cv2.rectangle(img, (pos[0], pos[1]), (pos[0] + width, pos[1] + height), (255, 0, 255), 2)

    cv2.imshow("Image", img)
    cv2.setMouseCallback("Image", on_click)

    if cv2.waitKey(1) == 27:
        # the "ESC" has been pressed => stop the execution of this script
        cv2.destroyWindow('Image')
        exit(0)
