import cv2
import pickle

width, height = 107, 48

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
    if events == cv2.EVENT_LBUTTONDOWN:
        # user has left-clicked ==> add the clicked position to the list

        posList.append((x, y))
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
    img = cv2.imread('carParkImg.png')
    for pos in posList:
        cv2.rectangle(img, pos, (pos[0] + width, pos[1] + height), (255, 0, 255), 2)

    cv2.imshow("Image", img)
    cv2.setMouseCallback("Image", on_click)
    cv2.waitKey(1)