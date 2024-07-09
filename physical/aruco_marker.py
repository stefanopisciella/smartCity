import cv2
import cv2.aruco as aruco

# Inizializza la webcam
cap = cv2.VideoCapture(0)

# Definisci il dizionario ArUco che stai utilizzando
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Crea i parametri di rilevamento ArUco
parameters = aruco.DetectorParameters()

while (True):
    # Leggi un frame dalla webcam
    ret, frame = cap.read()

    # Rileva i marker ArUco nell'immagine
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # Se sono stati rilevati dei marker, disegna i contorni attorno a loro
    if len(corners) > 0:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

    # Mostra l'immagine
    cv2.imshow('frame', frame)

    # Se viene premuto il tasto 'q', interrompi il ciclo
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Rilascia la webcam e distruggi tutte le finestre create
cap.release()
cv2.destroyAllWindows()
