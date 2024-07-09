import cv2
from pyzbar.pyzbar import decode

# Inizializza la webcam
cap = cv2.VideoCapture(0)

while True:
    # Leggi l'immagine dalla webcam
    ret, frame = cap.read()

    # Decodifica i QR codes presenti nell'immagine
    codes = decode(frame)

    for code in codes:
        # Estrai le coordinate del QR code rilevato
        x, y, w, h = code.rect

        # Disegna un rettangolo attorno al QR code
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Stampa i dati del QR code
        print("Dati del QR Code:", code.data.decode('utf-8'))

    # Mostra l'immagine con i QR codes evidenziati
    cv2.imshow('Webcam', frame)

    # Premi 'q' per uscire
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Rilascia la webcam e chiudi tutte le finestre
cap.release()
cv2.destroyAllWindows()
