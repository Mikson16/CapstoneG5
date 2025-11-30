import cv2
import math

# Inicializar cámara (cambia el 0 si es otra camara)
cap = cv2.VideoCapture(0)

# Variables globales
punto1 = None
punto2 = None

def click_event(event, x, y, flags, params):
    global punto1, punto2
    
    # Click Izquierdo: Primer punto
    if event == cv2.EVENT_LBUTTONDOWN:
        punto1 = (x, y)
        print(f"Punto 1 marcado en: {punto1}")
        
    # Click Derecho: Segundo punto y cálculo
    if event == cv2.EVENT_RBUTTONDOWN:
        punto2 = (x, y)
        print(f"Punto 2 marcado en: {punto2}")
        
        if punto1 is not None:
            # Calcular distancia Euclidiana en pixeles
            distancia_pixeles = math.sqrt((punto2[0] - punto1[0])**2 + (punto2[1] - punto1[1])**2)
            
            print("\nResultados:")
            print(f"Distancia en Pixeles: {distancia_pixeles:.2f} px")
            print("-" * 30)
            print("AHORA INGRESA LA DISTANCIA REAL EN TU CALCULADORA:")
            print(f"FACTOR = (Distancia Real en mm) / {distancia_pixeles:.2f}")
            print("-" * 30)

# Configurar ventana
cv2.namedWindow('Calibracion')
cv2.setMouseCallback('Calibracion', click_event)

print("--- INSTRUCCIONES ---")
print("1. Pon una regla visible en la cámara.")
print("2. Click IZQUIERDO en el cm 0.")
print("3. Click DERECHO en el cm 10 (o lo que midas).")
print("4. Mira la terminal para el resultado.")
print("5. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # Dibujar los puntos si existen
    if punto1:
        cv2.circle(frame, punto1, 5, (0, 255, 0), -1) # Verde
    if punto2:
        cv2.circle(frame, punto2, 5, (0, 0, 255), -1) # Rojo
        # Dibujar linea
        cv2.line(frame, punto1, punto2, (255, 0, 0), 2)

    cv2.imshow('Calibracion', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
