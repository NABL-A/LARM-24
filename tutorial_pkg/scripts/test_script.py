import cv2
import numpy as np

# Charger l'image
image = cv2.imread('Testgreen.png')

# Convertir l'image en HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Définir la plage de couleur verte en HSV
lower_green = np.array([35, 50, 50])  # Valeur min pour le vert
upper_green = np.array([85, 255, 255])  # Valeur max pour le vert

# Créer un masque pour les pixels verts
mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

# Appliquer des transformations morphologiques pour améliorer le masque (optionnel)
kernel = np.ones((5, 5), np.uint8)
mask_green = cv2.dilate(mask_green, kernel, iterations=2)  # Dilatation pour relier les objets proches
mask_green = cv2.erode(mask_green, kernel, iterations=1)  # Érosion pour supprimer les petites taches

# Trouver les contours dans le masque
contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Compter le nombre d'objets détectés
num_objects = len(contours)

print(f"Nombre d'objets détectés : {num_objects}")

# Optionnel: dessiner les contours sur l'image d'origine pour visualiser les objets détectés
image_with_contours = image.copy()
cv2.drawContours(image_with_contours, contours, -1, (0, 255, 0), 3)  # Dessiner les contours en vert

# Afficher l'image avec les contours
cv2.imshow("Contours détectés", image_with_contours)
cv2.waitKey(0)
cv2.destroyAllWindows()
