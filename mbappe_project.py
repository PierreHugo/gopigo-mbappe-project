###############################################################################
# Robot à utiliser : Dex104
###############################################################################

# ----------------- IMPORT -----------------------#
from easygopigo3 import EasyGoPiGo3
import math
import time
from copy import copy


# ---------- DECLARATIONS VARIABLES ---------- #
largeur_table = 1960    # en mm
portee_capteur = 2000   # en mm 
largeur_robot = 240     # en mm
poussee_de_balle = 150  # en mm
frequence_echantillonnage = 50
angle_servo = 83    # en degrés

gpg = EasyGoPiGo3()     # module robot
capteur = gpg.init_distance_sensor()    # démarrage du capteur de distance
servo = gpg.init_servo()    # caméra 


# ---------- PHASE 1 : détermination de la distance au bord droit ---------- #
print("==============================================================")
print("Initialisation...")
time.sleep(0.5)
print("==============================================================")
print("PHASE 1")
print("Analyse du terrain...")
gpg.turn_degrees(80)    # rotation de 90 degrés pour mesurer la distance OA (distance avec le bord latéral droit)
servo.rotate_servo(angle_servo)     # caméra en position initiale

liste_val_bd = []   # liste des valeurs acquises pour la distance au bord droit

for i in range (20):    # on fait 20 acquisitions de la distance
    dist = capteur.read_mm()
    print("Distance du bord droit : " + str(dist) + "mm")
    if dist != 3000 :   # on exclut les valeurs 3000 mm qui correspondent à des erreurs
        liste_val_bd.append(dist)
    time.sleep(0.1)

gpg.turn_degrees(-80)   # rotation de -90 degrés pour se remettre en position initiale


# ---------- PHASE 2 : calcul de toutes les données pour faire le scan ---------- #
time.sleep(0.5)
print("==============================================================")
print("PHASE 2")
print("Calculs des données...")
time.sleep(0.5)
distance_bord_droit = sum(liste_val_bd)/len(liste_val_bd)   # on fait la moyenne des valeurs acquises pour déterminer la distance du bord droit
distance_bord_gauche = largeur_table - distance_bord_droit - largeur_robot   # on détermine la distance du bord gauche par une soustraction
print("Valeurs finales : ")
print("Distance à droite : ",distance_bord_droit, "mm")
print("Distance à gauche : ",distance_bord_gauche, "mm")

alpha_1 = math.degrees(math.acos(distance_bord_droit/portee_capteur))   # on détermine alpha 1 avec pythagore + conversion en degrés
alpha_2 = math.degrees(math.acos(distance_bord_gauche/portee_capteur))  # on détermine alpha 2 avec pythagore + conversion en degrés
beta_1 = 90 - alpha_1   # on détermine la partie droite à balayer
beta_2 = 90 - alpha_2   # on détermine la partie gauche à balayer
theta = beta_2 + beta_1 # on détermine l'angle total à scanner
print("Angle de départ du scan : ",beta_1, "degrés")
print("Angle de fin du scan : ",beta_2, "degrés")
print("Surface totale du scan : ",theta, "degres")


# ---------- PHASE 3 : scan ---------- #
time.sleep(0.5)
print("==============================================================")
print("PHASE 3")
print("Démarrage du scan...")
time.sleep(0.5)
echantillon = theta / frequence_echantillonnage     # va nous fournir le pas, on tourne de [echantillon] degré puis on scanne
servo.rotate_servo(angle_servo-beta_1)      # on met la caméra à l'angle de départ
distances_dans_theta = []       # liste avec les distances pour chaque scan dans theta

for j in range (frequence_echantillonnage):
  servo.rotate_servo(angle_servo-beta_1-(j*-theta/frequence_echantillonnage)) 
  distances_dans_theta.append(str(capteur.read_mm()))
  time.sleep(0.1)

print("Scan terminé !")


# ---------- PHASE 4 : analyse des données ---------- #
time.sleep(0.5)
print("==============================================================")
print("PHASE 4")
print("Analyse des données...")
time.sleep(0.5)

# création de séries de listes temporaires pour déterminer le but
# celui-ci est caractérisé par le milieu des deux obstacles
# pour cela on détermine l'emplacement de ces deux obstacles et on note l'espace entre ses deux obstacles en termes d'échantillons
# la taille de cet espace correspond à la taille de la dernière liste temporaire qui ne contient plus que les termes entre les deux obstacles

liste_temp_1 = copy(distances_dans_theta)   # 1ère liste temporaire
del liste_temp_1[0]     # on élimine les 1eres et dernières valeurs, sources d'erreurs
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[0]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]
del liste_temp_1[-1]

liste_temp_2 = copy(liste_temp_1)   # 2e liste temporaire

for val in range (len(liste_temp_1)):
    if liste_temp_1[val] == '3000' and liste_temp_1[val+1] == '3000' and liste_temp_1[val+2] == '3000':
        print("1er poteau détecté !")
    # 3 termes consécutifs qui valent 3000 = on est dans le but  (on estime qu'il aura toujours une taille minimale de 3)    
        del liste_temp_2[0:val]     # élimination de toutes les valeurs avant le 1er obstacle

        for val_2 in range (len(liste_temp_2)):
            if liste_temp_2[val_2] != '3000':   # si le prochain terme est différent de 3000, on a atteint le deuxième obstacle
                print("2ème poteau détecté !")
                del liste_temp_2[val_2:]    # élimination de toutes les valeurs après le 2e obstacle
                print("Distances obtenues entre les poteaux : ", list_temp_2)
                print("Taille du but : ", len(list_temp_2))
                milieu_temp = int(len(liste_temp_2)/2)

                for val_3 in range (len(liste_temp_1)):
                    if liste_temp_1[val_3] == '3000' and liste_temp_1[val_3+1] == '3000' and liste_temp_1[val_3+2] == '3000':
                        milieu = val_3+milieu_temp   # numéro de l'échantillon du milieu de but
                        angle_milieu = beta_1-((milieu)*-theta/frequence_echantillonnage)   # angle précis du milieu du but
                        print("Angle du milieu du but : ", angle_milieu, "degrés")


# ---------- PHASE 5 : préparation à l'occasion de but ---------- #
                        time.sleep(0.5)
                        print("==============================================================")
                        print("PHASE 5")
                        print("Préparation à l'occasion de but...")
                        servo.rotate_servo(angle_servo)
                        time.sleep(0.5)
                        servo.rotate_servo(angle_servo-beta_1-((milieu2+8)*-theta/frequence_echantillonnage))   # orientation de la cam vers le but
                        print("Caméra : prête")
                        time.sleep(0.5)
                        gpg.turn_degrees(80-(angle_servo-beta_1-((milieu2+8)*-theta/frequence_echantillonnage)))    # orientation du robot vers le but
                        print("Robot : prêt")


 # ---------- PHASE 6 : tir au but ---------- # 
                        time.sleep(0.5)
                        print("==============================================================")
                        print("PHASE 6")
                        print("Frappe vers le but !")                      
                        gpg.drive_cm(poussee_de_balle)   # le robot emmène la balle vers le but
                        time.sleep(0.5)
                        print("==============================================================")
                        print("Fin du programme")
                        print("==============================================================")
                        break
