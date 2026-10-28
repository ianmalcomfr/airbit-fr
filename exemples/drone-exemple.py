# DRONE AIR:BIT — exemple complet avec l'extension française « airbit »
# (télécommande par radio, groupe 8). À utiliser après avoir ajouté
# l'extension « airbit » ET l'extension « radio ».

arme = False
finInclinaison = 0

def incliner(tangage: number, roulis: number):
    global finInclinaison
    airbit.regler_inclinaison(roulis, tangage)
    finInclinaison = input.running_time() + 1000   # 1 seconde

def on_received_string(ordre):
    global arme
    if ordre == "stop":
        arme = False
    elif ordre == "allumage":
        airbit.remettre_a_zero_stabilisation()
        airbit.regler_gaz(0)
        arme = True
    elif ordre == "gaz plus":
        airbit.regler_gaz(min(airbit.gaz() + 5, 90))
        if airbit.gaz() < 50:
            airbit.regler_gaz(50)
    elif ordre == "gaz moins":
        airbit.regler_gaz(max(airbit.gaz() - 5, 0))
    elif ordre == "avant":
        incliner(10, 0)
    elif ordre == "arriere":
        incliner(-10, 0)
    elif ordre == "droite":
        incliner(0, 10)
    elif ordre == "gauche":
        incliner(0, -10)
    elif ordre == "lacet droite":
        airbit.tourner_lacet(20)
    elif ordre == "lacet gauche":
        airbit.tourner_lacet(-20)
radio.on_received_string(on_received_string)

def on_button_pressed_ab():
    global arme
    arme = False
input.on_button_pressed(Button.AB, on_button_pressed_ab)

radio.set_group(8)
basic.show_number(8)
airbit.initialiser_drone()

def boucle_de_vol():
    global arme
    # while True évite la pause de 20 ms ajoutée par basic.forever
    while True:
        airbit.lire_gyroscope()
        airbit.calculer_angles()
        basic.pause(1)
        if input.running_time() > finInclinaison:
            airbit.regler_inclinaison(0, 0)            # retour à plat
        if abs(airbit.angle_de_roulis()) > 90:
            arme = False                               # drone retourné : on coupe
        airbit.stabiliser_pid()
        if arme and airbit.gaz() > 0:
            airbit.envoyer_stabilisation_aux_moteurs()
        elif arme:
            airbit.vitesse_moteurs(5, 5, 5, 5)         # ralenti
        else:
            airbit.regler_gaz(0)
            airbit.remettre_a_zero_stabilisation()
            airbit.vitesse_moteurs(0, 0, 0, 0)         # arrêt

basic.forever(boucle_de_vol)

def mesure_batterie():
    airbit.mesurer_batterie()
basic.forever(mesure_batterie)

# Écran : gaz à gauche, batterie à droite, point en haut au milieu = armé
def affichage():
    basic.clear_screen()
    airbit.barre_graduee(0, airbit.gaz())
    airbit.barre_graduee(4, airbit.niveau_batterie())
    if arme:
        led.plot(2, 0)
    basic.pause(200)
basic.forever(affichage)
