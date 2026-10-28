# Air:bit (français)

Extension MakeCode pour piloter le **drone Air:bit** avec une carte **micro:bit**, entièrement en français :
blocs, noms de fonctions Python/JavaScript et aide.

Elle est **autonome** : les variables de vol (gaz, consignes, angles, moteurs, réglages PID, batterie)
sont intégrées, il n'y a rien à créer dans ton programme.

## Installer dans un projet MakeCode

1. Ouvre https://makecode.microbit.org et crée (ou ouvre) un projet.
2. Clique sur **Extensions**.
3. Colle l'adresse GitHub de ce dépôt dans la barre de recherche, puis valide, puis clique sur la carte **Air:bit**.

L'extension `microbit-pxt-i2cpins` (bus I²C sur P2/P1) est ajoutée automatiquement.
Pour piloter par radio, ajoute aussi l'extension **radio** (incluse dans MakeCode).

## Programme minimal

```blocks
airbit.initialiserDrone()
basic.forever(function () {
    airbit.lireGyroscope()
    airbit.calculerAngles()
    airbit.stabiliserPid()
    airbit.envoyerStabilisationAuxMoteurs()
})
```

Un exemple complet (télécommande radio, arrêt d'urgence A+B, affichage gaz/batterie)
se trouve dans `exemples/drone-exemple.ts` et `exemples/drone-exemple.py`.

## Les blocs

| Groupe | Bloc | Python |
|---|---|---|
| Démarrage | initialiser le drone Air:bit | `airbit.initialiser_drone()` |
| | régler le bus I²C du drone (P2 / P1) | `airbit.regler_bus_i2c()` |
| | détecter le gyroscope | `airbit.detecter_gyroscope()` |
| | démarrer le gyroscope | `airbit.demarrer_gyroscope()` |
| | démarrer le contrôleur des moteurs | `airbit.demarrer_controleur_moteurs()` |
| | calibrer le gyroscope (drone à plat, immobile) | `airbit.calibrer_gyroscope()` |
| | contrôleur des moteurs détecté | `airbit.controleur_moteurs_detecte()` |
| | type de gyroscope | `airbit.type_de_gyroscope()` |
| Vol | régler les gaz à (0–100) | `airbit.regler_gaz(g)` |
| | gaz | `airbit.gaz()` |
| | régler l'inclinaison voulue roulis / tangage | `airbit.regler_inclinaison(roulis, tangage)` |
| | régler l'orientation voulue (lacet) | `airbit.regler_lacet(l)` |
| | tourner l'orientation voulue de | `airbit.tourner_lacet(deg)` |
| | lire le gyroscope | `airbit.lire_gyroscope()` |
| | calculer les angles | `airbit.calculer_angles()` |
| | stabiliser (PID) | `airbit.stabiliser_pid()` |
| | envoyer la stabilisation aux moteurs | `airbit.envoyer_stabilisation_aux_moteurs()` |
| | vitesse des moteurs A B C D (0–255) | `airbit.vitesse_moteurs(a, b, c, d)` |
| | remettre à zéro la stabilisation | `airbit.remettre_a_zero_stabilisation()` |
| | angle de roulis / tangage / lacet mesuré | `airbit.angle_de_roulis()` … |
| | vitesse calculée du moteur A/B/C/D | `airbit.vitesse_du_moteur(m)` |
| Réglages PID | régler le PID roulis/tangage P I D | `airbit.regler_pid_roulis_tangage(p, i, d)` |
| | régler le PID lacet P D | `airbit.regler_pid_lacet(p, d)` |
| Batterie | mesurer la batterie | `airbit.mesurer_batterie()` |
| | niveau de la batterie (%) | `airbit.niveau_batterie()` |
| | batterie en millivolts (lissée / instantanée) | `airbit.batterie_millivolts_lissee()` / `airbit.batterie_millivolts()` |
| Écran | barre graduée colonne x valeur | `airbit.barre_graduee(x, v)` |
| | point tournant | `airbit.point_tournant(x, y, rayon, vitesse)` |
| Avancé | écrire / lire le contrôleur des moteurs | `airbit.ecrire_controleur_moteurs(r, v)` / `airbit.lire_controleur_moteurs(r)` |

Réglages PID d'origine : roulis/tangage P 0,7 · I 0,004 · D 12 ; lacet P 5 · D 70.

## ⚠ Sécurité

Un drone peut blesser. Teste toujours **sans hélices** d'abord, garde une commande d'arrêt d'urgence
(`arme = faux` + `vitesse des moteurs 0 0 0 0`) et ne calibre que drone posé à plat et immobile.

## Origine

Code adapté du `custom.ts` fourni avec le drone Air:bit (traduction, extension autonome, quelques
protections : gaz et niveau de batterie bornés, calibration compatible avec les deux gyroscopes).

Cibles prises en charge : micro:bit (V1 et V2).
