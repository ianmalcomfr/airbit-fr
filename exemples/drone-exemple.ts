// DRONE AIR:BIT — exemple complet avec l'extension française « airbit »
// (télécommande par radio, groupe 8). À coller dans l'éditeur JavaScript,
// après avoir ajouté l'extension ET l'extension « radio ».

let arme = false
let finInclinaison = 0

function incliner(tangage: number, roulis: number) {
    airbit.reglerInclinaison(roulis, tangage)
    finInclinaison = input.runningTime() + 1000     // 1 seconde
}

radio.onReceivedString(function (ordre) {
    if (ordre == "stop") {
        arme = false
    } else if (ordre == "allumage") {
        airbit.remettreAZeroStabilisation()
        airbit.reglerGaz(0)
        arme = true
    } else if (ordre == "gaz plus") {
        airbit.reglerGaz(Math.min(airbit.gaz() + 5, 90))
        if (airbit.gaz() < 50) {
            airbit.reglerGaz(50)
        }
    } else if (ordre == "gaz moins") {
        airbit.reglerGaz(Math.max(airbit.gaz() - 5, 0))
    } else if (ordre == "avant") {
        incliner(10, 0)
    } else if (ordre == "arriere") {
        incliner(-10, 0)
    } else if (ordre == "droite") {
        incliner(0, 10)
    } else if (ordre == "gauche") {
        incliner(0, -10)
    } else if (ordre == "lacet droite") {
        airbit.tournerLacet(20)
    } else if (ordre == "lacet gauche") {
        airbit.tournerLacet(-20)
    }
})

input.onButtonPressed(Button.AB, function () {
    arme = false
})

radio.setGroup(8)
basic.showNumber(8)
airbit.initialiserDrone()

// Boucle de vol : le while(true) évite la pause de 20 ms ajoutée par « toujours »
basic.forever(function () {
    while (true) {
        airbit.lireGyroscope()
        airbit.calculerAngles()
        basic.pause(1)
        if (input.runningTime() > finInclinaison) {
            airbit.reglerInclinaison(0, 0)          // retour à plat
        }
        if (Math.abs(airbit.angleDeRoulis()) > 90) {
            arme = false                            // drone retourné : on coupe
        }
        airbit.stabiliserPid()
        if (arme && airbit.gaz() > 0) {
            airbit.envoyerStabilisationAuxMoteurs()
        } else if (arme) {
            airbit.vitesseMoteurs(5, 5, 5, 5)       // ralenti
        } else {
            airbit.reglerGaz(0)
            airbit.remettreAZeroStabilisation()
            airbit.vitesseMoteurs(0, 0, 0, 0)       // arrêt
        }
    }
})

basic.forever(function () {
    airbit.mesurerBatterie()
})

// Écran : gaz à gauche, batterie à droite, point en haut au milieu = armé
basic.forever(function () {
    basic.clearScreen()
    airbit.barreGraduee(0, airbit.gaz())
    airbit.barreGraduee(4, airbit.niveauBatterie())
    if (arme) {
        led.plot(2, 0)
    }
    basic.pause(200)
})
