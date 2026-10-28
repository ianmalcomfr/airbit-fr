/**
 * Extension Air:bit (français) — pilotage du drone Air:bit avec une carte BBC micro:bit.
 *
 * Adaptée du code d'origine fourni avec le drone Air:bit (fichier custom.ts) :
 *  - tous les blocs et les noms de fonctions sont en français ;
 *  - l'extension est AUTONOME : les variables de vol (gaz, consignes, angles, moteurs,
 *    réglages PID...) sont déclarées ici, plus besoin de les créer dans le programme.
 */

//% weight=100 color=#0fbc11 icon="" block="Air:bit"
//% groups=['Démarrage', 'Vol', 'Réglages PID', 'Batterie', 'Écran', 'Avancé']
namespace airbit {

    export enum Moteur {
        //% block="A"
        A = 0,
        //% block="B"
        B = 1,
        //% block="C"
        C = 2,
        //% block="D"
        D = 3
    }

    // ------------------------------------------------------------------
    //  Variables internes (état du drone)
    // ------------------------------------------------------------------

    // Consignes demandées par le pilote
    let gazValeur = 0          // 0..100
    let consigneRoulis = 0     // degrés
    let consigneTangage = 0    // degrés
    let consigneLacet = 0      // degrés

    // Angles mesurés par le gyroscope / accéléromètre
    let angleRoulis = 0
    let angleTangage = 0
    let angleLacet = 0

    // Vitesses calculées pour les 4 moteurs (0..255)
    let moteurA = 0
    let moteurB = 0
    let moteurC = 0
    let moteurD = 0

    // 0 = pas de gyroscope, 1 = ICM-20602, 2 = LSM6DSOX
    let typeGyro = 0
    let controleurMoteursPresent = false

    // Batterie (millivolts, valeur lissée)
    let batterieLissee = 3700
    const FACTEUR_BATTERIE = 5.94

    // Réglages de stabilisation (PID) d'origine
    let roulisTangageP = 0.7
    let roulisTangageI = 0.004
    let roulisTangageD = 12
    let lacetP = 5
    let lacetD = 70

    // Mesures brutes des capteurs
    let gyroX = 0
    let gyroY = 0
    let gyroZ = 0
    let accX = 0
    let accY = 0
    let accZ = 0
    let gyroXCalibration = 0
    let gyroYCalibration = 0
    let gyroZCalibration = 0
    let accRoulis = 0
    let accTangage = 0
    let accRoulisDecalage = 0
    let accTangageDecalage = 0
    let tempsPrecedent = 0

    // Termes du régulateur PID
    let roulisDiff = 0
    let tangageDiff = 0
    let lacetDiff = 0
    let derniereRoulisDiff = 0
    let derniereTangageDiff = 0
    let derniereLacetDiff = 0
    let roulisIntegrale = 0
    let tangageIntegrale = 0
    let lacetIntegrale = 0
    let corrRoulis = 0
    let corrTangage = 0
    let corrLacet = 0

    // Adresses I²C et registres
    const PCA_ADRESSE = 98
    const PCA_MODE1 = 0
    const PCA_MODE2 = 1
    const PCA_PWM0 = 2
    const PCA_PWM1 = 3
    const PCA_PWM2 = 4
    const PCA_PWM3 = 5
    const PCA_LEDOUT = 8
    const PCA_MODE2_CONFIG = 5
    const IMU1_ADRESSE = 104      // 0x68 : ICM-20602
    const IMU2_ADRESSE = 0x6A     // LSM6DSOX

    // ------------------------------------------------------------------
    //  DÉMARRAGE
    // ------------------------------------------------------------------

    /**
     * Séquence de démarrage complète du drone : bus I²C, détection et démarrage du
     * gyroscope, démarrage des moteurs (attend que la batterie soit branchée),
     * calibration (drone posé à plat, sans bouger). Affiche une coche quand c'est prêt.
     */
    //% blockId=airbit_initialiser
    //% block="initialiser le drone Air:bit"
    //% group='Démarrage' weight=100
    export function initialiserDrone() {
        reglerBusI2C()
        basic.pause(100)
        detecterGyroscope()
        demarrerGyroscope()
        basic.pause(100)
        demarrerControleurMoteurs()
        while (!controleurMoteursPresent) {
            basic.showString("Batterie ?")
            demarrerControleurMoteurs()
        }
        calibrerGyroscope()
        remettreAZeroStabilisation()
        basic.showIcon(IconNames.Yes)
    }

    /**
     * Règle le bus I²C du drone sur les broches P2 (SDA) et P1 (SCL).
     */
    //% blockId=airbit_bus_i2c
    //% block="régler le bus I²C du drone (P2 / P1)"
    //% group='Démarrage' weight=90
    export function reglerBusI2C() {
        i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1)
    }

    /**
     * Détecte le modèle de gyroscope présent sur le drone.
     */
    //% blockId=airbit_detecter_gyro
    //% block="détecter le gyroscope"
    //% group='Démarrage' weight=80
    export function detecterGyroscope() {
        typeGyro = 0

        // Test du LSM6DSOX à l'adresse 0x6A
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x0F, NumberFormat.UInt8BE, true)
        let qui = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.UInt8BE, false)
        if (qui == 0x6C) {
            typeGyro = 2
            return
        }

        // Test de l'ICM-20602 à l'adresse 0x68
        pins.i2cWriteNumber(IMU1_ADRESSE, 0x75, NumberFormat.UInt8BE, true)
        qui = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.UInt8BE, false)
        if (qui == 0x12) {
            typeGyro = 1
            return
        }
    }

    /**
     * Démarre et configure le gyroscope / accéléromètre.
     */
    //% blockId=airbit_demarrer_gyro
    //% block="démarrer le gyroscope"
    //% group='Démarrage' weight=70
    export function demarrerGyroscope() {
        if (typeGyro == 1) {
            demarrerGyroscope1()
        } else if (typeGyro == 2) {
            demarrerGyroscope2()
        }
    }

    /**
     * Démarre le contrôleur des moteurs (puce PCA). À répéter tant que la batterie
     * n'est pas branchée.
     */
    //% blockId=airbit_demarrer_moteurs
    //% block="démarrer le contrôleur des moteurs"
    //% group='Démarrage' weight=60
    export function demarrerControleurMoteurs() {
        ecrireControleurMoteurs(PCA_MODE1, 128)
        ecrireControleurMoteurs(PCA_MODE2, PCA_MODE2_CONFIG)
        ecrireControleurMoteurs(PCA_LEDOUT, 170)

        vitesseMoteurs(0, 0, 0, 0)

        // Auto-test : on essaie de relire un registre
        pins.i2cWriteNumber(PCA_ADRESSE, PCA_MODE2, NumberFormat.UInt8BE, true)
        let retour = pins.i2cReadNumber(PCA_ADRESSE, NumberFormat.UInt8BE, false)
        basic.clearScreen()
        controleurMoteursPresent = retour != 0
    }

    /**
     * Calibre le gyroscope. Poser le drone À PLAT et NE PAS LE BOUGER.
     */
    //% blockId=airbit_calibrer
    //% block="calibrer le gyroscope (drone à plat, immobile)"
    //% group='Démarrage' weight=50
    export function calibrerGyroscope() {
        gyroXCalibration = 0
        gyroYCalibration = 0
        gyroZCalibration = 0
        let compteurStable = 0
        let filtreSecousse = 0
        let filtreDelta = 0
        let ancienFiltreDelta = 0
        const roulisMax = 5      // inclinaison maximale tolérée avant calibration
        const tangageMax = 5
        basic.showString("C")
        lireGyroscope()
        calculerAngles()

        // 1) attendre que le drone soit à plat
        while (Math.abs(accRoulis) > roulisMax || Math.abs(accTangage) > tangageMax) {
            lireGyroscope()
            calculerAngles()
        }

        // 2) attendre qu'il soit immobile pendant 100 mesures
        while (compteurStable < 100) {
            lireGyroscope()
            filtreDelta = input.acceleration(Dimension.Strength) - ancienFiltreDelta
            ancienFiltreDelta = input.acceleration(Dimension.Strength)
            filtreSecousse = Math.abs(filtreDelta) * 0.1 + filtreSecousse * 0.9
            accTangage = -57.295 * Math.atan2(accY, accZ)
            accRoulis = -57.295 * Math.atan2(accX, accZ)
            basic.clearScreen()

            // barre de progression sur les colonnes 0 et 4 + niveau à bulle
            for (let i = 0; i < compteurStable / 20; i++) {
                led.plot(0, 4 - i)
                led.plot(4, 4 - i)
            }
            led.plot(accRoulis / 4 + 2.5, -accTangage / 4 + 2.5)
            led.plot(accRoulis / 4 + 3.5, -accTangage / 4 + 2.5)
            led.plot(accRoulis / 4 + 1.5, -accTangage / 4 + 2.5)
            led.plot(accRoulis / 4 + 2.5, -accTangage / 4 + 1.5)
            led.plot(accRoulis / 4 + 2.5, -accTangage / 4 + 3.5)

            if (Math.abs(accRoulis) < roulisMax && Math.abs(accTangage) < tangageMax && filtreSecousse < 14) {
                compteurStable += 1
            } else {
                compteurStable = 0
            }
            basic.pause(20)
        }

        // 3) moyenne de 100 mesures = décalage (« zéro ») du gyroscope
        for (let index = 0; index < 100; index++) {
            lireGyroscope()
            gyroXCalibration += gyroX
            gyroYCalibration += gyroY
            gyroZCalibration += gyroZ
            basic.pause(5)
        }
        gyroXCalibration = gyroXCalibration / 100
        gyroYCalibration = gyroYCalibration / 100
        gyroZCalibration = gyroZCalibration / 100
        accTangage = -57.295 * Math.atan2(accY, accZ)
        accRoulis = -57.295 * Math.atan2(accX, accZ)
        accTangageDecalage = accTangage
        accRoulisDecalage = accRoulis

        basic.showIcon(IconNames.Yes)
    }

    /**
     * Vrai si le contrôleur des moteurs répond (batterie branchée).
     */
    //% blockId=airbit_moteurs_presents
    //% block="contrôleur des moteurs détecté"
    //% group='Démarrage' weight=40
    export function controleurMoteursDetecte(): boolean {
        return controleurMoteursPresent
    }

    /**
     * Type de gyroscope : 0 = aucun, 1 = ICM-20602, 2 = LSM6DSOX.
     */
    //% blockId=airbit_type_gyro
    //% block="type de gyroscope"
    //% group='Démarrage' weight=30
    export function typeDeGyroscope(): number {
        return typeGyro
    }

    // ------------------------------------------------------------------
    //  VOL
    // ------------------------------------------------------------------

    /**
     * Règle les gaz (puissance des moteurs), de 0 à 100.
     */
    //% blockId=airbit_regler_gaz
    //% block="régler les gaz à $gaz"
    //% gaz.min=0 gaz.max=100 gaz.defl=0
    //% group='Vol' weight=100
    export function reglerGaz(gaz: number) {
        gazValeur = Math.constrain(gaz, 0, 100)
    }

    /**
     * Gaz actuels (0 à 100).
     */
    //% blockId=airbit_gaz
    //% block="gaz"
    //% group='Vol' weight=95
    export function gaz(): number {
        return gazValeur
    }

    /**
     * Règle l'inclinaison voulue : roulis (droite + / gauche −) et tangage
     * (avant + / arrière −), en degrés. 0 = drone à plat.
     */
    //% blockId=airbit_regler_inclinaison
    //% block="régler l'inclinaison voulue roulis $roulis tangage $tangage"
    //% roulis.min=-45 roulis.max=45 roulis.defl=0
    //% tangage.min=-45 tangage.max=45 tangage.defl=0
    //% inlineInputMode=inline
    //% group='Vol' weight=90
    export function reglerInclinaison(roulis: number, tangage: number) {
        consigneRoulis = roulis
        consigneTangage = tangage
    }

    /**
     * Règle l'orientation voulue (lacet), en degrés.
     */
    //% blockId=airbit_regler_lacet
    //% block="régler l'orientation voulue (lacet) à $lacet"
    //% lacet.defl=0
    //% group='Vol' weight=85
    export function reglerLacet(lacet: number) {
        consigneLacet = lacet
    }

    /**
     * Fait tourner le drone : ajoute un angle (en degrés) à l'orientation voulue.
     */
    //% blockId=airbit_tourner
    //% block="tourner l'orientation voulue de $degres"
    //% degres.defl=20
    //% group='Vol' weight=80
    export function tournerLacet(degres: number) {
        consigneLacet += degres
    }

    /**
     * Lit le gyroscope et l'accéléromètre.
     */
    //% blockId=airbit_lire_gyro
    //% block="lire le gyroscope"
    //% group='Vol' weight=70
    export function lireGyroscope() {
        if (typeGyro == 1) {
            lireGyroscope1()
        } else if (typeGyro == 2) {
            lireGyroscope2()
        }
    }

    /**
     * Calcule le roulis, le tangage et le lacet du drone à partir des mesures.
     */
    //% blockId=airbit_calculer_angles
    //% block="calculer les angles"
    //% group='Vol' weight=60
    export function calculerAngles() {
        const ECHELLE_GYRO = 1 / 16.4 / 1000

        const duree = input.runningTime() - tempsPrecedent
        tempsPrecedent = input.runningTime()
        accTangage = (-57.295 * Math.atan2(accY, accZ)) - accTangageDecalage
        accRoulis = (-57.295 * Math.atan2(accX, accZ)) - accRoulisDecalage
        // Variation d'angle depuis la dernière mesure
        const deltaX = (gyroX - gyroXCalibration) * duree * -ECHELLE_GYRO
        const deltaY = (gyroY - gyroYCalibration) * duree * ECHELLE_GYRO
        const deltaZ = (gyroZ - gyroZCalibration) * duree * -ECHELLE_GYRO
        // Filtre complémentaire : 99,5 % gyroscope + 0,5 % accéléromètre
        angleRoulis = (deltaY + angleRoulis) * 0.995 + accRoulis * 0.005
        angleTangage = (deltaX + angleTangage) * 0.995 + accTangage * 0.005
        angleLacet = deltaZ + angleLacet
    }

    /**
     * Calcule la vitesse des 4 moteurs avec le régulateur PID pour que le drone
     * atteigne l'inclinaison voulue (à appeler à chaque tour de la boucle de vol).
     */
    //% blockId=airbit_stabiliser
    //% block="stabiliser (PID)"
    //% group='Vol' weight=50
    export function stabiliserPid() {
        roulisDiff = consigneRoulis - angleRoulis
        tangageDiff = consigneTangage - angleTangage
        lacetDiff = consigneLacet - angleLacet
        const roulisDerivee = roulisDiff - derniereRoulisDiff
        const tangageDerivee = tangageDiff - derniereTangageDiff
        const lacetDerivee = lacetDiff - derniereLacetDiff

        derniereRoulisDiff = roulisDiff
        derniereTangageDiff = tangageDiff
        derniereLacetDiff = lacetDiff

        const plageI = 100       // erreur maximale qui fait grandir l'intégrale
        const limiteI = 7        // correction maximale ajoutée par l'intégrale
        const limiteLacet = 120  // correction maximale du lacet

        if (gazValeur > 50) {    // évite l'accumulation avant le décollage
            if (roulisDiff > -plageI && roulisDiff < plageI) {
                roulisIntegrale += roulisDiff
            }
            if (tangageDiff > -plageI && tangageDiff < plageI) {
                tangageIntegrale += tangageDiff
            }
        }

        let corrIRoulis = Math.constrain(roulisIntegrale * roulisTangageI, -limiteI, limiteI)
        let corrITangage = Math.constrain(tangageIntegrale * roulisTangageI, -limiteI, limiteI)

        corrRoulis = roulisDiff * roulisTangageP + corrIRoulis + roulisDerivee * roulisTangageD
        corrTangage = tangageDiff * roulisTangageP + corrITangage + tangageDerivee * roulisTangageD
        corrLacet = lacetDiff * lacetP + lacetDerivee * lacetD
        corrLacet = Math.constrain(corrLacet, -limiteLacet, limiteLacet)
        const gazEchelle = gazValeur * 2.55

        moteurA = Math.round(gazEchelle + corrRoulis + corrTangage + corrLacet)
        moteurB = Math.round(gazEchelle + corrRoulis - corrTangage - corrLacet)
        moteurC = Math.round(gazEchelle - corrRoulis + corrTangage - corrLacet)
        moteurD = Math.round(gazEchelle - corrRoulis - corrTangage + corrLacet)
        moteurA = Math.constrain(moteurA, 0, 255)
        moteurB = Math.constrain(moteurB, 0, 255)
        moteurC = Math.constrain(moteurC, 0, 255)
        moteurD = Math.constrain(moteurD, 0, 255)
    }

    /**
     * Envoie aux 4 moteurs les vitesses calculées par « stabiliser (PID) ».
     */
    //% blockId=airbit_envoyer_stabilisation
    //% block="envoyer la stabilisation aux moteurs"
    //% group='Vol' weight=45
    export function envoyerStabilisationAuxMoteurs() {
        vitesseMoteurs(moteurA, moteurB, moteurC, moteurD)
    }

    /**
     * Règle la vitesse de chaque moteur, de 0 à 255. Mettre 0 partout pour arrêter.
     */
    //% blockId=airbit_vitesse_moteurs
    //% block="vitesse des moteurs A $a B $b C $c D $d"
    //% a.min=0 a.max=255 b.min=0 b.max=255 c.min=0 c.max=255 d.min=0 d.max=255
    //% inlineInputMode=inline
    //% group='Vol' weight=40
    export function vitesseMoteurs(a: number, b: number, c: number, d: number) {
        pins.i2cWriteNumber(PCA_ADRESSE, PCA_PWM0 << 8 | d, NumberFormat.UInt16BE, false)
        pins.i2cWriteNumber(PCA_ADRESSE, PCA_PWM1 << 8 | c, NumberFormat.UInt16BE, false)
        pins.i2cWriteNumber(PCA_ADRESSE, PCA_PWM2 << 8 | b, NumberFormat.UInt16BE, false)
        pins.i2cWriteNumber(PCA_ADRESSE, PCA_PWM3 << 8 | a, NumberFormat.UInt16BE, false)
    }

    /**
     * Efface les mémoires du régulateur PID et les angles (à faire avant un décollage
     * et quand le drone est arrêté).
     */
    //% blockId=airbit_raz_stabilisation
    //% block="remettre à zéro la stabilisation"
    //% group='Vol' weight=30
    export function remettreAZeroStabilisation() {
        angleRoulis = 0
        angleTangage = 0
        angleLacet = 0
        roulisDiff = 0
        tangageDiff = 0
        lacetDiff = 0
        derniereRoulisDiff = 0
        derniereTangageDiff = 0
        derniereLacetDiff = 0
        roulisIntegrale = 0
        tangageIntegrale = 0
        lacetIntegrale = 0
        consigneLacet = 0
        corrRoulis = 0
        corrTangage = 0
        corrLacet = 0
    }

    /**
     * Angle de roulis mesuré (en degrés, + = penché à droite).
     */
    //% blockId=airbit_angle_roulis
    //% block="angle de roulis mesuré"
    //% group='Vol' weight=20
    export function angleDeRoulis(): number {
        return angleRoulis
    }

    /**
     * Angle de tangage mesuré (en degrés, + = penché vers l'avant).
     */
    //% blockId=airbit_angle_tangage
    //% block="angle de tangage mesuré"
    //% group='Vol' weight=19
    export function angleDeTangage(): number {
        return angleTangage
    }

    /**
     * Angle de lacet mesuré (en degrés, orientation du drone).
     */
    //% blockId=airbit_angle_lacet
    //% block="angle de lacet mesuré"
    //% group='Vol' weight=18
    export function angleDeLacet(): number {
        return angleLacet
    }

    /**
     * Vitesse calculée pour un moteur (0 à 255).
     */
    //% blockId=airbit_vitesse_moteur
    //% block="vitesse calculée du moteur $moteur"
    //% group='Vol' weight=10
    export function vitesseDuMoteur(moteur: Moteur): number {
        switch (moteur) {
            case Moteur.A: return moteurA
            case Moteur.B: return moteurB
            case Moteur.C: return moteurC
            default: return moteurD
        }
    }

    // ------------------------------------------------------------------
    //  RÉGLAGES PID
    // ------------------------------------------------------------------

    /**
     * Règle le PID du roulis et du tangage. Valeurs d'origine : P 0.7, I 0.004, D 12.
     * P = réaction à l'erreur, I = correction de l'erreur accumulée, D = amortissement.
     */
    //% blockId=airbit_pid_roulis_tangage
    //% block="régler le PID roulis/tangage P $p I $i D $d"
    //% p.defl=0.7 i.defl=0.004 d.defl=12
    //% inlineInputMode=inline
    //% group='Réglages PID' weight=100
    export function reglerPidRoulisTangage(p: number, i: number, d: number) {
        roulisTangageP = p
        roulisTangageI = i
        roulisTangageD = d
    }

    /**
     * Règle le PID du lacet (rotation). Valeurs d'origine : P 5, D 70.
     */
    //% blockId=airbit_pid_lacet
    //% block="régler le PID lacet P $p D $d"
    //% p.defl=5 d.defl=70
    //% inlineInputMode=inline
    //% group='Réglages PID' weight=90
    export function reglerPidLacet(p: number, d: number) {
        lacetP = p
        lacetD = d
    }

    // ------------------------------------------------------------------
    //  BATTERIE
    // ------------------------------------------------------------------

    /**
     * Mesure la tension de la batterie et la lisse (filtre passe-bas). À appeler
     * régulièrement, par exemple dans « toujours ».
     */
    //% blockId=airbit_mesurer_batterie
    //% block="mesurer la batterie"
    //% group='Batterie' weight=100
    export function mesurerBatterie() {
        batterieLissee = Math.round(pins.analogReadPin(AnalogPin.P0) * FACTEUR_BATTERIE * 0.1 + batterieLissee * 0.9)
    }

    /**
     * Niveau de la batterie en pourcentage (0 à 100).
     */
    //% blockId=airbit_niveau_batterie
    //% block="niveau de la batterie (%)"
    //% group='Batterie' weight=90
    export function niveauBatterie(): number {
        mesurerBatterie()
        return Math.constrain(Math.map(batterieLissee, 3400, 4200, 0, 100), 0, 100)
    }

    /**
     * Tension de la batterie en millivolts, valeur lissée.
     */
    //% blockId=airbit_batterie_lissee
    //% block="batterie en millivolts (lissée)"
    //% group='Batterie' weight=80
    export function batterieMillivoltsLissee(): number {
        return batterieLissee
    }

    /**
     * Tension de la batterie en millivolts, mesure instantanée.
     */
    //% blockId=airbit_batterie_mv
    //% block="batterie en millivolts (instantanée)"
    //% group='Batterie' weight=70
    export function batterieMillivolts(): number {
        return Math.round(pins.analogReadPin(AnalogPin.P0) * FACTEUR_BATTERIE)
    }

    // ------------------------------------------------------------------
    //  ÉCRAN
    // ------------------------------------------------------------------

    /**
     * Dessine une barre verticale graduée sur une colonne de l'écran (valeur de 0 à 100).
     */
    //% blockId=airbit_barre
    //% block="barre graduée colonne $x valeur $valeur"
    //% x.min=0 x.max=4
    //% valeur.min=0 valeur.max=100
    //% group='Écran' weight=100
    export function barreGraduee(x: number, valeur: number) {
        valeur = Math.constrain(valeur, 0, 100)
        for (let index = 0; index <= valeur / 20; index++) {
            led.plot(x, 4 - index)
        }
        led.plotBrightness(x, 4 - Math.floor(valeur / 20), 12.75 * (valeur % 20))
    }

    /**
     * Dessine un point qui tourne autour d'un centre. Vitesse de −100 à 100
     * (négatif = sens inverse des aiguilles d'une montre).
     */
    //% blockId=airbit_point_tournant
    //% block="point tournant centre x $x y $y rayon $rayon vitesse $vitesse"
    //% x.min=0 x.max=4 x.defl=2
    //% y.min=0 y.max=4 y.defl=2
    //% rayon.min=1 rayon.max=4 rayon.defl=2
    //% vitesse.min=-100 vitesse.max=100 vitesse.defl=10
    //% inlineInputMode=inline
    //% group='Écran' weight=90
    export function pointTournant(x: number, y: number, rayon: number, vitesse: number) {
        led.plot(
            x + 0.5 + (rayon + 0.5) * Math.cos(input.runningTime() / 10000 * 6.283 * vitesse),
            y + 0.5 + (rayon + 0.5) * Math.sin(input.runningTime() / 10000 * 6.283 * vitesse)
        )
    }

    // ------------------------------------------------------------------
    //  AVANCÉ
    // ------------------------------------------------------------------

    /**
     * Écrit une valeur dans un registre du contrôleur des moteurs (puce PCA).
     */
    //% blockId=airbit_ecrire_pca
    //% block="écrire dans le contrôleur des moteurs registre $registre valeur $valeur"
    //% inlineInputMode=inline
    //% group='Avancé' weight=100 advanced=true
    export function ecrireControleurMoteurs(registre: number, valeur: number) {
        pins.i2cWriteNumber(PCA_ADRESSE, registre << 8 | valeur, NumberFormat.UInt16BE, false)
    }

    /**
     * Lit un registre du contrôleur des moteurs (puce PCA).
     */
    //% blockId=airbit_lire_pca
    //% block="lire le contrôleur des moteurs registre $registre"
    //% group='Avancé' weight=90 advanced=true
    export function lireControleurMoteurs(registre: number): number {
        pins.i2cWriteNumber(PCA_ADRESSE, registre, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(PCA_ADRESSE, NumberFormat.UInt8BE, false)
    }

    // ------------------------------------------------------------------
    //  Fonctions internes (non visibles dans la boîte à outils)
    // ------------------------------------------------------------------

    // Démarrage du gyroscope de type 2 (LSM6DSOX)
    function demarrerGyroscope2() {
        // Réinitialisation (CTRL3_C : SW_RESET)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x12 << 8) | 0x01, NumberFormat.UInt16BE, false)
        basic.pause(50)
        // Incrémentation automatique des registres (IF_INC)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x12 << 8) | 0x04, NumberFormat.UInt16BE, false)
        // Gyroscope : 416 Hz, ±2000 °/s (CTRL2_G)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x11 << 8) | 0x6C, NumberFormat.UInt16BE, false)
        // Filtre passe-bas du gyroscope (CTRL6_C)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x15 << 8) | 0x00, NumberFormat.UInt16BE, false)
        // Accéléromètre : 208 Hz, ±4 g (CTRL1_XL)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x10 << 8) | 0x58, NumberFormat.UInt16BE, false)
        // Filtre passe-bas de l'accéléromètre (CTRL8_XL)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x17 << 8) | 0x09, NumberFormat.UInt16BE, false)
        // Gyroscope en haute performance (CTRL7_G)
        pins.i2cWriteNumber(IMU2_ADRESSE, (0x16 << 8) | 0x00, NumberFormat.UInt16BE, false)
        basic.pause(20)
    }

    // Démarrage du gyroscope de type 1 (ICM-20602)
    function demarrerGyroscope1() {
        // Réinitialisation complète
        pins.i2cWriteNumber(IMU1_ADRESSE, 107 << 8 | 0x80, NumberFormat.UInt16BE, false)
        basic.pause(500)
        // Gyroscope ±2000 °/s
        pins.i2cWriteNumber(IMU1_ADRESSE, 27 << 8 | 0x18, NumberFormat.UInt16BE, false)
        // Horloge sur la PLL interne
        pins.i2cWriteNumber(IMU1_ADRESSE, 107 << 8 | 0x01, NumberFormat.UInt16BE, false)
        // Réinitialisation des chemins de signal
        pins.i2cWriteNumber(IMU1_ADRESSE, 105 << 8 | 0x07, NumberFormat.UInt16BE, false)
        // FIFO désactivée
        pins.i2cWriteNumber(IMU1_ADRESSE, 106 << 8 | 0x00, NumberFormat.UInt16BE, false)
        // Filtre du gyroscope (250 Hz)
        pins.i2cWriteNumber(IMU1_ADRESSE, 1 << 8 | 0, NumberFormat.UInt16BE, false)
        // Accéléromètre ±4 g
        pins.i2cWriteNumber(IMU1_ADRESSE, 28 << 8 | 0x08, NumberFormat.UInt16BE, false)
        // Filtre de l'accéléromètre (10,2 Hz)
        pins.i2cWriteNumber(IMU1_ADRESSE, 29 << 8 | 5, NumberFormat.UInt16BE, false)
    }

    // Lecture du gyroscope de type 2
    function lireGyroscope2() {
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x22, NumberFormat.UInt8BE, true)
        gyroX = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x24, NumberFormat.UInt8BE, true)
        gyroY = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x26, NumberFormat.UInt8BE, true)
        gyroZ = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x28, NumberFormat.UInt8BE, true)
        accX = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x2A, NumberFormat.UInt8BE, true)
        accY = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)
        pins.i2cWriteNumber(IMU2_ADRESSE, 0x2C, NumberFormat.UInt8BE, true)
        accZ = pins.i2cReadNumber(IMU2_ADRESSE, NumberFormat.Int16LE, false)

        accX = -accX
        accY = -accY
        gyroX = -gyroX
        gyroY = -gyroY
    }

    // Lecture du gyroscope de type 1
    function lireGyroscope1() {
        pins.i2cWriteNumber(IMU1_ADRESSE, 67, NumberFormat.Int8LE, true)
        gyroX = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, true)
        gyroY = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, true)
        gyroZ = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, false)
        pins.i2cWriteNumber(IMU1_ADRESSE, 59, NumberFormat.Int8LE, true)
        accX = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, true)
        accY = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, true)
        accZ = pins.i2cReadNumber(IMU1_ADRESSE, NumberFormat.Int16BE, false)
    }
}
