# Asserv NG

Asserv Next Generation ! Rien que ça.... 
L'idée ici était de faire un asserv pour un robot de coupe de france de robotique (eurobot) avec un certain nombre de caractéristiques :
 * Que des unités SI. Ici, pas d'unités incompréhensibles à la con, mais tous les calculs se font en flottant ( d'où la nécéssité d'une FPU et d'une carte performante) 
 * Un décodage hardware des encodeurs (géré ici par des timers directement sur le microprocesseur)
 * Un premier étage d'asserv des roues en vitesse, puis par dessus, un asservissement polaire. L'avantage est de pouvoir gérer plus facilement les courbes d'accélérations
 * un système de visualisation en temps réel des données calculés par les boucles de controle.  
 * Un OS temps réel 

## Avant de compiler

Mon asserv utilise ChibiOS comme OS temps réel, c'est un sous module du repo qu'il faut initialiser après le clone :

```
git submodule update --init
```

Le compilateur arm-none-eabi est nécessaire pour la compilation et openocd pour flasher la carte:

```
 sudo apt install openocd binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```


## compiler

```
make
```

## Lancer

~~Pour le moment, il suffit de copier build/asservNucleo.bin dans le dossier de la carte, mais un flash via openocd est dans les tuyaux...~~	

```
make flash
```
