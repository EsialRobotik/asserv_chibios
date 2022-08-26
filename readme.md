# Asserv Nucleo

## Quels objectifs?

De part notre expérience en tant que participant à la coupe de France de robotique, nous savons qu'un asservissement est une brique de base obligatoire pour accéder à des travaux intéressants sur un robot ( Si le robot ne sait pas où il est, ou ne sait pas aller à un endroit précis sur la table, le bras le plus classe ou la méca la plus chiadé, ne sert à rien :) ).

Malheureusement cette brique n'est pas facile à appréhender :
 * Il y a peu de personnes ou de ressources qui permettent de s'initier "simplement" à la question
 * La transmission de connaissances sur le sujet est difficile entre générations (petit souvenir ému de mon arrivée dans le club de mon école où on m'a montré un PCB rouge avec marqué "carte d'asserv" dessus et ZERO documentation !)
 * Cela nécessite des connaissances en code, math, éléctronique et mécanique. C'est normal en robotique, mais pour des équipes jeunes, atteindre le niveau minimale dans toutes ces disciplines peut être très long et la finalité pas forcement très motivante.

Donc, lorsque j'ai décidé fin 2019 de refondre notre asservissement je me suis donné quelques objectifs :
  * Faire aussi accessible que possible : que ce soit en terme de compréhension du fonctionnement ou du réglage 
  * Fournir un système de visualisation de données, de façon a pouvoir comprendre facilement le comportement des formules mathématiques utilisés et donc le comportement final du robot. 
  * Avoir une base de code qui soit pensée pour être extensible et partageable entre équipes. Et donc qui soient instanciable facilement  sur d'autres microcontrôleur et en utilisant un matériel différent du notre. 
* Bien-sur avoir une asserv performante. L'idée étant que si une équipe veut être utilisatrice de l'asserv en quasi "boite noire" cela est possible, mais si quelqu'un veut implémenter des choses plus avancée et devenir un "poweruser" c'est possible.

## Quels caractéristiques ?

Donc si on devait compiler une liste de caractéristiques de ce projet façon liste à la Prévert: 

* Que des unités SI. Ici, pas d'unités incompréhensibles à la con, donc tous les calculs se font en flottant ( d'où la nécessité d'une FPU et d'une carte performante)
* Un décodage hardware des encodeurs (géré dans notre instance par des timers directement sur le microprocesseur)
* Un premier étage d'asserv des roues en vitesse, puis par dessus, un asservissement polaire. L'avantage est de pouvoir gérer plus facilement les courbes d'accélérations ( et aussi la décélération, mais moins facilement...)
* un système de visualisation en temps réel des données calculés par les boucles de contrôle.
* Un OS temps réel
* Des documentations sur le fonctionnement minimal à connaître, et sur la façon de régler l'asserv. 
* Des documentations plus avancées sur le fonctionnement interne, les méthodes alternatives existantes, ....


## Pourquoi faire ?

Soyons honnête, l'objectif premier était pour notre équipe d'avoir une meilleur asserv, moins buggé et plus compréhensible pour toute l'équipe. Le second objectif était simplement pour montrer à Planète science qu'en tant que ~~personne trop vieux pour ces conneries~~ ancien de la coupe de France de robotique, nous "jouons le jeu". Nous ne voulons pas pour écraser la concurrence, nous voulons qu'il y ait de la concurrence ! 
Lorsque j'ai commencé à travailler sur un asservissement de robot de coupe de France de robotique, aversive fournit par Microb Robotique m'a énormément appris et aidé. A nous de faire pareil maintenant. 

## Contact

Pour du support, une question ou me payer une bière à la prochaine coupe: robotique [at] epicea.org
Ou plus simplement sur le serveur [Discord de la coupe](https://discord.gg/tteC3Cp), pseudo : cheff[Esial Robotik]


## Avant de compiler

Mon asserv utilise ChibiOS comme OS temps réel, c'est un sous module du repo qu'il faut initialiser après le clone :

```
git submodule update --init
```

Le compilateur arm-none-eabi est nécessaire pour la compilation et openocd pour flasher la carte:

```
 sudo apt install openocd binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib gdb-multiarch
```


## compiler

```
make ROBOT=THE_ROBOT_TO_BUILD
```
THE_ROBOT_TO_BUILD : Princess, PMI, baseRoulanteTest, PMX

## Lancer

~~Pour le moment, il suffit de copier build/asservNucleo.bin dans le dossier de la carte, mais un flash via openocd est dans les tuyaux...~~	

```
make flash
```

## Debugger

D'abord, on lance le programme en mode débug avec la commande : 
```
make debug
```

Puis, __dans un autre shell__ , on lance le débugger comme suit : 
```
gdb-multiarch build/asservNucleo.elf  -ex "target remote :3333"
```

Ensuite, c'est du gdb classique en shell.... 
