# Code de test de détection de signaux d'hydrophones - Sous-marin autonome (ASUQTR)

**Réalisé par Louis Lavallée**

## Contexte du projet
Ce projet fut réalisé dans le cadre du projet de fin d'étude en équipe du baccalauréat en génie électrique à l'UQTR. Le projet d'équipe était centralisé sur la poursuite du développement du sous-marin autonome du club étudiant [ASUQTR](https://oraprdnt.uqtr.uquebec.ca/portail/gscw031?owa_no_site=8035). L'objectif du club ASUQTR est de participer à la compétition internationale [Robosub](https://robosub.org/) où chaque équipe doit concevoir un sous-marin et lui faire accomplir des tâches et missions de manière entièrement autonome.

## Objectif
Parmi les défis à relever lors de la compétition Robosub, il y a une zone de la piscine à atteindre en suivant un signal produit par un pinger sous-marin. À l'aide plusieurs hydrophones et d'un calcul de TDOA (Time Difference of Arrival), il est possible de déterminer la direction d'où provient le signal et de s'en servir comme repère spatial. Le code présenté ici permet de convertir un signal analogique provenant d'un hydrophone en format numérique afin de détecter des fréquences précises grâce à l'algorithme de Goertzel. Le code ne fait le calcul que pour un seul signal. Pour appliquer le calcul du TDOA il faudra convertir cinq signaux et faire cinq calculs différents.

### Sous-objectifs:


## Documentation
**Mapping des pins des MCU avec** ***STM32CubeMX***\
**Programmation des registres des MCU avec** ***STM32CubeProgrammer***\
**Programmation des MCU avec** ***STM32CubeIDE***

Fichier main.c du G474 du PCB: [main.c G474](main_g474.c)\
Fichier main.c de la carte nucleo-F446RE: [main.c F446](main_f446.c)
