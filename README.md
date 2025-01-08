Git spécifique pour le projet LARM, groupe A2-S4

# Structure du GIT du groupe A2-S4

## Dossier launch

Ce dossier contient deux fichiers YAML : simulation_launch.yaml et tbot_launch.yaml. Ces fichiers permettent de lancer les différents nodes permettant de lancer respectivement la simulation sur gazebo et le mouvement du robot dans un espace réel. Ne pas oublier de télécharger la librairie suivante :

    cd $ROS_WORKSPACE
    git clone https://github.com/imt-mobisyst/pkg-interfaces.git
    colcon build --base-path pkg-interfaces
    source ./install/setup.bash

### Fichier simulation_launch.yaml

Ce fichier permet d'ouvrir le logiciel gazebo et de faire avancer le robot.
