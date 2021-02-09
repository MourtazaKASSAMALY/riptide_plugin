## Installer ROS & Gazebo

http://wiki.ros.org/melodic/Installation/Ubuntu pour ROS Melodic sur Ubuntu Bionic 18.04 

## Create a ROS Workspace 

```bash
$ cd ~
$ mkdir -p workspace_riptide/src
$ cd workspace_riptide
$ catkin_make
$ cd src
```

## Installer UUV Simulator

https://uuvsimulator.github.io/installation/

## Installer le package du simulateur et du contrôleur

``` bash
$ cd ~/workspace_riptide/src
$ git clone https://github.com/MourtazaKASSAMALY/riptide_plugin.git
$ cd ~/workspace_riptide
$ catkin_make
```

Procédure au choix :

1.1 - Lancer gazebo avec un monde d'UUV Simulator : 

```bash 
$ roslaunch uuv_gazebo_worlds ocean_waves.launch
```

1.2 - Faire apparaître le Riptide dans la simulation : 

```bash
$ roslaunch riptide_description upload.launch
```

1.3 - Lancer l'interface simulation/contrôleur :

```bash
$ rosrun riptide_description remap.py
```

1.4 - Lancer un contrôleur (en cap ou en waypoint) :

```bash
$ roslaunch riptide_controller controller_heading_without_drivers.launch heading:=desired_heading depth:=desired_depth
$ roslaunch riptide_controller controller_waypoint_without_drivers.launch json:=path_to_json_file
```

Ou bien lancer tout en même temps :

```bash
$ roslaunch riptide_description launch_heading.launch
$ roslaunch riptide_description launch_waypoint.launch
```

## Riptide 2020

Lien fichier suivi : https://enstabretagne-my.sharepoint.com/:x:/g/personal/corentin_lemoine_ensta-bretagne_org/EUwVULg2j5NJgw7PNhc_FJoBiTzT8TCcGzkvBepUGHh1pg?e=4%3AbiCFwa&at=9&CID=44b48342-2e5a-3598-4f73-f3c62bffd64f

## Ajouter un dépôt distant :

 * Configuration au début :
 ```bash
    $ git remode add <nom_du_depot> https://github.com/HamidHacene/Riptide_2020.git
    $ git fetch <nom_du_depot>
  ```
  
  * Pour récupérer les modifications du dépôt distant : 

 ```bash
    $ git fetch <nom_du_depot>
    $ git merge <nom_du_depot>/main main
    $ git push origin main
 ```
