# wm_admittance

L'algorithme d'admittance pour seulement être utilisé sur un seul bras pour l'instant.

La classe principale est un «singleton» et permet d'effectuer un mode d'admittance sur les
joints fournis dans le fichier «sara_admittance.yaml».

Par exemple, pour le bras droit, il y a 7 actionneurs, dont 5 kinovas et 2 dynamixels. L'admittance est effectué sur les joints kinovas, mais les joints dynamixels sont tout de  même pris en considération.

Pour effectuer un calcul d'admittance:
```cpp
WMAdmittance* lAdmittance = WMAdmittance::getInstance();
lAdmittance->process();
```

Il faudra que cette fonction soit appelée par une horloge ROS. Exemple:

```cpp
while (ros::ok()) 
{
    chw.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period);

    lAdmittance->process(); // Fonction appelée dans la boucle principale de ROS (après le update et avant le write)

    chw.write(ros::Time::now(), period);
    period.sleep();
}
```

Voici un exemple pour récupérer les nouvelles vitesses:
```cpp
if (aAdmittance->isAdmittanceEnabled())
{
    for (int i = 0; i < 6; i++) 
    {
        Vel[i] = aAdmittance->getAdmittanceVelocityFromJoint(aIndexByJointNameMap[i]);
    }
}
```

Il faut au préalable configurer le fichier sara_admittance.yaml pour fournir les noms des joints et des référentiels:
```
sara_admittance:
    joint_names:
        - right_shoulder_roll_joint
        - right_shoulder_pitch_joint
        - right_shoulder_yaw_joint
        - right_elbow_pitch_joint
        - right_elbow_yaw_joint

    # These values define names of published transforms.
    # These values should be the same as
    # the ones in the joint_names array.
    right_shoulder_roll_joint:
        - right_clavicular_link

    right_shoulder_pitch_joint:    
        - right_upper_arm_upper_link

    right_shoulder_yaw_joint:    
        - right_upper_arm_lower_link

    right_elbow_pitch_joint:    
        - right_forearm_upper_link

    right_elbow_yaw_joint:    
        - right_wrist_upper_link
```

Par défaut, le mode admittance et le mode d'afficahge (debug) sont désactivés. Il est possible d'activer ces options avec la reconfiguration dynamique. Il faut lancer cette commande:
```sh
rosrun rqt_reconfigure rqt_reconfigure
```
ou
```sh
rqt
```

Les coefficients de la fonction de transfert peuvent aussi être modifiés par la reconfiguration dynamique. À la modification, les changements seront sauvegardés dans le fichier de configuration filter_admittance.yaml.

Finalement, il est possible de regénérer la documentation DOxygen en utilisant cette commande en se situant dans la racine du répertoire wm_admittance:
```sh
 rosdoc_lite .
```
Une documentation a déjà été générée dans le dossier **/doc** au préalable.
