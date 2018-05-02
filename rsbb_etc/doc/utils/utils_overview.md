Utility Scripts Overview
=================================================

The rsbb_utils package provides the utility scripts specifically developed to acquire informations that are later used by the benchmark scripts.

The informations acquired by the scripts are usually saved in the rsbb_etc package's directory, so that the benchmark scripts can access to the generated files and load the informations.

The following is a brief description of the utility scripts currently installed:

## FBM1 Object Transform Acquirer

The script is executed with the following command
```bash
roslaunch rsbb_utils acquire_objects.launch
```

The utility script acquires the transform of the object used in FBM1: Object Perception.

The script reads the list of objects and their description from rsbb_etc/objects_informations/items_list.yaml, and for each object acquires the transform of the object relatively to the positioner.
The transform informations are saved in rsbb_etc/objects_informations/items_with_pose.yaml.

The acquired information is directly used by the FBM1: Object Perception benchmark script (benchmark code: HOPF), so after acquisition no other action is necessary.

## FBM2 Robot-Markerset Transform Acquirer

The script is executed with the following command
```bash
roslaunch rsbb_utils acquire_markerset.launch
```

The utility script acquires the transform of the markerset relative to the robot's odometric center, used in FBM2: Navigation.

The script prompts the team's name.
The transform informations are saved in rsbb_etc/transforms/transform-team_name.yaml.

The acquired information is directly used by the FBM2: Navigation benchmark script (benchmark code: HNF), so after acquisition no other action is necessary.
