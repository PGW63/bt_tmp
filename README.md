# BehaviorTree.CPP

```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
```


https://github.com/BehaviorTree/BehaviorTree.CPP.git

# BehaviorTree.ROS2
```
  git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
```

https://github.com/BehaviorTree/BehaviorTree.ROS2.git

# Vosper 
Vosper.py is added. 

```
      def reset(self):
        """Reset Vosk recognizer state"""
        if self.vosk:
            self.vosk.Reset()
            self.recording_whisper = False
```


# How to use

We must turn these nodes

## Detector

```
  ros2 run task yolo2_node
```

## Nav2

```
  ros2 launch rby1_navigation carter_navigation.launch.py
```

## Semantic Navigator

```
  ros2 run rby1_custom_pkg semantic_navigator2
```

## describe person cation

```
  ros2 run task auto_vlm_captioner_node
```

## speak tts

```
  ros2 launch tts_bringup tts.launch.py
```

## Listen 

```
  ros2 launch vosper_ros vosper.launch.py
```

## Run BT

```
  ros2 run planning_tutorial bt_main_node
```


