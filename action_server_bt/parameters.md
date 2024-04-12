# Action Server Bt Parameters

Default Config
```yaml
action_server_bt:
  ros__parameters:
    action_name: action_server_bt
    behavior_tick_frequency: 100.0
    behavior_trees: '{}'
    groot2_port: 1667.0
    plugins: '{}'
    ros_plugins: '{}'

```

## action_name

The name the Action Server takes requests from

* Type: `string`
* Default Value: "action_server_bt"
* Read only: True

## behavior_tick_frequency

Frequency in Hz to tick() the Behavior tree at

* Type: `int`
* Default Value: 100
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## groot2_port

Server port value to publish Groot2 messages on

* Type: `int`
* Default Value: 1667
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## plugins

List of 'package_name/subfolder' containing BehaviorTree plugins to load into the factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## ros_plugins

List of 'package_name/subfolder' containing BehaviorTree ROS plugins to load into the factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## behavior_trees

List of 'package_name/subfolder' containing SubTrees to load into the BehaviorTree factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates
