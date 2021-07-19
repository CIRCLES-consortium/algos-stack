# ONNX2ROS

This is a lightweight ROS node that executes an ONNX neural network model of 
three inputs and a single output.
There are two modes of the node: a prompt mode and a synchronous mode.

As of now, only the prompt mode is supported in the current RAV4 ROS application.

## Prompt Mode (Default)

The default prompt mode of the node subscribes to three topics:
+ `/vel`: velocity of ego vehicle of type `geometry_msgs/Twist`
+ `/leader_vel`: velocity of lead vehicle of type `geometry_msgs/Twist`
+ `/headway_est`: velocity of ego vehicle of type `std_msgs/Float64`

It publishes to the following topic at 20 Hz:
+ `/v_des`: desired velocity of type `std_msgs/Float64`

To test an `example.onnx` model with input `/vel = x`, `/leader_vel = y`
and `/headway_est = z`, use the following launch file:
```angular2html
<launch>
    <param name="model" type="string" value="path/to/example.onnx"/>
    <param name="mode" type="string" value="prompt"/>
    <param name="v" type="double" value="x"/>
    <param name="lv" type="double" value="y"/>
    <param name="h" type="double" value="z"/>
    <node pkg="onnx2ros" type="prompt_mode" name="controller" output="screen"/>
    <node pkg="onnx2ros" type="mock_sensors.py" name="sensors" output="screen"/>
</launch>

```
See `launch/test_prompt_mode.launch` for a sample use case.

## Synchronous Mode (Experimental)

The synchronous mode utilizes built-in synchronization feature in 
`messsage_filters`.
However, it requires inputs of types that include timestamps. 

The synchronous mode of the node subscribes to three topics:
+ `/vel`: velocity of ego vehicle of type `geometry_msgs/TwistStamped`
+ `/leader_vel`: velocity of lead vehicle of type `geometry_msgs/TwistStamped`
+ `/headway_est`: velocity of ego vehicle of type `geometry_msgs/TwistStamped`

It publishes to the following topic at 20 Hz:
+ `/v_des`: desired velocity of type `geometry_msgs/TwistStamped`

The interface is mostly identical to prompt mode.
See `launch/test_sync_mode.launch` for a sample use case.

## Python Reader

The python script `script/ort_reader.py` can be used to 
quickly check for correct exporting of a neural network model to ONNX format.
To run the script on an `example.onnx` model with input `/vel = x`, 
`/leader_vel = y` and `/headway_est = z`, run
```angular2html
python ort_reader.py -m path/to/example.onnx -v x -lv y -s z
```
