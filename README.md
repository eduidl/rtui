# rtui

rtui is ROS Terminal User Interface

## Support

- Python
  - 3.8+
  - Latest Poetry does not support Python 3.6
- ROS1
  - noetic
- ROS2
  - humble
  - ironic

## Install

Via [pipx](https://github.com/pypa/pipx) (Recommended)

```sh-session
$ pipx install git+https://github.com/eduidl/rtui.git
```

Pip

```sh-session
$ pip3 install --user git+https://github.com/eduidl/rtui.git
```

## Demo

![demo](doc/demo.gif)

## Usage

```
$ rtui --help
Usage: rtui [OPTIONS] COMMAND [ARGS]...

  Terminal User Interface for ROS User

Options:
  --help  Show this message and exit.

Commands:
  actions   Inspect ROS actions (ROS1 is not supported)
  nodes     Inspect ROS nodes
  services  Inspect ROS services
  topics    Inspect ROS topics
```

- nodes/topics/services/actions
  - get a list of nodes, topics, or etc.
  - get an information about specific node, topic, or etc.
  - mouse operation
    - click link of a node, a topic, or etc.
  - keyboard operation
    - `b/f`: Trace history backward and forward
    - `r`: Once more get list of nodes, topics or etc.
    - `e`: Start or finish topic subscriptions
    - `q`: Terminate app
