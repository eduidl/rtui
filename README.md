# rtui

[![PyPI - Version](https://img.shields.io/pypi/v/rtui-app)](https://pypi.org/project/rtui-app/)

rtui is ROS Terminal User Interface

## Support

- Python
  - 3.8+
- ROS1
  - noetic
- ROS2
  - humble
  - ironic

## Install

> **Note**  
> Package name is changed because rtui is already used by other package.  
> If you have already installed rtui, please uninstall it and install rtui-app.

Via [pipx](https://github.com/pypa/pipx) (Recommended)

```sh-session
$ pipx install rtui-app
```

Pip

```sh-session
$ pip3 install --user rtui-app
```

## Demo

[demo](https://github.com/eduidl/rtui/assets/25898373/901f58a8-98f6-4f23-82d6-404d15d5f35b)

## Usage

```
Usage: rtui [OPTIONS] COMMAND [ARGS]...

  Terminal User Interface for ROS User

Options:
  --help  Show this message and exit.

Commands:
  action   Inspect ROS actions
  node     Inspect ROS nodes (default)
  service  Inspect ROS services
  topic    Inspect ROS topics
  type     Inspect ROS types
```

- node/topic/service/action/type
  - get a list of nodes, topics, or etc.
  - get an information about specific node, topic, or etc.
  - mouse operation
    - click link of a node, a topic, or etc.
  - keyboard operation
    - `b/f`: Trace history backward and forward
    - `r`: Once more get list of nodes, topics or etc.
    - `q`: Terminate app
