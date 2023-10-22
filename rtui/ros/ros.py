from __future__ import annotations

import typing as t
from abc import ABC, abstractmethod
from datetime import datetime
from os import environ

from .entity import ActionInfo, NodeInfo, ServiceInfo, TopicInfo

SEPARATOR = "/"
UNKNOWN_TYPE = "<unknown type>"


class RosInterface(ABC):
    @abstractmethod
    def terminate(self) -> None:
        ...

    @abstractmethod
    def now(self) -> datetime:
        ...

    @abstractmethod
    def list_nodes(self) -> list[str]:
        ...

    @abstractmethod
    def list_topics(self) -> list[str]:
        ...

    @abstractmethod
    def list_services(self) -> list[str]:
        ...

    @abstractmethod
    def list_actions(self) -> list[str]:
        ...

    @abstractmethod
    def get_node_info(self, node_name: str) -> NodeInfo:
        ...

    @abstractmethod
    def get_topic_info(self, topic_name: str) -> TopicInfo:
        ...

    @abstractmethod
    def get_service_info(self, service_name: str) -> ServiceInfo:
        ...

    @abstractmethod
    def get_action_info(self, action_name: str) -> ActionInfo:
        ...


def is_ros1() -> bool:
    return environ.get("ROS_VERSION") == "1"


def is_ros2() -> bool:
    return environ.get("ROS_VERSION") == "2"


def get_ros_cls() -> t.Type["RosInterface"]:
    ros_version = environ.get("ROS_VERSION")

    if ros_version == "1":
        from .ros1 import Ros1

        return Ros1
    elif ros_version == "2":
        from .ros2 import Ros2

        return Ros2
    elif ros_version is None:
        raise RuntimeError(
            "ROS_VERSION is not set. Please source /opt/ros/<ROS distro>/setup.bash etc."
        )
    else:
        raise RuntimeError(f"unknonw ROS version: {ros_version}")


def init_ros() -> "RosInterface":
    cls = get_ros_cls()
    return cls()
