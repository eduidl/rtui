from __future__ import annotations

import typing as t
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum, auto
from os import environ

SEPARATOR = "/"
UNKNOWN_TYPE = "<unknown type>"


class RosEntityType(IntEnum):
    Dummy = auto()
    Node = auto()
    Topic = auto()
    Service = auto()
    Action = auto()


@dataclass(frozen=True, order=True)
class RosEntity:
    name: str
    type: RosEntityType

    @classmethod
    def new_dummy(cls, name: str) -> "RosEntity":
        return cls(name, RosEntityType.Dummy)

    @classmethod
    def new_node(cls, path: str) -> "RosEntity":
        return cls(path, RosEntityType.Node)

    @classmethod
    def new_topic(cls, path: str) -> "RosEntity":
        return cls(path, RosEntityType.Topic)

    @classmethod
    def new_service(cls, path: str) -> "RosEntity":
        return cls(path, RosEntityType.Service)

    @classmethod
    def new_action(cls, path: str) -> "RosEntity":
        return cls(path, RosEntityType.Action)


@dataclass(repr=True)
class NodeInfo:
    name: str
    publishers: list[tuple[str, str | None]]
    subscribers: list[tuple[str, str | None]]
    service_servers: list[tuple[str, str | None]]
    service_clients: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_servers: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_clients: list[tuple[str, str | None]] | None = None  # not support for ros1


@dataclass(repr=True)
class TopicInfo:
    name: str
    types: list[str] = field(default_factory=list)
    publishers: list[tuple[str, str | None]] = field(default_factory=list)
    subscribers: list[tuple[str, str | None]] = field(default_factory=list)


@dataclass(repr=True)
class ServiceInfo:
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[str] | None = None  # not support for ros2


# ROS2 only
@dataclass(repr=True)
class ActionInfo:
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[tuple[str, str]] = field(default_factory=list)
    clients: list[tuple[str, str]] = field(default_factory=list)


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
