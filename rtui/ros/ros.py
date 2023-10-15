from __future__ import annotations

import typing as t
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum, auto
from os import environ

DISPLAY_ARRAY_LENGTH_MAX = 15
SEPARATOR = "/"
UNKNOWN_TYPE = "<unknown type>"


class RosEntityType(IntEnum):
    Node = auto()
    Topic = auto()
    Service = auto()
    Action = auto()


@dataclass(frozen=True, order=True)
class RosEntity:
    type: RosEntityType
    name: str

    @classmethod
    def new_node(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.Node, name)

    @classmethod
    def new_topic(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.Topic, name)

    @classmethod
    def new_service(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.Service, name)

    @classmethod
    def new_action(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.Action, name)


class RosEntityInfo(ABC):
    @abstractmethod
    def to_textual(self) -> str:
        ...


@dataclass(repr=True)
class NodeInfo(RosEntityInfo):
    name: str
    publishers: list[tuple[str, str | None]]
    subscribers: list[tuple[str, str | None]]
    service_servers: list[tuple[str, str | None]]
    service_clients: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_servers: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_clients: list[tuple[str, str | None]] | None = None  # not support for ros1
    is_ros2: bool = True

    def to_textual(self) -> str:
        def common(
            values: list[tuple[str, str | None]] | list[tuple[str, str]] | None,
            callback: str,
        ) -> str:
            if not values:
                return " None"

            out = ""
            for name, type in values:
                out += f"\n  [@click={callback}('{name}')]{name}[/] \\[{type or '<unknown type>'}]"

            return out

        text = f"""[b]Node:[/b] {self.name}

[b]Publishers:[/b]{common(self.publishers, "topic_link")}

[b]Subscribers:[/b]{common(self.subscribers, "topic_link")}

[b]Service Servers:[/b]{common(self.service_servers, "service_link")}
"""
        if self.is_ros2:
            text += f"""
[b]Service Clients:[/b]{common(self.service_clients, "service_link")}

[b]Action Servers:[/b]{common(self.action_servers, "action_link")}

[b]Action Clients:[/b]{common(self.action_clients, "action_link")}
"""

        return text


@dataclass(repr=True)
class TopicInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    publishers: list[tuple[str, str | None]] = field(default_factory=list)
    subscribers: list[tuple[str, str | None]] = field(default_factory=list)

    def to_textual(self) -> str:
        def common(nodes: list[tuple[str, str | None]]) -> str:
            if not nodes:
                return " None"

            out = ""
            for node, type_ in nodes:
                out += f"\n  [@click=node_link('{node}')]{node}[/]"
                if type_ is not None:
                    out += f" [{type_}]"

            return out

        return f"""[b]Topic:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or '<unknown type>'}

[b]Publishers:[/b]{common(self.publishers)}

[b]Subscribers:[/b]{common(self.subscribers)}
"""


@dataclass(repr=True)
class ServiceInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[str] | None = None  # not support for ros2
    is_ros2: bool = True

    def to_textual(self) -> str:
        def common(nodes: list[str] | None) -> str:
            if not nodes:
                return " None"

            out = ""
            for node in nodes:
                out += f"\n  [@click=node_link('{node}')]{node}[/]"

            return out

        text = f"""[b]Service:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or '<unknown type>'}
"""

        if not self.is_ros2:
            text += f"""
[b]Servers:[/b]{common(self.servers)}
"""

        return text


# ROS2 only
@dataclass(repr=True)
class ActionInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[tuple[str, str]] = field(default_factory=list)
    clients: list[tuple[str, str]] = field(default_factory=list)

    def to_textual(self) -> str:
        def common(nodes: list[tuple[str, str]]) -> str:
            if not nodes:
                return " None"

            out = ""
            for node, type_ in nodes:
                out += f"\n  [@click=node_link('{node}')]{node}[/] \\[{type_}]"

            return out

        return f"""[b]Action:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or '<unknown type>'}

[b]Action Servers:[/b]{common(self.servers)}

[b]Action Clients:[/b]{common(self.clients)}
"""


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

    def get_entity_info(self, entity: RosEntity) -> RosEntityInfo:
        if entity.type == RosEntityType.Node:
            return self.get_node_info(entity.name)
        elif entity.type == RosEntityType.Topic:
            return self.get_topic_info(entity.name)
        elif entity.type == RosEntityType.Service:
            return self.get_service_info(entity.name)
        elif entity.type == RosEntityType.Action:
            return self.get_action_info(entity.name)
        else:
            raise ValueError(f"unknown entity type: {entity.type}")

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

    @abstractmethod
    def subscribe_topic(
        self, topic_name: str, callback: t.Callable[..., t.Any]
    ) -> t.Any:
        ...

    @abstractmethod
    def unregister_subscriber(self, sub: t.Any) -> None:
        ...

    @classmethod
    @abstractmethod
    def format_msg(cls, msg: t.Any, indent: int = 0) -> str:
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
