from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, auto


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
    def new_node(cls, path: str) -> "RosEntity":
        return cls(RosEntityType.Node, path)

    @classmethod
    def new_topic(cls, path: str) -> "RosEntity":
        return cls(RosEntityType.Topic, path)

    @classmethod
    def new_service(cls, path: str) -> "RosEntity":
        return cls(RosEntityType.Service, path)

    @classmethod
    def new_action(cls, path: str) -> "RosEntity":
        return cls(RosEntityType.Action, path)


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
