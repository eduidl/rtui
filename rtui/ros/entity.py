from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum, auto

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
                out += f"\n  [@click={callback}('{name}')]{name}[/] \\[{type or UNKNOWN_TYPE}]"

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

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}

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

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}
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

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}

[b]Action Servers:[/b]{common(self.servers)}

[b]Action Clients:[/b]{common(self.clients)}
"""
