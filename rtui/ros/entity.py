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


def _common_entity(entities: list[tuple[str, str | None]], callback: str) -> str:
    if not entities:
        return " None"

    out = ""
    for name, type_ in entities:
        out += f"\n [@click={callback}('{name}')]{name}[/]"
        if type_ is not None:
            out += f" \\[{type_}]"

    return out


@dataclass(repr=True)
class NodeInfo(RosEntityInfo):
    name: str
    publishers: list[tuple[str, str | None]]
    subscribers: list[tuple[str, str | None]]
    service_servers: list[tuple[str, str | None]]
    service_clients: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_servers: list[tuple[str, str | None]] | None = None  # not support for ros1
    action_clients: list[tuple[str, str | None]] | None = None  # not support for ros1

    def to_textual(self) -> str:
        text = f"""[b]Node:[/b] {self.name}

[b]Publishers:[/b]{_common_entity(self.publishers, "topic_link")}

[b]Subscribers:[/b]{_common_entity(self.subscribers, "topic_link")}

[b]Service Servers:[/b]{_common_entity(self.service_servers, "service_link")}
"""
        if self.service_clients is not None:
            text += f"\n[b]Service Clients:[/b]{_common_entity(self.service_clients, 'service_link')}\n"

        if self.action_servers is not None:
            text += f"\n[b]Action Servers:[/b]{_common_entity(self.action_servers, 'action_link')}\n"

        if self.action_clients is not None:
            text += f"\n[b]Action Clients:[/b]{_common_entity(self.action_clients, 'action_link')}\n"

        return text


@dataclass(repr=True)
class TopicInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    publishers: list[tuple[str, str | None]] = field(default_factory=list)
    subscribers: list[tuple[str, str | None]] = field(default_factory=list)

    def to_textual(self) -> str:
        return f"""[b]Topic:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}

[b]Publishers:[/b]{_common_entity(self.publishers, "node_link")}

[b]Subscribers:[/b]{_common_entity(self.subscribers, "node_link")}
"""


@dataclass(repr=True)
class ServiceInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[str, str | None] | None = None  # not support for ros2

    def to_textual(self) -> str:
        text = f"""[b]Service:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}
"""

        if self.servers is not None:
            text += f"\n[b]Servers:[/b]{_common_entity(self.servers, 'node_link')}\n"

        return text


# ROS2 only
@dataclass(repr=True)
class ActionInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[tuple[str, str]] = field(default_factory=list)
    clients: list[tuple[str, str]] = field(default_factory=list)

    def to_textual(self) -> str:
        return f"""[b]Action:[/b] {self.name}

[b]Type:[/b] {', '.join(self.types) or UNKNOWN_TYPE}

[b]Action Servers:[/b]{_common_entity(self.servers, "node_link")}

[b]Action Clients:[/b]{_common_entity(self.clients, "node_link")}
"""
