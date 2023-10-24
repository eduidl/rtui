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
    MsgType = auto()
    SrvType = auto()
    ActionType = auto()

    def has_definition(self) -> bool:
        return self in {
            RosEntityType.MsgType,
            RosEntityType.SrvType,
            RosEntityType.ActionType,
        }


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

    @classmethod
    def new_msg_type(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.MsgType, name)

    @classmethod
    def new_srv_type(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.SrvType, name)

    @classmethod
    def new_action_type(cls, name: str) -> "RosEntity":
        return cls(RosEntityType.ActionType, name)


@dataclass(frozen=True, order=True)
class TreeKey:
    name: str
    group: str | None = field(default=None, hash=False, compare=False)

    @property
    def full_name(self) -> str:
        if self.group is None:
            return self.name
        else:
            return f"{self.group}{self.name}"


class RosEntityInfo(ABC):
    @abstractmethod
    def to_textual(self) -> str:
        ...


def _common_link(name: str, callback: str) -> str:
    return f"[@click={callback}('{name}')]{name}[/]"


def _common_entities_with_type(
    entities: list[tuple[str, str | None]], callback: str, type_callback: str
) -> str:
    if not entities:
        return " None"

    out = ""
    for i, (name, type_) in enumerate(entities):
        if i > 0 and i % 5 == 0:
            out += "\n"

        out += f"\n  {_common_link(name, callback)}"
        if type_ is not None:
            out += f" \\[{_common_link(type_, type_callback)}]"

    return out


def _common_entities(entities: list[str], callback: str) -> str:
    if not entities:
        return " None"

    out = ""
    for entity in entities:
        out += f"\n  {_common_link(entity, callback)}"

    return out


def _common_types(types: list[str], callback: str) -> str:
    if not types:
        return UNKNOWN_TYPE

    return ", ".join(_common_link(type, callback) for type in types)


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

[b]Publishers:[/b]{_common_entities_with_type(self.publishers, "topic_link", "msg_type_link")}

[b]Subscribers:[/b]{_common_entities_with_type(self.subscribers, "topic_link", "msg_type_link")}

[b]Service Servers:[/b]{_common_entities_with_type(self.service_servers, "service_link", "srv_type_link")}
"""
        if self.service_clients is not None:
            tmp = _common_entities_with_type(
                self.service_clients, "service_link", "srv_type_link"
            )
            text += f"\n[b]Service Clients:[/b]{tmp}\n"

        if self.action_servers is not None:
            tmp = _common_entities_with_type(
                self.action_servers, "action_link", "action_type_link"
            )
            text += f"\n[b]Action Servers:[/b]{tmp}\n"

        if self.action_clients is not None:
            tmp = _common_entities_with_type(
                self.action_clients, "action_link", "action_type_link"
            )
            text += f"\n[b]Action Clients:[/b]{tmp}\n"

        return text


@dataclass(repr=True)
class TopicInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    publishers: list[tuple[str, str | None]] = field(default_factory=list)
    subscribers: list[tuple[str, str | None]] = field(default_factory=list)

    def to_textual(self) -> str:
        return f"""[b]Topic:[/b] {self.name}

[b]Type:[/b] {_common_types(self.types, "msg_type_link")}

[b]Publishers:[/b]{_common_entities_with_type(self.publishers, "node_link", "msg_type_link")}

[b]Subscribers:[/b]{_common_entities_with_type(self.subscribers, "node_link", "msg_type_link")}
"""


@dataclass(repr=True)
class ServiceInfo(RosEntityInfo):
    name: str
    types: list[str] = field(default_factory=list)
    servers: list[str, str | None] | None = None  # not support for ros2

    def to_textual(self) -> str:
        text = f"""[b]Service:[/b] {self.name}

[b]Type:[/b] {_common_types(self.types, "srv_type_link")}
"""

        if self.servers is not None:
            tmp = _common_entities_with_type(self.servers, "node_link", "srv_type_link")
            text += f"\n[b]Servers:[/b]{tmp}\n"

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

[b]Type:[/b] {_common_types(self.types, "action_type_link")}

[b]Action Servers:[/b]{_common_entities_with_type(self.servers, "node_link", "action_type_link")}

[b]Action Clients:[/b]{_common_entities_with_type(self.clients, "node_link", "action_type_link")}
"""


@dataclass(repr=True)
class MsgTypeInfo(RosEntityInfo):
    name: str
    topics: list[str] = field(default_factory=list)

    def to_textual(self) -> str:
        text = f"""[b]Type:[/b] {self.name}

[b]Topics:[/b]{_common_entities(self.topics, "topic_link")}
"""

        return text


@dataclass(repr=True)
class SrvTypeInfo(RosEntityInfo):
    name: str
    services: list[str] | None = None  # no support for ros1

    def to_textual(self) -> str:
        text = f"[b]Type:[/b] {self.name}"

        if self.services is not None:
            text += f"""\n[b]Services:[/b]{_common_entities(self.services, "service_link")}\n"""

        return text


# ROS2 only
@dataclass(repr=True)
class ActionTypeInfo(RosEntityInfo):
    name: str
    actions: list[str] = field(default_factory=list)

    def to_textual(self) -> str:
        return f"""[b]Type:[/b] {self.name}

[b]Services:[/b]{_common_entities(self.actions, "action_link")}
"""
