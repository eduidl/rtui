from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto


class RosVersion(Enum):
    ROS1 = auto()
    ROS2 = auto()


class RosInterface(ABC):
    @abstractmethod
    def terminate(self) -> None:
        ...

    @classmethod
    @abstractmethod
    def version(cls) -> RosVersion:
        ...

    @abstractmethod
    def get_node_publishers(self, node_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_node_subscribers(self, node_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_node_service_servers(self, node_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_node_service_clients(
        self, node_name: str
    ) -> list[tuple[str, str | None]] | None:
        ...

    @abstractmethod
    def get_node_action_servers(
        self, node_name: str
    ) -> list[tuple[str, str | None]] | None:
        ...

    @abstractmethod
    def get_node_action_clients(
        self, node_name: str
    ) -> list[tuple[str, str | None]] | None:
        ...

    @abstractmethod
    def get_topic_types(self, topic_name: str) -> list[str]:
        ...

    @abstractmethod
    def get_topic_publishers(self, topic_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_topic_subscribers(self, topic_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_service_types(self, service_name: str) -> list[str]:
        ...

    @abstractmethod
    def get_service_servers(self, service_name: str) -> list[tuple[str, str | None]]:
        ...

    @abstractmethod
    def get_action_types(self, action_name: str) -> list[str]:
        ...

    @abstractmethod
    def get_action_servers(self, action_name: str) -> list[tuple[str, str]]:
        ...

    @abstractmethod
    def get_action_clients(self, action_name: str) -> list[tuple[str, str]]:
        ...

    @abstractmethod
    def get_msg_definition(self, msg_type: str) -> str:
        ...

    @abstractmethod
    def get_srv_definition(self, srv_type: str) -> str:
        ...

    @abstractmethod
    def get_action_definition(self, action_type: str) -> str:
        ...

    @abstractmethod
    def list_nodes(self) -> list[str]:
        ...

    @abstractmethod
    def list_topics(self, type: str | None) -> list[str]:
        ...

    @abstractmethod
    def list_services(self, type: str | None) -> list[str]:
        ...

    @abstractmethod
    def list_actions(self, type: str | None) -> list[str]:
        ...

    @abstractmethod
    def list_msg_types(self) -> list[str]:
        ...

    @abstractmethod
    def list_srv_types(self) -> list[str]:
        ...

    @abstractmethod
    def list_action_types(self) -> list[str]:
        ...
