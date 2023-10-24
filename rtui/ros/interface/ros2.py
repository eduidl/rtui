from __future__ import annotations

import typing as t
from pathlib import Path
from threading import Thread
from time import sleep

import rclpy
import ros2action.api
import ros2node.api
import ros2service.api
import ros2topic.api
from rclpy.action import (
    get_action_client_names_and_types_by_node,
    get_action_names_and_types,
    get_action_server_names_and_types_by_node,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py import (
    get_action_interfaces,
    get_interface_path,
    get_message_interfaces,
    get_service_interfaces,
)

from .base import RosInterface, RosVersion


def _get_full_path(namespace: str, name: str) -> str:
    if namespace == "/":
        return f"/{name}"
    else:
        return f"{namespace}/{name}"


def _flatten_node_info(
    entities: list[tuple[t.Any]],
) -> t.Generator[tuple[str, str | None]]:
    for entity in entities:
        if not entity.types:
            yield entity.name, None
        for type_ in entity.types:
            yield entity.name, type_


def _flatten_name_types(
    name_types: tuple[str, list[str]]
) -> t.Generator[tuple[str, str], None, None]:
    for name, types in name_types:
        if not types:
            yield name, None
        else:
            for type_ in types:
                yield name, type_


def _list_types_common(interfaces: dict[str, list[str]]) -> list[str]:
    full_types = []
    for package, type_names in interfaces.items():
        for type_name in type_names:
            full_types.append(f"{package}/{type_name}")

    return sorted(full_types)


class Ros2(RosInterface):
    node: Node

    def __init__(self, start_parameter_services: bool = False) -> None:
        rclpy.init()
        self.node = rclpy.create_node(
            "_rtui",
            enable_rosout=False,
            start_parameter_services=start_parameter_services,
            parameter_overrides=[],
        )

        Node.get_action_names_and_types = get_action_names_and_types
        Node.get_action_server_names_and_types_by_node = (
            get_action_server_names_and_types_by_node
        )
        Node.get_action_client_names_and_types_by_node = (
            get_action_client_names_and_types_by_node
        )

        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        self.thread = Thread(target=executor.spin, daemon=True)
        self.thread.start()

        sleep(0.01)

        super().__init__()

    def terminate(self) -> None:
        rclpy.shutdown()
        self.thread.join()

    @classmethod
    def version(_cls) -> RosVersion:
        return RosVersion.ROS2

    def get_node_publishers(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_publisher_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_node_subscribers(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_subscriber_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_node_service_servers(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_service_server_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_node_service_clients(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_service_client_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_node_action_servers(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_action_server_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_node_action_clients(self, node_name: str) -> list[tuple[str, str | None]]:
        return list(
            _flatten_node_info(
                ros2node.api.get_action_client_info(
                    node=self.node, remote_node_name=node_name
                )
            )
        )

    def get_topic_types(self, topic_name: str) -> list[str]:
        names_and_types: list[
            tuple[str, list[str]]
        ] = ros2topic.api.get_topic_names_and_types(
            node=self.node, include_hidden_topics=True
        )
        for name, types in names_and_types:
            if name == topic_name:
                return types

        return []

    def get_topic_publishers(self, topic_name: str) -> list[tuple[str, str]]:
        pubs = self.node.get_publishers_info_by_topic(topic_name)
        return list(
            (_get_full_path(comm.node_namespace, comm.node_name), comm.topic_type)
            for comm in pubs
        )

    def get_topic_subscribers(self, topic_name: str) -> list[tuple[str, str]]:
        subs = self.node.get_subscriptions_info_by_topic(topic_name)

        return list(
            (_get_full_path(comm.node_namespace, comm.node_name), comm.topic_type)
            for comm in subs
        )

    def get_service_types(self, service_name: str) -> list[str]:
        names_and_types = ros2service.api.get_service_names_and_types(
            node=self.node, include_hidden_services=True
        )
        for name, types in names_and_types:
            if name == service_name:
                return types

        return []

    def get_service_servers(self, service_name: str) -> None:
        """
        Unsupported for ROS2 because of the lack of API.
        """
        return None

    def get_action_types(self, action_name: str) -> list[str]:
        names_and_types = ros2action.api.get_action_names_and_types(node=self.node)
        for name, types in names_and_types:
            if name == action_name:
                return types

        return []

    def get_action_servers(self, action_name: str) -> list[str]:
        _, servers = ros2action.api.get_action_clients_and_servers(
            node=self.node, action_name=action_name
        )
        return list(_flatten_name_types(servers))

    def get_action_clients(self, action_name: str) -> list[str]:
        clients, _ = ros2action.api.get_action_clients_and_servers(
            node=self.node, action_name=action_name
        )
        return list(_flatten_name_types(clients))

    @staticmethod
    def __common_get_type_definition(type: str) -> str:
        return Path(get_interface_path(type)).read_text()

    def get_msg_definition(self, msg_type: str) -> str:
        return self.__common_get_type_definition(msg_type)

    def get_srv_definition(self, msg_type: str) -> str:
        return self.__common_get_type_definition(msg_type)

    def get_action_definition(self, msg_type: str) -> str:
        return self.__common_get_type_definition(msg_type)

    def list_nodes(self) -> list[str]:
        nodes = ros2node.api.get_node_names(node=self.node)
        return sorted({node.full_name for node in nodes})

    def list_topics(self, type: str | None = None) -> list[str]:
        topics = ros2topic.api.get_topic_names_and_types(node=self.node)
        return sorted({name for name, types in topics if type is None or type in types})

    def list_services(self, type: str | None = None) -> list[str]:
        services = ros2service.api.get_service_names_and_types(node=self.node)
        names = sorted(
            {name for name, types in services if type is None or type in types}
        )
        return names

    def list_actions(self, type: str | None = None) -> list[str]:
        actions = ros2action.api.get_action_names_and_types(node=self.node)
        names = sorted(
            {name for name, types in actions if type is None or type in types}
        )
        return names

    def list_msg_types(self) -> list[str]:
        return _list_types_common(get_message_interfaces())

    def list_srv_types(self) -> list[str]:
        return _list_types_common(get_service_interfaces())

    def list_action_types(self) -> list[str]:
        return _list_types_common(get_action_interfaces())
