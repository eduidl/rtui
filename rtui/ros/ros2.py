from __future__ import annotations

import array
import typing as t
from datetime import datetime
from threading import Thread
from time import sleep

import numpy as np
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
from rclpy.qos import QoSPresetProfiles
from rclpy.subscription import Subscription
from rosidl_runtime_py.utilities import get_message

from .ros import (
    DISPLAY_ARRAY_LENGTH_MAX,
    ActionInfo,
    NodeInfo,
    RosInterface,
    ServiceInfo,
    TopicInfo,
)


def split_full_path(path: str) -> tuple[str, str]:
    splits = path.split("/")
    return "/".join(splits[:-1]), splits[-1]


def get_full_path(namespace: str, name: str) -> str:
    if namespace == "/":
        return f"/{name}"
    else:
        return f"{namespace}/{name}"


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

    def now(self) -> datetime:
        return datetime.fromtimestamp(self.node.get_clock().now().nanoseconds * 1e-9)

    def list_nodes(self) -> list[str]:
        nodes = ros2node.api.get_node_names(node=self.node)
        return sorted({node.full_name for node in nodes})

    def list_topics(self) -> list[str]:
        topics = ros2topic.api.get_topic_names_and_types(node=self.node)
        names = sorted({name for name, _ in topics})
        return names

    def list_services(self) -> list[str]:
        services = ros2service.api.get_service_names_and_types(node=self.node)
        names = sorted({name for name, _ in services})
        return names

    def list_actions(self) -> list[str]:
        actions = ros2action.api.get_action_names_and_types(node=self.node)
        names = sorted({name for name, _ in actions})
        return names

    def get_node_info(self, node_name: str) -> NodeInfo:
        kwargs = dict(node=self.node, remote_node_name=node_name)

        pubs = ros2node.api.get_publisher_info(**kwargs)
        subs = ros2node.api.get_subscriber_info(**kwargs)
        servers = ros2node.api.get_service_server_info(**kwargs)
        clients = ros2node.api.get_service_client_info(**kwargs)
        action_servers = ros2node.api.get_action_server_info(**kwargs)
        action_clients = ros2node.api.get_action_client_info(**kwargs)

        def flatten(topics: t.Any) -> t.Generator[tuple[str, str], None, None]:
            for topic in topics:
                if not topic.types:
                    yield topic.name, "<unknown type>"
                for type_ in topic.types:
                    yield topic.name, type_

        return NodeInfo(
            name=node_name,
            publishers=list(flatten(pubs)),
            subscribers=list(flatten(subs)),
            service_servers=list(flatten(servers)),
            service_clients=list(flatten(clients)),
            action_servers=list(flatten(action_servers)),
            action_clients=list(flatten(action_clients)),
        )

    def __get_topic_types(self, topic_name: str) -> list[str]:
        names_and_types: list[
            tuple[str, list[str]]
        ] = ros2topic.api.get_topic_names_and_types(
            node=self.node, include_hidden_topics=True
        )
        for name, types in names_and_types:
            if name == topic_name:
                return types

        return []

    def get_topic_info(self, topic_name: str) -> TopicInfo:
        pubs = self.node.get_publishers_info_by_topic(topic_name)
        subs = self.node.get_subscriptions_info_by_topic(topic_name)

        return TopicInfo(
            name=topic_name,
            types=self.__get_topic_types(topic_name),
            publishers=[
                (get_full_path(comm.node_namespace, comm.node_name), comm.topic_type)
                for comm in pubs
            ],
            subscribers=[
                (get_full_path(comm.node_namespace, comm.node_name), comm.topic_type)
                for comm in subs
            ],
        )

    def __get_service_types(self, service_name: str) -> list[str]:
        names_and_types = ros2service.api.get_service_names_and_types(
            node=self.node, include_hidden_services=True
        )
        for name, types in names_and_types:
            if name == service_name:
                return types

        return []

    def get_service_info(self, service_name: str) -> ServiceInfo:
        return ServiceInfo(
            name=service_name,
            types=self.__get_service_types(service_name),
        )

    def __get_action_types(self, action_name: str) -> list[str]:
        names_and_types = ros2action.api.get_action_names_and_types(node=self.node)
        for name, types in names_and_types:
            if name == action_name:
                return types

        return []

    def get_action_info(self, action_name: str) -> ActionInfo:
        clients, servers = ros2action.api.get_action_clients_and_servers(
            node=self.node, action_name=action_name
        )

        def flatten(
            name_types: list[tuple[str, list[str]]]
        ) -> t.Generator[tuple[str, str], None, None]:
            for name, types in name_types:
                if not types:
                    yield name, "<unknown type>"
                else:
                    for type_ in types:
                        yield name, type_

        return ActionInfo(
            name=action_name,
            types=self.__get_action_types(action_name),
            servers=list(flatten(servers)),
            clients=list(flatten(clients)),
        )

    def subscribe_topic(
        self, topic_name: str, callback: t.Callable[..., t.Any]
    ) -> Subscription | None:
        types = self.__get_topic_types(topic_name)
        if types:
            qos = QoSPresetProfiles.get_from_short_key("sensor_data")
            return self.node.create_subscription(
                get_message(types[0]), topic_name, callback, qos
            )
        else:
            return None

    def unregister_subscriber(self, sub: Subscription) -> None:
        self.node.destroy_subscription(sub)

    @classmethod
    def format_msg(cls, msg: t.Any, indent: int = 0) -> str:
        out = ""

        type_: str
        for field, type_ in msg.get_fields_and_field_types().items():
            val = getattr(msg, field)
            out += f"\n{' ' * indent}{field}:"
            if isinstance(val, (bool, bytes, int, float)):
                out += f" {val!r}"
            elif isinstance(val, str):
                out += f' "{val}"'
            elif isinstance(val, (list, array.array, np.ndarray)):
                length = len(val)
                if length == 0:
                    out += " []"
                elif length > DISPLAY_ARRAY_LENGTH_MAX:
                    if "[" in type_:
                        out += f' "<{type_}>"'
                    else:
                        out += f' "<{type_}, length: {length}>"'
                elif isinstance(val[0], (bool, bytes, int, float, str)):
                    out += f" {list(val)}"
                else:
                    for v in val:
                        out += f"\n{' ' * (indent + 2)}-"
                        out += cls.format_msg(v, indent + 4)
            else:
                out += cls.format_msg(val, indent + 2)

        return out
