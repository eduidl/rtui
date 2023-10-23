from __future__ import annotations

from os import environ

from .entity import (
    ActionInfo,
    NodeInfo,
    RosEntity,
    RosEntityInfo,
    RosEntityType,
    ServiceInfo,
    TopicInfo,
)
from .interface import RosInterface, RosVersion


class RosClient:
    interface: RosInterface

    def __init__(self) -> None:
        ros_version = environ.get("ROS_VERSION")

        if ros_version == "1":
            from .interface.ros1 import Ros1

            self.interface = Ros1()
        elif ros_version == "2":
            from .interface.ros2 import Ros2

            self.interface = Ros2()
        elif ros_version is None:
            raise RuntimeError(
                "ROS_VERSION is not set. Please source /opt/ros/<ROS distro>/setup.bash etc."
            )
        else:
            raise RuntimeError(f"unknonw ROS version: {ros_version}")

    def is_ros1(self) -> bool:
        return self.interface.version() == RosVersion.ROS1

    def is_ros2(self) -> bool:
        return self.interface.version() == RosVersion.ROS2

    def terminate(self) -> None:
        self.interface.terminate()

    def get_node_info(self, node_name: str) -> NodeInfo:
        return NodeInfo(
            name=node_name,
            publishers=self.interface.get_node_publishers(node_name),
            subscribers=self.interface.get_node_subscribers(node_name),
            service_servers=self.interface.get_node_service_servers(node_name),
            service_clients=self.interface.get_node_service_clients(node_name),
            action_servers=self.interface.get_node_action_servers(node_name),
            action_clients=self.interface.get_node_action_clients(node_name),
        )

    def get_topic_info(self, topic_name: str) -> TopicInfo:
        return TopicInfo(
            name=topic_name,
            types=self.interface.get_topic_types(topic_name),
            publishers=self.interface.get_topic_publishers(topic_name),
            subscribers=self.interface.get_topic_subscribers(topic_name),
        )

    def get_service_info(self, service_name: str) -> ServiceInfo:
        return ServiceInfo(
            name=service_name,
            types=self.interface.get_service_types(service_name),
            servers=self.interface.get_service_servers(service_name),
        )

    def get_action_info(self, action_name: str) -> ActionInfo:
        return ActionInfo(
            name=action_name,
            types=self.interface.get_action_types(action_name),
            servers=self.interface.get_action_servers(action_name),
            clients=self.interface.get_action_clients(action_name),
        )

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

    def list_entities(self, entity_type: RosEntityType) -> list[str]:
        if entity_type == RosEntityType.Node:
            return self.interface.list_nodes()
        elif entity_type == RosEntityType.Topic:
            return self.interface.list_topics()
        elif entity_type == RosEntityType.Service:
            return self.interface.list_services()
        elif entity_type == RosEntityType.Action:
            return self.interface.list_actions()
        else:
            raise ValueError(f"unknown entity type: {entity_type}")
