from __future__ import annotations

from os import environ
from typing import Generator

from .entity import (
    ActionInfo,
    ActionTypeInfo,
    MsgTypeInfo,
    NodeInfo,
    RosEntity,
    RosEntityInfo,
    RosEntityType,
    ServiceInfo,
    SrvTypeInfo,
    TopicInfo,
    TreeKey,
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

    def available(self, entity_type: RosEntityType) -> bool:
        if entity_type in (RosEntityType.Action, RosEntityType.ActionType):
            return self.interface.version() == RosVersion.ROS2
        else:
            return True

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

    def get_msg_type_info(self, msg_type: str) -> MsgTypeInfo:
        return MsgTypeInfo(
            name=msg_type,
            topics=self.interface.list_topics(msg_type),
        )

    def get_srv_type_info(self, srv_type: str) -> SrvTypeInfo:
        return SrvTypeInfo(
            name=srv_type,
            services=self.interface.list_services(srv_type),
        )

    def get_action_type_info(self, action_type: str) -> ActionTypeInfo:
        return ActionTypeInfo(
            name=action_type,
            actions=self.interface.list_actions(action_type),
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
        elif entity.type == RosEntityType.MsgType:
            return self.get_msg_type_info(entity.name)
        elif entity.type == RosEntityType.SrvType:
            return self.get_srv_type_info(entity.name)
        elif entity.type == RosEntityType.ActionType:
            return self.get_action_type_info(entity.name)
        else:
            raise ValueError(f"invalid entity type: {entity.type}")

    def get_type_definition(self, entity: RosEntity) -> str:
        if entity.type == RosEntityType.MsgType:
            return self.interface.get_msg_definition(entity.name)
        elif entity.type == RosEntityType.SrvType:
            return self.interface.get_srv_definition(entity.name)
        elif entity.type == RosEntityType.ActionType:
            return self.interface.get_action_definition(entity.name)
        else:
            raise ValueError(f"invalid entity type: {entity.type}")

    @staticmethod
    def __common_list_entities(entities: list[str]) -> Generator[TreeKey, None, None]:
        for entity in entities:
            items = entity.split("/", 2)
            if len(items) < 2:
                raise ValueError(f"invalid entity name: {entity}")

            assert items[0] == ""
            if len(items) == 2:
                yield TreeKey(name=entity)
            else:
                yield TreeKey(name=f"/{items[2]}", group=f"/{items[1]}")

    def list_nodes(self) -> list[TreeKey]:
        return list(self.__common_list_entities(self.interface.list_nodes()))

    def list_topics(self) -> list[TreeKey]:
        return list(self.__common_list_entities(self.interface.list_topics()))

    def list_services(self) -> list[TreeKey]:
        return list(self.__common_list_entities(self.interface.list_services()))

    def list_actions(self) -> list[TreeKey]:
        return list(self.__common_list_entities(self.interface.list_actions()))

    @staticmethod
    def __common_list_types(types: list[str]) -> Generator[TreeKey, None, None]:
        for type in types:
            items = type.split("/")
            yield TreeKey(name=f"/{items[-1]}", group="/".join(items[:-1]))

    def list_msg_types(self) -> list[TreeKey]:
        return list(self.__common_list_types(self.interface.list_msg_types()))

    def list_srv_types(self) -> list[TreeKey]:
        return list(self.__common_list_types(self.interface.list_srv_types()))

    def list_action_types(self) -> list[TreeKey]:
        return list(self.__common_list_types(self.interface.list_action_types()))

    def list_entities(self, entity_type: RosEntityType) -> list[str]:
        if entity_type == RosEntityType.Node:
            return self.list_nodes()
        elif entity_type == RosEntityType.Topic:
            return self.list_topics()
        elif entity_type == RosEntityType.Service:
            return self.list_services()
        elif entity_type == RosEntityType.Action:
            return self.list_actions()
        elif entity_type == RosEntityType.MsgType:
            return self.list_msg_types()
        elif entity_type == RosEntityType.SrvType:
            return self.list_srv_types()
        elif entity_type == RosEntityType.ActionType:
            return self.list_action_types()
        else:
            raise ValueError(f"invalid entity type: {entity_type}")
