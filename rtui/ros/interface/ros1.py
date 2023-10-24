from __future__ import annotations

import sys
import typing as t
from threading import Thread

import rosgraph
import rosmsg
import rospy
import rosservice
from rosgraph import Master
from rospkg import RosPack
from typing_extensions import TypeAlias

from ..exception import RosMasterException
from .base import RosInterface, RosVersion

_RosMasterEachSystemState: TypeAlias = t.List[t.Tuple[str, t.List[str]]]
_RosMasterSystemState: TypeAlias = t.Tuple[
    _RosMasterEachSystemState, _RosMasterEachSystemState, _RosMasterEachSystemState
]


def _search_topic_type(
    topic_types: list[tuple[str, str]], topic_name: str
) -> str | None:
    matches: list[str] = [
        t_type for t_name, t_type in topic_types if t_name == topic_name
    ]
    if matches and matches[0] != rosgraph.names.ANYTYPE:
        return matches[0]
    return None


def _search_nodes(
    topics: _RosMasterEachSystemState, topic_name: str
) -> t.Generator[tuple[str, None], None, None]:
    for topic, nodes in topics:
        if topic != topic_name:
            continue

        for node in nodes:
            yield node, None


T = t.TypeVar("T")


class Ros1(RosInterface):
    master: Master = Master("/rtui_node")
    thread: Thread

    def __init__(self, **_kwargs: t.Any) -> None:
        try:
            rospy.init_node("rtui_node", anonymous=True, disable_rosout=True)
        except rospy.exceptions.ROSInitException:
            print("Fail to initialize ROS node. Is master running?")
            sys.exit(1)
        self.thread = Thread(target=lambda: rospy.spin(), daemon=True)
        self.thread.start()

        super().__init__()

    def __wrap_master_exception(self, func: t.Callable[..., T]) -> T:
        try:
            return func()
        except Exception as e:
            raise RosMasterException(e)

    def __get_system_state(self) -> _RosMasterSystemState:
        return self.__wrap_master_exception(self.master.getSystemState)

    def __get_topic_type(self) -> list[tuple[str, str]]:
        return self.__wrap_master_exception(self.master.getTopicTypes)

    def __get_service_type(self, service_name: str) -> str | None:
        uri = self.master.lookupService(service_name)
        return rosservice.get_service_headers(service_name, uri).get("type", None)

    def terminate(self) -> None:
        rospy.signal_shutdown("exit")
        self.thread.join()

    @classmethod
    def version(_cls) -> RosVersion:
        return RosVersion.ROS1

    def get_node_publishers(self, node_name: str) -> list[tuple[str, str | None]]:
        pubs, _, _ = self.__get_system_state()
        topic_types = self.__get_topic_type()
        return list(
            (topic, _search_topic_type(topic_types, topic))
            for topic, nodes in pubs
            if node_name in nodes
        )

    def get_node_subscribers(self, node_name: str) -> list[tuple[str, str | None]]:
        _, subs, _ = self.__get_system_state()
        topic_types = self.__get_topic_type()
        return list(
            (topic, _search_topic_type(topic_types, topic))
            for topic, nodes in subs
            if node_name in nodes
        )

    def get_node_service_servers(self, node_name: str) -> list[tuple[str, str | None]]:
        _, _, srvs = self.__get_system_state()
        return list(
            (srv, self.__get_service_type(srv))
            for srv, nodes in srvs
            if node_name in nodes
        )

    def get_node_service_clients(self, node_name: str) -> None:
        return None

    def get_node_action_servers(self, node_name: str) -> None:
        return None

    def get_node_action_clients(self, node_name: str) -> None:
        return None

    def get_topic_types(self, topic_name: str) -> list[str]:
        topic_types = self.__get_topic_type()
        topic_type = _search_topic_type(topic_types, topic_name)
        return [topic_type] if topic_type else []

    def get_topic_publishers(self, topic_name: str) -> list[tuple[str, str | None]]:
        pubs, _, _ = self.__get_system_state()
        return list(_search_nodes(pubs, topic_name))

    def get_topic_subscribers(self, topic_name: str) -> list[tuple[str, str | None]]:
        _, subs, _ = self.__get_system_state()
        return list(_search_nodes(subs, topic_name))

    def get_service_types(self, service_name: str) -> list[str]:
        type = self.__get_service_type(service_name)
        return [type] if type else []

    def get_service_servers(self, service_name: str) -> list[str]:
        _, _, srvs = self.__get_system_state()
        return list(_search_nodes(srvs, service_name))

    def get_action_types(self, action_name: str) -> list[str]:
        raise NotImplementedError("ROS1 does not support action")

    def get_action_servers(self, action_name: str) -> list[str]:
        raise NotImplementedError("ROS1 does not support action")

    def get_action_clients(self, action_name: str) -> list[str]:
        raise NotImplementedError("ROS1 does not support action")

    def get_msg_definition(self, msg_type: str) -> str:
        return rosmsg.get_msg_text(msg_type)

    def get_srv_definition(self, srv_type: str) -> str:
        return rosmsg.get_srv_text(srv_type)

    def get_action_definition(self, action_type: str) -> str:
        raise NotImplementedError("ROS1 does not support action")

    def list_nodes(self) -> list[str]:
        state = self.__get_system_state()
        nodes: list[str] = []
        for s in state:
            for _, l in s:
                nodes.extend((n for n in l if not n.startswith("/rtui_node")))
        return sorted(set(nodes))

    def list_topics(self, type: str | None = None) -> list[str] | None:
        topic_types = self.__get_topic_type()
        names = sorted(
            topic for topic, types in topic_types if type is None or type in types
        )
        return names

    def list_services(self, type: str | None = None) -> list[str]:
        if type is not None:
            # ROS1 does not support filtering by type
            return None

        _, _, srvs = self.__get_system_state()
        return sorted(s for s, _ in srvs if not s.startswith("/rtui_node"))

    def list_actions(self) -> t.NoReturn:
        raise NotImplementedError("ROS1 does not support action")

    @staticmethod
    def __common_list_types(kind: str) -> t.Generator[str, None, None]:
        for p, dir in sorted(rosmsg.iterate_packages(RosPack(), f".{kind}")):
            for file in rosmsg._list_types(dir, kind, f".{kind}"):
                yield f"{p}/{file}"

    def list_msg_types(self) -> list[str]:
        return self.__common_list_types("msg")

    def list_srv_types(self) -> list[str]:
        return self.__common_list_types("srv")

    def list_action_types(self) -> t.NoReturn:
        raise NotImplementedError("ROS1 does not support action")
