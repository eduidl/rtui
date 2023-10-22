from __future__ import annotations

import sys
import typing as t
from datetime import datetime
from threading import Thread

import rosgraph
import rospy
import rosservice
from rosgraph import Master
from typing_extensions import TypeAlias

from .exception import RosMasterException
from .ros import NodeInfo, RosInterface, ServiceInfo, TopicInfo

_RosMasterEachSystemState: TypeAlias = t.List[t.Tuple[str, t.List[str]]]
_RosMasterSystemState: TypeAlias = t.Tuple[
    _RosMasterEachSystemState, _RosMasterEachSystemState, _RosMasterEachSystemState
]


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

    def terminate(self) -> None:
        rospy.signal_shutdown("exit")
        self.thread.join()

    def now(self) -> datetime:
        return datetime.fromtimestamp(rospy.Time.now().to_sec())

    def get_service_type(self, service: str) -> str | None:
        uri = self.master.lookupService(service)
        return rosservice.get_service_headers(service, uri).get("type", None)

    def list_nodes(self) -> list[str]:
        state = self.__get_system_state()
        nodes: list[str] = []
        for s in state:
            for _, l in s:
                nodes.extend((n for n in l if not n.startswith("/rtui_node")))
        return sorted(set(nodes))

    def list_topics(self) -> list[str]:
        pubs, subs, _ = self.__get_system_state()
        pub_names = {pub for pub, _ in pubs}
        sub_names = {sub for sub, _ in subs}
        names = sorted(pub_names.union(sub_names))
        return names

    def list_services(self) -> list[str]:
        _, _, srvs = self.__get_system_state()
        return sorted(s for s, _ in srvs if not s.startswith("/rtui_node"))

    def list_actions(self) -> t.NoReturn:
        raise NotImplementedError("ROS1 does not support")

    def get_node_info(self, node_name: str) -> NodeInfo:
        pubs, subs, srvs = self.__get_system_state()
        topic_types = self.__get_topic_type()

        return NodeInfo(
            name=node_name,
            publishers=[
                (topic, search_topic_type(topic, topic_types))
                for topic, nodes in pubs
                if node_name in nodes
            ],
            subscribers=[
                (topic, search_topic_type(topic, topic_types))
                for topic, nodes in subs
                if node_name in nodes
            ],
            service_servers=[
                (srv, self.get_service_type(srv))
                for srv, nodes in srvs
                if node_name in nodes
            ],
        )

    def get_topic_info(self, topic_name: str) -> TopicInfo:
        pubs, subs, _ = self.__get_system_state()
        topic_types = self.__get_topic_type()

        topic_type = search_topic_type(topic_name, topic_types)

        def _filter_topic(
            topics: _RosMasterEachSystemState, topic_name: str
        ) -> t.Generator[tuple[str, None], None, None]:
            for topic, nodes in topics:
                if topic != topic_name:
                    continue

                for node in nodes:
                    yield node, None

        return TopicInfo(
            name=topic_name,
            types=[topic_type] if topic_type else [],
            publishers=list(_filter_topic(pubs, topic_name)),
            subscribers=list(_filter_topic(subs, topic_name)),
        )

    def get_service_info(self, service_name: str) -> ServiceInfo:
        _, _, srvs = self.__get_system_state()

        service_type = self.get_service_type(service_name)

        def _filter_srv(
            srvs: _RosMasterEachSystemState, srv_name: str
        ) -> t.Generator[str, None, None]:
            for srv, nodes in srvs:
                if srv != srv_name:
                    continue

                for node in nodes:
                    yield node

        return ServiceInfo(
            name=service_name,
            types=[service_type] if service_type else [],
            servers=list(_filter_srv(srvs, service_name)),
        )

    def get_action_info(self, action_name: str) -> t.NoReturn:
        raise NotImplementedError("ROS1 does not support")


def search_topic_type(topic: str, topic_types: list[tuple[str, str]]) -> str | None:
    matches = [t_type for t_name, t_type in topic_types if t_name == topic]
    if matches and matches[0] != rosgraph.names.ANYTYPE:
        return matches[0]
    return None
