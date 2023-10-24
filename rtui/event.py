from __future__ import annotations

from textual.message import Message

from .ros import RosEntity, RosEntityType


class RosEntitySelected(Message):
    entity: RosEntity

    def __init__(self, type: RosEntityType, name: str) -> None:
        super().__init__()
        self.entity = RosEntity(type=type, name=name)

    @classmethod
    def new_node(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.Node, name)

    @classmethod
    def new_topic(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.Topic, name)

    @classmethod
    def new_service(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.Service, name)

    @classmethod
    def new_action(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.Action, name)

    @classmethod
    def new_msg_type(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.MsgType, name)

    @classmethod
    def new_srv_type(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.SrvType, name)

    @classmethod
    def new_action_type(cls, name: str) -> "RosEntitySelected":
        return cls(RosEntityType.ActionType, name)
