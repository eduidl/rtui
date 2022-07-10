from __future__ import annotations

import rich.repr
from textual._types import MessageTarget
from textual.message import Message

from .ros import RosEntity


@rich.repr.auto
class RosEntityLinkClick(Message, bubble=True):
    entity: RosEntity

    def __init__(self, sender: MessageTarget, entity: RosEntity) -> None:
        self.entity = entity
        super().__init__(sender)

    def __rich_repr__(self) -> rich.repr.Result:
        yield "entity", self.entity
