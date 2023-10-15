from __future__ import annotations

import typing as t
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from threading import Lock

from rich.console import RenderableType
from rich.padding import PaddingDimensions
from rich.text import Text
from textual.geometry import Spacing
from textual.reactive import Reactive
from textual.widget import Widget

from ..ros import RosInterface


@dataclass
class MsgData:
    msg: t.Any
    received: datetime
    cache: str | None = None


class TopicEcho(Widget):
    ros: RosInterface
    topic_name: str
    sub: t.Any = None
    lock: Lock = Lock()
    msg_queue: deque[MsgData] = deque(maxlen=15)
    lines_max: Reactive[int] = Reactive(1, layout=True)

    def __init__(
        self,
        ros_itnerface: RosInterface,
        topic_name: str,
        padding: PaddingDimensions = (1, 1),
    ) -> None:
        self.ros = ros_itnerface
        self.topic_name = topic_name
        try:
            self.sub = self.ros.subscribe_topic(topic_name, self.enque_message)
        except Exception:
            pass
        super().__init__(name=f"echo: {topic_name}")
        self.padding = Spacing.unpack(padding)

    def enque_message(self, msg: t.Any) -> None:
        with self.lock:
            self.msg_queue.appendleft(MsgData(msg=msg, received=self.ros.now()))

    def render(self) -> RenderableType:
        if self.sub is None:
            return Text("Fail to subscribe!", style="red")

        def to_str(data: MsgData) -> str:
            if data.cache is None:
                data.cache = f"[{data.received}]{self.ros.format_msg(data.msg)}"

            return data.cache

        out = ""
        with self.lock:
            if self.msg_queue:
                out = "\n---\n".join((to_str(data) for data in self.msg_queue))

        self.lines_max = max(self.lines_max, out.count("\n") + 1)

        return Text(out)

    async def stop(self) -> None:
        if self.sub is not None:
            self.ros.unregister_subscriber(self.sub)
        self.msg_queue.clear()
