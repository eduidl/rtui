from __future__ import annotations

import typing as t
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from threading import Lock

from rich.console import RenderableType
from rich.padding import PaddingDimensions
from rich.text import Text
from textual.events import MouseMove
from textual.geometry import Spacing
from textual.reactive import Reactive
from textual.widget import Widget

from ..event import RosEntityLinkClick
from ..ros import RosEntity, RosInterface, TopicInfo


def text_from_topic_info(info: TopicInfo, hover_node: str) -> Text:
    def common(label: str, nodes: list[tuple[str, str | None]]) -> list[Text | str]:
        out: list[Text | str] = ["\n\n", Text(f"{label}:", style="bold")]
        if nodes:
            for node, type_ in nodes:
                link = Text(node)
                if node == hover_node:
                    link.stylize("underline")
                link.on(click=f"node_link('{node}')")
                link.apply_meta(dict(hover_node=node))
                out.extend(["\n  ", link])
                if type_ is not None:
                    out.append(f" [{type_}]")
        else:
            out.append(" None")

        return out

    return Text.assemble(
        Text("Topic:", style="bold"),
        f" {info.name}\n\n",
        Text("Type:", style="bold"),
        f" {', '.join(info.types) or '<unknown type>'}",
        *common("Publishers", info.publishers),
        *common("Subscriberes", info.subscribers),
        style="white",
        no_wrap=True,
        justify="left",
    )


class TopicView(Widget):
    ros: RosInterface
    topic_name: str
    hover_node: Reactive[str] = Reactive("", layout=True)

    def __init__(
        self,
        ros: RosInterface,
        topic_name: str,
        padding: PaddingDimensions = (1, 1),
    ) -> None:
        self.ros = ros
        self.topic_name = topic_name
        super().__init__(name=topic_name)
        self.padding = Spacing.unpack(padding)

    def render(self) -> RenderableType:
        info = self.ros.get_topic_info(self.topic_name)
        return text_from_topic_info(info, self.hover_node)

    async def on_mouse_move(self, event: MouseMove) -> None:
        self.hover_node = event.style.meta.get("hover_node", "")

    async def action_node_link(self, name: str) -> None:
        await self.post_message(RosEntityLinkClick(self, RosEntity.new_node(name)))


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
