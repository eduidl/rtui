from __future__ import annotations

from rich.console import RenderableType
from rich.padding import PaddingDimensions
from rich.text import Text
from textual.events import MouseMove
from textual.geometry import Spacing
from textual.reactive import Reactive
from textual.widget import Widget

from ..event import RosEntityLinkClick
from ..ros import NodeInfo, RosEntity, RosInterface
from ..ros.exception import RosMasterException


def text_from_node_info(info: NodeInfo, hover_entity: tuple[str, str]) -> Text:
    def common(
        label: str,
        values: list[tuple[str, str | None]] | list[tuple[str, str]] | None,
        callback: str,
    ) -> list[Text | str]:
        if values is None:
            return []

        out: list[Text | str] = ["\n\n", Text(f"{label}:", style="bold")]
        if values:
            for name, type in values:
                link = Text(name)
                if label == hover_entity[0] and name == hover_entity[1]:
                    link.stylize("underline")
                link.on(click=f"{callback}('{name}')")
                link.apply_meta(dict(hover_entity=(label, name)))
                out.extend(["\n  ", link, f" [{type or '<unknown type>'}]"])
        else:
            out.append(" None")

        return out

    return Text.assemble(
        Text("Node:", style="bold"),
        f" {info.name}",
        *common("Publishers", info.publishers, "topic_link"),
        *common("Subscribers", info.subscribers, "topic_link"),
        *common("Service Servers", info.service_servers, "service_link"),
        *common("Service Clients", info.service_clients, "service_link"),
        *common("Action Servers", info.action_servers, "action_link"),
        *common("Action Clients", info.action_clients, "action_link"),
        style="white",
        no_wrap=True,
        justify="left",
    )


class NodeView(Widget):
    ros: RosInterface
    node_name: str
    hover_entity: Reactive[tuple[str, str]] = Reactive(("", ""), layout=True)

    def __init__(
        self,
        ros: RosInterface,
        node_name: str,
        padding: PaddingDimensions = (1, 1),
    ) -> None:
        self.ros = ros
        self.node_name = node_name
        super().__init__(name=self.node_name)
        self.padding = Spacing.unpack(padding)

    def render(self) -> RenderableType:
        try:
            info = self.ros.get_node_info(self.node_name)
            return text_from_node_info(info, self.hover_entity)
        except RosMasterException as e:
            return Text.assemble(
                Text("Fail to communicate", style="red bold"), f"\n{e}"
            )

    async def on_mouse_move(self, event: MouseMove) -> None:
        self.hover_entity = event.style.meta.get("hover_entity", ("", ""))

    async def action_topic_link(self, name: str) -> None:
        await self.post_message(RosEntityLinkClick(self, RosEntity.new_topic(name)))

    async def action_service_link(self, name: str) -> None:
        await self.post_message(RosEntityLinkClick(self, RosEntity.new_service(name)))

    async def action_action_link(self, name: str) -> None:
        await self.post_message(RosEntityLinkClick(self, RosEntity.new_action(name)))
