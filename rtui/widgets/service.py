from __future__ import annotations

from rich.console import RenderableType
from rich.padding import PaddingDimensions
from rich.text import Text
from textual.events import MouseMove
from textual.geometry import Spacing
from textual.reactive import Reactive
from textual.widget import Widget

from ..event import RosEntityLinkClick
from ..ros import RosEntity, RosInterface, ServiceInfo


def text_from_service_info(info: ServiceInfo, hover_node: str) -> Text:
    def common(label: str, nodes: list[str] | None) -> list[Text | str]:
        if nodes is None:
            return []

        out: list[Text | str] = [Text(f"\n\n{label}:", style="bold")]
        if nodes:
            for node in nodes:
                link = Text(node)
                if node == hover_node:
                    link.stylize("underline")
                link.on(click=f"node_link('{node}')")
                link.apply_meta(dict(hover_node=node))
                out.extend(["\n  ", link])
        else:
            out.append(" None")

        return out

    return Text.assemble(
        Text("Service:", style="bold"),
        f" {info.name}",
        Text("\n\nType:", style="bold"),
        f" {', '.join(info.types) or '<unknown type>'}",
        *common("Servers", info.servers),
        style="white",
        no_wrap=True,
        justify="left",
    )


class ServiceView(Widget):
    ros: RosInterface
    service_name: str
    hover_node: Reactive[str] = Reactive("", layout=True)

    def __init__(
        self,
        ros: RosInterface,
        service_name: str,
        padding: PaddingDimensions = (1, 1),
    ) -> None:
        self.ros = ros
        self.service_name = service_name
        super().__init__(name=self.service_name)
        self.padding = Spacing.unpack(padding)

    def render(self) -> RenderableType:
        info = self.ros.get_service_info(self.service_name)
        return text_from_service_info(info, self.hover_node)

    async def on_mouse_move(self, event: MouseMove) -> None:
        self.hover_node = event.style.meta.get("hover_node", "")

    async def action_node_link(self, name: str) -> None:
        await self.post_message(RosEntityLinkClick(self, RosEntity.new_node(name)))
