from __future__ import annotations

import typing as t
import warnings
from enum import Enum, auto

from rich.padding import Padding
from rich.text import Text
from textual.app import App
from textual.driver import Driver
from textual.events import Mount
from textual.widgets import Footer, ScrollView, TreeClick, TreeControl

from ..event import RosEntityLinkClick
from ..ros import RosEntity, RosEntityType, RosInterface
from ..ros.exception import RosMasterException
from ..utility import History
from ..widgets import ActionView, NodeView, Separator, ServiceView, TopicEcho, TopicView

warnings.simplefilter("ignore", ResourceWarning)


class InspectMode(Enum):
    Nodes = auto()
    Topics = auto()
    Services = auto()
    Actions = auto()


class InspectApp(App):
    _ros: RosInterface
    _mode: InspectMode = InspectMode.Nodes
    _entity: RosEntity | None = None
    _history: History[RosEntity] = History(20)
    _topic_view: TopicEcho | None = None

    def __init__(
        self,
        ros: RosInterface,
        init_mode: InspectMode,
        screen: bool = True,
        driver_class: t.Type[Driver] | None = None,
        log: str = "",
        log_verbosity: int = 1,
        title: str = "Textual Application",
    ) -> None:
        self._ros = ros
        self._mode = init_mode

        super().__init__(
            screen=screen,
            driver_class=driver_class,
            log=log,
            log_verbosity=log_verbosity,
            title=title,
        )

    async def show_ros_entity(
        self, entity: RosEntity, append_history: bool = True
    ) -> None:
        prev_mode = self._mode

        await self.hide_topic_echo()

        self._entity = entity
        if entity.type == RosEntityType.Node:
            self._mode = InspectMode.Nodes
            await self.show_node(entity.name)
        elif entity.type == RosEntityType.Topic:
            self._mode = InspectMode.Topics
            await self.show_topic(entity.name)
        elif entity.type == RosEntityType.Service:
            self._mode = InspectMode.Services
            await self.show_service(entity.name)
        elif entity.type == RosEntityType.Action:
            self._mode = InspectMode.Actions
            await self.show_action(entity.name)

        if append_history:
            self._history.append(entity)

        if self._mode != prev_mode:
            await self.show_list()

    async def show_list(self) -> None:
        try:
            if self._mode == InspectMode.Nodes:
                tree = TreeControl("Nodes", RosEntity.new_dummy("/"))
                for name in self._ros.list_nodes():
                    await tree.add(tree.root.id, name, RosEntity.new_node(name))
            elif self._mode == InspectMode.Topics:
                tree = TreeControl("Topics", RosEntity.new_dummy("/"))
                for name in self._ros.list_topics():
                    await tree.add(tree.root.id, name, RosEntity.new_topic(name))
            elif self._mode == InspectMode.Services:
                tree = TreeControl("Services", RosEntity.new_dummy("/"))
                for name in self._ros.list_services():
                    await tree.add(tree.root.id, name, RosEntity.new_service(name))
            elif self._mode == InspectMode.Actions:
                tree = TreeControl("Actions", RosEntity.new_dummy("/"))
                for name in self._ros.list_actions():
                    await tree.add(tree.root.id, name, RosEntity.new_action(name))
            else:
                return
        except RosMasterException as e:
            await self.sidebar.update(
                Padding(
                    Text.assemble(
                        Text("Fail to communicate", style="red bold"), f"\n{e}"
                    ),
                    (1, 1),
                )
            )
            return

        await tree.root.expand()
        await self.sidebar.update(tree)

    async def show_node(self, node_name: str) -> None:
        await self.body.update(NodeView(self._ros, node_name))

    async def show_topic(self, topic_name: str) -> None:
        await self.body.update(TopicView(self._ros, topic_name))

    async def show_topic_echo(self) -> None:
        await self.hide_topic_echo()

        if self._entity:
            self._topic_view = TopicEcho(self._ros, self._entity.name)
            await self.echo_view.update(self._topic_view)
            self.horizontal.visible = True

            self.grid.place(
                main1=self.body,
                main2=self.horizontal,
                main3=self.echo_view,
            )

    async def hide_topic_echo(self) -> None:
        if self._topic_view:
            await self._topic_view.stop()
            self._topic_view = None
            self.horizontal.visible = False

            self.grid.place(main=self.body)

        await self.echo_view.update("")

    async def show_service(self, service_name: str) -> None:
        await self.body.update(ServiceView(self._ros, service_name))

    async def show_action(self, action_name: str) -> None:
        await self.body.update(ActionView(self._ros, action_name))

    async def on_load(self) -> None:
        await self.bind("b", "back", "Prev Page")
        await self.bind("f", "forward", "Next Page")
        await self.bind("r", "reload", "Reload")
        await self.bind("e", "toggle_echo", "Toggle Echo")
        await self.bind("q", "quit", "Quit")

    async def on_mount(self, _event: Mount) -> None:
        await self.view.dock(Footer(), edge="bottom")

        self.grid = await self.view.dock_grid(edge="left", name="left")

        self.grid.add_column(fraction=1, name="left")
        self.grid.add_column(size=1, name="vertical")
        self.grid.add_column(fraction=2, name="right")

        self.grid.add_row(fraction=1, name="top")
        self.grid.add_row(size=1, name="horizontal")
        self.grid.add_row(fraction=1, name="bottom")

        self.grid.add_areas(
            column="left,top-start|bottom-end",
            vertical="vertical,top-start|bottom-end",
            main="right,top-start|bottom-end",
            main1="right,top",
            main2="right,horizontal",
            main3="right,bottom",
        )

        self.sidebar = ScrollView(auto_width=True)
        self.body = ScrollView(auto_width=True)
        self.horizontal = Separator(horizontal=True)
        self.echo_view = ScrollView(auto_width=True)

        self.grid.place(
            column=self.sidebar,
            vertical=Separator(),
            main=self.body,
        )

        await self.show_list()

        def callback() -> None:
            self.body.window.widget.refresh(layout=True)

        self.set_interval(5.0, callback)

        def topic_view_callback() -> None:
            if self._topic_view:
                self._topic_view.refresh()

        self.set_interval(0.1, topic_view_callback)

    async def action_forward(self) -> None:
        entity = self._history.forward()
        if entity:
            await self.show_ros_entity(entity, append_history=False)

    async def action_back(self) -> None:
        entity = self._history.back()
        if entity:
            await self.show_ros_entity(entity, append_history=False)

    async def action_reload(self) -> None:
        await self.show_list()

    async def action_toggle_echo(self) -> None:
        if self._mode != InspectMode.Topics or self._entity is None:
            return

        if self._topic_view:
            await self.hide_topic_echo()
        else:
            await self.show_topic_echo()

    async def action_quit(self) -> None:
        await super().action_quit()

    async def handle_tree_click(self, message: TreeClick[RosEntity]) -> None:
        await self.show_ros_entity(message.node.data)

    async def handle_ros_entity_link_click(self, e: RosEntityLinkClick) -> None:
        await self.show_ros_entity(e.entity)
