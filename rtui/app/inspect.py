from __future__ import annotations

import typing as t
import warnings
from enum import Enum, auto

from textual.app import App
from textual.driver import Driver
from textual.events import Mount, Resize
from textual.widgets import Footer, ScrollView, TreeClick, TreeControl

from ..event import RosEntityLinkClick
from ..ros import RosEntity, RosEntityType, RosInterface
from ..utility import History
from ..widgets import ActionView, NodeView, Separator, ServiceView, TopicEcho, TopicView

warnings.simplefilter("ignore", ResourceWarning)


class InspectMode(Enum):
    Nodes = auto()
    Topics = auto()
    Services = auto()
    Actions = auto()


class InspectApp(App):
    _mode: InspectMode = InspectMode.Nodes
    _ros: RosInterface
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

        await tree.root.expand()
        await self.sidebar.update(tree)

    async def show_node(self, node_name: str) -> None:
        await self.body.update(NodeView(self._ros, node_name))
        await self.stop_topic_echo()

    async def show_topic(self, topic_name: str) -> None:
        await self.body.update(TopicView(self._ros, topic_name))

        await self.stop_topic_echo()
        self._topic_view = TopicEcho(self._ros, topic_name)
        await self.echo_view.update(self._topic_view)

    async def stop_topic_echo(self) -> None:
        if self._topic_view:
            await self._topic_view.stop()
            self._topic_view = None

        await self.echo_view.update("")

    async def show_service(self, service_name: str) -> None:
        await self.body.update(ServiceView(self._ros, service_name))
        await self.stop_topic_echo()

    async def show_action(self, action_name: str) -> None:
        await self.body.update(ActionView(self._ros, action_name))
        await self.stop_topic_echo()

    async def on_load(self) -> None:
        await self.bind("b", "back", "Back")
        await self.bind("f", "forward", "Forward")
        await self.bind("r", "reload", "Reload")
        await self.bind("q", "quit", "Quit")

    async def on_mount(self, _event: Mount) -> None:
        self.sidebar = ScrollView(auto_width=True)
        self.body = ScrollView(auto_width=True)
        self.echo_view = ScrollView(auto_width=True)

        await self.view.dock(Footer(), edge="bottom")
        await self.view.dock(
            self.sidebar,
            edge="left",
            size=(self.console.width - 2) // 3,
        )
        await self.view.dock(
            Separator(),
            edge="left",
            size=1,
        )
        await self.view.dock(
            self.body,
            edge="top",
            size=(self.console.height - 2) // 3,
        )
        await self.view.dock(
            Separator(horizontal=True),
            edge="top",
            size=1,
        )
        await self.view.dock(
            self.echo_view,
            edge="top",
        )

        await self.show_list()

        def callback() -> None:
            self.body.window.widget.refresh(layout=True)

        self.set_interval(5.0, callback)

        def topic_view_callback() -> None:
            if self._topic_view:
                self._topic_view.refresh()

        self.set_interval(0.1, topic_view_callback)

    async def on_resize(self, e: Resize) -> None:
        self.sidebar.layout_size = (e.width - 2) // 3
        self.body.layout_size = (e.height - 2) // 2

    async def action_forward(self) -> None:
        entity = self._history.forward()
        if entity:
            await self.show_ros_entity(entity, append_history=False)

    async def action_back(self) -> None:
        entity = self._history.back()
        if entity:
            await self.show_ros_entity(entity, append_history=False)

    async def action_quit(self) -> None:
        await super().action_quit()

    async def action_reload(self) -> None:
        await self.show_list()

    async def handle_tree_click(self, message: TreeClick[RosEntity]) -> None:
        await self.show_ros_entity(message.node.data)

    async def handle_ros_entity_link_click(self, e: RosEntityLinkClick) -> None:
        await self.show_ros_entity(e.entity)
