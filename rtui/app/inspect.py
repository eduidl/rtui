from __future__ import annotations

import warnings
from enum import Enum, auto

from textual.app import App
from textual.binding import Binding

from ..event import RosEntitySelected
from ..ros import RosEntity, RosEntityType, RosInterface
from ..screens import RosEntityInspection
from ..utility import History

warnings.simplefilter("ignore", ResourceWarning)


class InspectTarget(Enum):
    Nodes = auto()
    Topics = auto()
    Services = auto()
    Actions = auto()


class InspectApp(App):
    _ros: RosInterface
    _init_target: InspectTarget
    _history: History[RosEntity] = History(20)

    TITLE = "ROS Inspect"
    BINDINGS = [
        Binding("b", "back", "Prev Page", key_display="b"),
        Binding("f", "forward", "Next Page", key_display="f"),
        Binding("r", "reload", "Reload", key_display="r"),
        Binding("q", "quit", "Quit", key_display="q"),
    ]

    def __init__(
        self,
        ros: RosInterface,
        init_target: InspectTarget,
    ) -> None:
        super().__init__()

        self._ros = ros
        self._init_target = init_target
        self.add_mode("nodes", RosEntityInspection(ros, RosEntityType.Node))
        self.add_mode("topics", RosEntityInspection(ros, RosEntityType.Topic))
        self.add_mode("services", RosEntityInspection(ros, RosEntityType.Service))
        self.add_mode("actions", RosEntityInspection(ros, RosEntityType.Action))

    def show_ros_entity(self, entity: RosEntity, append_history: bool = True) -> None:
        if entity.type == RosEntityType.Node:
            self.switch_mode("nodes")
        elif entity.type == RosEntityType.Topic:
            self.switch_mode("topics")
        elif entity.type == RosEntityType.Service:
            self.switch_mode("services")
        elif entity.type == RosEntityType.Action:
            self.switch_mode("actions")
        else:
            return

        screen: RosEntityInspection = self.screen
        screen.set_entity_name(entity.name)

        if append_history:
            self._history.append(entity)

    def on_mount(self) -> None:
        self.switch_mode(self._init_target.name.lower())

    def action_forward(self) -> None:
        if entity := self._history.forward():
            self.show_ros_entity(entity, append_history=False)

    def action_back(self) -> None:
        if entity := self._history.back():
            self.show_ros_entity(entity, append_history=False)

    def action_reload(self) -> None:
        self.screen.force_update()

    async def action_quit(self) -> None:
        await super().action_quit()

    def on_ros_entity_selected(self, e: RosEntitySelected) -> None:
        self.show_ros_entity(e.entity)
