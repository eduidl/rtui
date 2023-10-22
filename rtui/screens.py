from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, ScrollableContainer
from textual.screen import Screen
from textual.widgets import Footer

from .ros import RosEntity, RosEntityType, RosInterface
from .widgets import RosEntityInfoPanel, RosEntityListPanel


class RosEntityInspection(Screen):
    _entity_type: RosEntityType
    _entity_name: str | None
    _list_panel: RosEntityListPanel
    _info_panel: RosEntityInfoPanel

    DEFAULT_CSS = """
    RosEntityListPanel {
        width: 30%;
        border-right: inner $primary;
    }
    """

    def __init__(self, ros: RosInterface, entity_type: RosEntityType) -> None:
        super().__init__()
        self._entity_type = entity_type
        self._entity_name = None
        self._list_panel = RosEntityListPanel(ros, entity_type)
        self._info_panel = RosEntityInfoPanel(
            ros,
            None,
            update_interval=5.0,
        )

    def set_entity_name(self, name: str) -> None:
        self._entity_name = name
        entity = RosEntity(type=self._entity_type, name=self._entity_name)
        self._info_panel.set_entity(entity)

    def force_update(self) -> None:
        self._list_panel.update_items()

    def toggle_echo(self) -> None:
        if self._entity_name is None or self._entity_type != RosEntityType.Topic:
            return

    def compose(self) -> ComposeResult:
        yield Footer()
        with Horizontal():
            yield self._list_panel
            with ScrollableContainer():
                yield self._info_panel
