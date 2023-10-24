from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, ScrollableContainer, Vertical
from textual.screen import Screen
from textual.widgets import Footer

from .ros import RosClient, RosEntity, RosEntityType
from .widgets import RosEntityInfoPanel, RosEntityListPanel, RosTypeDefinitionPanel


class RosEntityInspection(Screen):
    _entity_type: RosEntityType
    _entity_name: str | None
    _list_panel: RosEntityListPanel
    _info_panel: RosEntityInfoPanel
    _definition_panel: RosTypeDefinitionPanel | None = None

    DEFAULT_CSS = """
    .container {
        height: 100%;
        background: $panel;
    }

    RosEntityListPanel {
        padding-left: 2;
        width: 30%;
    }

    #main {
        border-left: inner $primary;
    }

    #main-upper {
        border-bottom: inner $primary;
    }

    .main-half {
        height: 50%;
    }
    """

    def __init__(self, ros: RosClient, entity_type: RosEntityType) -> None:
        super().__init__()
        self._entity_type = entity_type
        self._entity_name = None
        self._list_panel = RosEntityListPanel(ros, entity_type)
        self._info_panel = RosEntityInfoPanel(
            ros,
            None,
            update_interval=5.0,
        )
        if entity_type.has_definition():
            self._definition_panel = RosTypeDefinitionPanel(ros)

    def set_entity_name(self, name: str) -> None:
        self._entity_name = name
        entity = RosEntity(type=self._entity_type, name=self._entity_name)
        self._info_panel.set_entity(entity)
        if self._definition_panel is not None:
            self._definition_panel.set_entity(entity)

    def force_update(self) -> None:
        self._list_panel.update_items()

    def compose(self) -> ComposeResult:
        yield Footer()
        with Horizontal(classes="container"):
            yield self._list_panel

            with Vertical(id="main"):
                if self._definition_panel is None:
                    with ScrollableContainer():
                        yield self._info_panel
                else:
                    with ScrollableContainer(id="main-upper", classes="main-half"):
                        yield self._info_panel
                    with ScrollableContainer(classes="main-half"):
                        yield self._definition_panel
