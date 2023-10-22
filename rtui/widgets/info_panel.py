from __future__ import annotations

from textual.widgets import Static

from ..event import RosEntitySelected
from ..ros import RosEntity, RosInterface
from ..ros.exception import RosMasterException


class RosEntityInfoPanel(Static):
    _ros: RosInterface
    _entity: RosEntity | None = None
    _update_interval: float | None = None

    DEFAULT_CSS = """
    RosEntityInfoPanel {
        padding: 1;
    }
    """

    def __init__(
        self,
        ros: RosInterface,
        entity: RosEntity | None = None,
        update_interval: float | None = None,
        *,
        name: str | None = None,
        id: str | None = None,
        classes: str | None = None,
        disabled: bool = False,
    ) -> None:
        super().__init__(
            name=name,
            id=id,
            classes=classes,
            disabled=disabled,
        )

        self._ros = ros
        self._entity = entity
        self._update_interval = update_interval

    def on_mount(self) -> None:
        self.update_info()
        if self._update_interval is not None:
            self.set_interval(self._update_interval, self.update_info)

    def set_entity(self, entity: RosEntity) -> None:
        if entity != self._entity:
            self._entity = entity
            self.update_info()

    def update_info(self):
        if self._entity is None:
            self.update("")
            return

        try:
            info = self._ros.get_entity_info(self._entity).to_textual()
        except RosMasterException as e:
            info = f"[b][red]Fail to communicate to master[/][/]\n{e}"
        except Exception as e:
            info = f"[b][red]Fail to get information of {self._entity.name}[/][/]\n{e}"

        self.update(info)

    def action_node_link(self, name: str) -> None:
        self.post_message(RosEntitySelected.new_node(name))

    def action_topic_link(self, name: str) -> None:
        self.post_message(RosEntitySelected.new_topic(name))

    def action_service_link(self, name: str) -> None:
        self.post_message(RosEntitySelected.new_service(name))

    def action_action_link(self, name: str) -> None:
        self.post_message(RosEntitySelected.new_action(name))
