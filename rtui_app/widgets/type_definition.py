from __future__ import annotations

from textual.widgets import Static

from ..ros import RosClient, RosEntity


class RosTypeDefinitionPanel(Static):
    _ros: RosClient
    _entity: RosEntity | None = None

    DEFAULT_CSS = """
    RosTypeDefinitionPanel {
        padding: 1 2;
    }
    """

    def __init__(
        self,
        ros: RosClient,
        entity: RosEntity | None = None,
        *,
        name: str | None = None,
        id: str | None = None,
        classes: str | None = None,
    ) -> None:
        super().__init__(
            "",
            name=name,
            id=id,
            classes=classes,
        )

        self._ros = ros
        self._entity = entity
        self.update_content()

    def set_entity(self, entity: RosEntity) -> None:
        if entity != self._entity:
            self._entity = entity
            self.update_content()

    def update_content(self):
        if self._entity is None or not self._entity.type.has_definition():
            self.update("")
        else:
            self.update(self._ros.get_type_definition(self._entity))
