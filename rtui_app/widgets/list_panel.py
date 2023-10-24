from __future__ import annotations

from textual.app import ComposeResult
from textual.widgets import Static, Tree

from ..event import RosEntitySelected
from ..ros import RosClient, RosEntityType


class RosEntityListPanel(Static):
    _ros: RosClient
    _entity_type: RosEntityType
    _tree: Tree[str]

    def __init__(
        self,
        ros: RosClient,
        entity_type: RosEntityType,
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
        self._entity_type = entity_type
        self._tree = Tree(entity_type.name)
        self._tree.auto_expand = True
        self.update_items()

    def update_items(self) -> None:
        self._tree.clear()

        entities = self._ros.list_entities(self._entity_type)
        groups = {e.group for e in entities if e.group is not None}
        parents = {None: self._tree.root}

        for group in sorted(groups):
            parents[group] = self._tree.root.add(group)

        for entity in entities:
            parents[entity.group].add_leaf(entity.name, entity.full_name)

    def compose(self) -> ComposeResult:
        yield self._tree

    def on_tree_node_selected(self, e: Tree.NodeSelected[str]) -> None:
        if e.node.is_root or e.node.data is None:
            return
        self.post_message(RosEntitySelected(self._entity_type, e.node.data))
