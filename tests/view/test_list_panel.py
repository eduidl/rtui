from argparse import ArgumentParser

from textual.app import App, ComposeResult
from textual.containers import ScrollableContainer
from textual.widgets import Static

from rtui.event import RosEntitySelected
from rtui.ros import RosEntityType, RosInterface, init_ros
from rtui.widgets import RosEntityListPanel


class ExampleApp(App):
    ros: RosInterface
    entity_type: RosEntityType
    debug: Static = Static("")

    DEFAULT_CSS = """
    #upper {
        height: 50%;
        border-bottom: solid blue;
    }
    """

    def __init__(self, ros: RosInterface, entity_type: RosEntityType) -> None:
        self.ros = ros
        self.entity_type = entity_type
        super().__init__()

    def compose(self) -> ComposeResult:
        with ScrollableContainer(id="upper"):
            yield RosEntityListPanel(
                self.ros,
                self.entity_type,
            )
        yield self.debug

    def on_ros_entity_selected(self, e: RosEntitySelected) -> None:
        if e.entity.type == RosEntityType.Node:
            self.debug.update(f"Node selected {e.entity.name}")
        elif e.entity.type == RosEntityType.Topic:
            self.debug.update(f"Topic selected {e.entity.name}")
        elif e.entity.type == RosEntityType.Service:
            self.debug.update(f"Service selected {e.entity.name}")
        elif e.entity.type == RosEntityType.Action:
            self.debug.update(f"Action selected {e.entity.name}")


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--type", choices=["node", "topic", "service", "action"], default="node"
    )

    args = parser.parse_args()

    entity_type = None
    if args.type == "node":
        entity_type = RosEntityType.Node
    elif args.type == "topic":
        entity_type = RosEntityType.Topic
    elif args.type == "service":
        entity_type = RosEntityType.Service
    elif args.type == "action":
        entity_type = RosEntityType.Action

    if entity_type is None:
        raise RuntimeError("No entity specified")

    ros = init_ros()
    ExampleApp(ros, entity_type).run()


if __name__ == "__main__":
    main()
