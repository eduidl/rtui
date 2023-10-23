from argparse import ArgumentParser

from textual.app import App, ComposeResult
from textual.containers import ScrollableContainer
from textual.widgets import Static

from rtui.event import RosEntitySelected
from rtui.ros import RosClient, RosEntity, RosEntityType
from rtui.widgets import RosEntityInfoPanel


class ExampleApp(App):
    ros: RosClient
    entity: RosEntity
    debug: Static = Static("")

    DEFAULT_CSS = """
    #upper {
        height: 50%;
        border-bottom: solid blue;
    }
    """

    def __init__(self, ros: RosClient, entity: RosEntity) -> None:
        self.ros = ros
        self.entity = entity
        super().__init__()

    def compose(self) -> ComposeResult:
        with ScrollableContainer(id="upper"):
            yield RosEntityInfoPanel(
                self.ros,
                self.entity,
                5.0,
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
    parser.add_argument("--node", help="Node name")
    parser.add_argument("--topic", help="Topic name")
    parser.add_argument("--service", help="Service name")
    parser.add_argument("--action", help="Action name")

    args = parser.parse_args()

    entity = None
    if args.node:
        entity = RosEntity.new_node(args.node)
    elif args.topic:
        entity = RosEntity.new_topic(args.topic)
    elif args.service:
        entity = RosEntity.new_service(args.service)
    elif args.action:
        entity = RosEntity.new_action(args.action)

    if entity is None:
        raise RuntimeError("No entity specified")

    ros = RosClient()
    ExampleApp(ros, entity).run()


if __name__ == "__main__":
    main()
