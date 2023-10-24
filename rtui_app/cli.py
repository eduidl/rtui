from os import environ

import click

from .app import InspectApp
from .ros import RosClient, RosEntityType


def is_ros2() -> bool:
    return environ.get("ROS_VERSION") == "2"


def inspect_common(target: RosEntityType) -> None:
    ros = RosClient()
    try:
        app = InspectApp(ros=ros, init_target=target)
        app.run()
    finally:
        ros.terminate()


@click.command(help="Inspect ROS nodes (default)")
def node() -> None:
    inspect_common(RosEntityType.Node)


@click.command(hidden=True)
def nodes() -> None:
    inspect_common(RosEntityType.Node)


@click.command(help="Inspect ROS topics")
def topic() -> None:
    inspect_common(RosEntityType.Topic)


@click.command(hidden=True)
def topics() -> None:
    inspect_common(RosEntityType.Topic)


@click.command(help="Inspect ROS services")
def service() -> None:
    inspect_common(RosEntityType.Service)


@click.command(hidden=True)
def services() -> None:
    inspect_common(RosEntityType.Service)


@click.command(help="Inspect ROS actions")
def action() -> None:
    inspect_common(RosEntityType.Action)


@click.command(hidden=True)
def actions() -> None:
    inspect_common(RosEntityType.Action)


@click.group(help="Inspect ROS types")
def type() -> None:
    ...


@click.command(name="msg", help="Inspect ROS message types")
def type_msg() -> None:
    inspect_common(RosEntityType.MsgType)


@click.command(name="srv", help="Inspect ROS service types")
def type_srv() -> None:
    inspect_common(RosEntityType.SrvType)


@click.command(name="action", help="Inspect ROS action types")
def type_action() -> None:
    inspect_common(RosEntityType.ActionType)


@click.group(help="Terminal User Interface for ROS User", invoke_without_command=True)
@click.pass_context
def cli(ctx: click.Context) -> None:
    if ctx.invoked_subcommand is None:
        ctx.invoke(node)


def main() -> None:
    cli.add_command(node)
    cli.add_command(topic)
    cli.add_command(service)
    cli.add_command(type)
    type.add_command(type_msg)
    type.add_command(type_srv)

    # old
    cli.add_command(nodes)
    cli.add_command(topics)
    cli.add_command(services)

    if is_ros2():
        cli.add_command(action)
        type.add_command(type_action)

        # old
        cli.add_command(actions)

    cli()


if __name__ == "__main__":
    main()
