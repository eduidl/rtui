import click

from .app import InspectApp
from .ros import RosEntityType, init_ros, is_ros2


def inspect_common(target: RosEntityType) -> None:
    ros = init_ros()
    try:
        app = InspectApp(ros=ros, init_target=target)
        app.run()
    finally:
        ros.terminate()


@click.command(help="Inspect ROS nodes (default)")
def nodes() -> None:
    inspect_common(RosEntityType.Node)


@click.command(help="Inspect ROS topics")
def topics() -> None:
    inspect_common(RosEntityType.Topic)


@click.command(help="Inspect ROS services")
def services() -> None:
    inspect_common(RosEntityType.Service)


@click.command(help="Inspect ROS actions")
def actions() -> None:
    inspect_common(RosEntityType.Action)


@click.group(help="Terminal User Interface for ROS User", invoke_without_command=True)
@click.pass_context
def cli(ctx: click.Context) -> None:
    if ctx.invoked_subcommand is None:
        ctx.invoke(nodes)


def main() -> None:
    cli.add_command(nodes)
    cli.add_command(topics)
    cli.add_command(services)
    if is_ros2():
        cli.add_command(actions)
    cli()
    ...


if __name__ == "__main__":
    main()
