import click

from .app import InspectApp, InspectMode
from .ros import init_ros, is_ros2


def inspect_common(mode: InspectMode) -> None:
    ros = init_ros()
    try:
        InspectApp.run(ros=ros, init_mode=mode, title="ROS Inspect")
    finally:
        ros.terminate()


@click.command(help="Inspect ROS nodes (default)")
def nodes() -> None:
    inspect_common(InspectMode.Nodes)


@click.command(help="Inspect ROS topics")
def topics() -> None:
    inspect_common(InspectMode.Topics)


@click.command(help="Inspect ROS services")
def services() -> None:
    inspect_common(InspectMode.Services)


@click.command(help="Inspect ROS actions")
def actions() -> None:
    inspect_common(InspectMode.Actions)


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
