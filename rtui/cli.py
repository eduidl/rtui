import click

from .app import InspectApp, InspectMode
from .ros import init_ros, is_ros1


def inspect_common(mode: InspectMode) -> None:
    ros = init_ros()
    try:
        InspectApp.run(ros=ros, init_mode=mode, title="ROS Inspect")
    finally:
        ros.terminate()


@click.command
def nodes() -> None:
    inspect_common(InspectMode.Nodes)


@click.command
def topics() -> None:
    inspect_common(InspectMode.Topics)


@click.command
def services() -> None:
    inspect_common(InspectMode.Services)


@click.command
def actions() -> None:
    if is_ros1():
        print("actions command does not support ROS1")
        return
    inspect_common(InspectMode.Actions)


@click.group
def cli() -> None:
    pass


def main() -> None:
    cli.add_command(nodes)
    cli.add_command(topics)
    cli.add_command(services)
    cli.add_command(actions)
    cli()
    ...


if __name__ == "__main__":
    main()
