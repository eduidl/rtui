from __future__ import annotations

from rclpy.node import Node

def get_action_names_and_types(*, node: Node) -> list[tuple[str, list[str]]]: ...
def get_action_clients_and_servers(
    *, node: Node, action_name: str
) -> tuple[list[tuple[str, list[str]]], list[tuple[str, list[str]]]]: ...
