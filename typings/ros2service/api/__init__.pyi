from __future__ import annotations

from rclpy.node import Node

def get_service_names_and_types(
    *, node: Node, include_hidden_services: bool = False
) -> list[tuple[str, list[str]]]: ...
