from __future__ import annotations

import typing as t

from typing_extensions import TypeAlias

from . import names

_RosMasterEachSystemState: TypeAlias = t.List[t.Tuple[str, t.List[str]]]
_RosMasterSystemState: TypeAlias = t.Tuple[
    _RosMasterEachSystemState, _RosMasterEachSystemState, _RosMasterEachSystemState
]

class Master:
    def __init__(self, caller_id: str, master_uri: str | None = None) -> None: ...
    def lookupService(self, service_name: str) -> str: ...
    def getTopicTypes(self) -> list[tuple[str, str]]: ...
    def getSystemState(
        self,
    ) -> _RosMasterSystemState: ...

__all__ = ["names", "Master"]
