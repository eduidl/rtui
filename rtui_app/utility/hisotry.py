from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass
class History(Generic[T]):
    history: deque[T] = deque(maxlen=20)
    index: int = -1

    def __init__(self, maxlen: int) -> None:
        self.history = deque(maxlen=maxlen)

    def __clear_future(self) -> None:
        if self.index <= 0:
            return

        for _ in range(self.index):
            self.history.popleft()
        self.index = 0

    def empty(self) -> bool:
        return self.len() == 0

    def len(self) -> int:
        return len(self.history)

    def append(self, item: T) -> None:
        self.__clear_future()
        if item == self.current():
            return
        self.history.appendleft(item)
        self.index = 0

    def current(self) -> T | None:
        if self.index < 0:
            return None
        return self.history[self.index]

    def forward(self) -> T | None:
        if self.index <= 0:
            return None

        self.index -= 1
        return self.current()

    def back(self) -> T | None:
        if self.index == len(self.history) - 1:
            return None

        self.index += 1
        return self.current()
