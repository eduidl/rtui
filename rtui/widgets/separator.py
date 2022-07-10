from __future__ import annotations

from rich.box import ROUNDED, Box
from rich.console import Console, ConsoleOptions, RenderableType, RenderResult
from rich.jupyter import JupyterMixin
from rich.measure import Measurement
from rich.segment import Segment
from rich.style import StyleType
from textual.widget import Widget


class HorizontalLine(JupyterMixin):
    def __init__(
        self,
        box: Box = ROUNDED,
        width: int | None = None,
        *,
        style: StyleType = "none",
    ) -> None:
        self.box = box
        self.width = width
        self.style = style

    def __rich_console__(
        self, console: Console, options: ConsoleOptions
    ) -> RenderResult:
        style = console.get_style(self.style)
        box = self.box.substitute(options, console.safe_box)
        width = self.width or 100

        for _ in range(width):
            yield Segment(box.top, style)

    def __rich_measure__(
        self, console: Console, options: ConsoleOptions
    ) -> Measurement:
        return Measurement(self.width or 100, self.width or 100)


class VerticalLine(JupyterMixin):
    def __init__(
        self,
        box: Box = ROUNDED,
        height: int | None = None,
        *,
        style: StyleType = "none",
    ) -> None:
        self.box = box
        self.height = height
        self.style = style

    def __rich_console__(
        self, console: Console, options: ConsoleOptions
    ) -> RenderResult:
        style = console.get_style(self.style)
        box = self.box.substitute(options, console.safe_box)
        height = self.height or options.height or 20
        new_line = Segment.line()

        for i in range(height):
            yield Segment(box.mid_left, style)
            if i != height - 1:
                yield new_line

    def __rich_measure__(
        self, console: Console, options: ConsoleOptions
    ) -> Measurement:
        return Measurement(1, 1)


class Separator(Widget):
    box: Box
    horizontal: bool

    def __init__(
        self,
        *,
        box: Box = ROUNDED,
        name: str | None = None,
        horizontal: bool = False,
    ) -> None:
        self.box = box
        self.horizontal = horizontal
        super().__init__(name=name)

    def render(self) -> RenderableType:
        if self.horizontal:
            return HorizontalLine(box=self.box)
        else:
            return VerticalLine(box=self.box)
