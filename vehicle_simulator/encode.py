"""Turn the car's signals into CAN frames and read requests back off the bus. The DBC and alias.json are the only per-vehicle inputs."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import cantools

_TOP_KEYS = {"dbc", "interfaces", "interface_by_sender", "default_interface", "periods_ms", "default_period_ms", "fixed", "signals"}
_ROW_KEYS = {"signal", "plant", "request", "values", "scale"}


@dataclass(frozen=True)
class Binding:
    message_name: str
    signal_name: str
    name: str
    is_request: bool
    values: dict[str, int]
    reverse: dict[int, str]
    scale: float = 1.0


@dataclass(frozen=True)
class Frame:
    frame_id: int
    data: bytes
    interface: str
    extended: bool = False
    name: str = ""

    def wire(self) -> dict[str, Any]:
        return {"id": self.frame_id, "ext": self.extended, "data": self.data.hex()}


@dataclass
class Page:
    """One frame layout of a message. A message without a mux has one page."""

    selectors: dict[str, int]
    bindings: list[Binding]
    next_s: float = 0.0


@dataclass
class TxMessage:
    message: Any
    interface: str
    period_s: float
    pages: list[Page] = field(default_factory=list)


class Encoder:
    def __init__(
        self,
        dbc_path: Path,
        database: Any,
        bindings: list[Binding],
        fixed: dict[tuple[str, str], int],
        interfaces: dict[str, int],
        tx: list[TxMessage],
    ) -> None:
        self.dbc_path = dbc_path
        self.bindings = bindings
        self.interfaces = interfaces
        self.tx = tx
        self._db = database
        self._fixed = fixed
        self._rx: dict[tuple[str, int, bool], tuple[Any, list[Binding]]] = {}
        for item in tx:
            requests = [b for page in item.pages for b in page.bindings if b.is_request]
            if requests:
                key = (item.interface, item.message.frame_id, item.message.is_extended_frame)
                self._rx[key] = (item.message, requests)

    def names(self) -> list[str]:
        return [binding.name for binding in self.bindings]

    def check(self, known: Iterable[str]) -> None:
        """Every alias name must be something the car produces."""
        known = set(known)
        missing = sorted({name for name in self.names() if name not in known})
        if missing:
            raise ValueError(f"alias uses names the car does not have: {', '.join(missing)}")

    def encode_all(self, values: dict[str, Any]) -> list[Frame]:
        """Every page of every message, once."""
        return [self._frame(item, page, values) for item in self.tx for page in item.pages]

    def due(self, now_s: float, values: dict[str, Any]) -> list[Frame]:
        """Frames whose period has come round. A late loop sends once, not a burst."""
        frames: list[Frame] = []
        for item in self.tx:
            for page in item.pages:
                if now_s + 1e-9 < page.next_s:
                    continue
                frames.append(self._frame(item, page, values))
                page.next_s = max(page.next_s + item.period_s, now_s)
        return frames

    def decode(self, frames: Iterable[Frame]) -> dict[str, Any]:
        """Alias names from frames. The last page seen for a message wins."""
        recovered: dict[str, Any] = {}
        by_key = {(item.interface, item.message.frame_id): item for item in self.tx}
        for frame in frames:
            item = by_key.get((frame.interface, frame.frame_id))
            if item is None:
                continue
            raw = item.message.decode(frame.data, decode_choices=False, scaling=False)
            for page in item.pages:
                for binding in page.bindings:
                    if binding.signal_name in raw:
                        value = _from_raw(item.message.get_signal_by_name(binding.signal_name), binding, raw[binding.signal_name])
                        if value is not None:
                            recovered[binding.name] = value
        return recovered

    def requests(
        self,
        interface: str,
        frame_id: int,
        extended: bool,
        data: bytes,
        current: dict[str, Any],
    ) -> list[tuple[str, Any]]:
        """Requests in a frame from someone else that differ from what the car already asks for."""
        entry = self._rx.get((interface, frame_id, extended))
        if entry is None:
            return []
        message, bindings = entry
        try:
            raw = message.decode(data, decode_choices=False, scaling=False)
        except Exception:
            return []
        changes: list[tuple[str, Any]] = []
        for binding in bindings:
            if binding.signal_name not in raw:
                continue
            signal = message.get_signal_by_name(binding.signal_name)
            value = _from_raw(signal, binding, raw[binding.signal_name])
            if value is None:
                continue
            if _same(signal, binding, value, current.get(binding.name)):
                continue
            changes.append((binding.name, value))
        return changes

    def _frame(self, item: TxMessage, page: Page, values: dict[str, Any]) -> Frame:
        message = item.message
        bound = {binding.signal_name: binding for binding in page.bindings}
        payload: dict[str, int] = {}
        for signal in _active(message, page.selectors):
            if signal.name in page.selectors:
                payload[signal.name] = page.selectors[signal.name]
            elif signal.name in bound:
                binding = bound[signal.name]
                payload[signal.name] = _to_raw(signal, binding, values[binding.name])
            elif (message.name, signal.name) in self._fixed:
                payload[signal.name] = self._fixed[(message.name, signal.name)]
            else:
                payload[signal.name] = _physical_to_raw(signal, _filler(signal))
        data = message.encode(payload, scaling=False, strict=False)
        return Frame(message.frame_id, bytes(data), item.interface, message.is_extended_frame, message.name)


def load_encoder(alias_path: Path) -> Encoder:
    alias = json.loads(Path(alias_path).read_text())
    unknown = set(alias) - _TOP_KEYS
    if unknown:
        raise ValueError(f"alias has unknown keys: {', '.join(sorted(unknown))}")
    dbc_path = Path(alias["dbc"])
    if not dbc_path.is_absolute():
        dbc_path = (Path(alias_path).parent / dbc_path).resolve()
    database = cantools.database.load_file(dbc_path)

    interfaces = {str(name): int(port) for name, port in (alias.get("interfaces") or {"can0": 47000}).items()}
    by_sender = {str(k): str(v) for k, v in (alias.get("interface_by_sender") or {}).items()}
    default_interface = alias.get("default_interface", next(iter(interfaces)) if len(interfaces) == 1 else None)
    for name in [*by_sender.values(), *([default_interface] if default_interface else [])]:
        if name not in interfaces:
            raise ValueError(f"interface {name} is not in interfaces")
    periods = {str(k): float(v) for k, v in (alias.get("periods_ms") or {}).items()}
    default_period_ms = float(alias.get("default_period_ms", 1000))
    if default_period_ms <= 0 or any(v <= 0 for v in periods.values()):
        raise ValueError("periods must be positive")

    bindings = [_binding(database, row) for row in alias["signals"]]
    seen: set[tuple[str, str]] = set()
    for binding in bindings:
        key = (binding.message_name, binding.signal_name)
        if key in seen:
            raise ValueError(f"{binding.message_name}.{binding.signal_name} is on two alias rows")
        seen.add(key)

    fixed: dict[tuple[str, str], int] = {}
    for text, value in (alias.get("fixed") or {}).items():
        message, signal = _lookup(database, text)
        if (message.name, signal.name) in seen:
            raise ValueError(f"{text} is both fixed and on an alias row")
        fixed[(message.name, signal.name)] = _constant(signal, value)

    grouped: dict[str, list[Binding]] = {}
    for binding in bindings:
        grouped.setdefault(binding.message_name, []).append(binding)
    for message_name, _signal in fixed:
        grouped.setdefault(message_name, [])
    for name in periods:
        if name not in grouped:
            raise ValueError(f"periods_ms names {name}, which has no alias rows")

    tx: list[TxMessage] = []
    for message_name, rows in grouped.items():
        message = database.get_message_by_name(message_name)
        period_ms = periods.get(message_name) or message.cycle_time or default_period_ms
        item = TxMessage(message, _interface(message, by_sender, default_interface), period_ms / 1000.0)
        item.pages = _pages(message, rows, fixed)
        tx.append(item)
    tx.sort(key=lambda item: item.message.frame_id)
    return Encoder(dbc_path, database, bindings, fixed, interfaces, tx)


def _binding(database: Any, row: dict[str, Any]) -> Binding:
    unknown = set(row) - _ROW_KEYS
    if unknown:
        raise ValueError(f"alias row {row.get('signal')} has unknown keys: {', '.join(sorted(unknown))}")
    if ("plant" in row) == ("request" in row):
        raise ValueError(f"alias row {row.get('signal')} needs exactly one of plant or request")
    message, signal = _lookup(database, str(row["signal"]))
    values: dict[str, int] = {}
    for key, raw in (row.get("values") or {}).items():
        values[str(key)] = _choice(signal, raw)
    reverse: dict[int, str] = {}
    for key, raw in values.items():
        reverse.setdefault(raw, key)
    is_request = "request" in row
    scale = float(row.get("scale", 1.0))
    if scale == 0 or (values and "scale" in row):
        raise ValueError(f"alias scale for {message.name}.{signal.name} must be non-zero and not used with values")
    return Binding(message.name, signal.name, str(row["request" if is_request else "plant"]), is_request, values, reverse, scale)


def _lookup(database: Any, text: str) -> tuple[Any, Any]:
    if "." not in text:
        raise ValueError(f"alias signal must be Message.Signal, got {text}")
    message_name, signal_name = text.split(".", 1)
    try:
        message = database.get_message_by_name(message_name)
        return message, message.get_signal_by_name(signal_name)
    except KeyError as exc:
        raise ValueError(f"DBC has no signal {text}") from exc


def _choice(signal: Any, raw: Any) -> int:
    """An alias value is a raw number or a DBC value name."""
    if isinstance(raw, bool):
        return int(raw)
    if isinstance(raw, int):
        return raw
    if isinstance(raw, str):
        for number, label in (signal.choices or {}).items():
            if str(label) == raw:
                return int(number)
        raise ValueError(f"{signal.name} has no value named {raw}")
    raise ValueError(f"{signal.name} alias value {raw!r} must be an integer or a DBC value name")


def _constant(signal: Any, value: Any) -> int:
    """A fixed value is a DBC value name or a physical number."""
    if isinstance(value, str):
        return _choice(signal, value)
    return _physical_to_raw(signal, float(value))


def _interface(message: Any, by_sender: dict[str, str], default: str | None) -> str:
    targets = {by_sender[s] for s in message.senders if s in by_sender}
    if len(targets) > 1:
        raise ValueError(f"{message.name} senders sit on different interfaces")
    if targets:
        return targets.pop()
    if default is None:
        raise ValueError(f"{message.name} sender {message.senders} has no interface and there is no default_interface")
    return default


def _requirement(message: Any, signal: Any) -> dict[str, int]:
    """Selector values a signal needs before it is on the frame, walking up nested muxes."""
    needs: dict[str, int] = {}
    current = signal
    while current.multiplexer_signal:
        ids = current.multiplexer_ids or []
        if not ids:
            break
        needs[current.multiplexer_signal] = int(ids[0])
        current = message.get_signal_by_name(current.multiplexer_signal)
    return needs


def _pages(message: Any, rows: list[Binding], fixed: dict[tuple[str, str], int]) -> list[Page]:
    pages: dict[tuple[tuple[str, int], ...], list[Binding]] = {}
    free: list[Binding] = []
    for binding in rows:
        needs = _requirement(message, message.get_signal_by_name(binding.signal_name))
        if needs:
            pages.setdefault(tuple(sorted(needs.items())), []).append(binding)
        else:
            free.append(binding)
    if not pages:
        pages[()] = []
    result: list[Page] = []
    for key, bound in pages.items():
        selectors = dict(key)
        _settle(message, selectors, fixed)
        result.append(Page(selectors, [*free, *bound]))
    return result


def _settle(message: Any, selectors: dict[str, int], fixed: dict[tuple[str, str], int]) -> None:
    """Give every active selector a value until the page stops changing."""
    while True:
        added = False
        for signal in _active(message, selectors):
            if signal.is_multiplexer and signal.name not in selectors:
                pinned = fixed.get((message.name, signal.name))
                selectors[signal.name] = pinned if pinned is not None else _first_page(message, signal.name)
                added = True
        if not added:
            return


def _first_page(message: Any, selector: str) -> int:
    ids = sorted({int(i) for s in message.signals if s.multiplexer_signal == selector for i in (s.multiplexer_ids or [])})
    return ids[0] if ids else 0


def _active(message: Any, selectors: dict[str, int]) -> list[Any]:
    active = []
    for signal in message.signals:
        parent = signal.multiplexer_signal
        if parent is None:
            active.append(signal)
        elif parent in selectors and selectors[parent] in (signal.multiplexer_ids or []):
            active.append(signal)
    return active


def _to_raw(signal: Any, binding: Binding, value: Any) -> int:
    if binding.values:
        key = str(value)
        if key not in binding.values:
            raise ValueError(f"{binding.name} value {value!r} has no alias value for {signal.name}")
        return binding.values[key]
    if isinstance(value, str):
        raise ValueError(f"{binding.name} is text ({value!r}); {signal.name} needs an alias values map")
    return _physical_to_raw(signal, float(value) * binding.scale)


def _from_raw(signal: Any, binding: Binding, raw: Any) -> Any:
    number = int(raw)
    if binding.values:
        return binding.reverse.get(number)
    return (number * float(signal.scale) + float(signal.offset)) / binding.scale


def _same(signal: Any, binding: Binding, value: Any, current: Any) -> bool:
    if current is None:
        return False
    if binding.values:
        return str(value) == str(current)
    try:
        return abs(float(value) - float(current)) <= abs(float(signal.scale) / binding.scale) / 2.0
    except (TypeError, ValueError):
        return False


def _physical_to_raw(signal: Any, value: float) -> int:
    if signal.minimum is not None:
        value = max(float(signal.minimum), value)
    if signal.maximum is not None:
        value = min(float(signal.maximum), value)
    raw = round((value - float(signal.offset)) / float(signal.scale))
    if signal.is_signed:
        low, high = -(1 << (signal.length - 1)), (1 << (signal.length - 1)) - 1
    else:
        low, high = 0, (1 << signal.length) - 1
    return int(max(low, min(high, raw)))


def _filler(signal: Any) -> float:
    low = signal.minimum
    high = signal.maximum
    if low is not None and 0 < low:
        return float(low)
    if high is not None and 0 > high:
        return float(high)
    return 0.0
