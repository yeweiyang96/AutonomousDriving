"""A deliberately small Lua *data* reader; it never evaluates Lua code.

The checked-in references wrap literal tables in assignments/functions. We locate
an explicit token anchor and parse only literal tables, strings, numbers and
booleans. Expressions, calls and metatables are rejected, not executed.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re


class LuaDataError(ValueError):
    pass


@dataclass(frozen=True)
class Token:
    kind: str
    value: object
    offset: int


_NUMBER = re.compile(r"(?:0[xX][0-9a-fA-F]+|(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)")
_NAME = re.compile(r"[A-Za-z_][A-Za-z_0-9]*")
_LONG = re.compile(r"\[(=*)\[")


def tokenize(source: str) -> list[Token]:
    result: list[Token] = []
    i, length = 0, len(source)
    while i < length:
        if source[i].isspace():
            i += 1
            continue
        if source.startswith("--", i):
            match = _LONG.match(source, i + 2)
            if match:
                end = source.find("]" + match[1] + "]", match.end())
                if end < 0:
                    raise LuaDataError(f"Unterminated comment at {i}")
                i = end + len(match[1]) + 2
            else:
                end = source.find("\n", i + 2)
                i = length if end < 0 else end + 1
            continue
        start, char = i, source[i]
        if char in "\"'":
            quote, pieces = char, []
            i += 1
            while i < length and source[i] != quote:
                if source[i] != "\\":
                    if source[i] in "\r\n":
                        raise LuaDataError(f"Newline in string at {start}")
                    pieces.append(source[i])
                    i += 1
                    continue
                i += 1
                if i >= length:
                    raise LuaDataError(f"Unterminated escape at {start}")
                escape = source[i]
                if escape.isascii() and escape.isdigit():
                    end = i + 1
                    while end < min(i + 3, length) and source[end].isascii() and source[end].isdigit():
                        end += 1
                    byte = int(source[i:end])
                    if byte > 255:
                        raise LuaDataError(f"Lua decimal escape exceeds one byte at {i}")
                    pieces.append(chr(byte))
                    i = end
                else:
                    escapes = {"a": "\a", "b": "\b", "f": "\f", "n": "\n", "r": "\r", "t": "\t", "v": "\v", "\\": "\\", '"': '"', "'": "'"}
                    if escape not in escapes:
                        raise LuaDataError(f"Unsupported string escape at {i}")
                    pieces.append(escapes[escape])
                    i += 1
            if i >= length:
                raise LuaDataError(f"Unterminated string at {start}")
            result.append(Token("string", "".join(pieces), start))
            i += 1
            continue
        match = _NUMBER.match(source, i)
        if match:
            text = match[0]
            value = int(text, 16) if text.lower().startswith("0x") else float(text) if any(c in text for c in ".eE") else int(text)
            if isinstance(value, float) and not math.isfinite(value):
                raise LuaDataError(f"Non-finite number at {i}")
            result.append(Token("number", value, i))
            i = match.end()
            continue
        match = _NAME.match(source, i)
        if match:
            result.append(Token("name", match[0], i))
            i = match.end()
            continue
        result.append(Token("punct", char, i))
        i += 1
    result.append(Token("eof", None, length))
    return result


class LiteralReader:
    def __init__(self, source: str):
        self.tokens = tokenize(source)
        self.index = 0

    def _take(self, value: object) -> None:
        token = self.tokens[self.index]
        if token.value != value:
            raise LuaDataError(f"Expected {value!r} at {token.offset}, got {token.value!r}")
        self.index += 1

    def _value(self, depth: int = 0):
        if depth > 64:
            raise LuaDataError("Literal nesting exceeds 64")
        token = self.tokens[self.index]
        if token.value == "{" and token.kind == "punct":
            return self._table(depth + 1)
        if token.kind in ("string", "number"):
            self.index += 1
            return token.value
        if token.value == "-" and self.tokens[self.index + 1].kind == "number":
            self.index += 2
            return -self.tokens[self.index - 1].value
        if token.kind == "name" and token.value in ("true", "false", "nil"):
            self.index += 1
            return {"true": True, "false": False, "nil": None}[token.value]
        raise LuaDataError(f"Non-literal Lua value at {token.offset}: {token.value!r}")

    def _table(self, depth: int) -> dict:
        self._take("{")
        result, ordinal = {}, 1
        while self.tokens[self.index].value != "}":
            token = self.tokens[self.index]
            if token.value == "[" and token.kind == "punct":
                self.index += 1
                key = self._value(depth)
                if key is None or isinstance(key, (dict, bool)):
                    raise LuaDataError("Unsupported table key")
                self._take("]")
                self._take("=")
                value = self._value(depth)
            elif token.kind == "name" and self.tokens[self.index + 1].value == "=":
                key = token.value
                self.index += 2
                value = self._value(depth)
            else:
                key = ordinal
                ordinal += 1
                value = self._value(depth)
            if key in result:
                raise LuaDataError(f"Duplicate table key {key!r}")
            result[key] = value
            if self.tokens[self.index].value in (",", ";"):
                self.index += 1
            elif self.tokens[self.index].value != "}":
                raise LuaDataError(f"Expected table separator at {self.tokens[self.index].offset}")
        self._take("}")
        return result

    def table_after(self, *anchor: object) -> dict:
        """Read a literal table immediately after an exact token sequence."""
        for i in range(len(self.tokens) - len(anchor)):
            if tuple(token.value for token in self.tokens[i:i + len(anchor)]) == anchor:
                self.index = i + len(anchor)
                value = self._value()
                if not isinstance(value, dict):
                    raise LuaDataError(f"Anchor {anchor!r} does not precede a table")
                return value
        raise LuaDataError(f"Table anchor missing: {anchor!r}")


def array(value: object) -> list:
    if not isinstance(value, dict) or set(value) != set(range(1, len(value) + 1)):
        raise LuaDataError("Expected a contiguous Lua array")
    return [value[i] for i in range(1, len(value) + 1)]
