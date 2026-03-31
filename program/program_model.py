from __future__ import annotations

import ast
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class ProgramCommand:
    name: str
    args: list[Any]
    kwargs: dict[str, Any]
    notes: str = ""
    source: str = ""
    parameters_text: str = "-"

    def to_row(self) -> dict[str, str]:
        return {
            "command": self.source or self.to_script_line(),
            "parameters": self.parameters_text if self.parameters_text else "-",
            "notes": self.notes,
        }

    def to_script_line(self) -> str:
        parts = [self._render_value(value) for value in self.args]
        parts.extend(f"{key}={self._render_value(value)}" for key, value in self.kwargs.items())
        return f"{self.name}({', '.join(parts)})"

    @staticmethod
    def _render_value(value: Any) -> str:
        if isinstance(value, str):
            if value.replace("_", "").isalnum() and " " not in value:
                return value
            return json.dumps(value)
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (list, tuple)):
            inner = ", ".join(ProgramCommand._render_value(item) for item in value)
            open_char, close_char = ("[", "]") if isinstance(value, list) else ("(", ")")
            return f"{open_char}{inner}{close_char}"
        return str(value)


class ProgramModel:
    def __init__(self, commands: list[ProgramCommand] | None = None) -> None:
        self.commands = commands or []

    @classmethod
    def from_rows(cls, rows: list[dict[str, str]]) -> "ProgramModel":
        commands = [cls.parse_row(row) for row in rows if row.get("command", "").strip()]
        return cls(commands)

    @classmethod
    def parse_row(cls, row: dict[str, str]) -> ProgramCommand:
        command_text = (row.get("command") or "").strip()
        parameters_text = (row.get("parameters") or "-").strip()
        notes = (row.get("notes") or "").strip()
        expression = cls._compose_expression(command_text, parameters_text)
        name, args, kwargs = cls._parse_expression(expression)
        return ProgramCommand(
            name=name,
            args=args,
            kwargs=kwargs,
            notes=notes,
            source=expression,
            parameters_text=parameters_text,
        )

    @staticmethod
    def _compose_expression(command_text: str, parameters_text: str) -> str:
        if not command_text:
            raise ValueError("Program command must not be empty")
        if "(" in command_text and command_text.endswith(")"):
            return command_text
        if parameters_text and parameters_text != "-":
            return f"{command_text}({parameters_text})"
        return f"{command_text}()"

    @classmethod
    def _parse_expression(cls, expression: str) -> tuple[str, list[Any], dict[str, Any]]:
        parsed = ast.parse(expression, mode="eval")
        node = parsed.body
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                raise ValueError(f"Unsupported command expression: {expression}")
            name = node.func.id
            args = [cls._ast_to_value(arg) for arg in node.args]
            kwargs = {keyword.arg: cls._ast_to_value(keyword.value) for keyword in node.keywords}
            return name, args, kwargs
        if isinstance(node, ast.Name):
            return node.id, [], {}
        raise ValueError(f"Unsupported command expression: {expression}")

    @classmethod
    def _ast_to_value(cls, node: ast.AST) -> Any:
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Name):
            return node.id
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            operand = cls._ast_to_value(node.operand)
            if isinstance(operand, (int, float)):
                return -operand if isinstance(node.op, ast.USub) else operand
        if isinstance(node, ast.List):
            return [cls._ast_to_value(item) for item in node.elts]
        if isinstance(node, ast.Tuple):
            return tuple(cls._ast_to_value(item) for item in node.elts)
        raise ValueError(f"Unsupported parameter value: {ast.dump(node)}")

    def to_rows(self) -> list[dict[str, str]]:
        return [command.to_row() for command in self.commands]

    def to_script(self) -> str:
        lines = [command.to_script_line() for command in self.commands]
        return "\n".join(lines).strip() + ("\n" if lines else "")

    def save_json(self, destination: str | Path) -> Path:
        destination_path = Path(destination)
        destination_path.write_text(json.dumps(self.to_rows(), indent=2) + "\n", encoding="utf-8")
        return destination_path

    @classmethod
    def load_json(cls, source: str | Path) -> "ProgramModel":
        rows = json.loads(Path(source).read_text(encoding="utf-8"))
        if not isinstance(rows, list):
            raise ValueError("Program JSON must contain a list of rows")
        return cls.from_rows(rows)