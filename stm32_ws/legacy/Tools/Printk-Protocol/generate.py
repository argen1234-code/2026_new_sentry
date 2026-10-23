#!/usr/bin/env python3
"""Proto2 RT-only schema compiler.

The generator intentionally owns only the compact fixed-frame RT core.  It does
not generate the old full/debug runtime, host tools, transports, tasks, or any
board binding code.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    from jinja2 import Environment, FileSystemLoader, StrictUndefined
except ImportError as exc:  # pragma: no cover
    raise SystemExit("jinja2 is required: pip install jinja2") from exc

RT_MAX_PAYLOAD = 255

BUILTIN_TYPES: Dict[str, Tuple[str, int]] = {
    "bool": ("uint8_t", 1),
    "uint8_t": ("uint8_t", 1),
    "int8_t": ("int8_t", 1),
    "uint16_t": ("uint16_t", 2),
    "int16_t": ("int16_t", 2),
    "uint32_t": ("uint32_t", 4),
    "int32_t": ("int32_t", 4),
    "uint64_t": ("uint64_t", 8),
    "int64_t": ("int64_t", 8),
    "float": ("float", 4),
    "double": ("double", 8),
}

FLOAT_TYPES = {"float", "double"}
VALID_DIRECTIONS = {"host_to_device", "device_to_host", "bidirectional"}
INTEGER_TYPES = {k for k in BUILTIN_TYPES if k not in FLOAT_TYPES and k != "bool"}
C_CPP_KEYWORDS = {
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor", "bool",
    "break", "case", "catch", "char", "char8_t", "char16_t", "char32_t", "class", "compl",
    "concept", "const", "consteval", "constexpr", "constinit", "const_cast", "continue", "co_await",
    "co_return", "co_yield", "decltype", "default", "delete", "do", "double", "dynamic_cast",
    "else", "enum", "explicit", "export", "extern", "false", "float", "for", "friend", "goto",
    "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept", "not", "not_eq",
    "nullptr", "operator", "or", "or_eq", "private", "protected", "public", "register",
    "reinterpret_cast", "requires", "return", "short", "signed", "sizeof", "static", "static_assert",
    "static_cast", "struct", "switch", "template", "this", "thread_local", "throw", "true", "try",
    "typedef", "typeid", "typename", "union", "unsigned", "using", "virtual", "void", "volatile",
    "wchar_t", "while", "xor", "xor_eq",
}


@dataclass
class EnumMember:
    name: str
    value: int
    value_text: str
    line: int


@dataclass
class EnumDef:
    name: str
    base_type: str
    members: List[EnumMember] = field(default_factory=list)
    line: int = 1


@dataclass
class FieldDef:
    name: str
    type_name: str
    array_len: Optional[int]
    line: int


@dataclass
class MessageDef:
    name: str
    msg_id: int
    direction: str
    fields: List[FieldDef] = field(default_factory=list)
    line: int = 1


class SchemaError(ValueError):
    pass


def schema_error(path: str, line: int, message: str) -> SchemaError:
    return SchemaError(f"{path}:{line}: {message}")


def line_for_offset(text: str, offset: int) -> int:
    return text.count("\n", 0, max(0, offset)) + 1


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def to_snake(name: str) -> str:
    step1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    step2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", step1)
    return step2.lower()


def validate_identifier(path: str, line: int, kind: str, name: str) -> None:
    if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", name):
        raise schema_error(path, line, f"{kind} {name} is not a valid identifier")
    if name in C_CPP_KEYWORDS:
        raise schema_error(path, line, f"{kind} {name} uses reserved C/C++ keyword")


def parse_int(path: str, line: int, owner: str, raw: str) -> int:
    try:
        return int(raw, 0)
    except ValueError as exc:
        raise schema_error(path, line, f"{owner} has invalid integer literal {raw}") from exc


def enum_bounds(base_type: str) -> Tuple[int, int]:
    bits = BUILTIN_TYPES[base_type][1] * 8
    if base_type.startswith("u"):
        return 0, (1 << bits) - 1
    return -(1 << (bits - 1)), (1 << (bits - 1)) - 1


def is_hpp_schema(path: str) -> bool:
    return path.endswith(".hpp") or path.endswith(".h")


def canonical_schema_path(path: str) -> Optional[str]:
    p = Path(path)
    if p.suffix in {".hpp", ".h"}:
        name = p.name
        for suffix in (".schema.hpp", ".schema.h", ".hpp", ".h"):
            if name.endswith(suffix):
                return str(p.with_name(name[: -len(suffix)] + ".pfbs"))
    return None


def split_items(body: str) -> Iterable[str]:
    for raw in re.split(r"[;,]", body):
        item = raw.strip()
        if item:
            yield item


def parse_enums(clean: str, path: str) -> List[EnumDef]:
    enums: List[EnumDef] = []
    seen: Dict[str, int] = {}
    pattern = re.compile(r"\benum\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([A-Za-z_][A-Za-z0-9_]*)\s*\{(.*?)\}\s*;?", re.S)
    for match in pattern.finditer(clean):
        name = match.group(1)
        base_type = match.group(2)
        body = match.group(3)
        line = line_for_offset(clean, match.start())
        validate_identifier(path, line, "Enum", name)
        if name in seen:
            raise schema_error(path, line, f"Duplicate enum {name}")
        if base_type not in INTEGER_TYPES:
            raise schema_error(path, line, f"Enum {name} base type must be an integer stdint type")
        lo, hi = enum_bounds(base_type)
        members: List[EnumMember] = []
        member_names = set()
        member_values: Dict[int, str] = {}
        search_pos = 0
        for item in split_items(body):
            m = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+)$", item)
            if not m:
                raise schema_error(path, line, f"Invalid enum member in {name}: {item}")
            item_offset = body.find(item, search_pos)
            search_pos = item_offset + len(item) if item_offset >= 0 else search_pos
            member_line = line_for_offset(clean, match.start(3) + max(item_offset, 0))
            member_name = m.group(1)
            raw_value = m.group(2).strip()
            validate_identifier(path, member_line, "Enum member", member_name)
            if member_name in member_names:
                raise schema_error(path, member_line, f"Duplicate enum member {name}.{member_name}")
            value = parse_int(path, member_line, f"{name}.{member_name}", raw_value)
            if value < lo or value > hi:
                raise schema_error(path, member_line, f"Enum member {name}.{member_name} value out of {base_type} range")
            if value in member_values:
                raise schema_error(path, member_line, f"Enum member {name}.{member_name} reuses value {raw_value} from {member_values[value]}")
            member_names.add(member_name)
            member_values[value] = member_name
            members.append(EnumMember(member_name, value, raw_value, member_line))
        seen[name] = line
        enums.append(EnumDef(name=name, base_type=base_type, members=members, line=line))
    return enums


def parse_array_type(raw: str) -> Tuple[str, Optional[int]]:
    bracket = re.match(r"\[\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([0-9]+)\s*\]$", raw)
    if bracket:
        return bracket.group(1), int(bracket.group(2))
    c_array = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*\[\s*([0-9]+)\s*\]$", raw)
    if c_array:
        return c_array.group(1), int(c_array.group(2))
    return raw, None


def parse_messages(clean: str, path: str, enum_map: Dict[str, EnumDef]) -> List[MessageDef]:
    pattern = re.compile(r"\bmessage\s+([A-Za-z_][A-Za-z0-9_]*)\s*(.*?)\{(.*?)\}\s*;?", re.S)
    messages: List[MessageDef] = []
    names = set()
    ids: Dict[int, str] = {}
    for match in pattern.finditer(clean):
        name = match.group(1)
        anno_text = match.group(2)
        body = match.group(3)
        line = line_for_offset(clean, match.start())
        validate_identifier(path, line, "Message", name)
        if name in names:
            raise schema_error(path, line, f"Duplicate message {name}")
        annotations: Dict[str, str] = {}
        for key, value in re.findall(r"@([A-Za-z_][A-Za-z0-9_]*)\(([^)]+)\)", anno_text):
            annotations[key] = value.strip()
        fields: List[FieldDef] = []
        search_pos = 0
        for item in split_items(body):
            item_offset = body.find(item, search_pos)
            search_pos = item_offset + len(item) if item_offset >= 0 else search_pos
            item_line = line_for_offset(clean, match.start(3) + max(item_offset, 0))
            assign = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+)$", item)
            if assign:
                annotations[assign.group(1)] = assign.group(2).strip()
                continue
            pfbs = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.+)$", item)
            c_decl = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s+([A-Za-z_][A-Za-z0-9_]*)(\s*\[\s*([0-9]+)\s*\])?$", item)
            if pfbs:
                field_name = pfbs.group(1)
                type_name, array_len = parse_array_type(pfbs.group(2).strip())
            elif c_decl:
                type_name = c_decl.group(1)
                field_name = c_decl.group(2)
                array_len = int(c_decl.group(4)) if c_decl.group(4) else None
            else:
                raise schema_error(path, item_line, f"Invalid message item in {name}: {item}")
            validate_identifier(path, item_line, "Field", field_name)
            if type_name not in BUILTIN_TYPES and type_name not in enum_map:
                raise schema_error(path, item_line, f"Unknown type {type_name} in {name}.{field_name}")
            if array_len is not None and array_len <= 0:
                raise schema_error(path, item_line, f"Array field {name}.{field_name} must have positive length")
            if array_len is not None and type_size(type_name, enum_map) * array_len > 0xFFFF:
                raise schema_error(path, item_line, f"Array field {name}.{field_name} is too large")
            if any(f.name == field_name for f in fields):
                raise schema_error(path, item_line, f"Duplicate field {name}.{field_name}")
            fields.append(FieldDef(field_name, type_name, array_len, item_line))
        missing = [key for key in ("msg_id", "direction") if key not in annotations]
        if missing:
            raise schema_error(path, line, f"Message {name} missing annotations: {', '.join(missing)}")
        unknown = [key for key in annotations if key not in {"msg_id", "direction"}]
        if unknown:
            raise schema_error(path, line, f"Message {name} has unknown annotations: {', '.join(unknown)}")
        if annotations["direction"] not in VALID_DIRECTIONS:
            raise schema_error(path, line, f"Message {name} has invalid direction {annotations['direction']}")
        msg_id = parse_int(path, line, f"Message {name} msg_id", annotations["msg_id"])
        if msg_id in (0x00, 0xFF):
            raise schema_error(path, line, f"RT message {name} uses reserved msg_id 0x{msg_id:02X}")
        if msg_id < 0 or msg_id > 0xFF:
            raise schema_error(path, line, f"RT message {name} msg_id must fit in uint8_t")
        if msg_id in ids:
            raise schema_error(path, line, f"Duplicate msg_id 0x{msg_id:02X}: message {name} reuses value from {ids[msg_id]}")
        msg = MessageDef(name=name, msg_id=msg_id, direction=annotations["direction"], fields=fields, line=line)
        if message_size(msg, enum_map) > RT_MAX_PAYLOAD:
            raise schema_error(path, line, f"RT message {name} payload must be <= {RT_MAX_PAYLOAD} bytes")
        names.add(name)
        ids[msg_id] = name
        messages.append(msg)
    return messages


def parse_schema(text: str, path: str) -> Tuple[str, List[EnumDef], List[MessageDef]]:
    clean = strip_comments(text)
    namespace_match = re.search(r"\bnamespace\s+([A-Za-z0-9_: .]+)\s*;", clean)
    namespace = namespace_match.group(1).replace(" ", "") if namespace_match else ""
    namespace = namespace.replace("::", ".")
    if namespace != "proto2.rt":
        raise schema_error(path, 1, "RT-only generator requires namespace proto2::rt")
    enums = parse_enums(clean, path)
    enum_map = {enum.name: enum for enum in enums}
    messages = parse_messages(clean, path, enum_map)
    if not messages:
        raise schema_error(path, 1, "Schema contains no messages")
    return namespace, enums, messages


def c_type_name(type_name: str, enum_map: Dict[str, EnumDef]) -> str:
    if type_name in BUILTIN_TYPES:
        return BUILTIN_TYPES[type_name][0]
    return f"proto2_rt_enum_{to_snake(type_name)}_t"


def type_size(type_name: str, enum_map: Dict[str, EnumDef]) -> int:
    if type_name in BUILTIN_TYPES:
        return BUILTIN_TYPES[type_name][1]
    return BUILTIN_TYPES[enum_map[type_name].base_type][1]


def message_size(message: MessageDef, enum_map: Dict[str, EnumDef]) -> int:
    total = 0
    for f in message.fields:
        total += type_size(f.type_name, enum_map) * (f.array_len or 1)
    return total


def field_contexts(message: MessageDef, enum_map: Dict[str, EnumDef]) -> List[Dict[str, object]]:
    fields: List[Dict[str, object]] = []
    offset = 0
    for f in message.fields:
        size = type_size(f.type_name, enum_map)
        wire_type = f.type_name if f.type_name in BUILTIN_TYPES else enum_map[f.type_name].base_type
        ctx = {
            "name": f.name,
            "type_name": f.type_name,
            "wire_type": wire_type,
            "ctype": c_type_name(f.type_name, enum_map),
            "array_len": f.array_len,
            "array_bytes": size * f.array_len if f.array_len else 0,
            "size": size,
            "is_float": f.type_name in FLOAT_TYPES,
            "is_bool": f.type_name == "bool",
            "is_raw_byte_array": bool(f.array_len and wire_type in {"uint8_t", "int8_t"}),
            "offset": offset,
        }
        if f.array_len:
            ctx["decl"] = f"    {ctx['ctype']} {f.name}[{f.array_len}];"
        else:
            ctx["decl"] = f"    {ctx['ctype']} {f.name};"
        fields.append(ctx)
        offset += size * (f.array_len or 1)
    return fields


def message_context(message: MessageDef, enum_map: Dict[str, EnumDef]) -> Dict[str, object]:
    fields = field_contexts(message, enum_map)
    size = message_size(message, enum_map)
    return {
        "name": message.name,
        "snake": to_snake(message.name),
        "doc_block": "",
        "msg_id": f"0x{message.msg_id:02X}",
        "msg_id_value": message.msg_id,
        "size": size,
        "frame_size": size + 7,
        "direction": message.direction,
        "direction_macro": f"PROTO2_RT_DIRECTION_{message.direction.upper()}",
        "fields": fields,
        "struct_fields_block": "\n".join(str(f["decl"]) for f in fields) if fields else "    uint8_t _reserved_zero_size;",
        "storage_size": size if size else 1,
    }


def helper_context(messages: List[Dict[str, object]]) -> Dict[str, bool]:
    sizes = {2}  # RT header seq/crc helpers always need u16.
    f32 = False
    f64 = False
    for msg in messages:
        for field in msg["fields"]:
            size = int(field["size"])
            if field["array_len"] and not field["is_raw_byte_array"] and not field["is_bool"]:
                sizes.add(size)
            elif not field["array_len"]:
                sizes.add(size)
            if field["is_float"] and size == 4:
                f32 = True
            if field["is_float"] and size == 8:
                f64 = True
    if f32:
        sizes.add(4)
    if f64:
        sizes.add(8)
    return {"u16": 2 in sizes, "u32": 4 in sizes, "u64": 8 in sizes, "f32": f32, "f64": f64}


def build_context(namespace: str, enums: List[EnumDef], messages: List[MessageDef]) -> Dict[str, object]:
    enum_map = {enum.name: enum for enum in enums}
    rt_messages = [message_context(message, enum_map) for message in messages]
    max_payload = max([int(message["size"]) for message in rt_messages], default=0)
    return {
        "namespace": namespace,
        "helpers": helper_context(rt_messages),
        "enums": [
            {
                "name": enum.name,
                "snake": to_snake(enum.name),
                "doc_block": "",
                "ctype": BUILTIN_TYPES[enum.base_type][0],
                "members": [{"name": m.name, "value": m.value_text, "doc_line": ""} for m in enum.members],
            }
            for enum in enums
        ],
        "rt": {
            "max_payload": max_payload,
            "payload_storage": max_payload if max_payload else 1,
            "messages": rt_messages,
        },
    }


def canonical_schema(namespace: str, enums: List[EnumDef], messages: List[MessageDef]) -> str:
    out: List[str] = [f"namespace {namespace};", ""]
    for enum in enums:
        out.append(f"enum {enum.name}: {enum.base_type} {{")
        for member in enum.members:
            out.append(f"  {member.name} = {member.value_text};")
        out.append("}")
        out.append("")
    for msg in messages:
        out.append(f"message {msg.name} @msg_id(0x{msg.msg_id:02X}) @direction({msg.direction}) {{")
        for f in msg.fields:
            if f.array_len:
                out.append(f"  {f.name}: [{f.type_name}:{f.array_len}];")
            else:
                out.append(f"  {f.name}: {f.type_name};")
        out.append("}")
        out.append("")
    return "\n".join(out).rstrip() + "\n"


def generated_notice(schema_path: str) -> str:
    return (
        "/**\n"
        " * @file\n"
        " * @brief Proto2 RT \u81ea\u52a8\u751f\u6210\u6587\u4ef6\uff0c\u8bf7\u52ff\u624b\u52a8\u4fee\u6539\u3002\n"
        " *\n"
        f" * @see {schema_path.replace(os.sep, '/')}\n"
        " */\n"
    )


def template_env() -> Environment:
    return Environment(
        loader=FileSystemLoader(Path(__file__).with_name("templates")),
        trim_blocks=True,
        lstrip_blocks=True,
        keep_trailing_newline=True,
        undefined=StrictUndefined,
    )


def write_files_atomic(files: List[Tuple[Path, str]]) -> None:
    staged: List[Tuple[Path, Path]] = []
    try:
        for path, content in files:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
            tmp_path = Path(tmp)
            staged.append((tmp_path, path))
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(content)
        for tmp, final in staged:
            os.replace(tmp, final)
    finally:
        for tmp, _ in staged:
            if tmp.exists():
                tmp.unlink()


def render(out_dir: Path, context: Dict[str, object], schema_display: str) -> List[Path]:
    env = template_env()
    context = dict(context)
    context["generated_notice"] = generated_notice(schema_display)
    outputs = {
        "proto2_rt.h.j2": "proto2_rt.h",
        "proto2_rt.c.j2": "proto2_rt.c",
        "proto2_rt.hpp.j2": "proto2_rt.hpp",
    }
    files = [(out_dir / out_name, env.get_template(tpl).render(**context)) for tpl, out_name in outputs.items()]
    write_files_atomic(files)
    return [path for path, _ in files]


def write_manifest(out_dir: Path, schema_path: Path, managed: List[Path]) -> None:
    manifest = {
        "schema": schema_path.name,
        "managed": [path.name for path in managed] + ["proto2_rt_codegen_manifest.json"],
    }
    write_files_atomic([(out_dir / "proto2_rt_codegen_manifest.json", json.dumps(manifest, indent=2) + "\n")])


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a compact Proto2 RT C core from a schema.hpp file.")
    parser.add_argument("schema")
    parser.add_argument("--out-dir", default="generated")
    parser.add_argument("--bundle-dir", default=None, help="Optional bundle root; writes device/generated and schema only.")
    args = parser.parse_args()

    schema_path = Path(args.schema).resolve()
    text = schema_path.read_text(encoding="utf-8")
    namespace, enums, messages = parse_schema(text, str(schema_path))
    canonical = canonical_schema(namespace, enums, messages)

    canonical_path = canonical_schema_path(str(schema_path))
    if canonical_path:
        write_files_atomic([(Path(canonical_path), canonical)])

    if args.bundle_dir:
        bundle = Path(args.bundle_dir).resolve()
        out_dir = bundle / "device" / "generated"
        schema_out = bundle / "schema" / (Path(canonical_path).name if canonical_path else schema_path.name)
        write_files_atomic([(schema_out, canonical)])
    else:
        out_dir = Path(args.out_dir).resolve()

    context = build_context(namespace, enums, messages)
    schema_display = os.path.relpath(schema_path, out_dir.parent).replace("\\", "/")
    managed = render(out_dir, context, schema_display)
    write_manifest(out_dir, schema_path, managed)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SchemaError as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1) from exc
