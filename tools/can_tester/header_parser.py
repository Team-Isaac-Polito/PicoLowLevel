"""
Minimal C preprocessor for PicoLowLevel header files.

Parses ``#define`` macros from communication.h and mod_config.h at runtime,
correctly handling ``#if defined()``, ``#elif``, ``#else``, and ``#endif``
blocks so the Python CAN tester always reflects the firmware's actual
definitions — no manual sync needed.

Usage::

    from header_parser import parse_communication_header, parse_mod_config

    msg_defines = parse_communication_header()      # all CAN message IDs
    mod_defines = parse_mod_config("MK2_MOD1")      # module-specific config
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, Optional, Set

# ─── paths ───────────────────────────────────────────────────────────────────

_DEFAULT_INCLUDE_DIR = (
    Path(__file__).resolve().parent.parent.parent
    / "PicoLowLevel" / "include"
)

# ─── regex ───────────────────────────────────────────────────────────────────

# Matches:  #define NAME 0xFF  |  #define NAME 123
_DEFINE_VALUE_RE = re.compile(
    r"^\s*#define\s+(\w+)\s+(0[xX][0-9a-fA-F]+|\d+)"
)
# Matches:  #define NAME  (flag-style, no value)
_DEFINE_FLAG_RE = re.compile(
    r"^\s*#define\s+(\w+)\s*$"
)
# Matches:  #if defined(NAME)  |  #ifdef NAME
_IF_DEFINED_RE = re.compile(
    r"^\s*#if\s+defined\s*\(\s*(\w+)\s*\)|^\s*#ifdef\s+(\w+)"
)
# Matches:  #elif defined(NAME)
_ELIF_DEFINED_RE = re.compile(
    r"^\s*#elif\s+defined\s*\(\s*(\w+)\s*\)"
)
# Matches:  #ifndef NAME
_IFNDEF_RE = re.compile(
    r"^\s*#ifndef\s+(\w+)"
)


# ─── preprocessor engine ────────────────────────────────────────────────────

def preprocess_header(
    path: Path,
    predefined: Optional[Set[str]] = None,
) -> Dict[str, int]:
    """Parse a C header file through a minimal preprocessor.

    Handles ``#define``, ``#if defined()``, ``#ifdef``, ``#ifndef``,
    ``#elif defined()``, ``#else``, and ``#endif``.

    Args:
        path: Path to the ``.h`` file.
        predefined: Set of macro names to treat as already defined
                    (e.g. ``{"MK2_MOD1"}``).  These are used to evaluate
                    ``#if defined()`` conditionals.

    Returns:
        ``{MACRO_NAME: int_value}`` for every ``#define NAME <number>``
        that is active given the conditionals.
    """
    predefined = predefined or set()
    known_flags: Set[str] = set(predefined)  # tracks flag defines
    defines: Dict[str, int] = {}

    # Stack of (this_branch_active, any_branch_taken) per nesting level.
    # "this_branch_active" = are we currently emitting defines?
    # "any_branch_taken"   = has any branch of this #if/elif/else been active?
    cond_stack: list[tuple[bool, bool]] = []

    def _active() -> bool:
        """True when all enclosing conditionals are active."""
        return all(active for active, _ in cond_stack)

    with open(path, "r") as fh:
        for raw_line in fh:
            line = raw_line.strip()

            # ── #ifndef ─────────────────────────────────────────────
            m = _IFNDEF_RE.match(line)
            if m:
                name = m.group(1)
                active = name not in known_flags
                cond_stack.append((active, active))
                continue

            # ── #if defined(NAME) / #ifdef NAME ─────────────────────
            m = _IF_DEFINED_RE.match(line)
            if m:
                name = m.group(1) or m.group(2)
                active = name in known_flags
                cond_stack.append((active, active))
                continue

            # ── #elif defined(NAME) ─────────────────────────────────
            m = _ELIF_DEFINED_RE.match(line)
            if m and cond_stack:
                name = m.group(1)
                _, any_taken = cond_stack[-1]
                if any_taken:
                    # a previous branch was active → skip
                    cond_stack[-1] = (False, True)
                else:
                    active = name in known_flags
                    cond_stack[-1] = (active, active)
                continue

            # ── #else ───────────────────────────────────────────────
            if line.startswith("#else") and cond_stack:
                _, any_taken = cond_stack[-1]
                cond_stack[-1] = (not any_taken, True)
                continue

            # ── #endif ──────────────────────────────────────────────
            if line.startswith("#endif") and cond_stack:
                cond_stack.pop()
                continue

            # Only process defines when all enclosing conditionals active
            if not _active():
                # Still skip other directives
                continue

            # ── #define NAME VALUE (numeric) ────────────────────────
            m = _DEFINE_VALUE_RE.match(raw_line)
            if m:
                name, val = m.group(1), m.group(2)
                defines[name] = int(val, 0)
                known_flags.add(name)
                continue

            # ── #define NAME (flag) ─────────────────────────────────
            m = _DEFINE_FLAG_RE.match(raw_line)
            if m:
                known_flags.add(m.group(1))

    return defines


# ─── convenience wrappers ────────────────────────────────────────────────────

def parse_communication_header(
    include_dir: Optional[Path] = None,
) -> Dict[str, int]:
    """Parse ``communication.h`` and return all CAN message-type defines.

    Returns:
        ``{MACRO_NAME: int_value}`` — every ``#define`` with a numeric value,
        excluding include guards.

    Raises:
        FileNotFoundError: if communication.h is not at the expected path.
    """
    include_dir = include_dir or _DEFAULT_INCLUDE_DIR
    path = include_dir / "communication.h"

    if not path.exists():
        raise FileNotFoundError(
            f"communication.h not found at {path}. "
            f"Ensure you're running from within the PicoLowLevel repo."
        )

    return preprocess_header(path)


def parse_mod_config(
    module_variant: str = "MK2_MOD1",
    include_dir: Optional[Path] = None,
) -> Dict[str, int]:
    """Parse ``mod_config.h`` with a specific module variant active.

    The variant name (e.g. ``MK2_MOD1``) is passed as a predefined macro so
    that ``#if defined(MK2_MOD1)`` blocks resolve correctly.

    Args:
        module_variant: One of MK1_MOD1, MK1_MOD2, MK2_MOD1, MK2_MOD2, MK2_MOD3.
        include_dir: Override for the include directory path.

    Returns:
        ``{MACRO_NAME: int_value}`` for numeric defines active under the
        chosen variant.

    Raises:
        FileNotFoundError: if mod_config.h is not at the expected path.
    """
    include_dir = include_dir or _DEFAULT_INCLUDE_DIR
    path = include_dir / "mod_config.h"

    if not path.exists():
        raise FileNotFoundError(f"mod_config.h not found at {path}")

    return preprocess_header(path, predefined={module_variant})
