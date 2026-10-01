"""What a program declares, read from its source WITHOUT importing it (ast + tokenize): the console's program list and
typed parameter form, and the Runner's safety flags.

    summary     the first sentence of the docstring's first paragraph
    params      DEFAULTS -- dict(k=v, ...) or {...} -- in source order: key, default, type, help (the comment on the
                value's line plus the comment-only lines directly under it)
    drives      DRIVES (default True): the program commands the wheels.  False = it may run while the E-STOP is
                latched, and the link watchdog ignores it
    manual_ok   MANUAL_OK (default False): the console may drive by hand while it runs (watch only)

Presets are stored beside params.json as presets.json, {program: {preset: {k: v}}}; the mock's go to .mock/.

The program LIBRARY (the console's PROGRAMS list: GET /api/library, PUT /api/library/{name}) is the team's own word on
each program -- note, rating 0-5, status, tags, challenge, rank (1 = top of the list, null = none) -- stored beside
params.json as library.json (BW_LIBRARY overrides), {program: {note, rating, status, tags, challenge, rank, updated,
by}}.  The console lists ranked programs first (rank, then rating, then name).  programs/_library.json is a read-only
seed shipped with the code: the store's fields win over it field by field, and a save never writes the seed (the mock
agent serves the repo's programs/ directly, and a note typed on the laptop must never land in the repo).  The last run
of each program is read from the tail of its newest runs/<id>.jsonl (its `end` event), cached by (mtime, size).
"""
from __future__ import annotations

import ast
import io
import json
import math
import os
import re
import threading
import tokenize

from . import params as P

_CACHE: dict = {}
_LOCK = threading.Lock()
PRESET_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,32}$")


def first_sentence(doc: str) -> str:
    para = re.split(r"\n\s*\n", (doc or "").strip(), maxsplit=1)[0]
    s = " ".join(para.split())
    ends = [i for i in (s.find(". "), s.find("? "), s.find("! ")) if i >= 0]
    return s[:min(ends) + 1] if ends else s


def _type(v) -> str:
    if v is None:
        return "none"
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, int):
        return "int"
    if isinstance(v, float):
        return "float"
    if isinstance(v, str):
        return "str"
    if isinstance(v, (list, tuple)):
        return "list"
    return "expr"


def comments(src: str):
    """{line: comment text} and the set of comment-only lines."""
    out, only = {}, set()
    try:
        for tok in tokenize.generate_tokens(io.StringIO(src).readline):
            if tok.type == tokenize.COMMENT:
                ln = tok.start[0]
                out[ln] = tok.string.lstrip("#").strip()
                if not tok.line[:tok.start[1]].strip():
                    only.add(ln)
    except (tokenize.TokenError, IndentationError, SyntaxError):
        pass
    return out, only


def help_for(line: int, com: dict, only: set) -> str:
    """The comment on `line` plus the comment-only lines directly under it."""
    if line not in com or line in only:
        return ""
    parts = [com[line]]
    k = line + 1
    while k in only:
        parts.append(com[k])
        k += 1
    return " ".join(p for p in parts if p)


def _params(tree, com, only):
    node = None
    for st in tree.body:
        if isinstance(st, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "DEFAULTS" for t in st.targets):
            node = st.value
    if node is None:
        return []
    items = []
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "dict":
        items = [(kw.arg, kw.value) for kw in node.keywords if kw.arg]
    elif isinstance(node, ast.Dict):
        items = [(k.value, v) for k, v in zip(node.keys, node.values)
                 if isinstance(k, ast.Constant) and isinstance(k.value, str)]
    out = []
    for key, val in items:
        try:
            default = ast.literal_eval(val)
            typ = _type(default)
            if isinstance(default, tuple):
                default = list(default)
            if typ == "expr":
                default = ast.unparse(val)
        except (ValueError, SyntaxError, TypeError):
            default, typ = ast.unparse(val), "expr"
        out.append(dict(key=key, default=default, type=typ, help=help_for(val.end_lineno, com, only)))
    return out


def _flag(tree, name: str, default: bool) -> bool:
    for st in tree.body:
        if isinstance(st, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in st.targets):
            try:
                return bool(ast.literal_eval(st.value))
            except (ValueError, SyntaxError, TypeError):
                return default
    return default


# a program's file name: what the list shows, the library keys and the Runner imports as bwprog_<name> -- no leading _
# (the list hides those), no dot (a module name), no space or other letters (the preset and tag rules' alphabet)
PROG_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")
DECODE_HINT = "save the file as UTF-8"


def name_ok(name: str) -> bool:
    return bool(PROG_NAME.match(name or ""))


def decode_source(data: bytes) -> str:
    """The text of an uploaded program, decoded the way Python (and meta) will read it: a UTF-8 BOM or a coding cookie
    is honoured, anything else is UTF-8.  ValueError when it does not decode -- compile(bytes) ACCEPTS a latin-1 or
    cp1256 comment (upload 200, then GET /api/programs 500 on every later read: library test 2026-09-30)."""
    try:
        enc, _lines = tokenize.detect_encoding(io.BytesIO(data).readline)
        return data.decode(enc)
    except (SyntaxError, UnicodeDecodeError, LookupError) as e:
        raise ValueError("cannot decode: %s (%s)" % (e, DECODE_HINT))


def read_source(path: str):
    """(text, error) of a program file, read as Python reads it (tokenize.open: a BOM, a coding cookie).  A file that
    does not decode reads with U+FFFD for the bad bytes and error says so: one bad file must never empty the list."""
    try:
        with tokenize.open(path) as f:
            return f.read(), None
    except (SyntaxError, UnicodeDecodeError, LookupError) as e:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read(), "cannot decode: %s (%s)" % (e, DECODE_HINT)


def meta_error(path: str, err) -> dict:
    """The row of a program meta() could not read at all: listed, with the reason, never an exception."""
    name = os.path.splitext(os.path.basename(path))[0]
    try:
        st = os.stat(path)
        mt, sz = st.st_mtime, st.st_size
    except OSError:
        mt = sz = None
    return dict(name=name, summary="", doc="", docstring="", drives=True, manual_ok=False, mtime=mt, bytes=sz,
                params=[], error="cannot read: %s: %s" % (type(err).__name__, err))


def meta(path: str) -> dict:
    """{name, summary, doc, docstring, drives, manual_ok, mtime, bytes, params, error}; cached by (path, mtime)."""
    st = os.stat(path)
    key = (path, st.st_mtime, st.st_size)
    with _LOCK:
        if key in _CACHE:
            return _CACHE[key]
    src, err = read_source(path)
    name = os.path.splitext(os.path.basename(path))[0]
    out = dict(name=name, summary="", doc="", docstring="", drives=True, manual_ok=False, mtime=st.st_mtime,
               bytes=st.st_size, params=[], error=err)
    try:
        tree = ast.parse(src)
        doc = ast.get_docstring(tree, clean=True) or ""
        com, only = comments(src)
        out.update(summary=first_sentence(doc), docstring=doc, drives=_flag(tree, "DRIVES", True),
                   manual_ok=_flag(tree, "MANUAL_OK", False), params=_params(tree, com, only))
        out["doc"] = out["summary"]
    except SyntaxError as e:
        out["error"] = err or "syntax error line %s: %s" % (e.lineno, e.msg)
    except (ValueError, RecursionError, MemoryError) as e:
        out["error"] = err or "cannot parse: %s: %s" % (type(e).__name__, e)
    with _LOCK:
        for k in [k for k in _CACHE if k[0] == path]:
            del _CACHE[k]
        _CACHE[key] = out
    return out


# ---------------------------------------------------------------------------------------------------------- presets
def _presets_path() -> str:
    return os.path.join(os.path.dirname(P.path()), "presets.json")


def _load() -> dict:
    try:
        with open(_presets_path(), encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _save(d: dict) -> None:
    path = _presets_path()
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, path)


def presets(program: str) -> dict:
    return _load().get(program, {})


def preset(program: str, name: str):
    return presets(program).get(name)


def preset_put(program: str, name: str, values: dict) -> None:
    if not PRESET_NAME.match(name or ""):
        raise ValueError("preset names are 1-32 of A-Z a-z 0-9 _ . -")
    with _LOCK:
        d = _load()
        d.setdefault(program, {})[name] = dict(values or {})
        _save(d)


def preset_delete(program: str, name: str) -> bool:
    with _LOCK:
        d = _load()
        if name not in d.get(program, {}):
            return False
        del d[program][name]
        if not d[program]:
            del d[program]
        _save(d)
        return True


# ---------------------------------------------------------------------------------------------------------- library
STATUSES = ("best", "testing", "old", "broken")          # "" = none
CHALLENGES = ("open", "obstacle", "tool")                # "" = auto (race.programs, then the docstring, then DRIVES)
FIELDS = ("note", "rating", "status", "tags", "challenge", "rank")
TAG = re.compile(r"^[A-Za-z0-9_.+-]{1,24}$")
NOTE_MAX = 4000
TAGS_MAX = 8
RANK_MAX = 99                                            # rank 1 = the top of the console's list; None = no rank
_LIB_LOCK = threading.Lock()


def _whole(v) -> bool:
    """A JSON whole number: not a bool, not NaN / inf (int() of those raises, and not as ValueError for inf)."""
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) and v == int(v)


def rank_of(entry) -> int | None:
    """The rank of a library entry (seed or store) as the list reads it: a whole number 1..RANK_MAX, else None.  The
    seed is read raw (never through clean), so a hand-typed "2" or 0 there is no rank, never an error."""
    v = entry.get("rank") if isinstance(entry, dict) else None
    return int(v) if _whole(v) and 1 <= v <= RANK_MAX else None


def library_path() -> str:
    return os.environ.get("BW_LIBRARY") or os.path.join(os.path.dirname(P.path()), "library.json")


def seed_path(prog_dir: str) -> str:
    return os.path.join(prog_dir, "_library.json")


def _read_lib(path: str) -> dict:
    """{program: entry}; a missing file is {}.  ValueError when the file exists but is not a JSON object."""
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except FileNotFoundError:
        return {}
    except OSError:
        return {}
    if not isinstance(d, dict):
        raise ValueError("not a JSON object")
    return {k: v for k, v in d.items() if isinstance(k, str) and isinstance(v, dict) and not k.startswith("_")}


def library(prog_dir: str) -> dict:
    """The seed overlaid by the store, field by field: {program: entry}.  A broken seed or store reads as {} here
    (a save moves a broken store aside first, never over it)."""
    try:
        seed = _read_lib(seed_path(prog_dir))
    except ValueError:
        seed = {}
    try:
        with _LIB_LOCK:                                  # never open while a save replaces it (Windows refuses that)
            store = _read_lib(library_path())
    except ValueError:
        store = {}
    out = {k: dict(v) for k, v in seed.items()}
    for k, v in store.items():
        out[k] = dict(out.get(k, {}), **v)
    return out


def clean(patch: dict) -> dict:
    """The validated subset of a PUT body; ValueError names the first bad field (the whole save is refused)."""
    if not isinstance(patch, dict):
        raise ValueError("the body is a JSON object of %s" % ", ".join(FIELDS))
    out = {}
    for k, v in patch.items():
        if k not in FIELDS:
            raise ValueError("unknown field %s (one of %s)" % (k, ", ".join(FIELDS)))
        if k == "note":
            if v is None:
                v = ""
            if not isinstance(v, str):
                raise ValueError("note is text")
            v = v.replace("\r\n", "\n").replace("\r", "\n")
            if len(v) > NOTE_MAX:
                raise ValueError("note is at most %d characters (%d sent)" % (NOTE_MAX, len(v)))
        elif k == "rating":
            if isinstance(v, bool) or not isinstance(v, (int, float)) or v != int(v) or not 0 <= v <= 5:
                raise ValueError("rating is a whole number 0-5 (0 = not rated)")
            v = int(v)
        elif k == "status":
            v = "" if v is None else v
            if v not in ("",) + STATUSES:
                raise ValueError("status is one of %s, or empty" % ", ".join(STATUSES))
        elif k == "challenge":
            v = "" if v is None else v
            if v not in ("",) + CHALLENGES:
                raise ValueError("challenge is one of %s, or empty for auto" % ", ".join(CHALLENGES))
        elif k == "rank":                                # None / "" / 0 clear it (the store's None wins over the seed)
            if v is None or v == "" or (_whole(v) and v == 0):
                v = None
            elif not (_whole(v) and 1 <= v <= RANK_MAX):
                raise ValueError("rank is a whole number 1-%d (1 = top of the list), or empty / 0 for none" % RANK_MAX)
            else:
                v = int(v)
        elif k == "tags":
            if v is None:
                v = []
            if isinstance(v, str):
                v = v.split(",")
            if not isinstance(v, list) or not all(isinstance(t, str) for t in v):
                raise ValueError("tags is a list of words (or one comma-separated text)")
            seen = []
            for t in (t.strip() for t in v):
                if not t:
                    continue
                if not TAG.match(t):
                    raise ValueError("tag %r: 1-24 of A-Z a-z 0-9 _ . + -" % t[:30])
                if t.lower() not in [s.lower() for s in seen]:
                    seen.append(t)
            if len(seen) > TAGS_MAX:
                raise ValueError("at most %d tags" % TAGS_MAX)
            v = seen
        out[k] = v
    return out


def _write_atomic(path: str, d: dict) -> None:
    """tmp in the same directory, flushed and fsynced, then os.replace: a power cut leaves the old file or the new
    one, never half of one (the Pi has no clean shutdown on the mat).  On Windows (the laptop's mock agent)
    os.replace is refused while any other handle has the file open -- a reader in another process, an antivirus
    scan: measured 2 of 5 runs of test_concurrent_puts_all_land before readers took the lock -- so a refusal is
    retried for up to ~1 s."""
    import time as _t
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = "%s.%d.tmp" % (path, os.getpid())
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(d, f, indent=1, sort_keys=True, ensure_ascii=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    for k in range(20):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            if k == 19:
                try:
                    os.remove(tmp)
                except OSError:
                    pass
                raise
            _t.sleep(0.05)


class Conflict(Exception):
    """A note saved from an older copy: another device saved this program's note after the editor loaded it."""

    def __init__(self, entry: dict):
        super().__init__("the note was saved elsewhere after this copy was loaded")
        self.entry = entry


def frozen_names(prog_dir: str) -> set:
    """Programs tagged `frozen` in the seed OR the store (Fawaz's rule: printed / shipped files are never edited, a new
    version is a new file).  The seed's tag cannot be taken away from the page: removing it in the store's tags only
    changes what the list shows."""
    out = set()
    for src in ("seed", "store"):
        try:
            if src == "seed":
                d = _read_lib(seed_path(prog_dir))
            else:
                with _LIB_LOCK:
                    d = _read_lib(library_path())
        except ValueError:
            continue
        for k, v in d.items():
            if any(str(t).strip().lower() == "frozen" for t in (v.get("tags") or []) if isinstance(t, str)):
                out.add(k)
    return out


def library_put(prog_dir: str, name: str, patch: dict, by: str = "?", now: float | None = None,
                if_note: float | None = None) -> dict:
    """Merge the validated fields into the store's entry of `name` (updated, by stamped) and return the merged entry
    (seed + store).  A store that is not JSON is moved aside to library.json.bad-<time> first: a save over it would
    have dropped every other program's notes.  `if_note` (a note save from the page): the note_updated the editor
    loaded (0 = none); a newer one in the store raises Conflict -- a phone with an older copy never overwrites a
    newer note silently.  The note alone carries the stamp: a rating saved elsewhere is no conflict for a note."""
    import time as _t
    fields = clean(patch)
    path = library_path()
    with _LIB_LOCK:
        try:
            store = _read_lib(path)
        except ValueError:                               # not JSON, or not an object: keep it for a human
            try:
                os.replace(path, "%s.bad-%s" % (path, _t.strftime("%Y%m%d-%H%M%S")))
            except OSError:
                pass
            store = {}
        e = dict(store.get(name, {}))
        if if_note is not None and "note" in fields and float(e.get("note_updated") or 0.0) > float(if_note) + 1e-3 \
                and fields["note"] != e.get("note"):
            raise Conflict(e)
        stamp = round(now if now is not None else _t.time(), 3)
        if "note" in fields and fields["note"] != e.get("note"):
            e["note_updated"] = stamp
        e.update(fields)
        e["updated"] = stamp
        e["by"] = str(by or "?")[:60]
        store[name] = e
        _write_atomic(path, store)
    return library(prog_dir).get(name, e)


def challenge_auto(m: dict, race: list | None = None) -> str:
    """Which challenge a program is for, when nobody said: race mode's own mapping (params race.programs) first, then
    the first paragraph of its docstring ("Open Challenge" / "obstacle challenge"), then DRIVES False = a tool."""
    for r in race or []:
        if r in ("open", "obstacle"):
            return r
    doc = (m.get("docstring") or "").strip()
    head = re.split(r"\n\s*\n", doc, maxsplit=1)[0] if doc else ""
    if re.search(r"\bopen\s+challenge\b", head, re.I):
        return "open"
    if re.search(r"\bobstacle\s+challenge\b", head, re.I):
        return "obstacle"
    if m.get("drives") is False:
        return "tool"
    return ""


# ---------------------------------------------------------------------------------------------------------- last run
_END_CACHE: dict = {}
TAIL = (96 * 1024, 1024 * 1024)                          # bytes read from the end: first try, then once more


def run_index(run_dir: str) -> dict:
    """{program: [run ids oldest -> newest]} of runs/<id>.jsonl (the id starts with its local start time, so the
    name order is the time order)."""
    from . import runlog
    out: dict = {}
    try:
        names = os.listdir(run_dir)
    except OSError:
        return out
    for fn in sorted(names):
        if not fn.endswith(".jsonl") or fn == "tests.jsonl":
            continue
        rid = fn[:-6]
        prog = runlog.parse_id(rid).get("program")
        if prog:
            out.setdefault(prog, []).append(rid)
    return out


def _tail_end(path: str, size: int) -> dict:
    """Scan the file's tail backwards: the program's `end` event, its last error, the Runner's end marker."""
    res = dict(end=None, error=None, ended=False)
    for n in TAIL:
        with open(path, "rb") as f:
            f.seek(max(0, size - n))
            chunk = f.read(n)
        lines = chunk.split(b"\n")
        if size > n:
            lines = lines[1:]                            # the first line of a mid-file read is a fragment
        for raw in reversed(lines):
            if not raw.strip():
                continue
            if b"-- program ended --" in raw:
                res["ended"] = True
                continue
            has_end = b'"ev": "end"' in raw or b'"ev":"end"' in raw
            has_err = b'"error"' in raw and b'"log"' in raw
            if not (has_end or has_err):
                continue
            try:
                rec = json.loads(raw)
            except ValueError:
                continue
            x = rec.get("log") if isinstance(rec, dict) else None
            if not isinstance(x, dict):
                continue
            if x.get("ev") == "end" and res["end"] is None:
                res["end"] = dict(x, t=rec.get("t"))
            elif "error" in x and res["error"] is None:
                last = [l for l in str(x["error"]).strip().splitlines() if l.strip()]
                res["error"] = last[-1].strip()[:200] if last else "error"
            if res["end"] is not None:
                return res
        if size <= n:
            break
    return res


def last_run(path: str, rid: str, parse: bool = True) -> dict:
    """{id, started, bytes, reason, laps, seconds, corners, error, ended, parsed} of one run file.  `parse` False
    (a DRIVES program runs: this process is the program's) answers from the cache only -- parsed False otherwise."""
    from . import runlog
    out = dict(id=rid, started=runlog.parse_id(rid).get("started"), bytes=None, reason=None, laps=None, seconds=None,
               corners=None, error=None, ended=None, parsed=False)
    try:
        st = os.stat(path)
    except OSError:
        return out
    out["bytes"] = st.st_size
    key = (st.st_mtime, st.st_size)
    with _LIB_LOCK:
        hit = _END_CACHE.get(path)
    if hit and hit[0] == key:
        return dict(out, **hit[1])
    if not parse:
        return out
    try:
        r = _tail_end(path, st.st_size)
    except OSError:
        return out
    end = r["end"] or {}
    got = dict(parsed=True, ended=r["ended"], error=r["error"], laps=end.get("laps"), seconds=end.get("seconds"),
               corners=end.get("corners"),
               reason=end.get("reason") if end else ("error" if r["error"] else "ended" if r["ended"] else "incomplete"))
    with _LIB_LOCK:
        if len(_END_CACHE) > 512:
            _END_CACHE.clear()
        _END_CACHE[path] = (key, got)
    return dict(out, **got)
