#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""strip_comments.py -- 生成 C/C++ 源文件的"无注释"版本。

用法：
    python .selfcheck/strip_comments.py <输入文件> <输出文件> [--lines]

模式（默认 --compact）：
    compact 只删注释：**原本就是空行的行保留**，整行都是注释的行整行删掉，
            行首注释（`/* … */ code`）留下的缩进也去掉，连续空行折叠成 1 行，
            文件首尾的空行去掉。结果看起来就是"正常的干净代码"，但行号不再与
            带注释版对应。
    --lines 只删注释、**换行一律保留**，行号与带注释版逐行对应（编译报错行号、
            .selfcheck/README.md 里引用的行号都仍然有效），代价是长注释块的位置
            剩下一片空行。

两种模式都保证：字符串/字符字面量里的 // 与 /* 不动（含 \\ \" \' 转义）、
// 行注释以反斜杠结尾时按 C 规则继续吃掉下一行、去掉 CR、保留原文件的 UTF-8
BOM 有无状态。等价性由 `.selfcheck/make_nocomment.ps1` 用"预处理后的记号流
逐字符相同"证明，所以 compact 模式删掉换行是安全的。

为什么用 Python：需要一个真正的词法状态机（字符串里的 "//"、字符字面量里的
'/'、跨行块注释、行尾续行），Python 写起来又短又不容易错；docs/ 里的文档生成
脚本本来也是 Python。

输出一行统计：ok <文件> mode=<模式> in=<字节> out=<字节> lines=<行数> nonascii=<剩余非 ASCII 字节>
"""
import os
import sys


def _rstrip(out):
    while out and out[-1] in " \t":
        out.pop()


def strip_comments(text):
    """去掉注释；换行数与输入完全一致（第二阶段再决定要不要删空行）。"""
    out = []
    i = 0
    n = len(text)
    state = "code"          # code | line | block | string | char
    while i < n:
        c = text[i]
        if state == "code":
            if c == "/" and i + 1 < n:
                d = text[i + 1]
                if d == "/":
                    state = "line"
                    i += 2
                    continue
                if d == "*":
                    state = "block"
                    i += 2
                    continue
            if c == '"':
                state = "string"
                out.append(c)
                i += 1
                continue
            if c == "'":
                state = "char"
                out.append(c)
                i += 1
                continue
            if c == "\n":
                _rstrip(out)
                out.append("\n")
                i += 1
                continue
            if c == "\r":       # 统一成 LF（本仓库源文件本来就没有 CR）
                i += 1
                continue
            out.append(c)
            i += 1
            continue

        if state == "line":
            if c == "\\" and i + 1 < n and text[i + 1] == "\n":
                out.append("\n")        # 续行的注释：整行都是注释
                i += 2
                continue
            if c == "\n":
                state = "code"
                _rstrip(out)
                out.append("\n")
                i += 1
                continue
            i += 1
            continue

        if state == "block":
            if c == "*" and i + 1 < n and text[i + 1] == "/":
                state = "code"
                i += 2
                continue
            if c == "\n":
                _rstrip(out)
                out.append("\n")
                i += 1
                continue
            i += 1
            continue

        # string / char
        q = '"' if state == "string" else "'"
        if c == "\\":
            out.append(c)
            if i + 1 < n:
                out.append(text[i + 1])
                i += 2
            else:
                i += 1
            continue
        out.append(c)
        if c == q:
            state = "code"
        i += 1
        continue

    if state in ("block", "string", "char"):
        sys.stderr.write("warning: unterminated %s state at EOF\n" % state)
    return "".join(out)


def compact(text, stripped):
    """把"纯注释行"整行删掉，保留作者原本的空行（连续空行折叠成 1 行）。"""
    src = text.split("\n")
    dst = stripped.split("\n")
    if len(src) != len(dst):
        raise AssertionError("line count changed while stripping")

    kept = []
    for s, d in zip(src, dst):
        if d.strip() == "":
            if s.strip() == "":         # 作者本来就写了空行 -> 保留
                kept.append("")
            # 否则这一行只是注释 -> 整行丢掉
            continue
        if s.lstrip().startswith("/"):  # 行首是注释，后面才是代码
            d = d.lstrip()
        kept.append(d.rstrip())

    out = []
    for ln in kept:                     # 连续空行折叠成 1 行
        if ln == "" and out and out[-1] == "":
            continue
        out.append(ln)
    while out and out[0] == "":
        out.pop(0)
    while out and out[-1] == "":
        out.pop()

    res = "\n".join(out)
    return res + "\n" if res else ""


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    flags = [a for a in argv[1:] if a.startswith("--")]
    if len(args) != 2 or [f for f in flags if f not in ("--lines", "--compact")]:
        sys.stderr.write(__doc__)
        return 2
    mode = "lines" if "--lines" in flags else "compact"
    src, dst = args

    with open(src, "rb") as fh:
        raw = fh.read()
    bom = raw[:3] == b"\xef\xbb\xbf"
    body = raw[3:] if bom else raw
    text = body.decode("utf-8")

    stripped = strip_comments(text)
    if stripped.count("\n") != text.count("\n"):
        sys.stderr.write("error: line count changed for %s\n" % src)
        return 1

    result = stripped if mode == "lines" else compact(text, stripped)
    out = result.encode("utf-8")
    with open(dst, "wb") as fh:
        if bom:
            fh.write(b"\xef\xbb\xbf")
        fh.write(out)

    nonascii = sum(1 for b in out if b > 0x7F)
    print("ok %s mode=%s in=%d out=%d lines=%d nonascii=%d"
          % (os.path.basename(src), mode, len(raw), len(out) + (3 if bom else 0),
             result.count("\n"), nonascii))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
