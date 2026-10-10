# -*- coding: utf-8 -*-
"""校验 docs/meArm_review_guide.docx：结构计数 + 关键内容抽查 + 乱码检查。"""
import io
import os
import re
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_content  # noqa: E402

DOCX = os.path.join(HERE, "meArm_review_guide.docx")
REPORT = os.path.join(os.path.dirname(HERE), ".selfcheck", "out",
                      "review_docx_validate_report.txt")

doc = review_content.DOC
kinds = {}
for item in doc:
    kinds[item[0]] = kinds.get(item[0], 0) + 1
n_tables = kinds.get("tbl", 0)
n_paras = sum(1 for i in doc if i[0] in ("p", "pc", "code"))
n_bullets = sum(len(i[1]) for i in doc if i[0] in ("bul", "num"))
table_rows = [len(i[3]) for i in doc if i[0] == "tbl"]
chars = 0
for item in doc:
    if item[0] in ("p", "pc", "code", "title", "h1", "h2", "h3"):
        chars += len(item[1])
    elif item[0] in ("bul", "num"):
        chars += sum(len(t) for t in item[1])
    elif item[0] == "tbl":
        chars += len(item[1]) + sum(len(c) for r in item[3] for c in r)

z = zipfile.ZipFile(DOCX)
xml = z.read("word/document.xml").decode("utf-8")
n_w_p = xml.count("<w:p ")
n_w_tbl = xml.count("<w:tbl>")
n_w_drawing = xml.count("<w:drawing>")
n_omath = xml.count("<m:oMath>")
bad = xml.count("\ufffd")

KEY = ["系统总览", "各板块实现逻辑", "示教点记录", "轨迹生成", "速度控制",
       "异常处理", "未完成功能", "评审速查", "constant_and_positions",
       "path_core.h", "servo_drive", "Catmull-Rom", "moveToPoint",
       "31656", "38.4", "WEARM_SHOULDER_HEIGHT", "DRAW_MAX_DPS"]
missing = [k for k in KEY if k not in xml]

lines = []
lines.append("== 内容统计（review_content.DOC） ==")
lines.append("条目类型计数: %s" % kinds)
lines.append("段落数(p/pc/code): %d；列表项(bul/num): %d" % (n_paras, n_bullets))
lines.append("表格数: %d；表格数据行合计: %d（最大单表 %d 行）"
             % (n_tables, sum(table_rows), max(table_rows) if table_rows else 0))
lines.append("正文字符数（含表格，未计 ** 标记）: %d" % chars)
lines.append("")
lines.append("== docx 结构（word/document.xml） ==")
lines.append("zip 项: %d；document.xml %d 字节" % (len(z.namelist()), len(xml)))
lines.append("w:p %d；w:tbl %d；w:drawing %d；m:oMath %d"
             % (n_w_p, n_w_tbl, n_w_drawing, n_omath))
lines.append("U+FFFD 替换字符: %d" % bad)
lines.append("")
lines.append("== 关键内容抽查 ==")
lines.append("缺失关键词: %s" % (missing if missing else "无（%d 项全部命中）" % len(KEY)))
lines.append("文档大小: %d 字节" % os.path.getsize(DOCX))

txt = "\n".join(lines)
io.open(REPORT, "w", encoding="utf-8", newline="\n").write(txt)
print(txt)
