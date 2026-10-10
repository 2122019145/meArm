# -*- coding: utf-8 -*-
"""校验 docs/meArm_program_guide.docx：结构、必备主题、行号一致性、乱码。

用法:
    python docs/validate_program_guide.py [docx路径]

检查项（任一失败即以非零码退出，并把报告写到 .selfcheck/out/）:
  1) 段落数、表格数、正文字符数（估算页数用）
  2) 必备章节关键词是否齐全（任务一~五 + 四个专题 + 验证）
  3) 单文件版行号引用是否与 single/meArm/meArm.ino 的实际分块一致
  4) 是否出现替换字符 / 常见 GBK 误读乱码
"""
import os
import re
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SINGLE = os.path.join(ROOT, "single", "meArm", "meArm.ino")
OUTDIR = os.path.join(ROOT, ".selfcheck", "out")

REQUIRED = [
    "任务一", "任务二", "任务三", "任务四", "任务五",
    "示教点记录", "轨迹生成", "速度控制", "异常处理",
    "忙让位", "卡死", "串口", "运动学", "未在实体 Uno 上烧录验证",
]

# 模块 -> 单文件里期望的起始行号（用 "// ===== 名字 =====" 标记定位）
MODULE_BANNERS = [
    ("weArm_config.h", 4),
    ("constant_and_positions.h", 250),
    ("protocol_constants.h", 487),
    ("servo_drive.h", 579),
    ("move.h", 655),
    ("serial_protocol.h", 795),
    ("pick_place.h", 915),
    ("button_control.h", 999),
    ("draw_control.h", 1134),
    ("joystick_control.h", 1421),
    ("path_core.h", 1557),
    ("servo_drive.cpp", 1662),
    ("constant_and_positions.cpp", 1792),
    ("protocol_constants.cpp", 2654),
    ("move.cpp", 2717),
    ("joystick_control.cpp", 2934),
    ("pick_place.cpp", 3458),
    ("draw_control.cpp", 3964),
    ("button_control.cpp", 5364),
    ("serial_protocol.cpp", 6142),
    ("meArm.ino", 7106),
]

BAD_PATTERNS = ["锛", "鍏", "锟斤拷", "\ufffd", "鐨", "鏄"]


def docx_text(path):
    """把所有 w:t 文本抓出来（保持段落顺序），再加表格里的文本。"""
    with zipfile.ZipFile(path) as z:
        xml = z.read("word/document.xml").decode("utf-8")
    xml = xml.replace("</w:p>", "\n</w:p>")
    parts = re.findall(r"<w:t[^>]*>(.*?)</w:t>", xml, flags=re.S)
    text = "".join(parts)
    # 段落标记也补成换行，便于统计
    text = text.replace("</w:p>", "\n")
    return text


def main():
    docx = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, "meArm_program_guide.docx")
    problems = []
    if not os.path.exists(docx):
        print("FAIL: missing %s" % docx)
        return 2

    text = docx_text(docx)
    cjk = len(re.findall(r"[\u4e00-\u9fff]", text))

    with zipfile.ZipFile(docx) as z:
        xml = z.read("word/document.xml").decode("utf-8")
    n_par = xml.count("<w:p>") + xml.count("<w:p ")
    n_tbl = xml.count("<w:tbl>")

    for kw in REQUIRED:
        if kw not in text:
            problems.append("缺少必备主题: %s" % kw)

    for bad in BAD_PATTERNS:
        if bad in text:
            problems.append("疑似乱码/替换字符: %r" % bad)

    # 行号一致性：文档里写的模块行号应与单文件实际 banner 一致
    if os.path.exists(SINGLE):
        lines = open(SINGLE, encoding="utf-8").read().split("\n")
        actual = {}
        for i, ln in enumerate(lines, 1):
            m = re.match(r"^// ===== (.+?) =====$", ln.strip())
            if m:
                actual[m.group(1)] = i
        for name, expect in MODULE_BANNERS:
            got = actual.get(name)
            if got != expect:
                problems.append("行号漂移: %s 期望 %d 实际 %r" % (name, expect, got))
            stem = name[:-4] if name.endswith(".cpp") else name
            if name not in text and (stem + ".h/.cpp") not in text:
                problems.append("文档未提到模块: %s" % name)
    else:
        problems.append("找不到 %s，无法核对行号" % SINGLE)

    report = []
    report.append("docx=%s" % docx)
    report.append("段落=%d 表格=%d 中文字数=%d（页数以 Word 统计为准，"
                  "生成脚本产出后另用 Word 复核）" % (n_par, n_tbl, cjk))
    report.append("必备主题齐备=%s" % (not problems,))
    report.append("问题数=%d" % len(problems))
    for p in problems:
        report.append("  !! %s" % p)
    body = "\n".join(report)
    print(body)

    os.makedirs(OUTDIR, exist_ok=True)
    with open(os.path.join(OUTDIR, "program_guide_validate_report.txt"), "w",
              encoding="utf-8") as f:
        f.write(body + "\n")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
