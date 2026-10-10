# -*- coding: utf-8 -*-
"""生成《meArm 机械臂运动学正解与逆解推导》Word 文档。

公式全部写成 Word 原生公式对象（OMML，m:oMath），可在 Word 中继续编辑，
不是图片。文本为中文，公式符号与源码 constant_and_positions.cpp 逐式对应。

用法: python make_kinematics_docx.py [输出路径]
"""
import os
import sys

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import parse_xml
from docx.oxml.ns import qn
from docx.shared import Pt, RGBColor

MNS = 'http://schemas.openxmlformats.org/officeDocument/2006/math'
OUT_DEFAULT = r"D:\dsh1\meArm\docs\kinematics_derivation.docx"


# --------------------------------------------------------------------------- #
# OMML 基础构件
# --------------------------------------------------------------------------- #
def esc(t):
    return (t.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))


def r(t):
    """数学斜体变量/文本 run（Word 对拉丁字母默认斜体）。"""
    return '<m:r><m:t xml:space="preserve">%s</m:t></m:r>' % esc(t)


def op(t):
    """直立（非斜体）run：函数名、数字、单位、符号文字。"""
    return ('<m:r><m:rPr><m:sty m:val="p"/></m:rPr>'
            '<m:t xml:space="preserve">%s</m:t></m:r>' % esc(t))


def frac(a, b):
    return '<m:f><m:num>%s</m:num><m:den>%s</m:den></m:f>' % (a, b)


def sup(a, b):
    return '<m:sSup><m:e>%s</m:e><m:sup>%s</m:sup></m:sSup>' % (a, b)


def sub(a, b):
    return '<m:sSub><m:e>%s</m:e><m:sub>%s</m:sub></m:sSub>' % (a, b)


def subsup(a, b, c):
    return '<m:sSubSup><m:e>%s</m:e><m:sub>%s</m:sub><m:sup>%s</m:sup></m:sSubSup>' % (a, b, c)


def rad(a):
    return ('<m:rad><m:radPr><m:degHide m:val="1"/></m:radPr>'
            '<m:deg/><m:e>%s</m:e></m:rad>' % a)


def paren(a, beg='(', end=')'):
    return ('<m:d><m:dPr><m:begChr m:val="%s"/><m:endChr m:val="%s"/></m:dPr>'
            '<m:e>%s</m:e></m:d>' % (beg, end, a))


def fnc(name, arg):
    return '<m:func><m:fName>%s</m:fName><m:e>%s</m:e></m:func>' % (name, arg)


def matrix(rows, beg='[', end=']'):
    """OMML 矩阵（m:m / m:mr / m:e）。"""
    mrs = ''.join('<m:mr>' + ''.join('<m:e>%s</m:e>' % c for c in row) + '</m:mr>'
                  for row in rows)
    return ('<m:d><m:dPr><m:begChr m:val="%s"/><m:endChr m:val="%s"/></m:dPr>'
            '<m:e><m:m>%s</m:m></m:e></m:d>' % (beg, end, mrs))


def vec(a):
    """带上划线的矢量记号（用 m:acc + 上划线组合近似为 m:bar）。"""
    return ('<m:acc><m:accPr><m:chr m:val="\u0304"/></m:accPr>'
            '<m:e>%s</m:e></m:acc>' % a)


# 常用符号
AL = r('α')
BE = r('β')
DE = r('δ')
TH = r('θ')
RHO = r('ρ')
L1 = sub(r('L'), op('1'))
L2 = sub(r('L'), op('2'))
HA = sub(r('h'), op('arm'))
HH = r('h')
XP = sub(r('x'), op('p'))
ZV = sub(r('z'), op('v'))
RHOP = sub(RHO, op('P'))
RV = r('R')
X = r('x')
Y = r('y')
Z = r('z')
BB = r('b')
RR = r('r')
CC = r('c')
COS = op('cos')
SIN = op('sin')
ATAN2 = op('atan2')
ACOS = op('acos')
PI = op('π')
DEG = op('°')
PLUS = r(' + ')
MINUS = r(' − ')
EQ = r(' = ')
COMMA = r(', ')
TIMES = r(' · ')


def disp(body):
    return '<m:oMath xmlns:m="%s">%s</m:oMath>' % (MNS, body)


def disp_para(body):
    return ('<m:oMathPara xmlns:m="%s"><m:oMath>%s</m:oMath></m:oMathPara>'
            % (MNS, body))


# --------------------------------------------------------------------------- #
# 文档排版辅助
# --------------------------------------------------------------------------- #
def set_cjk_font(style_or_run, latin='Times New Roman', cjk='宋体', size=None, bold=None):
    f = style_or_run.font
    f.name = latin
    if size is not None:
        f.size = Pt(size)
    if bold is not None:
        f.bold = bold
    rpr = style_or_run.element.get_or_add_rPr()
    rfonts = rpr.get_or_add_rFonts()
    rfonts.set(qn('w:ascii'), latin)
    rfonts.set(qn('w:hAnsi'), latin)
    rfonts.set(qn('w:eastAsia'), cjk)


def add_heading(doc, text, level=1):
    p = doc.add_heading('', level=level)
    run = p.add_run(text)
    sizes = {0: 20, 1: 15, 2: 13, 3: 11.5}
    set_cjk_font(run, latin='Times New Roman', cjk='黑体',
                 size=sizes.get(level, 11.5), bold=True)
    run.font.color.rgb = RGBColor(0x1F, 0x35, 0x64)
    return p


def add_para(doc, text, size=10.5, bold=False, indent=True, align=None, italic=False):
    p = doc.add_paragraph()
    if indent:
        p.paragraph_format.first_line_indent = Pt(21)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.line_spacing = 1.28
    if align is not None:
        p.alignment = align
    run = p.add_run(text)
    run.italic = italic
    set_cjk_font(run, size=size, bold=bold)
    return p


def add_eq(doc, body, label=None, size=11):
    """居中显示公式（Word 原生公式对象），可选右侧编号。"""
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(3)
    if label:
        body = body + op('\u2003\u2003(' + label + ')')
    p._p.append(parse_xml(disp_para(body)))
    return p


def add_inline_math(paragraph, body):
    """在普通段落里插入行内公式。"""
    paragraph._p.append(parse_xml(disp(body)))


def add_mixed(doc, parts, size=10.5, indent=True):
    """parts: [('t',文本) | ('b',粗体文本) | ('m',OMML片段), ...]"""
    p = doc.add_paragraph()
    if indent:
        p.paragraph_format.first_line_indent = Pt(21)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.line_spacing = 1.28
    for kind, val in parts:
        if kind == 'm':
            add_inline_math(p, val)
        else:
            run = p.add_run(val)
            set_cjk_font(run, size=size, bold=(kind == 'b'))
    return p


def add_table(doc, headers, rows, widths=None, size=9.5):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ''
        pr = hdr[i].paragraphs[0]
        pr.alignment = WD_ALIGN_PARAGRAPH.CENTER
        run = pr.add_run(h)
        set_cjk_font(run, size=size, bold=True)
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ''
            pr = cells[i].paragraphs[0]
            run = pr.add_run(str(v))
            set_cjk_font(run, size=size)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Pt(w)
    return t


def add_caption(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(8)
    run = p.add_run(text)
    set_cjk_font(run, size=9, bold=False)
    run.font.color.rgb = RGBColor(0x44, 0x44, 0x44)
    return p


def add_bullet(doc, text, size=10.5):
    p = doc.add_paragraph(style='List Bullet')
    p.paragraph_format.space_after = Pt(2)
    p.paragraph_format.line_spacing = 1.25
    run = p.add_run(text)
    set_cjk_font(run, size=size)
    return p


# --------------------------------------------------------------------------- #
# 生成文档
# --------------------------------------------------------------------------- #
def build(path):
    doc = Document()

    # 默认字体
    normal = doc.styles['Normal']
    set_cjk_font(normal, latin='Times New Roman', cjk='宋体', size=10.5)
    normal.paragraph_format.line_spacing = 1.28
    for sec in doc.sections:
        sec.top_margin = Pt(64)
        sec.bottom_margin = Pt(64)
        sec.left_margin = Pt(72)
        sec.right_margin = Pt(72)

    # ---------------- 封面标题 ----------------
    t = doc.add_paragraph()
    t.alignment = WD_ALIGN_PARAGRAPH.CENTER
    tr = t.add_run('meArm 机械臂 运动学正解与逆解推导')
    set_cjk_font(tr, cjk='黑体', size=22, bold=True)
    tr.font.color.rgb = RGBColor(0x1F, 0x35, 0x64)

    st = doc.add_paragraph()
    st.alignment = WD_ALIGN_PARAGRAPH.CENTER
    sr = st.add_run('三自由度位置运动学（基座回转 + 上臂 + 下臂）\n'
                    '源码基线：v1.6.3（commit 7a1d818） · 公式与固件逐式对应 · 附独立数值复算')
    set_cjk_font(sr, size=10.5)
    sr.font.color.rgb = RGBColor(0x44, 0x44, 0x44)

    doc.add_paragraph()

    # ---------------- 1 引言 ----------------
    add_heading(doc, '1  引言与文档范围', 1)
    add_para(doc, '本文给出 meArm 四自由度机械臂运动学正解（关节角 → 末端坐标）与逆解'
                  '（末端坐标 → 关节角）的完整推导。所有公式都与固件源码逐式对应，'
                  '每条结论都标注了源码位置，并给出独立复算的数值证据（第 8 节），'
                  '评审人员无需阅读源码即可复核本文内容。')

    add_mixed(doc, [
        ('t', '被控点（末端参考点）取为'),
        ('b', '下臂末端、即夹具根部'),
        ('t', '。第 4 个舵机（夹爪开合角 '),
        ('m', r('f')),
        ('t', '）只改变夹爪的张开度，不改变该参考点的位置，因此参与位置解算的自由度是 3 个：'
              '基座回转 '),
        ('m', r('b')),
        ('t', '、上臂俯仰 '),
        ('m', r('r')),
        ('t', '、下臂俯仰 '),
        ('m', r('c')),
        ('t', '。'),
    ])

    add_para(doc, '本文只讨论"位置运动学"，不涉及舵机脉宽映射、速度规划与轨迹插值；'
                  '这些内容分别由 servo_drive.cpp、move.cpp 与 draw_control.cpp 负责，'
                  '不属于运动学方程本身。')

    # ---------------- 2 机构 ----------------
    add_heading(doc, '2  机构描述与坐标系', 1)

    add_heading(doc, '2.1  机构与几何参数', 2)
    add_para(doc, '机械臂由基座回转关节（回转轴竖直）和一组平面两连杆（上臂、下臂）组成，'
                  '末端夹具刚性固定在下臂末端。上臂与下臂在同一竖直平面内摆动，'
                  '该平面由基座回转角确定；这就是典型的"柱面坐标 + 平面两连杆"结构。')
    add_para(doc, '几何参数取自固件常量：上臂与下臂等长（L1 = L2 = 20），'
                  '下臂末端沿自身方向再不伸出（h_arm = 0）。')

    add_table(doc,
              ['符号', '含义', '取值', '源码位置'],
              [['L₁', '上臂长度', '20.0', 'constant_and_positions.cpp:37'],
               ['L₂', '下臂长度', '20.0', 'constant_and_positions.cpp:37'],
               ['h_arm', '下臂末端沿下臂再伸出的长度', '0.0', 'constant_and_positions.cpp:37'],
               ['b / r / c / f', '基座/上臂/下臂/夹爪 关节角（度）', '见行程表', 'servoLimit'],
               ['b、r、c 行程', '关节硬限位', '[0°, 180°]', 'constant_and_positions.cpp'],
               ['f 行程', '夹爪硬限位', '[60°, 150°]', 'constant_and_positions.cpp']],
              widths=[70, 165, 62, 200])
    add_caption(doc, '表 1  机构几何参数与关节行程')

    add_heading(doc, '2.2  坐标系', 2)
    add_para(doc, '固件只有一个直角坐标系：原点 O′ 在肩关节回转轴与上臂俯仰轴的交点上，'
                  'x′ 为开机面朝方向，z′ 竖直向上，右手系；'
                  '固件变量 Pos.rec 存的就是末端在这套坐标系里的位置。'
                  '由于 h_arm = 0，z′ 同时也就是末端相对肩关节的高度。')
    add_para(doc, '串口 x/y/z 指令用的是同一套坐标系，但它的三个数并不是末端位置，'
                  '而是三个关节角（x→b、y→r、z→c，单位为度）：'
                  '指令由 protoApplyAngles() 一次性写入 Pos.ser，再经 clampServoAngles() '
                  '按 servoLimit 夹取、recFromServo() 正解刷新出 Pos.rec。'
                  '因此固件里没有"用户坐标系/地面坐标系"，也没有肩高换算——'
                  'v1.3.0～v1.6.3 曾把 x/y/z 解释为地面坐标系下的末端点，'
                  '该版本已备份（分支 backup/xyz-cartesian，即 tag v1.6.3）。')

    add_heading(doc, '2.3  关节角定义（实测标定）', 2)
    add_para(doc, '关节角都以"舵机中立位 90°"为基准，其物理含义由整机实测标定确定：')
    add_bullet(doc, 'b = 90° 时基座面朝 +x′；b 减小转向 +y′ 侧。因此"基座方位角"取为 '
                    'θ = 90° − b，即平面朝前方向相对 +x′ 轴绕 z′ 转过的角度（绕 +z′ 逆时针为正）。')
    add_bullet(doc, 'r = 90° 时上臂竖直向上（+z′）。可见 r 本身就是上臂的绝对方向角，'
                    '即 α = r（自 +x′ 起、朝 +z′ 为正）。')
    add_bullet(doc, 'c = 90° 时下臂水平朝前（+x′），与上臂夹角为 90°。设下臂绝对方向角为 β，'
                    '则 β = r − c，即 c = α − β 是下臂相对上臂"往前折"的角度。')

    add_para(doc, '上述 α = r、β = r − c 的取法不是任选的，而是由两处整机实测标定唯一确定：'
                  '(r, c) = (90°, 90°) 时末端位于 (20.00, 20.00)，(r, c) = (90°, 105°) 时末端'
                  '位于 (19.32, 14.82)（内部坐标，单位与 L1/L2 相同）。'
                  '另外两个候选取法 β = r + c − 180° 与 β = r + c 只能命中其中一个点，'
                  '代入两处标定后均被排除。')

    add_heading(doc, '2.4  符号表', 2)
    add_table(doc,
              ['符号', '含义'],
              [['θ', '基座方位角（平面朝前方向与 +x′ 的夹角），θ = 90° − b'],
               ['α', '上臂绝对方向角，α = r'],
               ['β', '下臂绝对方向角，β = r − c'],
               ['δ', '两连杆夹角（肘关节角），δ = α − β = c'],
               ['ρ', '末端在平面内的半径，ρ = √(x² + y²)'],
               ['z_v', '末端相对肩关节平面的高度，z_v = z′ − h_arm'],
               ['R', '肩关节到末端的距离，R = √(ρ² + z_v²)'],
               ['ρ_P', '平面分支取向：+ρ（正向平面）或 −ρ（反折平面）'],
               ['x_p', '平面内水平坐标（末端到基座回转轴的水平距离）'],
               ['h_arm', '夹具根部沿下臂再伸出的长度（本机 0）'],
               ['h', '肩关节轴离地高度（本机 20.0）']],
              widths=[52, 445])
    add_caption(doc, '表 2  符号表')

    # ---------------- 3 正运动学 ----------------
    add_heading(doc, '3  正运动学推导', 1)

    add_heading(doc, '3.1  平面内两连杆（相量形式）', 2)
    add_para(doc, '先在由基座回转角确定的竖直平面内求解。以肩关节 O′ 为极点，'
                  '平面内取水平方向为实轴、竖直向上为虚轴，末端位置可用复数表示。'
                  '两连杆在该平面内可看作两个矢量的叠加：上臂矢量模长 L1、方向角 α；'
                  '下臂矢量模长 L2、方向角 β。于是末端点 P 满足')
    add_eq(doc, r('P') + EQ + L1 + sup(r('e'), r('i') + AL) + PLUS + L2 + sup(r('e'), r('i') + BE), '2')
    add_para(doc, '把实部与虚部分开，即得到平面正解（x_p 为平面内水平坐标，'
                  'z′ 为相对肩关节的高度）：')
    add_eq(doc, XP + EQ + L1 + fnc(COS, AL) + PLUS + L2 + fnc(COS, BE), '3')
    add_eq(doc, sup(r('z'), op('′')) + EQ + L1 + fnc(SIN, AL) + PLUS + L2 + fnc(SIN, BE) + PLUS + HA, '4')
    add_para(doc, '其中平面内水平坐标 x_p 就是末端到基座回转轴的水平距离 ρ，'
                  '这一点在第 4 节逆解中会直接用到。')

    add_heading(doc, '3.2  基座回转', 2)
    add_para(doc, '基座回转把平面内的 x_p 投影到三维空间。由 2.3 节的定义，'
                  '基座方位角为 θ = 90° − b（换算成弧度使用）：')
    add_eq(doc, TH + EQ + paren(op('90°') + MINUS + BB) + TIMES + frac(PI, op('180')), '5')
    add_eq(doc, X + EQ + XP + fnc(COS, TH) + COMMA + Y + EQ + XP + fnc(SIN, TH), '6')

    add_heading(doc, '3.3  完整正解方程', 2)
    add_para(doc, '把 3.1 与 3.2 合并，并代入 α = r、β = r − c，得到完整的正运动学方程'
                  '（即固件函数 recFromServo 所实现的内容）：')
    add_eq(doc, X + EQ + paren(L1 + fnc(COS, RR) + PLUS + L2 + fnc(COS, paren(RR + MINUS + CC))) +
           fnc(COS, TH), '7')
    add_eq(doc, Y + EQ + paren(L1 + fnc(COS, RR) + PLUS + L2 + fnc(COS, paren(RR + MINUS + CC))) +
           fnc(SIN, TH), '8')
    add_eq(doc, sup(r('z'), op('′')) + EQ + L1 + fnc(SIN, RR) + PLUS + L2 + fnc(SIN, paren(RR + MINUS + CC)) +
           PLUS + HA, '9')
    add_mixed(doc, [
        ('t', '其中 '),
        ('m', TH + EQ + paren(op('90°') + MINUS + BB) + TIMES + frac(PI, op('180'))),
        ('t', '。三式中的三角函数自变量均为弧度，'
              '故实际计算时需把角度制的关节角乘以 π/180；夹爪角 '),
        ('m', r('f')),
        ('t', ' 不出现在以上任何一式中，'
              '证明它不影响末端参考点位置。'),
    ])
    add_mixed(doc, [
        ('t', '若写成矩阵形式，正解相当于先做平面内的连杆叠加、再绕竖直轴旋转 θ：'),
    ])
    add_eq(doc, paren(r('x') + r('; ') + r('y') + r('; ') + sup(r('z'), op('′')), '[', ']') +
           EQ + matrix([[op('cos') + TH, op('−sin') + TH, op('0')],
                        [op('sin') + TH, op(' cos') + TH, op('0')],
                        [op('0'), op('0'), op('1')]]) +
           paren(XP + r('; ') + op('0') + r('; ') + sup(r('z'), op('′')), '[', ']'), '10')

    add_heading(doc, '3.4  正解的标定验证', 2)
    add_para(doc, '下表用式 (3)、式 (4) 复算固件注释中记载的四个标定行（内部坐标、'
                  '单位与 L1/L2 相同，取 b = 90° 故 x = x_p、y = 0）：')
    add_table(doc,
              ['(r, c)', 'α = r', 'β = r − c', '平面坐标 (x_p, z′)', '固件注释值', '误差'],
              [['(90°, 90°)', '90°', '0°', '(20.000, 20.000)', '(20.00, 20.00)', '0.0000 / 0.0000'],
               ['(90°, 105°)', '90°', '−15°', '(19.318, 14.824)', '(19.32, 14.82)', '0.0015 / 0.0036'],
               ['(45°, 90°)', '45°', '−45°', '(28.284, 0.000)', '(28.28, 0.00)', '0.0043 / 0.0000'],
               ['(45°, 0°)', '45°', '45°', '(28.284, 28.284)', '(28.28, 28.28)', '0.0043 / 0.0043']],
              widths=[62, 42, 52, 118, 92, 100], size=8.5)
    add_caption(doc, '表 3  正解在四个标定点上的复算结果（误差为四舍五入到两位小数造成的差异）')

    add_heading(doc, '3.5  几个关键位形', 2)
    add_table(doc,
              ['位形', '条件', '结果'],
              [['两臂完全伸直', 'c = 0°（β = α）', 'R = L1 + L2 = 40'],
               ['两臂完全折回', 'c = 180°（β = α − 180°）', 'R = |L1 − L2| = 0'],
               ['开机初始位姿', '(b, r, c) = (90°, 90°, 90°)', '内部 (20, 0, 20) = 用户 (20, 0, 40)'],
               ['平面朝 +x′', 'θ = 0°（b = 90°）', 'y = 0，末端落在 x′ 轴上'],
               ['平面朝 +y′', 'θ = 90°（b = 0°）', 'x = 0，末端落在 y′ 轴上']],
              widths=[110, 150, 235])
    add_caption(doc, '表 4  典型位形')

    # ---------------- 4 逆运动学 ----------------
    add_heading(doc, '4  逆运动学推导', 1)

    add_heading(doc, '4.1  回转角 b', 2)
    add_para(doc, '末端的方位只由基座回转决定。由式 (6) 有 y / x = tan θ，而 θ = 90° − b，'
                  '故回转角为')
    add_eq(doc, BB + EQ + op('90°') + MINUS + fnc(ATAN2, Y + COMMA + X), '11')
    add_para(doc, '注意式 (11) 只在末端位于 x ≥ 0 一侧时给出合法的回转角。'
                  '当末端位于 x < 0 一侧时，atan2(y, x) 落在 (90°, 270°)，'
                  '式 (11) 会算出 b ∈ (190°, 270°)，超出 b 的行程 [0°, 180°]。'
                  '此时应改用一个"反折平面"的等价表示：把平面内水平坐标取为 ρ_P = −ρ，'
                  '相当于认为末端朝平面的反方向伸出、而基座再转过 180°，于是')
    add_eq(doc, BB + EQ + op('90°') + MINUS + fnc(ATAN2, r('−') + Y + COMMA + r('−') + X), '12')
    add_para(doc, '式 (11) 与式 (12) 给出的是同一个三维点的两种平面表示，'
                  '两者都必须参与候选，最后按行程与残差择优（见 4.6、4.7 节）。')

    add_heading(doc, '4.2  平面半径与肩–末端距离', 2)
    add_eq(doc, RHO + EQ + rad(sup(X, op('2')) + PLUS + sup(Y, op('2'))), '13')
    add_eq(doc, ZV + EQ + sup(r('z'), op('′')) + MINUS + HA, '14')
    add_eq(doc, sup(RV, op('2')) + EQ + sup(RHO, op('2')) + PLUS + sup(ZV, op('2')), '15')
    add_mixed(doc, [
        ('t', '由几何关系可知：无论 '),
        ('m', AL),
        ('t', ' 取何值，'),
        ('m', RV),
        ('t', ' 只取决于两杆夹角 '),
        ('m', DE),
        ('t', '（即关节角 '),
        ('m', CC),
        ('t', '），与整体姿态无关。这正是下面用余弦定理先解 '),
        ('m', DE),
        ('t', ' 的依据。'),
    ])

    add_heading(doc, '4.3  余弦定理求肘角', 2)
    add_para(doc, '把式 (2) 写成模长形式：')
    add_eq(doc, sup(RV, op('2')) + EQ + sup(L1, op('2')) + PLUS + sup(L2, op('2')) + PLUS +
           op('2') + L1 + L2 + fnc(COS, DE), '16')
    add_para(doc, '整理即得肘角（两杆夹角）的余弦：')
    add_eq(doc, fnc(COS, DE) + EQ + frac(sup(RV, op('2')) + MINUS + sup(L1, op('2')) + MINUS + sup(L2, op('2')),
                                        op('2') + L1 + L2), '17')
    add_eq(doc, DE + EQ + op('±') + fnc(ACOS, paren(frac(
        sup(RV, op('2')) + MINUS + sup(L1, op('2')) + MINUS + sup(L2, op('2')),
        op('2') + L1 + L2))), '18')
    add_para(doc, '式 (18) 的绝对值由余弦定理唯一确定，符号 ± 对应"肘部在上/在下"'
                  '两个镜像姿态，两个分支都需要检验其关节行程。')

    add_heading(doc, '4.4  相量法求下臂与上臂方向角', 2)
    add_para(doc, '确定了 |δ| 之后，还需要求出两杆的绝对方向角。把式 (2) 与 α = β + δ 联立：')
    add_eq(doc, r('P') + EQ + L1 + sup(r('e'), r('i') + AL) + PLUS + L2 + sup(r('e'), r('i') + BE) +
           EQ + sup(r('e'), r('i') + BE) + paren(L1 + sup(r('e'), r('i') + DE) + PLUS + L2), '19')
    add_para(doc, '两边同乘 e^(−iβ)，把未知的 β 从指数中分离出来：')
    add_eq(doc, sup(r('e'), r('i') + BE) + EQ + frac(r('P'), L1 + sup(r('e'), r('i') + DE) + PLUS + L2), '20')
    add_para(doc, '取两边幅角（arg P = atan2(z_v, ρ_P)），即得下臂方向角，'
                  '再由 α = β + δ 得上臂方向角：')
    add_eq(doc, BE + EQ + fnc(ATAN2, ZV + COMMA + RHOP) + MINUS +
           fnc(ATAN2, L1 + fnc(SIN, DE) + COMMA + L2 + PLUS + L1 + fnc(COS, DE)), '21')
    add_eq(doc, AL + EQ + BE + PLUS + DE, '22')
    add_mixed(doc, [
        ('t', '本机两杆等长（'),
        ('m', L1 + EQ + L2 + EQ + op('20')),
        ('t', '），式 (21) 中分母的两项可以互换，'
              '因此它与固件中写成的 atan2(L2·sin δ, L1 + L2·cos δ)（'
              'constant_and_positions.cpp:517）在数值上完全相同。'),
    ])
    add_para(doc, '推导要点：必须先把式 (2) 两边乘 e^(−iβ)（或等价地提出公因子 e^(iβ)），'
                  '让 L1、L2 两项的幅角关系直接分开；若改去凑 (α + β) 之和，'
                  '会把向量的模与幅角混在一起，得不到正确解。')

    add_heading(doc, '4.5  逆解公式汇总', 2)
    add_para(doc, '把 4.1–4.4 的结果汇总成固件实际执行的求解流程（角度均为度）：')
    add_table(doc,
              ['步骤', '公式', '说明'],
              [['1', 'ρ = √(x² + y²)，z_v = z′ − h_arm，R² = ρ² + z_v²', '平面半径与肩–末端距离'],
               ['2', 'cos δ = (R² − L1² − L2²) / (2 L1 L2)', '余弦定理，δ = ±acos(·)'],
               ['3', 'β = atan2(z_v, ρ_P) − atan2(L1 sin δ, L2 + L1 cos δ)', '相量法'],
               ['4', 'α = β + δ', '上臂绝对方向角'],
               ['5', 'r = α，c = δ', '折算回关节角（度）'],
               ['6', 'b = 90° − atan2(y, x) 或 90° − atan2(−y, −x)', '正向 / 反折两个平面分支'],
               ['7', '回代式 (3)(4) 计算残差，超差整组丢弃', '自洽性校验，阈值 0.05']],
              widths=[30, 250, 215], size=9)
    add_caption(doc, '表 5  逆解流程（与 getAngleEx 逐句对应）')

    add_heading(doc, '4.6  分支枚举与择优', 2)
    add_para(doc, '逆解在平面内存在两个独立的二值自由度，共 4 个候选姿态：')
    add_bullet(doc, '肘部在上 / 在下：δ = +|c| 或 δ = −|c|（式 (18) 的 ± 号）。')
    add_bullet(doc, '平面正向 / 反折：ρ_P = +ρ（末端朝平面 +x 方向）或 ρ_P = −ρ（式 (11) 与式 (12)）。')
    add_para(doc, '固件对 4 个候选逐一求解，先按关节行程筛选（见 4.7 节），'
                  '再取"正解回代残差最小"的一个作为最终解。')
    add_para(doc, '反折分支不是冗余的：当目标落在 x < 0 一侧时，只有反折平面才能给出落在 '
                  '[0°, 180°] 内的回转角；如果忽略该分支而把超行程的回转角强行吸附到 180°，'
                  '会引入可观的坐标误差（固件调试记录中曾出现 0.6077 个单位的偏差）。')

    add_heading(doc, '4.7  可达条件与行程筛选', 2)
    add_para(doc, '几何上存在解的必要条件是"两杆能构成三角形"：')
    add_eq(doc, op('|') + L1 + MINUS + L2 + op('|') + r(' ≤ ') + RV + r(' ≤ ') + L1 + PLUS + L2, '23')
    add_para(doc, '本机 L1 = L2 = 20，故 R ∈ [0, 40]；R = 40 为完全伸直，R = 0 为完全折回。'
                  '固件用一个很小的容差（1e-9）放宽该判据，'
                  '以免"恰好完全伸直"的浮点舍入（R 算出 40.000000000000004）被误判为不可达。')
    add_para(doc, '几何可达之后还要满足关节行程与自洽性，固件的筛选顺序为：')
    add_bullet(doc, '回转角必须在 b 的行程内：b ∈ [0°, 180°]（带 1e-9 容差）。')
    add_bullet(doc, '解出的 r、c 必须分别落在行程内：r ∈ [0°, 180°]，c ∈ [0°, 180°]。'
                    '该筛选必须在"限位吸附"之前完成，否则越限的候选会被静默吸附成看似可用的解。')
    add_bullet(doc, '正解回代残差必须 ≤ 0.05：把候选 (α, β) 代入式 (3)(4) 得 (x_f, z_f)，'
                    '要求该点与目标点的距离足够小。')
    add_para(doc, '以上任一条件不满足即丢弃该候选；4 个候选全部不满足时函数返回失败，'
                  '并保持原有姿态不变（固件契约：返回失败时不得修改 Pos.ser）。')

    add_heading(doc, '4.8  奇异位形', 2)
    add_bullet(doc, 'ρ = 0（末端正好落在基座回转轴线上）：回转角在数学上不可确定，'
                    '固件取中立值 b = 90°（朝 +x′），是否真的可用交由行程与残差判定。')
    add_bullet(doc, 'R = L1 + L2 = 40（两臂完全伸直）：δ = 0，γ 方向与两杆方向重合，'
                    '工作空间边界（雅可比矩阵降秩）。')
    add_bullet(doc, 'R = 0（完全折回）：δ = 180°，末端与肩关节重合，'
                    '同样属于边界奇异位形。')

    # ---------------- 5 工程判据 ----------------
    add_heading(doc, '5  工程判据（与数学推导配套的实现约束）', 1)

    add_heading(doc, '5.1  正解回代自检', 2)
    add_para(doc, '逆解算出的关节角必须能让正解重新落在目标点上。固件在选出候选后'
                  '立即回代式 (3)(4)，残差超阈值即整组丢弃：')
    _d1 = L1 + fnc(COS, AL) + PLUS + L2 + fnc(COS, BE) + MINUS + RHOP
    _d2 = L1 + fnc(SIN, AL) + PLUS + L2 + fnc(SIN, BE) + MINUS + ZV
    add_eq(doc, r('e') + EQ + rad(sup(paren(_d1), op('2')) + PLUS + sup(paren(_d2), op('2'))) +
           r(' ≤ ') + op('0.05'), '24')
    add_para(doc, '上式即"平面内两点欧氏距离"，固件要求 e ≤ 0.05；'
                  '这个阈值远小于机构精度（约 0.1° 对应约 0.03 个长度单位），'
                  '但足以挡住错误分支与异常数值。', indent=True)

    add_heading(doc, '5.2  回转角归一化', 2)
    add_para(doc, '式 (11) 的值域是 [−90°, 270°]，固件按下列顺序归一化到 [0°, 360°)：'
                  '负值加 360°；不小于 360° 的减去 360°；'
                  '与 360° 相差不足 1e-6 的折回 0°。'
                  '最后一句是必需的：在 32 位浮点下 (−1e−10) + 360 会被舍入成整 360.0，'
                  '而 64 位双精度下同样会算出 359.9999999998，'
                  '两种情况都会让"回转角恰好为 0°"这一整类解被判为超行程而丢弃。')

    add_heading(doc, '5.3  限位处理策略', 2)
    add_para(doc, '关节硬限位（servoLimit）是固件唯一的软件边界。逆解内部先按 4.7 节显式筛选，'
                  '再可选地执行"吸附"或"拒绝"：')
    add_bullet(doc, 'CLAMP 策略（默认）：越限角度吸附到最近限位。')
    add_bullet(doc, 'REJECT 策略：出现越限角度即返回失败。')
    add_para(doc, '两种策略都建立在 4.7 节的显式筛选之上。固件当前的坐标入口只有两条——'
                  '绘图轨迹与 A/B/C 取放——两者都在启动前做完整路径校验，一律要求严格可达；'
                  '串口 x/y/z 写的是关节角，不经过逆解，越限时由 clampServoAngles() 夹取。')

    # ---------------- 6 工作空间 ----------------
    add_heading(doc, '6  工作空间', 1)
    add_para(doc, '由式 (23) 与 R² = ρ² + z_v² 可知，在 (ρ, z_v) 平面内可达域是以肩关节'
                  '为圆心、半径 40 的圆盘（本机 L1 = L2）。再叠加基座回转范围与软护栏，'
                  '得到实际工作空间：')
    add_bullet(doc, '平面可达域：R ≤ 40 的圆盘（R = 0 为完全折回，R = 40 为完全伸直）。')
    add_bullet(doc, '基座回转 b ∈ [0°, 180°] ⇒ θ ∈ [−90°, 90°]：'
                    '正向平面覆盖 x ≥ 0 的一半，反折平面覆盖 x ≤ 0 的一半，'
                    '两者合计使末端可达 x ∈ [−40, 40]、y ∈ [−40, 40]。')
    add_bullet(doc, '位置软护栏：固件 limit 取 x、y ∈ [−40, 40]、z′ ∈ [−20, 40]，'
                    '与关节行程算出的可达包络完全一致（余量 0），'
                    '明显越界的坐标请求在解算之前就被挡掉。')
    add_table(doc,
              ['方向', '可达范围'],
              [['x', '[−40.0, 40.0]'],
               ['y', '[−40.0, 40.0]'],
               ['z′', '[−20.0, 40.0]'],
               ['径向 R = √(x²+y²+z′²)', '[0.0, 40.0]']],
              widths=[140, 300])
    add_caption(doc, '表 6  可达范围（与固件 limit 软护栏及实测包络一致）')

    # ---------------- 7 源码对应 ----------------
    add_heading(doc, '7  与固件源码的对应关系', 1)
    add_table(doc,
              ['公式/概念', '固件实现', '位置'],
              [['平面正解 x_p、z′', 'fkPlanar()', 'constant_and_positions.cpp:395–401'],
               ['完整正解（含基座回转）', 'recFromServo()', 'constant_and_positions.cpp:403–428'],
               ['可达性判据（式 23）', 'isReachable()', 'constant_and_positions.cpp:349–371'],
               ['逆解主流程', 'getAngleEx()', 'constant_and_positions.cpp:540 起'],
               ['相量法分支求解（式 21）', 'ikBranch()', 'constant_and_positions.cpp:514–524'],
               ['回转角归一化', 'normRevAngle()', 'constant_and_positions.cpp:533–538'],
               ['串口角度指令（x/y/z，不逆解）', 'protoApplyAngles()', 'serial_protocol.cpp:831'],
               ['角度行解析（x,y,z 三轴任选）', 'protoParseAxisLine()', 'serial_protocol.cpp:727'],
               ['连杆参数 L1、L2、h_arm', 'arm1 = {20, 20, 0}', 'constant_and_positions.cpp:37'],
               ['关节硬限位', 'servoLimit', 'constant_and_positions.cpp'],
               ['位置软护栏', 'limit', 'constant_and_positions.cpp'],
               ['坐标入口（统一走同一套解算）', 'moveToPoint()', 'move.cpp（move.h:97–140）']],
              widths=[135, 130, 232], size=9)
    add_caption(doc, '表 7  公式与源码的对应关系')

    # ---------------- 8 数值验证 ----------------
    add_heading(doc, '8  独立数值验证', 1)
    add_para(doc, '为核实上述公式与固件实现一致，本文用独立的 Python 复算脚本'
                  '（与固件同一组公式、同一组常量，不调用固件代码）做了三类验证。'
                  '脚本与完整报告随文档一并保存：.selfcheck/out/kin_verify.py 与 '
                  '.selfcheck/out/kin_verify_report.txt。')

    add_heading(doc, '8.1  标定点复算', 2)
    add_para(doc, '式 (3)(4) 在四个标定行上的结果见表 3，与固件注释记载的数值一致'
                  '（差异仅为四舍五入到两位小数所致）。')

    add_heading(doc, '8.2  逆解–正解往返一致性', 2)
    add_para(doc, '在关节行程内随机取 200000 组关节角 (b, r, c)，先用正解算出末端坐标，'
                  '再对同一坐标做逆解，然后把逆解结果重新正解、与原始坐标比较：')
    add_table(doc,
              ['样本数', '逆解失败', '最大位置往返误差', '结论'],
              [['200000', '0 组（0.000%）', '2.266 × 10⁻¹⁰ 单位', '逆解在整个行程内自洽']],
              widths=[70, 110, 150, 167])
    add_caption(doc, '表 8  往返一致性验证结果')
    add_para(doc, '最大误差量级为 1e−10，仅来自浮点舍入，'
                  '远小于舵机可分辨的最小角度（约 0.1° ⇒ 约 0.03 个长度单位），'
                  '可认为正解与逆解互为精确逆运算。')

    add_heading(doc, '8.3  代表点的逆解结果', 2)
    add_para(doc, '下表给出若干代表点（末端坐标，原点在肩关节，单位同 L1/L2）的逆解与回代结果，'
                  '可用于人工抽查：')
    add_table(doc,
              ['末端坐标 (x, y, z′)', '逆解 (b, r, c)', '回代（正解）'],
              [['(20, 0, 20) 初始位姿', '(90.000, 90.000, 90.000)', '(20.000, 0.000, 20.000)'],
               ['(20, 0, 0)', '(90.000, 60.000, 120.000)', '(20.000, 0.000, 0.000)'],
               ['(0, 20, 20)', '(0.000, 90.000, 90.000)', '(0.000, 20.000, 20.000)'],
               ['(30, 0, 0)', '(90.000, 41.410, 82.819)', '(30.000, 0.000, 0.000)'],
               ['(0, 0, 30)', '(90.000, 131.410, 82.819)', '(0.000, 0.000, 30.000)'],
               ['(−10, 0, 10)', '无合法分支', '几何可达但无解满足关节行程']],
              widths=[155, 140, 165], size=8.5)
    add_caption(doc, '表 9  代表点逆解与回代')
    add_para(doc, '最后一行值得注意：该点几何上满足式 (23)（R ≈ 14.14 < 40），'
                  '但 4 个候选姿态解出的 r、c 均超出 [0°, 180°]，'
                  '因此在固件中同样返回"不可达/无合法分支"。'
                  '这说明式 (23) 只是必要条件，最终可行性由关节行程共同决定。')

    # ---------------- 9 公式汇总 ----------------
    add_heading(doc, '9  公式汇总', 1)
    add_para(doc, '正运动学（关节角 → 末端坐标，内部坐标系）：')
    add_eq(doc, TH + EQ + paren(op('90°') + MINUS + BB) + TIMES + frac(PI, op('180')), None)
    add_eq(doc, X + EQ + paren(L1 + fnc(COS, paren(RR + TIMES + frac(PI, op('180'))) ) +
           PLUS + L2 + fnc(COS, paren(paren(RR + MINUS + CC) + TIMES + frac(PI, op('180')))) ) + fnc(COS, TH), None)
    add_eq(doc, Y + EQ + paren(L1 + fnc(COS, paren(RR + TIMES + frac(PI, op('180'))) ) +
           PLUS + L2 + fnc(COS, paren(paren(RR + MINUS + CC) + TIMES + frac(PI, op('180')))) ) + fnc(SIN, TH), None)
    add_eq(doc, sup(r('z'), op('′')) + EQ + L1 + fnc(SIN, paren(RR + TIMES + frac(PI, op('180'))) ) +
           PLUS + L2 + fnc(SIN, paren(paren(RR + MINUS + CC) + TIMES + frac(PI, op('180')))) + PLUS + HA, None)

    add_para(doc, '逆运动学（末端坐标 → 关节角，内部坐标系）：')
    add_eq(doc, RHO + EQ + rad(sup(X, op('2')) + PLUS + sup(Y, op('2'))) + COMMA +
           ZV + EQ + sup(r('z'), op('′')) + MINUS + HA + COMMA +
           sup(RV, op('2')) + EQ + sup(RHO, op('2')) + PLUS + sup(ZV, op('2')), None)
    add_eq(doc, fnc(COS, DE) + EQ + frac(sup(RV, op('2')) + MINUS + sup(L1, op('2')) + MINUS + sup(L2, op('2')),
                                        op('2') + L1 + L2) + COMMA + DE + EQ + op('±') +
           fnc(ACOS, paren(frac(sup(RV, op('2')) + MINUS + sup(L1, op('2')) + MINUS + sup(L2, op('2')),
                                op('2') + L1 + L2))), None)
    add_eq(doc, BE + EQ + fnc(ATAN2, ZV + COMMA + RHOP) + MINUS +
           fnc(ATAN2, L1 + fnc(SIN, DE) + COMMA + L2 + PLUS + L1 + fnc(COS, DE)) + COMMA +
           AL + EQ + BE + PLUS + DE, None)
    add_eq(doc, BB + EQ + op('90°') + MINUS + fnc(ATAN2, Y + COMMA + X) +
           op('  (正向平面)  ') + r('或') + op('  90°') + MINUS + fnc(ATAN2, r('−') + Y + COMMA + r('−') + X) +
           op('  (反折平面)'), None)
    add_eq(doc, RR + EQ + AL + COMMA + CC + EQ + DE + COMMA +
           op('条件: |') + L1 + MINUS + L2 + op('|') + r(' ≤ ') + RV + r(' ≤ ') + L1 + PLUS + L2, None)

    # ---------------- 附录 ----------------
    add_heading(doc, '附录 A  复现本文数值结果', 1)
    add_para(doc, '本文所有数值均由下列脚本复算得到（Python 3，无第三方依赖）：')
    add_para(doc, r'D:\dsh1\meArm\.selfcheck\out\kin_verify.py', indent=False)
    add_para(doc, '运行方式（在仓库根目录执行）：', indent=False)
    add_para(doc, r'python .selfcheck\out\kin_verify.py', indent=False)
    add_para(doc, '脚本会输出标定点复算、20 万组往返一致性、工作空间与代表点四类结果，'
                  '并写入 .selfcheck/out/kin_verify_report.txt。', indent=False)

    add_heading(doc, '附录 B  结论摘要', 1)
    add_bullet(doc, '正运动学：由 α = r、β = r − c 得到平面坐标，再绕基座方位角 θ = 90° − b 旋转，'
                    '即式 (7)–(9)。')
    add_bullet(doc, '逆运动学：余弦定理定 |c|，相量法定 β 与 α（式 (17)–(22)），'
                    '基座回转角由式 (11)/(12) 给出，共 4 个候选姿态择优。')
    add_bullet(doc, '判据：几何可达式 (23) + 关节行程筛选 + 正解回代残差 ≤ 0.05。')
    add_bullet(doc, '本文公式经 20 万组独立复算验证，最大往返误差 2.266 × 10⁻¹⁰ 单位。')

    doc.save(path)
    return path


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else OUT_DEFAULT
    os.makedirs(os.path.dirname(out), exist_ok=True)
    p = build(out)
    sys.stdout.write('docx written: %s (%d bytes)\n' % (p, os.path.getsize(p)))
