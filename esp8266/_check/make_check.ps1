# 从 meArm_wifi.ino 里切出内嵌网页，生成 page.js（给 node --check）和三个假数据测试页。
# 注意：必须用 UTF8 读源文件，否则中文变乱码，后续 JS 检查会假失败。
$ErrorActionPreference = 'Stop'
$ino = 'D:\dsh1\meArm\esp8266\meArm_wifi\meArm_wifi.ino'
$dir = 'D:\dsh1\meArm\esp8266\_check'
$src = [System.IO.File]::ReadAllText($ino, [System.Text.Encoding]::UTF8)

$a = $src.IndexOf('R"HTML(') + 7
$b = $src.IndexOf(')HTML"', $a)
if ($a -lt 7 -or $b -lt 0) { throw 'R"HTML( ... )HTML" not found' }
$html = $src.Substring($a, $b - $a)
[System.IO.File]::WriteAllText("$dir\page.html", $html, (New-Object System.Text.UTF8Encoding $false))

# 网页自己的那段 JS（最后一个 <script>）
$i = $html.LastIndexOf('<script>') + 8
$j = $html.IndexOf('</script>', $i)
[System.IO.File]::WriteAllText("$dir\page.js", $html.Substring($i, $j - $i), (New-Object System.Text.UTF8Encoding $false))

function New-TestPage([string]$file, [string]$mock) {
  $out = $html -replace '(?s)<body>', ("<body>`n<script>`n" + $mock + "`n</script>")
  [System.IO.File]::WriteAllText("$dir\$file", $out, (New-Object System.Text.UTF8Encoding $false))
}

# --- 1) V-test 风格：有 4 个关节滑条、one=1、老式 state（没有 cards → 走兜底分支）
New-TestPage 'page_test.html' @'
const _d={buttons:[{g:"状态/安全",l:"查询状态",c:"?",f:0,d:0},{g:"状态/安全",l:"急停·保持",c:"X",f:2,d:0},
 {g:"使能与位姿",l:"使能+回初始位姿",c:"E",f:3,d:1},{g:"摇杆/调速",l:"摇杆开",c:"JOY",f:0,d:0}],
 joints:{one:1,list:[{n:"B",lo:0,hi:180,v:90},{n:"R",lo:10,hi:170,v:90},{n:"F",lo:20,hi:150,v:90},{n:"C",lo:50,hi:130,v:120}]},
 state:{state:"IDLE",pose:"B=90.00 R=90.00 F=90.00 C=120.00",joy:"JOY:0 CLAW_LOCK:1",ram:"RAM:1 samples=300 ms=30000",
  q:0,heap:34108,up:42,net:"STA mywifi -55dBm 192.168.1.23",new:["E: moving to B90 R90 F90 C120","DONE:E_HOME; joystick ON"]}};
window.fetch=(u)=>{u=String(u);let r=_d.buttons;if(u.indexOf('/joints')>=0)r=_d.joints;if(u.indexOf('/state')>=0)r=_d.state;
 return Promise.resolve({json:()=>Promise.resolve(r),text:()=>Promise.resolve(JSON.stringify(r))});};
'@

# --- 2) meArm v1.6.5 风格：3 个关节、one=0、老式 state
New-TestPage 'page_test2.html' @'
const _d={buttons:[{g:"取放(自动)",l:"取放 A",c:"A",f:2,d:1},{g:"绘图",l:"绘图开始",c:"G",f:0,d:1},
 {g:"状态/安全",l:"查询位姿",c:"!P",f:0,d:0}],
 joints:{one:0,list:[{n:"x",lo:0,hi:180,v:90},{n:"y",lo:0,hi:180,v:90},{n:"z",lo:0,hi:180,v:90}]},
 state:{state:"IDLE",pose:"x=90 y=90 z=90",joy:"-",ram:"-",q:0,heap:34000,up:5,
  net:"STA mywifi -60dBm 192.168.1.9",new:["READY"]}};
window.fetch=(u)=>{u=String(u);let r=_d.buttons;if(u.indexOf('/joints')>=0)r=_d.joints;if(u.indexOf('/state')>=0)r=_d.state;
 return Promise.resolve({json:()=>Promise.resolve(r),text:()=>Promise.resolve(JSON.stringify(r))});};
'@

# --- 3) Task5 风格：没有关节滑条、state 带 cards、中文状态与笔尖坐标
New-TestPage 'page_test3.html' @'
const _d={buttons:[{g:"绘制",l:"直线演示 (键4×1)",c:"7",f:3,d:1},{g:"绘制",l:"折线 (键3×2)",c:"5",f:1,d:1},
 {g:"示教",l:"进入示教/记录点 (键1)",c:"1",f:1,d:1},{g:"控制",l:"暂停 (键2×1)",c:"2",f:0,d:0},
 {g:"控制",l:"取消·回待机 (键3×1)",c:"4",f:2,d:0},{g:"状态",l:"状态查询",c:"?",f:0,d:0}],
 joints:{one:0,list:[]},
 state:{state:"绘制中",pose:"笔尖 12.3, 45.6, 7.8",msg:">>> 基础图形绘制（字母 N）",joy:"-",ram:"-",q:0,
  heap:34100,up:12,net:"AP meArm-3F2A1B 192.168.4.1",
  cards:[["状态","绘制中"],["当前动作",">>> 基础图形绘制（字母 N）"],["笔尖坐标","笔尖 12.3, 45.6, 7.8"],
   ["示教点","2 个"],["WiFi","AP meArm-3F2A1B 192.168.4.1"],["堆内存"," 34100 B"],["运行"," 12 s"],["队列"," 0 条"]],
  new:[">>> 折线绘制开始","! 示教点不足，先进入示教模式记录点"]}};
window.fetch=(u)=>{u=String(u);let r=_d.buttons;if(u.indexOf('/joints')>=0)r=_d.joints;if(u.indexOf('/state')>=0)r=_d.state;
 return Promise.resolve({json:()=>Promise.resolve(r),text:()=>Promise.resolve(JSON.stringify(r))});};
'@

"page.html = $((Get-Item "$dir\page.html").Length) B"
