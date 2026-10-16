/*
 * keysim.js -- 把 arduino_keypad.ino 的 keysTick() 状态机逐行搬到 JS，
 * 用 5ms 步长的合成按键波形复核"哪一下按击会发出哪条指令"。
 * 与 C++ 版保持一致的关键点：消抖 30ms、连击窗口 350ms、长按 700ms、
 * 长按触发后取消本次点击结算、电平变化即重启消抖计时。
 */
const DEBOUNCE_MS = 30, CLICK_WINDOW = 350, LONG_PRESS_MS = 700, STEP = 5;

// ---- 与 .ino 里 PRESET_MEARM 的两张表逐条对应（pin 用字符串编号） ----
const PINS = ['8', '9', '10', '11'];
const KEYS = [
  ['8', 1, 'N'], ['8', 2, 'O'], ['8', 3, 'S'],
  ['9', 1, 'R'], ['9', 2, 'P'], ['9', 3, 'H'],
  ['10', 1, 'A'], ['10', 2, 'B'], ['10', 3, 'C'],
  ['11', 1, 'm'], ['11', 2, 'mS'], ['11', 3, 'mE'],
];
const LONGS = [['8', 'M'], ['9', 'L'], ['10', '!'], ['11', '0']];

const clickCmd = (pin, n) => { const e = KEYS.find(k => k[0] === pin && k[1] === n); return e ? e[2] : null; };
const longCmd  = (pin)     => { const e = LONGS.find(l => l[0] === pin); return e ? e[1] : null; };

const LABEL = { '8': '键1', '9': '键2', '10': '键3', '11': '键4' };

function run(intervals, durationMs) {
  const ks = PINS.map(() => ({ stable: false, prev: false, longFired: false, clicks: 0,
                              debounceAt: 0, releaseAt: 0, pressAt: 0 }));
  const sent = [];
  const pressed = (pin, t) => intervals.some(([p, a, b]) => p === pin && t >= a && t < b);

  for (let now = 0; now <= durationMs; now += STEP) {
    for (let i = 0; i < PINS.length; i++) {
      const k = ks[i], pin = PINS[i];
      const raw = pressed(pin, now);
      if (raw !== k.prev) { k.prev = raw; k.debounceAt = now; }
      else if (now - k.debounceAt >= DEBOUNCE_MS && raw !== k.stable) {
        k.stable = raw;
        if (raw) { if (k.clicks < 9) k.clicks++; k.pressAt = now; k.longFired = false; }
        else k.releaseAt = now;
      }
      if (LONG_PRESS_MS > 0 && k.stable && !k.longFired && k.clicks > 0 &&
          now - k.pressAt >= LONG_PRESS_MS) {
        k.longFired = true; k.clicks = 0;
        const c = longCmd(pin);
        sent.push([now, pin, c, c ? '长按' : '长按无映射']);
      }
      if (k.clicks > 0 && !k.stable && now - k.releaseAt >= CLICK_WINDOW) {
        const n = k.clicks; k.clicks = 0;
        const c = clickCmd(pin, n);
        sent.push([now, pin, c, c ? `${n}击` : `${n}击无映射`]);
      }
    }
  }
  return sent;
}

let pass = 0, fail = 0;
function check(name, intervals, duration, expectCmds) {
  const got = run(intervals, duration).map(s => s[2]).filter(c => c !== null);
  const ok = JSON.stringify(got) === JSON.stringify(expectCmds);
  if (ok) pass++; else fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name.padEnd(34)} 期望=[${expectCmds}] 实际=[${got}]`);
}

// ---- 场景 ----
check('键1 单击 → N',                 [['8', 100, 200]],                        900,  ['N']);
check('键1 双击 → O',                 [['8', 100, 200], ['8', 350, 450]],        1100, ['O']);
check('键1 三击 → S',                 [['8', 100, 200], ['8', 350, 450], ['8', 600, 700]], 1400, ['S']);
check('键1 长按 900ms → M（不发 N）', [['8', 100, 1000]],                       1600, ['M']);
check('键1 按 600ms 松开 → N',        [['8', 100, 700]],                        1200, ['N']);
check('键2 三击 → H',                 [['9', 100, 200], ['9', 350, 450], ['9', 600, 700]], 1400, ['H']);
check('键3 单击 → A',                 [['10', 100, 200]],                       900,  ['A']);
check('键3 双击 → B',                 [['10', 100, 200], ['10', 350, 450]],       1100, ['B']);
check('键3 三击 → C',                 [['10', 100, 200], ['10', 350, 450], ['10', 600, 700]], 1400, ['C']);
check('键3 长按 → !',                 [['10', 100, 1000]],                      1600, ['!']);
check('键4 单击 → m',                 [['11', 100, 200]],                       900,  ['m']);
check('键4 双击 → mS',                [['11', 100, 200], ['11', 350, 450]],       1100, ['mS']);
check('键4 三击 → mE',                [['11', 100, 200], ['11', 350, 450], ['11', 600, 700]], 1400, ['mE']);
check('键4 长按 → 0',                 [['11', 100, 1000]],                      1600, ['0']);
check('键1 四击（表里没有）→ 不发',   [['8', 100, 200], ['8', 350, 450], ['8', 600, 700], ['8', 850, 950]], 1700, []);
check('12ms 抖动 + 单击 → 只一发 N',  [['8', 100, 112], ['8', 118, 130], ['8', 132, 240]], 1000, ['N']);
check('键2 两次单击间隔 700ms → R,R', [['9', 100, 200], ['9', 900, 1000]],     1700, ['R', 'R']);
check('两键几乎同时按 → N,R 各一发',  [['8', 100, 200], ['9', 100, 200]],      1000, ['N', 'R']);
check('键1 单击 + 长按（间隔松手）',  [['8', 100, 200], ['8', 600, 1500]],     2100, ['N', 'M']);

console.log(`\n合计 ${pass + fail} 个场景：PASS ${pass}，FAIL ${fail}`);
process.exit(fail === 0 ? 0 : 1);
