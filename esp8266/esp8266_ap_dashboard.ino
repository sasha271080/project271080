/**
  ******************************************************************************
  *  esp8266_ap_dashboard.ino
  *  Wi-Fi панель для контроллера коллекторного двигателя на STM32F103C8T6.
  *
  *  ПЛАТА: Wemos D1 mini (ESP8266EX, 4 МБ flash)
  *  ЯДРО: "esp8266 by ESP8266 Community" 3.x
  *  ПЛАТА В IDE: LOLIN(WEMOS) D1 R2 & mini
  *
  *  ЧТО ДЕЛАЕТ
  *    1. Поднимает точку доступа (или AP+STA, если вписать домашнюю сеть).
  *    2. Отдаёт веб-панель на http://192.168.4.1/ — телеметрия и команды.
  *    3. Держит «прозрачный» TCP-сервер на порту 23 — сырой мост UART <-> TCP
  *       для Serial Studio / Node-RED / com0com + pid_tuner.py.
  *    4. Раз в секунду шлёт на STM32 "PG" — иначе при виртуальном задатчике
  *       оборотов прошивка сработает по TRIP_COMM через 5 с без команд.
  *
  *  СВЯЗЬ С STM32 (USART3, см. Core/Src/mc_wifi.c):
  *      STM32 PB10 (TX) ----> D1 mini D7 / GPIO13 (RX)
  *      STM32 PB11 (RX) <---- D1 mini D8 / GPIO15 (TX)
  *      STM32 GND       ----  D1 mini GND        (обязательно)
  *      Питание модуля  — только 5V (VIN), не 3.3V Blue Pill.
  *
  *  ПОЧЕМУ swap(): штатные GPIO1/GPIO3 заняты загрузчиком и отладочным
  *  выводом (74880 бод). swap() переводит UART0 на GPIO13/GPIO15, и мусор
  *  при старте в STM32 не уходит.
  *
  *  ПРО СКОРОСТЬ 460800. Это не «с запасом»: кадр до 712 байт каждые 100 мс.
  *  ESP8266 тянет 460800, но буфер приёма по умолчанию 256 байт — его
  *  сносит за 5 мс. Поэтому RX_BUFFER_SIZE = 4096 (запас ~88 мс) и приём
  *  вычитывается дважды за оборот loop(). Если линия всё равно сыпет
  *  ошибками — уменьшите WIFI_BAUD в mc_wifi.h и LINK_BAUD здесь до 230400
  *  (7 КБ/с влезает с двойным запасом).
  *
  *  ЧЕГО СКЕТЧ НЕ ДЕЛАЕТ: не разбирает телеметрию в структуру и не хранит
  *  историю на сервере. Последний целый JSON-кадр отдаётся как есть
  *  (/api/data), график строит браузер. Меньше кода на ESP — меньше
  *  поводов потерять байты на 460800.
  ******************************************************************************
  */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <string.h>

/* ==========================================================================
 *  НАСТРОЙКИ
 * ========================================================================== */

#define FW_VERSION      "1.0"

/* Должно совпадать с WIFI_BAUD в Core/Inc/mc_wifi.h */
#define LINK_BAUD       460800u

/* Буфер приёма UART. 4096 байт = 89 мс при 460800. Открытие страницы панели
   (~20 КБ) на медленном клиенте этот запас может съесть: тогда пара кадров
   теряется, панель догоняет на следующем. Видно по fps в /api/info. */
#define RX_BUFFER_SIZE  4096

/* Максимальный кадр. В прошивке sbuf = 1024 байта, берём с запасом. */
#define FRAME_MAX       1280

#define HTTP_PORT       80
#define BRIDGE_PORT     23

/* Пинг живости для TRIP_COMM (в прошивке таймаут 5 с). */
#define PING_PERIOD_MS  1000u

/* Панель считается «живой», если браузер опрашивал данные не дольше этого. */
#define POLL_ALIVE_MS   3000u

/* Кадр старше — показываем «нет данных». */
#define STALE_MS        1500u

/* Отсчётов АЦП на 1 А. Должно совпадать с I_CNT_PER_A в Core/Inc/mc.h
   (шунт 0.075 Ом, ACPL-C790 x8.2, Vref 3.3 В, 12 бит). */
static const float I_CNT_PER_A = 763.0f;

/* --- Точка доступа. Пароль короче 8 символов ESP8266 не примет. --- */
static const char AP_SSID[] = "MotorCtrl";
static const char AP_PASS[] = "motor271080";

/* --- Домашняя сеть. Пусто = работать только точкой доступа.
       Заполнено = AP+STA, панель доступна и по 192.168.4.1, и по IP роутера. --- */
static const char STA_SSID[] = "";
static const char STA_PASS[] = "";

/* ==========================================================================
 *  ГЛОБАЛЬНЫЕ ОБЪЕКТЫ И БУФЕРЫ
 * ========================================================================== */

static ESP8266WebServer web(HTTP_PORT);
static WiFiServer        bridge(BRIDGE_PORT);
static WiFiClient        bridgeCli;

static bool     staMode   = false;
static bool     mdnsReady = false;

/* --- последний целый JSON-кадр от STM32 --- */
static char     lastFrame[FRAME_MAX];
static uint16_t lastLen     = 0;
static uint32_t lastFrameMs = 0;

/* --- сборка кадра из потока --- */
static char     fbuf[FRAME_MAX];
static uint16_t flen  = 0;
static uint8_t  depth = 0;

/* --- буфер моста: копим строку, отдаём в TCP целиком --- */
static uint8_t  stage[FRAME_MAX];
static uint16_t stageLen = 0;

/* --- буфер команд из TCP: даёт работать командам без '\n' (тюнер шлёт 'j') --- */
static uint8_t  tcpBuf[96];
static uint8_t  tcpLen    = 0;
static uint32_t tcpLastMs = 0;

/* --- счётчики --- */
static uint32_t framesOk     = 0;
static uint32_t framesBad    = 0;
static uint32_t bridgeDrops  = 0;
static uint32_t bridgeBytes  = 0;
static uint32_t cmdsSent     = 0;
static uint32_t lastPollMs   = 0;
static uint32_t lastPingMs   = 0;
static uint32_t fpsMark      = 0;
static uint32_t fpsCount     = 0;
static uint32_t fpsVal       = 0;

/* Кадр считается свежим, если он не старше STALE_MS. Устаревший кадр панель
   всё равно получает (чтобы показать последние значения), но с пометкой
   X-Stm: stale — и браузер гасит цифры и не тянет график дальше. */
static bool frameFresh()
{
  return (lastLen > 0) && ((millis() - lastFrameMs) <= STALE_MS);
}

/* ==========================================================================
 *  ВЕБ-ПАНЕЛЬ (в flash, ОЗУ не занимает)
 * ========================================================================== */

static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Мотор — панель</title>
<style>
:root{--bg:#0e1116;--card:#161b23;--line:#232c39;--txt:#e6edf6;--dim:#8b98a9;--acc:#39d3a4;--warn:#f2b73c;--bad:#ee5b5b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.45 system-ui,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:1150px;margin:0 auto;padding:14px}
h1{font-size:18px;margin:0 0 2px;font-weight:650}
.sub{color:var(--dim);font-size:12px;font-weight:400;margin-left:6px}
.chips{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0 14px}
.chip{padding:4px 10px;border-radius:20px;background:#1b222c;border:1px solid var(--line);font-size:12px;color:var(--dim)}
.chip.ok{color:#08130f;background:var(--acc);border-color:var(--acc);font-weight:600}
.chip.bad{color:#fff;background:var(--bad);border-color:var(--bad);font-weight:600}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(255px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
.card h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--dim);margin:0 0 10px;font-weight:600}
.big{font-size:38px;font-weight:700;line-height:1.05;font-variant-numeric:tabular-nums}
.big small{font-size:13px;color:var(--dim);font-weight:400;margin-left:4px}
.row{display:flex;justify-content:space-between;gap:8px;font-size:13px;margin-top:6px;color:var(--dim)}
.row b{color:var(--txt);font-variant-numeric:tabular-nums;font-weight:600}
b.v-ok{color:var(--acc)} b.v-bad{color:var(--bad)} b.v-warn{color:var(--warn)}
.bar{height:8px;border-radius:6px;background:#0d1218;overflow:hidden;margin-top:6px;border:1px solid var(--line)}
.bar i{display:block;height:100%;width:0;background:var(--acc);transition:width .2s}
.bar i.warn{background:var(--warn)}
.bar i.bad{background:var(--bad)}
button{font:inherit;color:var(--txt);background:#1d2530;border:1px solid var(--line);border-radius:9px;padding:8px 10px;cursor:pointer}
button:hover{border-color:#3a4757}
button:active{transform:translateY(1px)}
button.pri{background:var(--acc);color:#08130f;border-color:var(--acc);font-weight:600}
button.dng{background:#3a1d1d;border-color:#5a2a2a;color:#ffb3b3}
.btns{display:flex;gap:8px;flex-wrap:wrap}
.seg{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:6px}
.seg button{padding:8px 4px}
.seg button.on{background:#20313a;border-color:var(--acc);color:var(--acc);font-weight:600}
label.f{display:flex;flex-direction:column;gap:4px;font-size:11px;color:var(--dim)}
input[type=number],input[type=range]{font:inherit;background:#0d1218;border:1px solid var(--line);border-radius:8px;color:var(--txt);padding:6px 8px;width:100%}
input[type=range]{padding:0;height:28px}
.form{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}
.tgl{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px dashed #202836;font-size:13px}
.tgl:last-child{border-bottom:0}
.sw{position:relative;width:41px;height:23px;flex:0 0 auto}
.sw input{position:absolute;opacity:0;width:100%;height:100%;margin:0;cursor:pointer}
.sw span{position:absolute;inset:0;background:#2a3340;border-radius:20px;transition:.15s}
.sw span:before{content:"";position:absolute;width:17px;height:17px;left:3px;top:3px;background:#8b98a9;border-radius:50%;transition:.15s}
.sw input:checked+span{background:var(--acc)}
.sw input:checked+span:before{transform:translateX(18px);background:#08130f}
details{margin-top:0}
summary{cursor:pointer;color:var(--dim);font-size:12px;font-weight:600;letter-spacing:.06em;text-transform:uppercase}
canvas{width:100%;height:150px;display:block}
#log{font:11px/1.5 ui-monospace,Consolas,monospace;color:var(--dim);max-height:110px;overflow:auto;background:#0d1218;border:1px solid var(--line);border-radius:8px;padding:8px;margin-top:8px;white-space:pre-wrap}
.hint{color:var(--dim);font-size:12px;margin:8px 0 0}
.wide{grid-column:1/-1}
.wrap.stale .big,.wrap.stale .row b{opacity:.4}
.wrap.stale .bar i{opacity:.4}
</style>
</head>
<body>
<div class="wrap">
  <h1>Контроллер ДПТ<span class="sub">STM32F103C8T6 · ESP8266 D1 mini</span></h1>
  <div class="chips">
    <span id="stLink" class="chip">STM32: —</span>
    <span id="stWifi" class="chip">Wi-Fi: —</span>
    <span id="chipTrip" class="chip" hidden></span>
  </div>

  <div class="grid">

    <div class="card">
      <h2>Обороты</h2>
      <div class="big"><span id="rpm">—</span><small>об/мин</small></div>
      <div class="row"><span>задание</span><b id="rpmSet">—</b></div>
      <div class="row"><span>потенциометр PA2</span><b id="pot">—</b></div>
    </div>

    <div class="card">
      <h2>Ток и ШИМ</h2>
      <div class="big"><span id="amp">—</span><small>А</small></div>
      <div class="row"><span>порог</span><b id="clA">—</b></div>
      <div class="bar"><i id="iBar"></i></div>
      <div class="row"><span>RMS тока</span><b id="rmsA">—</b></div>
      <div class="row"><span>заполнение ШИМ</span><b id="duty">—</b></div>
      <div class="bar"><i id="dBar"></i></div>
      <div class="row"><span>напряжение двигателя</span><b id="mv">—</b></div>
    </div>

    <div class="card">
      <h2>Состояние</h2>
      <div class="row"><span>инвертор</span><b id="sOn">—</b></div>
      <div class="row"><span>авария</span><b id="sTrip">—</b></div>
      <div class="row"><span>режим защиты</span><b id="sProt">—</b></div>
      <div class="row"><span>момент</span><b id="sTl">—</b></div>
      <div class="row"><span>тепловая / макс.</span><b id="sTeMe">—</b></div>
      <div class="row"><span>ПИД</span><b id="sAuto">—</b></div>
      <div class="row"><span>задатчик</span><b id="sVs">—</b></div>
      <div class="row"><span>линия</span><b id="cnt">—</b></div>
    </div>

    <div class="card">
      <h2>Управление</h2>
      <div class="btns">
        <button class="pri" data-cmd="ON">Пуск</button>
        <button data-cmd="OF">Стоп</button>
        <button data-cmd="HB">Кнопка</button>
        <button class="dng" data-cmd="RS">Сброс аварии</button>
      </div>
      <div class="row" style="margin-top:10px"><span>режим защиты</span></div>
      <div class="seg" id="modes">
        <button data-mode="0" title="мягкое выключение">0</button>
        <button data-mode="1" title="отладка">1</button>
        <button data-mode="2" title="тест: защита отключена">2</button>
        <button data-mode="3" title="тяговый, min-селектор">3</button>
      </div>
      <div style="margin-top:8px">
        <div class="tgl">Торк-лимит PA3<label class="sw"><input type="checkbox" id="tl" data-on="tl"><span></span></label></div>
        <div class="tgl">Тепловая I&sup2;t<label class="sw"><input type="checkbox" id="te" data-on="te"><span></span></label></div>
        <div class="tgl">Максимальная &times;2<label class="sw"><input type="checkbox" id="me" data-on="me"><span></span></label></div>
        <div class="tgl">Авто-ПИД<label class="sw"><input type="checkbox" id="auto" data-on="auto"><span></span></label></div>
        <div class="tgl">Контроль пульсаций<label class="sw"><input type="checkbox" id="rip" data-on="rip"><span></span></label></div>
      </div>
    </div>

    <div class="card">
      <h2>Задатчик оборотов</h2>
      <input type="range" id="rv" min="30" max="6000" step="10" value="1000">
      <div class="row"><span>RV</span><b><span id="rvVal">—</span> об/мин</b></div>
      <div class="btns" style="margin-top:8px">
        <button class="pri" id="rvGo">Задать</button>
        <button data-cmd="RP">На потенциометр</button>
      </div>
      <div class="row"><span>рабочий диапазон RL..RH</span><b id="rvRange">—</b></div>
      <p class="hint">При задатчике прошивка ждёт команды: панель шлёт PG раз в секунду. Закрыли браузер — через 5 с сработает TRIP_COMM.</p>
    </div>

    <div class="card">
      <h2>ПИД скорости</h2>
      <div class="form">
        <label class="f">Kp<input type="number" id="kp" min="0" max="5000" step="1"></label>
        <label class="f">Ki<input type="number" id="ki" min="0" max="5000" step="1"></label>
        <label class="f">Kd<input type="number" id="kd" min="0" max="5000" step="1"></label>
      </div>
      <div class="btns" style="margin-top:8px">
        <button class="pri" id="pidGo">Применить</button>
        <button data-cmd="a">Авто</button>
        <button data-cmd="m">Ручной</button>
      </div>
      <div class="row"><span>действующие</span><b id="pidEff">—</b></div>
    </div>

    <div class="card">
      <h2>Плавность и сеть</h2>
      <div class="form">
        <label class="f">SS<input type="number" id="ss" min="1" max="100" step="1"></label>
        <label class="f">RU<input type="number" id="ru" min="1" max="100" step="1"></label>
        <label class="f">RD<input type="number" id="rd" min="1" max="100" step="1"></label>
      </div>
      <div class="form" style="grid-template-columns:1fr 1fr">
        <label class="f">Сеть, В (UV)<input type="number" id="uv" min="100" max="300" step="1"></label>
        <label class="f">Пауза, мс (P)<input type="number" id="pz" min="100" max="30000" step="100" placeholder="3000"></label>
      </div>
      <div class="btns" style="margin-top:8px">
        <button class="pri" id="smoothGo">Применить</button>
        <button data-cmd="w">Сохранить в Flash</button>
      </div>
    </div>

    <div class="card wide">
      <h2>Обороты и ток</h2>
      <canvas id="ch"></canvas>
      <div id="log"></div>
    </div>

    <details class="card wide">
      <summary>Дополнительные параметры</summary>
      <div class="form" style="grid-template-columns:repeat(4,1fr)">
        <label class="f">FR<input type="number" id="fr" min="1" max="4" step="1"></label>
        <label class="f">FD<input type="number" id="fd" min="1" max="4" step="1"></label>
        <label class="f">SL<input type="number" id="sl" min="1" max="8" step="1"></label>
        <label class="f">BT, мс<input type="number" id="bt" min="0" max="5000" step="50"></label>
        <label class="f">CP<input type="number" id="cp" min="1" max="5000" step="1"></label>
        <label class="f">CI<input type="number" id="ci" min="0" max="5000" step="1"></label>
        <label class="f">CL, отсч.<input type="number" id="clv" min="50" max="3000" step="1"></label>
        <label class="f">CN, отсч.<input type="number" id="cnv" min="5" max="3000" step="1"></label>
      </div>
      <div class="btns" style="margin-top:8px">
        <button class="pri" id="advGo">Применить</button>
        <button data-cmd="ZD">Выгрузить зоны</button>
        <button data-cmd="ZR">Зоны по умолчанию</button>
        <button data-cmd="j">Формат JSON</button>
        <button data-cmd="o">O-кадры</button>
        <button data-cmd="WE1">Мост ВКЛ</button>
        <button data-cmd="WE0">Мост ВЫКЛ</button>
      </div>
      <p class="hint">WE0 выключает телеметрию и команды по Wi-Fi до перезагрузки STM32 — панель перестанет получать кадры.</p>
    </details>

  </div>
</div>
<script>
var $=function(i){return document.getElementById(i)};
var ICNT=763;
var TRIP=['норма','перегрузка','пик тока','тепловая I\u00B2t','энкодер','разгон','стоянка под током','датчик тока','обрыв якоря','потеря связи','watchdog','аппаратный BKIN'];
var PROT=['0 · мягкий','1 · отладка','2 · ТЕСТ','3 · тяговый'];
var ON_SW=['кнопка PB12','ON','OF'];
var st=null, hist=[], logs=[], chartT=0;

function addLog(s){
  var t=new Date();
  logs.push(('0'+t.getHours()).slice(-2)+':'+('0'+t.getMinutes()).slice(-2)+':'+('0'+t.getSeconds()).slice(-2)+' '+s);
  if(logs.length>40) logs.shift();
  $('log').textContent=logs.join('\n');
}

function cmd(c){
  fetch('/api/cmd?c='+encodeURIComponent(c),{cache:'no-store'}).then(function(r){
    addLog((r.ok?'\u2192 ':'\u2715 ')+c);
  }).catch(function(){ addLog('\u2715 '+c+' — нет связи'); });
}

document.addEventListener('click',function(e){
  var b=e.target.closest('[data-cmd]');
  if(b){
    if(b.dataset.cmd==='w'){
      if(st && st.on){ alert('Мотор работает (on=1): прошивка проигнорирует запись в Flash. Сначала остановите двигатель.'); return; }
      if(!confirm('Записать настройки в Flash STM32?')) return;
    }
    if(b.dataset.cmd==='WE0' && !confirm('Мост Wi-Fi выключится до перезагрузки STM32 — панель перестанет получать данные. Продолжить?')) return;
    cmd(b.dataset.cmd); return;
  }
  var m=e.target.closest('[data-mode]');
  if(m) cmd(m.dataset.mode);
});

document.addEventListener('change',function(e){
  var t=e.target, key=t.dataset.on;
  if(!key) return;
  var pairs={tl:['TL0','TL1'],te:['TE0','TE1'],me:['ME0','ME1'],auto:['m','a'],rip:['s','r']};
  var p=pairs[key];
  if(p) cmd(p[t.checked?1:0]);
});

function link(ok,txt){
  var el=$('stLink'); el.textContent=txt; el.className='chip '+(ok?'ok':'bad');
}

async function poll(){
  try{
    var r=await fetch('/api/data',{cache:'no-store'});
    var h=r.headers.get('X-Stm')||'none';
    var age=parseInt(r.headers.get('X-Frame-Age')||'0',10)||0;
    var j=JSON.parse(await r.text());
    if(!j || j.rpm===undefined){
      dim(true);
      link(false, h==='stale'?('STM32: молчит '+Math.round(age/1000)+' с'):'STM32: нет кадров');
      return;
    }
    st=j; render(j);
    if(h==='ok'){ link(true,'STM32: онлайн'); dim(false); pushHist(j); }
    else { link(false,'STM32: молчит '+Math.round(age/1000)+' с'); dim(true); }
  }catch(e){ dim(true); link(false,'STM32: нет связи'); }
}

function dim(on){
  var w=document.querySelector('.wrap');
  if(w) w.classList.toggle('stale',!!on);
}

function bar(id,p){
  var el=$(id); el.style.width=Math.max(0,Math.min(100,p))+'%';
  el.className=p>95?'bad':(p>75?'warn':'');
}

function setTxt(id,v,cls){
  var el=$(id); el.textContent=v;
  el.className=cls||'';
}

function setVal(id,v){ var el=$(id); if(el && document.activeElement!==el) el.value=v; }
function setChk(id,v){ var el=$(id); if(el && document.activeElement!==el) el.checked=!!v; }

function render(j){
  $('rpm').textContent=j.rpm;
  $('rpmSet').textContent=j.rpm_set+' об/мин';
  $('pot').textContent=j.pot;

  var amp=j.cur/ICNT, cl=j.cl/ICNT, rms=j.rms/ICNT;
  $('amp').textContent=amp.toFixed(2);
  $('clA').textContent=cl.toFixed(2)+' А';
  $('rmsA').textContent=rms.toFixed(2)+' А';
  bar('iBar', cl>0 ? amp/cl*100 : 0);

  $('duty').textContent=(j.duty/10).toFixed(1)+' %';
  bar('dBar', j.duty/9);
  $('mv').textContent=j.mv+' В';

  setTxt('sOn',(j.on?'ВКЛ':'ВЫКЛ')+' · '+(ON_SW[j.on_sw]||'—'), j.on?'v-ok':'v-bad');
  setTxt('sTrip', j.trip?(TRIP[j.trip]||('код '+j.trip)):'нет', j.trip?'v-bad':'v-ok');
  setTxt('sProt', PROT[j.prot]||('режим '+j.prot), j.prot===2?'v-warn':'');
  setTxt('sTl', j.tl?'PA3 (ограничитель)':'полный ток', j.tl?'':'v-warn');
  setTxt('sTeMe',(j.te?'TE вкл':'TE выкл')+' / '+(j.me?'ME вкл':'ME выкл'), (j.te&&j.me)?'v-ok':'');
  setTxt('sAuto',(j.auto?'авто':'ручной')+(j.rip?' · пульсации':''), j.auto?'v-ok':'');
  setTxt('sVs', j.vs?('RV '+j.vr):'потенциометр', j.vs?'v-warn':'');

  var ct=$('chipTrip');
  if(j.trip){
    ct.hidden=false; ct.textContent='АВАРИЯ: '+(TRIP[j.trip]||j.trip); ct.className='chip bad';
  } else ct.hidden=true;

  var btns=document.querySelectorAll('[data-mode]');
  for(var i=0;i<btns.length;i++) btns[i].classList.toggle('on', +btns[i].dataset.mode===j.prot);

  setChk('tl',j.tl); setChk('te',j.te); setChk('me',j.me);
  setChk('auto',j.auto); setChk('rip',j.rip);

  var rv=$('rv');
  rv.min=j.rlo; rv.max=j.rhi;
  if(document.activeElement!==rv) rv.value=j.vs?j.vr:j.rpm_set;
  $('rvVal').textContent=rv.value;
  $('rvRange').textContent=j.rlo+'…'+j.rhi;

  setVal('kp',j.kpu); setVal('ki',j.kiu); setVal('kd',j.kdu);
  $('pidEff').textContent=j.kp+' / '+j.ki+' / '+j.kd;
  setVal('ss',j.ss); setVal('ru',j.ru); setVal('rd',j.rd); setVal('uv',j.uv);
  setVal('fr',j.fr); setVal('fd',j.fd); setVal('sl',j.sl);
  setVal('cp',j.cikp); setVal('ci',j.ciki); setVal('bt',j.bt);
  setVal('clv',j.cls); setVal('cnv',j.cn);
}

function pushHist(j){
  hist.push({r:j.rpm,a:j.cur/ICNT});
  if(hist.length>140) hist.shift();
  var now=Date.now();
  if(now-chartT>400){ chartT=now; drawChart(); }
}

function drawChart(){
  var c=$('ch'), dpr=window.devicePixelRatio||1;
  var w=c.clientWidth||600, h=c.clientHeight||150;
  if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){ c.width=Math.round(w*dpr); c.height=Math.round(h*dpr); }
  var g=c.getContext('2d');
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  g.strokeStyle='#202836'; g.lineWidth=1;
  for(var i=0;i<=4;i++){
    var y=10+(h-20)*i/4;
    g.beginPath(); g.moveTo(0,y); g.lineTo(w,y); g.stroke();
  }
  if(hist.length<2) return;
  var rmax=1, amax=0.01;
  for(var k=0;k<hist.length;k++){ if(hist[k].r>rmax) rmax=hist[k].r; if(hist[k].a>amax) amax=hist[k].a; }
  rmax*=1.15; amax*=1.15;
  plot(g,w,h,hist,'r',rmax,'#39d3a4');
  plot(g,w,h,hist,'a',amax,'#f2b73c');
  g.fillStyle='#8b98a9'; g.font='11px system-ui';
  g.fillText('об/мин до '+Math.round(rmax)+'  ·  ток до '+amax.toFixed(2)+' А',8,12);
}

function plot(g,w,h,arr,key,max,color){
  g.strokeStyle=color; g.lineWidth=1.6; g.beginPath();
  var n=arr.length;
  for(var i=0;i<n;i++){
    var x=w*i/(n-1), y=h-10-(h-20)*Math.min(1,arr[i][key]/max);
    if(i) g.lineTo(x,y); else g.moveTo(x,y);
  }
  g.stroke();
}

async function info(){
  try{
    var j=await (await fetch('/api/info',{cache:'no-store'})).json();
    $('stWifi').textContent='Wi-Fi: '+j.mode+' '+j.ip+(j.clients?(' · клиентов '+j.clients):'')+(j.rssi?(' · '+j.rssi+' dBm'):'');
    $('stWifi').className='chip '+(j.sta?'ok':'');
    $('cnt').textContent='кадры '+j.frames+' · битых '+j.bad+' · мост '+(j.bridge?'да':'—')+' · команд '+j.cmd;
  }catch(e){}
}

$('rvGo').onclick=function(){ cmd('RV'+$('rv').value); };
$('rv').addEventListener('change',function(){ cmd('RV'+$('rv').value); });
$('pidGo').onclick=function(){
  var kp=+$('kp').value, ki=+$('ki').value, kd=+$('kd').value;
  if(isNaN(kp)||isNaN(ki)||isNaN(kd)) return;
  cmd('KP'+Math.max(0,Math.min(5000,Math.round(kp))));
  cmd('KI'+Math.max(0,Math.min(5000,Math.round(ki))));
  cmd('KD'+Math.max(0,Math.min(5000,Math.round(kd))));
};
$('smoothGo').onclick=function(){
  var ss=$('ss').value, ru=$('ru').value, rd=$('rd').value, uv=$('uv').value, pz=$('pz').value;
  if(ss) cmd('SS'+ss);
  if(ru) cmd('RU'+ru);
  if(rd) cmd('RD'+rd);
  if(uv) cmd('UV'+uv);
  if(pz) cmd('P'+pz);
};
$('advGo').onclick=function(){
  var m=[['fr','FR'],['fd','FD'],['sl','SL'],['cp','CP'],['ci','CI'],['bt','BT'],['clv','CL'],['cnv','CN']];
  for(var i=0;i<m.length;i++){
    var v=$(m[i][0]).value;
    if(v!=='') cmd(m[i][1]+v);
  }
};

setInterval(poll,300);
setInterval(info,5000);
poll(); info(); drawChart();
window.addEventListener('resize',drawChart);
</script>
</body>
</html>)HTML";

/* ==========================================================================
 *  МОСТ UART -> TCP
 *  Копим строку целиком: пропуск кадра лучше, чем кадр, разорванный
 *  посередине (тюнер тогда ругается на JSON).
 * ========================================================================== */

static bool bridgeConnected()
{
  return bridgeCli.connected();
}

static void bridgeFlush()
{
  if (stageLen == 0) return;

  if (bridgeConnected())
  {
    int space = bridgeCli.availableForWrite();
    if (space >= (int)stageLen)
    {
      bridgeCli.write((const uint8_t *)stage, stageLen);
      bridgeBytes += stageLen;
    }
    else
    {
      bridgeDrops++;          /* клиент не успевает — теряем кадр целиком */
    }
  }
  stageLen = 0;
}

static void bridgeFeed(const uint8_t *p, size_t n)
{
  if (!bridgeConnected()) { stageLen = 0; return; }

  for (size_t i = 0; i < n; i++)
  {
    stage[stageLen++] = p[i];
    if (p[i] == '\n' || stageLen >= FRAME_MAX) bridgeFlush();
  }
}

/* ==========================================================================
 *  СБОРКА JSON-КАДРА ДЛЯ ПАНЕЛИ
 *
 *  В телеметрии нет строк — только числа и массивы, поэтому хватает счёта
 *  фигурных скобок. Мусор и O-кадры (бинарные) просто не дают кадра.
 * ========================================================================== */

static void publishFrame()
{
  fbuf[flen] = '\0';
  memcpy(lastFrame, fbuf, (size_t)flen + 1u);
  lastLen     = flen;
  lastFrameMs = millis();
  framesOk++;
  fpsCount++;
}

static void frameFeed(const uint8_t *p, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    char c = (char)p[i];

    if (depth == 0)
    {
      if (c == '{') { fbuf[0] = c; flen = 1; depth = 1; }
      continue;
    }

    if (flen >= (FRAME_MAX - 1))
    {
      depth = 0; flen = 0; framesBad++;   /* кадр длиннее буфера — брак */
      continue;
    }

    fbuf[flen++] = c;

    if (c == '{') depth++;
    else if (c == '}') { depth--; if (depth == 0) publishFrame(); }
  }
}

/* ==========================================================================
 *  КОМАНДЫ
 * ========================================================================== */

static void linkSend(const String &c)
{
  Serial.write((const uint8_t *)c.c_str(), c.length());
  Serial.write('\n');
  cmdsSent++;
}

static bool digitsOnly(const char *p, size_t n)
{
  if (n == 0 || n > 7) return false;
  for (size_t i = 0; i < n; i++)
    if (p[i] < '0' || p[i] > '9') return false;
  return true;
}

/* Белый список команд из Core/Src/mc_cmd.c. Всё остальное — 400.
   Смысл не в защите от «хакера» (сеть локальная), а в том, чтобы опечатка
   в URL не отправила в моторный контроллер случайную строку. */
static bool cmdAllowed(const String &c)
{
  const char *s = c.c_str();
  size_t n = c.length();

  if (n < 1 || n > 40) return false;
  for (size_t i = 0; i < n; i++)
    if (s[i] < 0x21 || s[i] > 0x7e) return false;   /* без пробелов и управляющих */

  if (n == 1)
    return (s[0] == 'j' || s[0] == 'o' || s[0] == 'a' || s[0] == 'm' ||
            s[0] == 'r' || s[0] == 's' || s[0] == 'w' ||
            (s[0] >= '0' && s[0] <= '3'));

  static const char *WORD[] = { "ON", "OF", "HB", "RP", "PG", "RS", "ZD", "ZR", "CF" };
  for (size_t i = 0; i < sizeof(WORD) / sizeof(WORD[0]); i++)
    if (c == WORD[i]) return true;

  static const char *NUM[] = {
    "KP", "KI", "KD", "SS", "RU", "RD", "UV", "RV", "RL", "RH",
    "TE", "ME", "TL", "BT", "OL", "OP", "OT", "TQ", "TC", "TB", "TZ",
    "TR", "TM", "TF", "TS", "CP", "CI", "CL", "CN", "FR", "FD", "SL",
    "WE", "P"
  };
  for (size_t i = 0; i < sizeof(NUM) / sizeof(NUM[0]); i++)
  {
    size_t L = strlen(NUM[i]);
    if (n > L && strncmp(s, NUM[i], L) == 0 && digitsOnly(s + L, n - L)) return true;
  }

  /* Z<n>P/I/D/R<значение> — таблица зон */
  if (n >= 4 && s[0] == 'Z' && s[1] >= '0' && s[1] <= '5' &&
      (s[2] == 'P' || s[2] == 'I' || s[2] == 'D' || s[2] == 'R') &&
      digitsOnly(s + 3, n - 3))
    return true;

  return false;
}

/* ==========================================================================
 *  ПРИЁМ ИЗ UART
 * ========================================================================== */

static uint8_t chunk[256];

static void uartPump()
{
  int avail = Serial.available();

  while (avail > 0)
  {
    size_t want = (avail > (int)sizeof(chunk)) ? sizeof(chunk) : (size_t)avail;
    size_t got  = Serial.readBytes((char *)chunk, want);
    if (got == 0) break;

    bridgeFeed(chunk, got);
    frameFeed(chunk, got);

    avail = Serial.available();
  }
}

/* ==========================================================================
 *  TCP-МОСТ: приём команд
 * ========================================================================== */

static void tcpFlush(bool addNewline)
{
  if (tcpLen == 0) return;
  Serial.write(tcpBuf, tcpLen);
  if (addNewline) Serial.write('\n');
  cmdsSent++;
  tcpLen = 0;
}

static void tcpFeed(const uint8_t *p, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    uint8_t c = p[i];

    if (tcpLen >= sizeof(tcpBuf)) tcpFlush(true);   /* слишком длинная строка */

    tcpBuf[tcpLen++] = c;
    tcpLastMs = millis();

    if (c == '\n' || c == '\r') tcpFlush(false);
  }
}

/* Тюнер шлёт одиночный 'j' без перевода строки (pid_tuner.py: ser.write(b'j')),
   а прошивка разбирает команду только по '\n'. Поэтому незавершённую строку
   закрываем сами, если пауза больше 25 мс. */
static void tcpIdle()
{
  if (tcpLen > 0 && (millis() - tcpLastMs) > 25u) tcpFlush(true);
}

static void bridgeAccept()
{
  WiFiClient nc = bridge.accept();   /* core 3.x; в 2.x было available() */
  if (!nc) return;

  if (bridgeConnected()) bridgeCli.stop();   /* один клиент за раз */
  bridgeCli = nc;
  bridgeCli.setNoDelay(true);
  stageLen = 0;
}

static void bridgePump()
{
  if (!bridgeConnected()) return;

  int n = bridgeCli.available();
  if (n <= 0) return;

  uint8_t b[64];
  while (n > 0)
  {
    size_t want = (n > (int)sizeof(b)) ? sizeof(b) : (size_t)n;
    size_t got  = bridgeCli.read(b, want);
    if (got == 0) break;
    tcpFeed(b, got);
    n -= (int)got;
  }
}

/* ==========================================================================
 *  ЖИВОСТЬ КАНАЛА
 *
 *  Прошивка ставит TRIP_COMM, если rpm_virt_sw && on && нет команд 5 с.
 *  Пингуем PG, только пока панель реально опрашивается или подключён TCP-мост:
 *  закрыли браузер — и защита снова работает как задумано.
 * ========================================================================== */

static void keepAlive()
{
  bool active = bridgeConnected() ||
                (lastPollMs != 0 && (millis() - lastPollMs) < POLL_ALIVE_MS);

  if (!active) return;

  if (millis() - lastPingMs >= PING_PERIOD_MS)
  {
    lastPingMs = millis();
    linkSend(String("PG"));
  }
}

/* ==========================================================================
 *  HTTP
 * ========================================================================== */

static void handleRoot()
{
  web.sendHeader("Cache-Control", "no-store, must-revalidate");
  web.send_P(200, "text/html", PAGE);
}

static void handleData()
{
  lastPollMs = millis();

  web.sendHeader("Cache-Control", "no-store");
  web.sendHeader("Access-Control-Allow-Origin", "*");

  if (lastLen == 0)
  {
    web.sendHeader("X-Stm", "none");
    web.sendHeader("X-Frame-Age", "0");
    web.send(200, "application/json", "{}");
    return;
  }

  uint32_t ageMs = millis() - lastFrameMs;

  web.sendHeader("X-Stm", frameFresh() ? "ok" : "stale");
  web.sendHeader("X-Frame-Age", String(ageMs));
  web.send(200, "application/json", lastFrame);
}

static void handleCmd()
{
  lastPollMs = millis();

  if (!web.hasArg("c"))
  {
    web.send(400, "text/plain", "no c");
    return;
  }

  String c = web.arg("c");
  c.trim();

  if (!cmdAllowed(c))
  {
    web.send(400, "text/plain", "bad cmd");
    return;
  }

  linkSend(c);
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "text/plain", "ok");
}

static String currentIp()
{
  if (staMode && WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
  return WiFi.softAPIP().toString();
}

static void handleInfo()
{
  String j = "{";
  j += "\"fw\":\"" FW_VERSION "\",";
  j += "\"mode\":\"";
  j += staMode ? "AP+STA" : "AP";
  j += "\",";
  j += "\"ip\":\"";
  j += currentIp();
  j += "\",";
  j += "\"sta\":";
  j += (WiFi.status() == WL_CONNECTED) ? 1 : 0;
  j += ",\"rssi\":";
  j += (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  j += ",\"clients\":";
  j += WiFi.softAPgetStationNum();
  j += ",\"frames\":";
  j += framesOk;
  j += ",\"bad\":";
  j += framesBad;
  j += ",\"bridge\":";
  j += bridgeConnected() ? 1 : 0;
  j += ",\"drops\":";
  j += bridgeDrops;
  j += ",\"btx\":";
  j += bridgeBytes;
  j += ",\"cmd\":";
  j += cmdsSent;
  j += ",\"fps\":";
  j += fpsVal;
  j += ",\"up\":";
  j += millis() / 1000u;
  j += "}";

  web.sendHeader("Cache-Control", "no-store");
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.send(200, "application/json", j);
}

static void handleFavicon()
{
  web.send(204, "text/plain", "");
}

/* ==========================================================================
 *  СЛУЖЕБНОЕ
 * ========================================================================== */

static void statsTick()
{
  uint32_t now = millis();
  if (now - fpsMark >= 1000u)
  {
    fpsVal   = fpsCount;
    fpsCount = 0;
    fpsMark  = now;
  }
}

/* ==========================================================================
 *  SETUP / LOOP
 * ========================================================================== */

void setup()
{
  /* Буфер приёма задаётся ДО begin: по умолчанию 256 байт, на 460800 это 5 мс. */
  Serial.setRxBufferSize(RX_BUFFER_SIZE);
  Serial.begin(LINK_BAUD);
  Serial.swap();                 /* UART0 -> GPIO13 (RX) / GPIO15 (TX) */
  Serial.setDebugOutput(false);  /* чтобы отладочный вывод не ушёл в STM32 */

  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  staMode = (STA_SSID[0] != '\0');

  if (staMode)
  {
    /* Панель доступна сразу в точке доступа, подключение к роутеру идёт фоном:
       блокирующий while(WiFi.status()!=WL_CONNECTED) здесь недопустим — на
       460800 UART переполнится за время ожидания. */
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);
    WiFi.begin(STA_SSID, STA_PASS);
  }
  else
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);
  }

  web.on("/", handleRoot);
  web.on("/api/data", handleData);
  web.on("/api/cmd", handleCmd);
  web.on("/api/info", handleInfo);
  web.on("/favicon.ico", handleFavicon);
  web.begin();

  bridge.begin();
  bridge.setNoDelay(true);

  mdnsReady = MDNS.begin("motor");
  if (mdnsReady) MDNS.addService("http", "tcp", HTTP_PORT);

  fpsMark = millis();
}

void loop()
{
  uartPump();        /* UART первым: на 460800 каждая миллисекунда дорога */

  web.handleClient();

  uartPump();
  bridgeAccept();
  bridgePump();
  tcpIdle();

  keepAlive();
  statsTick();

  if (mdnsReady) MDNS.update();
}
