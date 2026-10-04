/*
 * ESP32-C3 神光同步 —— 被控端（Slave） V3
 *   - 支持多路 WS2812B 灯带（最多 MAX_STRIPS 路），引脚 / 灯珠数运行时可配
 *   - 灯珠数量无需预知：各路颗数由 PC 端填写并写入 NVS，固件按实际长度渲染
 *   - 各路可「拼接成一条长灯带」或「独立同时跑同一灯效」
 *   - 板载 GPIO8 单色 LED 可跟随灯带亮度同步
 *   - 通过串口接收 PC 指令，实时切换灯效 / 颜色 / 亮度 / 速度
 *   - 支持：参数化灯效、多色渐变、分段控制、编排轮播、音频律动、像素画
 *   - 全部设置写入 NVS，PC 断开或板子复位后保持
 *
 * 依赖库：Adafruit NeoPixel
 *
 * 串口协议（每行一条，\n 结尾，大小写不敏感）：
 *   PING                       -> 回 PONG
 *   GET                        -> 回 STATE/STRIPS/STRIP/SEG/STEP/PIX ... END 状态块
 *   OFF                        关闭
 *   MODE 0..14                 0关闭 1静态 2呼吸 3彩虹 4流星 5流光 6渐变
 *                              7正弦波 8心跳 9警灯 10星空 11色块 12双色交替
 *                              13自定义像素 14音乐律动
 *   MODE 15                    立体声律动：左/右声道各自从两端向中间律动，
 *                              无声音时回落 IDLE 指定的灯效
 *   IDLE 0..14                 立体声律动无声时使用的灯效
 *   ---- 灯带配置（存独立 NVS key，改 cfg 结构）----
 *   STRIPS n merge             启用前 n 路（1..MAX_STRIPS）并设定 0=独立 / 1=拼接
 *   STRIPSET i pin count en    配置第 i 路：引脚 / 灯珠数 / 是否启用
 *   MERGE 0|1                  仅切换拼接模式
 *   STRIPCLR                   恢复默认：1 路 GPIO10 / 14 颗 / 拼接
 *   ---- 灯效参数 ----
 *   COLOR r g b                起色      COLOR2 r g b   终色
 *   HUE 0..255                 色相跨度（彩虹/渐变展开）
 *   DIR 0|1                    方向
 *   TAIL 1..逻辑长度           尾迹长度（流星/流光）
 *   SPACING 0..6               灯间距（色块/双色交替）
 *   MIRROR 0|1                 镜像对称
 *   RANDOM 0..100              随机强度（星空）
 *   GAMMA 0..100               亮度曲线（0=线性）
 *   GAIN 0..100                音频灵敏度
 *   BRIGHT 0..255 / SPEED 1..100 / SYNC 0|1
 *   SEG i start len mode r g b 写入第 i 段；SEGCLR 清空所有段
 *   PIX i r g b                单颗像素；PIXCLR 清空
 *   PIXB start v v v ...       分块写入像素画（每 3 个整数组一颗，可多次调用）
 *   STEP i mode secs r1 g1 b1 r2 g2 b2   写入第 i 个编排步骤；STEPCLR 清空
 *   PLAY 0|1                   编排开始/停止；PLAYNEXT 手动下一步
 *   LEVEL v [l r]              音频电平（不写 NVS）；给出 l r 时为左右声道
 *
 * 渲染优先级：编排运行中 > 存在分段 > 全局灯效
 * 逻辑坐标：拼接模式下 = 各路首尾相接的总长；独立模式下 = 各路各自的长度
 * 接线：每条灯带 5V 独立供电，GND 与开发板共地，DIN -> 所选 GPIO
 * 可用 GPIO（C3）：0~7、10（8=板载LED，9=BOOT，11 起为 Flash/USB，勿用）
 * 开发板选项：USB CDC On Boot = Enabled（否则串口收不到）
 */

#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>

#define STRIP_PIN           10       // 默认引脚
#define STRIP_COUNT         14       // 默认灯珠数（同时决定遗留 pix 字段大小）
#define ONBOARD_PIN         8
#define ONBOARD_ACTIVE_LOW  0        // 板载 LED 低电平点亮则改成 1

#define PWM_FREQ_HZ         5000
#define PWM_RES_BITS        10
#define DUTY_MAX            ((1 << PWM_RES_BITS) - 1)

#define MAX_SEG             4
#define MAX_STEP            8
#define CFG_VERSION         3
#define STEREO_SILENT       4        // 判定「无声音」的电平阈值

#define MAX_STRIPS          4        // 最多支持的灯带路数
#define MAX_PER_STRIP       128      // 单路灯珠上限（4×128 = 512）
#define MAX_TOTAL_PIXELS    512      // 逻辑灯珠总量上限（含拼接）
#define STRIP_CFG_VERSION   1

enum Effect : uint8_t {
  E_OFF = 0, E_STATIC, E_BREATH, E_RAINBOW, E_COMET, E_FLOW,
  E_GRADIENT, E_SINE, E_HEARTBEAT, E_ALERT,
  E_STARFIELD, E_BLOCK, E_ALTERNATE, E_CUSTOM, E_MUSIC, E_STEREO, E_COUNT
};

struct Segment { uint8_t start, len, mode, r, g, b; };
struct Step    { uint8_t mode, r1, g1, b1, r2, g2, b2, speed; uint16_t secs; };

struct Settings {
  uint8_t  version;
  uint8_t  mode;
  uint16_t bright;
  uint8_t  speed;
  uint8_t  onboardSync;
  uint8_t  gamma;                 // 0..100 -> 指数 1.0..3.0
  uint8_t  r1, g1, b1;            // 起色
  uint8_t  r2, g2, b2;            // 终色
  uint8_t  hueSpan;
  uint8_t  dir;
  uint8_t  tail;
  uint8_t  spacing;
  uint8_t  mirror;
  uint8_t  randomAmt;
  uint8_t  musicGain;
  uint8_t  segCount;
  Segment  segs[MAX_SEG];
  uint8_t  pix[STRIP_COUNT * 3];  // 遗留字段：仅用于维持 NVS 布局，实际像素画见 pixBuf
  uint8_t  stepCount;
  uint8_t  playing;
  uint8_t  idleMode;              // 立体声律动无声音时回落的灯效（占用旧版填充字节）
  Step     steps[MAX_STEP];
};

Settings   cfg;
Preferences prefs;

// 布局约束：结构体总大小与旧版 v2 相同（170），且 idleMode 正好落在旧版的填充字节上，
// 这样老固件保存的配置在新固件里仍然按原偏移读取，不会串位。
static_assert(sizeof(Settings) == 170, "Settings 大小变化，需同步处理 NVS 迁移");
static_assert(offsetof(Settings, idleMode) == 89, "idleMode 必须占用旧版填充字节");

// ---- 多灯带配置（独立 NVS key，避免动 cfg 布局）----
struct StripDesc { uint16_t count; uint8_t pin; uint8_t enabled; };
struct StripCfg {
  uint8_t   version;
  uint8_t   numStrips;            // 启用的路数（1..MAX_STRIPS）
  uint8_t   merge;                // 0=各路独立 1=拼接成一条长灯带
  uint8_t   reserved;
  StripDesc strips[MAX_STRIPS];
};
StripCfg sc;                      // 灯带硬件配置

Adafruit_NeoPixel *strips[MAX_STRIPS] = { nullptr };
uint16_t stripLen[MAX_STRIPS];    // 每路实际灯珠数
uint16_t stripOfs[MAX_STRIPS];    // 拼接模式下该路在逻辑坐标里的起点；独立模式恒为 0
uint8_t  activeStrips = 0;        // 实际生效的路数（跳过禁用/非法项）
uint16_t logicalLen   = 0;        // 逻辑总长（拼接=各路之和，独立=最长一路）
int8_t   curTarget = -1;          // 渲染上下文：-1 跨路(拼接)，>=0 指定某一路(独立)
uint16_t curLen    = 0;           // 渲染长度上下文
uint8_t  pixBuf[MAX_TOTAL_PIXELS * 3];   // 像素画缓冲（逻辑坐标）
uint8_t  starVal[MAX_TOTAL_PIXELS];      // 星空各颗余辉

bool     dirty   = false;         // cfg 是否有改动待写入
uint32_t dirtyMs = 0;
bool     pixDirty = false;        // 像素画是否有改动待写入
uint32_t pixDirtyMs = 0;

uint32_t lastTick = 0, lastFrame = 0, animMs = 0;
uint8_t  curStep = 0;
uint32_t stepStart = 0;
uint8_t  musicLevel = 0;          // 最近一次音频电平（已按灵敏度缩放）
uint8_t  musicLevelL = 0;         // 左声道电平
uint8_t  musicLevelR = 0;         // 右声道电平
uint32_t levelMs = 0;
uint8_t  gammaLut[256];
String   line;

void markDirty()    { dirty = true; dirtyMs = millis(); }
void markPixDirty() { pixDirty = true; pixDirtyMs = millis(); }

// ---------------- 默认值 / 持久化（NVS） ----------------
void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.version     = CFG_VERSION;
  cfg.mode        = E_BREATH;
  cfg.bright      = 120;
  cfg.speed       = 50;
  cfg.onboardSync = 1;
  cfg.gamma       = 0;
  cfg.r1 = 0;   cfg.g1 = 150; cfg.b1 = 255;
  cfg.r2 = 0;   cfg.g2 = 0;   cfg.b2 = 0;
  cfg.hueSpan     = 255;
  cfg.dir         = 0;
  cfg.tail        = 6;
  cfg.spacing     = 1;
  cfg.mirror      = 0;
  cfg.randomAmt   = 20;
  cfg.musicGain   = 0;
  cfg.idleMode    = E_BREATH;      // 无声时默认回落呼吸
  cfg.segCount    = 0;
  cfg.stepCount   = 0;
  cfg.playing     = 0;
}

void loadSettings() {
  setDefaults();
  prefs.begin("argb", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(Settings)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
    // 旧版（v2）没有 idleMode，该字节只是结构体填充位，取到的值不可信
    if (cfg.version != CFG_VERSION) cfg.idleMode = E_BREATH;
  } else {
    // 旧版本单键回退，保证升级后不丢设置
    uint8_t m = prefs.getUChar("mode", 255);
    if (m != 255) {
      cfg.mode        = (m < E_COUNT) ? m : E_BREATH;
      cfg.r1          = prefs.getUChar("r", cfg.r1);
      cfg.g1          = prefs.getUChar("g", cfg.g1);
      cfg.b1          = prefs.getUChar("b", cfg.b1);
      cfg.bright      = prefs.getUShort("bright", cfg.bright);
      cfg.speed       = prefs.getUChar("speed", cfg.speed);
      cfg.onboardSync = prefs.getBool("sync", true) ? 1 : 0;
    }
  }
  prefs.end();

  // 兜底校验
  if (cfg.mode >= E_COUNT) cfg.mode = E_BREATH;
  if (cfg.speed < 1)   cfg.speed = 1;
  if (cfg.speed > 100) cfg.speed = 100;
  if (cfg.bright > 255) cfg.bright = 255;
  if (cfg.segCount > MAX_SEG)  cfg.segCount = MAX_SEG;
  if (cfg.stepCount > MAX_STEP) cfg.stepCount = MAX_STEP;
  for (uint8_t i = 0; i < cfg.segCount; i++) {
    if (cfg.segs[i].len < 1) cfg.segs[i].len = 1;
    if (cfg.segs[i].start >= MAX_TOTAL_PIXELS) cfg.segs[i].start = 0;
    if (cfg.segs[i].start + cfg.segs[i].len > MAX_TOTAL_PIXELS)
      cfg.segs[i].len = MAX_TOTAL_PIXELS - cfg.segs[i].start;
    if (cfg.segs[i].mode >= E_COUNT) cfg.segs[i].mode = E_BREATH;
  }
  for (uint8_t i = 0; i < cfg.stepCount; i++)
    if (cfg.steps[i].mode >= E_COUNT) cfg.steps[i].mode = E_BREATH;
  if (cfg.idleMode >= E_COUNT || cfg.idleMode == E_STEREO) cfg.idleMode = E_BREATH;
}

void saveSettings() {
  cfg.version = CFG_VERSION;
  prefs.begin("argb", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}

// ---- 灯带配置 / 像素画的独立持久化 ----
bool pinAllowed(uint8_t p) { return (p <= 7) || (p == 10); }   // C3 可用 IO

void setStripDefaults() {
  memset(&sc, 0, sizeof(sc));
  sc.version   = STRIP_CFG_VERSION;
  sc.numStrips = 1;
  sc.merge     = 1;
  sc.strips[0].pin     = STRIP_PIN;
  sc.strips[0].count   = STRIP_COUNT;
  sc.strips[0].enabled = 1;
}

void loadStrips() {
  setStripDefaults();
  prefs.begin("argb", true);
  if (prefs.getBytesLength("strips") == sizeof(StripCfg)) {
    prefs.getBytes("strips", &sc, sizeof(sc));
    if (sc.version != STRIP_CFG_VERSION) setStripDefaults();
  }
  prefs.end();
  if (sc.numStrips < 1) sc.numStrips = 1;
  if (sc.numStrips > MAX_STRIPS) sc.numStrips = MAX_STRIPS;
  for (uint8_t i = 0; i < MAX_STRIPS; i++) {
    if (sc.strips[i].count > MAX_PER_STRIP) sc.strips[i].count = MAX_PER_STRIP;
    if (!pinAllowed(sc.strips[i].pin)) { sc.strips[i].pin = STRIP_PIN; sc.strips[i].enabled = 0; }
  }
}

void loadPix() {
  memset(pixBuf, 0, sizeof(pixBuf));
  prefs.begin("argb", true);
  if (prefs.getBytesLength("pix") == sizeof(pixBuf))
    prefs.getBytes("pix", pixBuf, sizeof(pixBuf));
  prefs.end();
}

void saveStrips() {
  sc.version = STRIP_CFG_VERSION;
  prefs.begin("argb", false);
  prefs.putBytes("strips", &sc, sizeof(sc));
  prefs.end();
}

void savePix() {
  prefs.begin("argb", false);
  prefs.putBytes("pix", pixBuf, sizeof(pixBuf));
  prefs.end();
}

// ---------------- 板载 LED ----------------
void onboardSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ONBOARD_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ONBOARD_PIN, 0);
#endif
}

void onboardWrite(uint32_t level) {              // level: 0 ~ DUTY_MAX
  uint32_t duty = ONBOARD_ACTIVE_LOW ? (DUTY_MAX - level) : level;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ONBOARD_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void syncOnboard() {
  if (!cfg.onboardSync) { onboardWrite(0); return; }
  uint32_t sum = 0, total = 0;
  for (uint8_t s = 0; s < activeStrips; s++) {
    if (!strips[s]) continue;
    for (uint16_t i = 0; i < stripLen[s]; i++) {
      uint32_t c = strips[s]->getPixelColor(i);
      sum += ((c >> 16) & 0xFF) + ((c >> 8) & 0xFF) + (c & 0xFF);
    }
    total += stripLen[s];
  }
  if (!total) { onboardWrite(0); return; }
  uint64_t maxSum = (uint64_t)total * 3 * 255 * 255;
  onboardWrite((uint64_t)sum * cfg.bright * DUTY_MAX / maxSum);
}

// ---------------- 多灯带构建 ----------------
// 按 sc 里的运行时配置重建各路 NeoPixel；颗数未知也能用，按填写的数量分配
void rebuildStrips() {
  for (uint8_t i = 0; i < MAX_STRIPS; i++) {
    if (strips[i]) { delete strips[i]; strips[i] = nullptr; }
    stripLen[i] = 0;
    stripOfs[i] = 0;
  }
  activeStrips = 0;
  uint16_t ofs = 0, maxLen = 0;
  for (uint8_t i = 0; i < sc.numStrips && i < MAX_STRIPS; i++) {
    StripDesc &d = sc.strips[i];
    if (!d.enabled || d.count == 0 || !pinAllowed(d.pin)) continue;
    uint16_t c = d.count;
    if (c > MAX_PER_STRIP) c = MAX_PER_STRIP;
    if (ofs + c > MAX_TOTAL_PIXELS) break;         // 超出总量预算则忽略后续路
    Adafruit_NeoPixel *np = new Adafruit_NeoPixel(c, d.pin, NEO_GRB + NEO_KHZ800);
    if (!np) break;
    np->begin();
    np->setBrightness(cfg.bright);
    np->clear();
    np->show();
    strips[activeStrips] = np;
    stripLen[activeStrips] = c;
    stripOfs[activeStrips] = sc.merge ? ofs : 0;
    ofs += c;
    if (c > maxLen) maxLen = c;
    activeStrips++;
  }
  logicalLen = sc.merge ? ofs : maxLen;
  if (logicalLen > MAX_TOTAL_PIXELS) logicalLen = MAX_TOTAL_PIXELS;
  curTarget = -1;
  curLen    = logicalLen;

  // 配置变化后把分段 / 像素画夹回新的逻辑长度
  for (uint8_t i = 0; i < cfg.segCount; i++) {
    if (logicalLen == 0) { cfg.segCount = 0; break; }
    if (cfg.segs[i].start >= logicalLen) cfg.segs[i].start = logicalLen - 1;
    if (cfg.segs[i].start + cfg.segs[i].len > logicalLen)
      cfg.segs[i].len = logicalLen - cfg.segs[i].start;
    if (cfg.segs[i].len < 1) cfg.segs[i].len = 1;
  }
  if (cfg.tail > logicalLen) cfg.tail = logicalLen ? logicalLen : 1;
}

// 清空当前渲染目标（跨路或单路）
void blankTarget() {
  if (curTarget >= 0) {
    if (curTarget < (int8_t)activeStrips) strips[curTarget]->clear();
  } else {
    for (uint8_t s = 0; s < activeStrips; s++) strips[s]->clear();
  }
}

void showAll() {
  for (uint8_t s = 0; s < activeStrips; s++) strips[s]->show();
}

// ---------------- 工具 ----------------
void buildGamma() {
  float g = 1.0f + (float)cfg.gamma / 50.0f;     // 0->1.0, 100->3.0
  for (int i = 0; i < 256; i++) {
    float v = powf(i / 255.0f, g) * 255.0f;
    gammaLut[i] = (uint8_t)(v + 0.5f);
  }
}

inline uint8_t lerp8(uint8_t a, uint8_t b, uint8_t t) {
  return (uint8_t)((int)a + ((int)b - (int)a) * (int)t / 255);
}

// 逻辑坐标 -> 实际灯带：独立模式画到当前目标路，拼接模式按 offset 路由
inline void setPx(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t gr = gammaLut[r], gg = gammaLut[g], gb = gammaLut[b];
  if (curTarget >= 0) {
    if (curTarget < (int8_t)activeStrips && i < stripLen[curTarget])
      strips[curTarget]->setPixelColor(i, gr, gg, gb);
    return;
  }
  for (uint8_t s = 0; s < activeStrips; s++) {
    if (i >= stripOfs[s] && i < stripOfs[s] + stripLen[s]) {
      strips[s]->setPixelColor(i - stripOfs[s], gr, gg, gb);
      return;
    }
  }
}

// 方向 / 镜像映射：把逻辑位置 p 映射到段内物理偏移
inline uint16_t mapPos(uint16_t len, uint16_t p) {
  if (cfg.dir) p = len - 1 - p;
  if (cfg.mirror) {
    uint16_t q = len - 1 - p;
    if (q < p) p = q;
  }
  return p;
}

inline uint32_t packRGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

uint32_t wheel(uint8_t pos) {                    // HSV 色环 -> RGB
  pos = 255 - pos;
  if (pos < 85)  return packRGB(255 - pos * 3, 0, pos * 3);
  if (pos < 170) { pos -= 85; return packRGB(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return packRGB(pos * 3, 255 - pos * 3, 0);
}

uint32_t frameIntervalFor(uint8_t spd) { return map(spd, 1, 100, 60, 8); }

// ---------------- 各灯效（按 animMs 画一帧，非阻塞；均带范围） ----------------
void fxFilled(uint16_t start, uint16_t len, uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < len; i++) setPx(start + i, r, g, b);
}

void fxBlank(uint16_t start, uint16_t len) { fxFilled(start, len, 0, 0, 0); }

void fxGradient(uint16_t start, uint16_t len,
                uint8_t r1, uint8_t g1, uint8_t b1,
                uint8_t r2, uint8_t g2, uint8_t b2) {
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    uint8_t t = (len > 1) ? (uint8_t)((uint32_t)q * 255 / (len - 1)) : 0;
    setPx(start + p, lerp8(r1, r2, t), lerp8(g1, g2, t), lerp8(b1, b2, t));
  }
}

void fxBreath(uint16_t start, uint16_t len,
              uint8_t r1, uint8_t g1, uint8_t b1,
              uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  const uint32_t period = 2400;
  float k = (1.0f - cosf((float)(ph % period) / period * 2.0f * PI)) * 0.5f;
  uint8_t t = (uint8_t)(k * 255.0f);
  fxFilled(start, len, lerp8(r1, r2, t), lerp8(g1, g2, t), lerp8(b1, b2, t));
}

void fxRainbow(uint16_t start, uint16_t len, uint32_t ph) {
  uint8_t base = (ph / 12) & 0xFF;
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    uint8_t h = base + (uint8_t)((uint32_t)q * cfg.hueSpan / len);
    uint32_t c = wheel(h);
    setPx(start + p, (c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
  }
}

void fxComet(uint16_t start, uint16_t len,
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  int16_t tail = cfg.tail;
  if (tail < 1) tail = 1;
  if (tail > (int16_t)len) tail = len;
  int16_t span = len + tail;
  int16_t head = (int16_t)((ph / 40) % span);
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    int16_t d = head - (int16_t)q;
    if (d < 0) d = -d;
    if (d < tail) {
      uint8_t t = (uint8_t)((tail - d) * 255 / tail);
      setPx(start + p, lerp8(r2, r1, t), lerp8(g2, g1, t), lerp8(b2, b1, t));
    } else {
      setPx(start + p, 0, 0, 0);
    }
  }
}

void fxFlow(uint16_t start, uint16_t len, uint32_t ph) {
  uint8_t base = (ph / 10) & 0xFF;
  int16_t head = (int16_t)((ph / 60) % len);
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    uint32_t c = wheel(base + (uint8_t)((uint32_t)q * 200 / len));
    uint8_t r = ((c >> 16) & 0xFF) / 6;
    uint8_t g = ((c >> 8) & 0xFF) / 6;
    uint8_t b = (c & 0xFF) / 6;
    if ((int16_t)q == head) { r = (c >> 16) & 0xFF; g = (c >> 8) & 0xFF; b = c & 0xFF; }
    setPx(start + p, r, g, b);
  }
}

void fxSine(uint16_t start, uint16_t len,
            uint8_t r1, uint8_t g1, uint8_t b1,
            uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  const uint32_t period = 2200;
  float base = (float)(ph % period) / period * 2.0f * PI;
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    float s = 0.5f + 0.5f * sinf(base - (float)q * 0.9f);
    uint8_t t = (uint8_t)(s * 255.0f);
    setPx(start + p, lerp8(r1, r2, t), lerp8(g1, g2, t), lerp8(b1, b2, t));
  }
}

void fxHeartbeat(uint16_t start, uint16_t len,
                 uint8_t r1, uint8_t g1, uint8_t b1,
                 uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  const uint32_t period = 1200;
  float t = (float)(ph % period) / period;
  float e = expf(-powf((t - 0.10f) / 0.055f, 2.0f))
          + 0.55f * expf(-powf((t - 0.30f) / 0.070f, 2.0f));
  if (e > 1.0f) e = 1.0f;
  uint8_t k = (uint8_t)(e * 255.0f);
  fxFilled(start, len, lerp8(r1, r2, k), lerp8(g1, g2, k), lerp8(b1, b2, k));
}

void fxAlert(uint16_t start, uint16_t len,
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  bool a = ((ph / 380) % 2) == 0;
  fxFilled(start, len, a ? r1 : r2, a ? g1 : g2, a ? b1 : b2);
}

void fxStarfield(uint16_t start, uint16_t len,
                 uint8_t r1, uint8_t g1, uint8_t b1) {
  uint16_t end = start + len;
  if (end > MAX_TOTAL_PIXELS) end = MAX_TOTAL_PIXELS;
  for (uint16_t i = start; i < end; i++)
    starVal[i] = (starVal[i] > 12) ? (uint8_t)(starVal[i] - 12) : 0;

  uint8_t tries = 1 + cfg.randomAmt / 25;        // 随机强度越高，落星越多
  if (len) {
    for (uint8_t t = 0; t < tries; t++)
      if ((int)random(100) < (int)cfg.randomAmt)
        starVal[start + random(len)] = 255;
  }

  for (uint16_t p = 0; p < len; p++) {
    uint16_t idx = start + p;
    uint8_t v = starVal[idx];
    setPx(idx, lerp8(0, r1, v), lerp8(0, g1, v), lerp8(0, b1, v));
  }
}

void fxBlock(uint16_t start, uint16_t len,
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  int16_t w = 1 + cfg.spacing;
  int16_t period = w + cfg.spacing;
  int16_t off = (int16_t)((ph / 60) % period);
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    int16_t m = ((int16_t)q + off) % period;
    if (m < 0) m += period;
    if (m < w) setPx(start + p, r1, g1, b1);
    else       setPx(start + p, r2, g2, b2);
  }
}

void fxAlternate(uint16_t start, uint16_t len,
                 uint8_t r1, uint8_t g1, uint8_t b1,
                 uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  int16_t step = 1 + cfg.spacing;
  int16_t off = (int16_t)((ph / 80) % (2 * step));
  for (uint16_t p = 0; p < len; p++) {
    uint16_t q = mapPos(len, p);
    int16_t m = (int16_t)q + off;
    if (m < 0) m += 2 * step;
    bool a = ((m / step) & 1) == 0;
    setPx(start + p, a ? r1 : r2, a ? g1 : g2, a ? b1 : b2);
  }
}

void fxCustom(uint16_t start, uint16_t len) {
  for (uint16_t p = 0; p < len; p++) {
    uint16_t i = start + p;                      // 像素画按逻辑位置直出
    if (i >= MAX_TOTAL_PIXELS) break;
    setPx(i, pixBuf[i * 3], pixBuf[i * 3 + 1], pixBuf[i * 3 + 2]);
  }
}

void fxMusic(uint16_t start, uint16_t len,
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  bool live = levelMs && (millis() - levelMs < 500);
  float k;
  if (live) {
    k = musicLevel / 255.0f;
  } else {                                       // PC 断开：回落慢呼吸，避免全黑
    const uint32_t period = 2600;
    k = 0.10f + 0.15f * (1.0f - cosf((float)(ph % period) / period * 2.0f * PI)) * 0.5f;
  }
  uint8_t t = (uint8_t)(k * 255.0f);
  float amp = 0.15f + 0.85f * k;
  fxFilled(start, len,
           (uint8_t)(lerp8(r1, r2, t) * amp),
           (uint8_t)(lerp8(g1, g2, t) * amp),
           (uint8_t)(lerp8(b1, b2, t) * amp));
}

// 从外侧端向内点亮 n 颗：外侧最亮、越靠中间越暗
void barFromOuter(uint16_t start, uint16_t len, bool fromLeft, uint16_t n,
                  uint8_t r, uint8_t g, uint8_t b) {
  if (n > len) n = len;
  for (uint16_t p = 0; p < n; p++) {
    uint8_t fade = (uint8_t)(255 - (uint32_t)p * 150 / len);
    uint16_t idx = fromLeft ? (start + p) : (start + len - 1 - p);
    setPx(idx, (uint8_t)((uint32_t)r * fade / 255),
              (uint8_t)((uint32_t)g * fade / 255),
              (uint8_t)((uint32_t)b * fade / 255));
  }
}

// 立体声律动：左半屏由左声道从外侧向中间点亮，右半屏由右声道同理；
// 无声音时回落到用户设定的「无声灯效」idleMode
void fxStereo(uint16_t start, uint16_t len,
              uint8_t r1, uint8_t g1, uint8_t b1,
              uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  bool live = levelMs && (millis() - levelMs < 500);
  uint8_t lvL = live ? musicLevelL : 0;
  uint8_t lvR = live ? musicLevelR : 0;

  if (lvL < STEREO_SILENT && lvR < STEREO_SILENT) {   // 静音 -> 用户默认灯效
    uint8_t m = cfg.idleMode;
    if (m >= E_COUNT || m == E_STEREO) m = E_BREATH;
    renderRange(start, len, m, r1, g1, b1, r2, g2, b2, ph);
    return;
  }

  fxBlank(start, len);
  uint16_t half = len / 2;
  if (half == 0) half = len;
  bool has2 = (r2 | g2 | b2) != 0;                   // 未设终色时两侧同色
  barFromOuter(start, half, true,
               (uint16_t)((uint32_t)lvL * half * 2 / 255), r1, g1, b1);
  barFromOuter(start + len - half, half, false,
               (uint16_t)((uint32_t)lvR * half * 2 / 255),
               has2 ? r2 : r1, has2 ? g2 : g1, has2 ? b2 : b1);
}

// ---------------- 渲染调度 ----------------
void renderRange(uint16_t start, uint16_t len, uint8_t mode,
                 uint8_t r1, uint8_t g1, uint8_t b1,
                 uint8_t r2, uint8_t g2, uint8_t b2, uint32_t ph) {
  if (len == 0 || start >= curLen) return;
  if (start + len > curLen) len = curLen - start;
  switch (mode) {
    case E_OFF:       fxBlank(start, len); break;
    case E_STATIC:    fxFilled(start, len, r1, g1, b1); break;
    case E_BREATH:    fxBreath(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_RAINBOW:   fxRainbow(start, len, ph); break;
    case E_COMET:     fxComet(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_FLOW:      fxFlow(start, len, ph); break;
    case E_GRADIENT:  fxGradient(start, len, r1, g1, b1, r2, g2, b2); break;
    case E_SINE:      fxSine(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_HEARTBEAT: fxHeartbeat(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_ALERT:     fxAlert(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_STARFIELD: fxStarfield(start, len, r1, g1, b1); break;
    case E_BLOCK:     fxBlock(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_ALTERNATE: fxAlternate(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_CUSTOM:    fxCustom(start, len); break;
    case E_MUSIC:     fxMusic(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    case E_STEREO:    fxStereo(start, len, r1, g1, b1, r2, g2, b2, ph); break;
    default:          fxBlank(start, len); break;
  }
}

// 把同一灯效铺到全部逻辑灯珠：拼接模式一次跨路渲染，独立模式逐路各自渲染
void renderEffect(uint8_t mode, uint8_t r1, uint8_t g1, uint8_t b1,
                  uint8_t r2, uint8_t g2, uint8_t b2) {
  if (!activeStrips) return;
  if (sc.merge) {
    curTarget = -1; curLen = logicalLen;
    renderRange(0, logicalLen, mode, r1, g1, b1, r2, g2, b2, animMs);
  } else {
    for (uint8_t s = 0; s < activeStrips; s++) {
      curTarget = (int8_t)s; curLen = stripLen[s];
      renderRange(0, stripLen[s], mode, r1, g1, b1, r2, g2, b2, animMs);
    }
    curTarget = -1; curLen = logicalLen;
  }
}

void renderSegments() {
  if (!activeStrips) return;
  if (sc.merge) {
    curTarget = -1; curLen = logicalLen;
    blankTarget();
    for (uint8_t s = 0; s < cfg.segCount; s++) {
      Segment &sg = cfg.segs[s];
      renderRange(sg.start, sg.len, sg.mode, sg.r, sg.g, sg.b, 0, 0, 0, animMs);
    }
  } else {
    for (uint8_t k = 0; k < activeStrips; k++) {
      curTarget = (int8_t)k; curLen = stripLen[k];
      blankTarget();
      for (uint8_t s = 0; s < cfg.segCount; s++) {
        Segment &sg = cfg.segs[s];
        renderRange(sg.start, sg.len, sg.mode, sg.r, sg.g, sg.b, 0, 0, 0, animMs);
      }
    }
    curTarget = -1; curLen = logicalLen;
  }
}

void render() {
  if (cfg.playing && cfg.stepCount > 0) {        // 1) 编排优先
    Step &st = cfg.steps[curStep];
    renderEffect(st.mode, st.r1, st.g1, st.b1, st.r2, st.g2, st.b2);
  } else if (cfg.segCount > 0 && logicalLen > 0) {   // 2) 分段
    renderSegments();
  } else {                                       // 3) 全局
    renderEffect(cfg.mode, cfg.r1, cfg.g1, cfg.b1, cfg.r2, cfg.g2, cfg.b2);
  }
  showAll();
  syncOnboard();
}

// ---------------- 状态回包 ----------------
void sendState() {
  Serial.printf("STATE %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                (int)cfg.mode, cfg.r1, cfg.g1, cfg.b1,
                (int)cfg.bright, (int)cfg.speed, cfg.onboardSync ? 1 : 0,
                cfg.r2, cfg.g2, cfg.b2, cfg.hueSpan,
                cfg.dir ? 1 : 0, cfg.tail, cfg.spacing, cfg.mirror ? 1 : 0,
                cfg.randomAmt, cfg.gamma, cfg.musicGain,
                cfg.segCount, cfg.stepCount, cfg.playing ? 1 : 0,
                (int)cfg.idleMode);
  for (uint8_t s = 0; s < cfg.segCount; s++) {
    Segment &g = cfg.segs[s];
    Serial.printf("SEG %d %d %d %d %d %d %d\n",
                  s, g.start, g.len, g.mode, g.r, g.g, g.b);
  }
  for (uint8_t s = 0; s < cfg.stepCount; s++) {
    Step &t = cfg.steps[s];
    Serial.printf("STEP %d %d %d %d %d %d %d %d %d\n",
                  s, t.mode, t.secs, t.r1, t.g1, t.b1, t.r2, t.g2, t.b2);
  }
  // 灯带描述：路数 / 拼接模式 / 逻辑总长 / 实际生效路数
  Serial.printf("STRIPS %d %d %d %d\n",
                (int)sc.numStrips, sc.merge ? 1 : 0, (int)logicalLen, (int)activeStrips);
  for (uint8_t i = 0; i < sc.numStrips; i++)
    Serial.printf("STRIP %d %d %d %d\n",
                  i, sc.strips[i].pin, sc.strips[i].count, sc.strips[i].enabled ? 1 : 0);
  for (uint16_t i = 0; i < logicalLen && i < MAX_TOTAL_PIXELS; i++)
    Serial.printf("PIX %d %d %d %d\n", i,
                  pixBuf[i * 3], pixBuf[i * 3 + 1], pixBuf[i * 3 + 2]);
  Serial.println("END");
}

// ---------------- 编排 ----------------
void nextStep() {
  if (cfg.stepCount == 0) return;
  curStep = (curStep + 1) % cfg.stepCount;
  stepStart = millis();
  animMs = 0;
}

// ---------------- 指令解析 ----------------
void applyCmd(String s) {
  s.trim();
  if (!s.length()) return;
  String up = s; up.toUpperCase();
  int i1, i2, i3, i4, i5, i6, i7, i8, i9;

  if (up == "PING") { Serial.println("PONG"); return; }
  if (up == "GET")  { sendState(); return; }
  if (up == "OFF")  { cfg.mode = E_OFF; cfg.playing = 0; markDirty();
                      Serial.println("OK OFF"); return; }

  // ---- 灯带配置（运行时改引脚 / 颗数 / 拼接，立即重建并写 NVS）----
  if (up == "STRIPCLR") {
    setStripDefaults();
    saveStrips();
    rebuildStrips();
    Serial.println("OK STRIPCLR");
    return;
  }
  if (sscanf(up.c_str(), "STRIPSET %d %d %d %d", &i1, &i2, &i3, &i4) == 4) {
    if (i1 >= 0 && i1 < MAX_STRIPS && pinAllowed((uint8_t)i2) &&
        i3 >= 0 && i3 <= MAX_PER_STRIP) {
      sc.strips[i1].pin     = (uint8_t)i2;
      sc.strips[i1].count   = (uint16_t)i3;
      sc.strips[i1].enabled = (i4 != 0) ? 1 : 0;
      saveStrips();
      rebuildStrips();
      Serial.printf("OK STRIPSET %d %d %d %d\n", i1, sc.strips[i1].pin,
                    sc.strips[i1].count, sc.strips[i1].enabled);
    } else Serial.println("ERR STRIPSET");
    return;
  }
  if (sscanf(up.c_str(), "STRIPS %d %d", &i1, &i2) == 2) {
    if (i1 >= 1 && i1 <= MAX_STRIPS && (i2 == 0 || i2 == 1)) {
      sc.numStrips = (uint8_t)i1;
      sc.merge     = (uint8_t)i2;
      saveStrips();
      rebuildStrips();
      Serial.printf("OK STRIPS %d %d\n", sc.numStrips, sc.merge);
    } else Serial.println("ERR STRIPS");
    return;
  }
  if (sscanf(up.c_str(), "MERGE %d", &i1) == 1) {
    sc.merge = (i1 != 0) ? 1 : 0;
    saveStrips();
    rebuildStrips();
    Serial.printf("OK MERGE %d\n", sc.merge);
    return;
  }

  // ---- 编排 ----
  if (up == "PLAYNEXT") {
    if (cfg.stepCount > 0) { cfg.playing = 1; nextStep(); markDirty();
                             Serial.println("OK PLAYNEXT"); }
    else Serial.println("ERR PLAYNEXT");
    return;
  }
  if (sscanf(up.c_str(), "PLAY %d", &i1) == 1) {
    cfg.playing = (i1 != 0) && (cfg.stepCount > 0);
    if (cfg.playing) { curStep = 0; stepStart = millis(); animMs = 0; }
    markDirty();
    Serial.printf("OK PLAY %d\n", cfg.playing ? 1 : 0);
    return;
  }
  if (up == "STEPCLR") { cfg.stepCount = 0; cfg.playing = 0; markDirty();
                         Serial.println("OK STEPCLR"); return; }
  if (sscanf(up.c_str(), "STEP %d %d %d %d %d %d %d %d %d",
             &i1, &i2, &i3, &i4, &i5, &i6, &i7, &i8, &i9) == 9) {
    if (i1 >= 0 && i1 < MAX_STEP) {
      Step &st = cfg.steps[i1];
      st.mode = constrain(i2, 0, E_COUNT - 1);
      st.secs = constrain(i3, 1, 3600);
      st.r1 = constrain(i4, 0, 255); st.g1 = constrain(i5, 0, 255); st.b1 = constrain(i6, 0, 255);
      st.r2 = constrain(i7, 0, 255); st.g2 = constrain(i8, 0, 255); st.b2 = constrain(i9, 0, 255);
      st.speed = cfg.speed;
      if (cfg.stepCount <= i1) cfg.stepCount = i1 + 1;
      markDirty();
      Serial.printf("OK STEP %d\n", i1);
    } else Serial.println("ERR STEP");
    return;
  }

  // ---- 分段 ----
  if (up == "SEGCLR") { cfg.segCount = 0; markDirty();
                        Serial.println("OK SEGCLR"); return; }
  if (sscanf(up.c_str(), "SEG %d %d %d %d %d %d %d",
             &i1, &i2, &i3, &i4, &i5, &i6, &i7) == 7) {
    if (i1 >= 0 && i1 < MAX_SEG && logicalLen > 0) {
      Segment &sg = cfg.segs[i1];
      sg.start = constrain(i2, 0, logicalLen - 1);
      sg.len   = constrain(i3, 1, logicalLen - sg.start);
      sg.mode  = constrain(i4, 0, E_COUNT - 1);
      sg.r = constrain(i5, 0, 255); sg.g = constrain(i6, 0, 255); sg.b = constrain(i7, 0, 255);
      if (cfg.segCount <= i1) cfg.segCount = i1 + 1;
      cfg.playing = 0;                          // 编辑分段即退出编排
      markDirty();
      Serial.printf("OK SEG %d\n", i1);
    } else Serial.println("ERR SEG");
    return;
  }

  // ---- 像素画 ----
  if (up == "PIXCLR") { memset(pixBuf, 0, sizeof(pixBuf)); cfg.playing = 0;
                        markPixDirty();
                        Serial.println("OK PIXCLR"); return; }
  // 分块写入：PIXB <起始灯珠> <r g b> <r g b> ...（每 3 个整数组一颗）
  if (up.startsWith("PIXB ")) {
    const char *p = up.c_str() + 5;
    char *endp;
    long st = strtol(p, &endp, 10);
    if (endp == p) { Serial.println("ERR PIXB"); return; }
    p = endp;
    uint16_t i = (uint16_t)constrain((int)st, 0, (int)MAX_TOTAL_PIXELS);
    uint8_t comp = 0;
    uint16_t written = 0;
    while (i < MAX_TOTAL_PIXELS) {
      long v = strtol(p, &endp, 10);
      if (endp == p) break;
      p = endp;
      pixBuf[i * 3 + comp] = (uint8_t)constrain((int)v, 0, 255);
      if (++comp == 3) { comp = 0; i++; written++; }
    }
    cfg.playing = 0;
    markPixDirty();
    Serial.printf("OK PIXB %d %d\n", (int)st, written);
    return;
  }
  if (sscanf(up.c_str(), "PIX %d %d %d %d", &i1, &i2, &i3, &i4) == 4) {
    if (i1 >= 0 && i1 < MAX_TOTAL_PIXELS && i1 < logicalLen) {
      pixBuf[i1 * 3]     = constrain(i2, 0, 255);
      pixBuf[i1 * 3 + 1] = constrain(i3, 0, 255);
      pixBuf[i1 * 3 + 2] = constrain(i4, 0, 255);
      cfg.playing = 0;
      markPixDirty();
      Serial.printf("OK PIX %d\n", i1);
    } else Serial.println("ERR PIX");
    return;
  }

  // ---- 音频（高频，不写 NVS） ----
  if (sscanf(up.c_str(), "LEVEL %d %d %d", &i1, &i2, &i3) == 3) {
    int g = 100 + cfg.musicGain;
    musicLevel  = (uint8_t)constrain((i1 + i2) / 2 * g / 100, 0, 255);
    musicLevelL = (uint8_t)constrain(i2 * g / 100, 0, 255);
    musicLevelR = (uint8_t)constrain(i3 * g / 100, 0, 255);
    levelMs = millis();
    return;
  }
  if (sscanf(up.c_str(), "LEVEL %d", &i1) == 1) {
    musicLevel = musicLevelL = musicLevelR =
        (uint8_t)constrain(i1 * (100 + cfg.musicGain) / 100, 0, 255);
    levelMs = millis();
    return;
  }

  // ---- 颜色 ----
  if (up.startsWith("COLOR2") &&
      sscanf(up.c_str(), "COLOR2 %d %d %d", &i1, &i2, &i3) == 3) {
    cfg.r2 = constrain(i1, 0, 255); cfg.g2 = constrain(i2, 0, 255); cfg.b2 = constrain(i3, 0, 255);
    markDirty();
    Serial.printf("OK COLOR2 %d %d %d\n", cfg.r2, cfg.g2, cfg.b2);
    return;
  }
  if (sscanf(up.c_str(), "COLOR %d %d %d", &i1, &i2, &i3) == 3) {
    cfg.r1 = constrain(i1, 0, 255); cfg.g1 = constrain(i2, 0, 255); cfg.b1 = constrain(i3, 0, 255);
    if (cfg.mode == E_OFF) cfg.mode = E_STATIC;
    markDirty();
    Serial.printf("OK COLOR %d %d %d\n", cfg.r1, cfg.g1, cfg.b1);
    return;
  }

  // ---- 模式 ----
  if (sscanf(up.c_str(), "MODE %d", &i1) == 1) {
    if (i1 >= 0 && i1 < E_COUNT) {
      cfg.mode = (Effect)i1;
      cfg.playing = 0;                          // 选灯效即退出编排；分段仍优先
      markDirty();
      Serial.printf("OK MODE %d\n", i1);
    } else Serial.println("ERR MODE");
    return;
  }

  // ---- 立体声律动无声时回落的灯效 ----
  if (sscanf(up.c_str(), "IDLE %d", &i1) == 1) {
    if (i1 >= 0 && i1 < E_COUNT && i1 != E_STEREO) {
      cfg.idleMode = (uint8_t)i1;
      markDirty();
      Serial.printf("OK IDLE %d\n", i1);
    } else Serial.println("ERR IDLE");
    return;
  }

  // ---- 参数 ----
  if (sscanf(up.c_str(), "HUE %d", &i1) == 1) {
    cfg.hueSpan = constrain(i1, 0, 255); markDirty();
    Serial.printf("OK HUE %d\n", cfg.hueSpan); return;
  }
  if (sscanf(up.c_str(), "DIR %d", &i1) == 1) {
    cfg.dir = (i1 != 0); markDirty();
    Serial.printf("OK DIR %d\n", cfg.dir ? 1 : 0); return;
  }
  if (sscanf(up.c_str(), "TAIL %d", &i1) == 1) {
    int hi = logicalLen ? (int)logicalLen : 1;
    cfg.tail = constrain(i1, 1, hi); markDirty();
    Serial.printf("OK TAIL %d\n", cfg.tail); return;
  }
  if (sscanf(up.c_str(), "SPACING %d", &i1) == 1) {
    cfg.spacing = constrain(i1, 0, 6); markDirty();
    Serial.printf("OK SPACING %d\n", cfg.spacing); return;
  }
  if (sscanf(up.c_str(), "MIRROR %d", &i1) == 1) {
    cfg.mirror = (i1 != 0); markDirty();
    Serial.printf("OK MIRROR %d\n", cfg.mirror ? 1 : 0); return;
  }
  if (sscanf(up.c_str(), "RANDOM %d", &i1) == 1) {
    cfg.randomAmt = constrain(i1, 0, 100); markDirty();
    Serial.printf("OK RANDOM %d\n", cfg.randomAmt); return;
  }
  if (sscanf(up.c_str(), "GAMMA %d", &i1) == 1) {
    cfg.gamma = constrain(i1, 0, 100); buildGamma(); markDirty();
    Serial.printf("OK GAMMA %d\n", cfg.gamma); return;
  }
  if (sscanf(up.c_str(), "GAIN %d", &i1) == 1) {
    cfg.musicGain = constrain(i1, 0, 100); markDirty();
    Serial.printf("OK GAIN %d\n", cfg.musicGain); return;
  }
  if (sscanf(up.c_str(), "BRIGHT %d", &i1) == 1) {
    cfg.bright = constrain(i1, 0, 255);
    for (uint8_t s = 0; s < activeStrips; s++)
      if (strips[s]) strips[s]->setBrightness(cfg.bright);
    markDirty();
    Serial.printf("OK BRIGHT %d\n", cfg.bright); return;
  }
  if (sscanf(up.c_str(), "SPEED %d", &i1) == 1) {
    cfg.speed = constrain(i1, 1, 100); markDirty();
    Serial.printf("OK SPEED %d\n", cfg.speed); return;
  }
  if (sscanf(up.c_str(), "SYNC %d", &i1) == 1) {
    cfg.onboardSync = (i1 != 0); markDirty();
    Serial.printf("OK SYNC %d\n", cfg.onboardSync ? 1 : 0); return;
  }

  Serial.printf("ERR %s\n", s.c_str());
}

// ---------------- 主流程 ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(micros());

  loadSettings();                   // 恢复上次 PC 断开前保存的设置
  loadStrips();                     // 灯带硬件配置（引脚 / 颗数 / 拼接）
  loadPix();                        // 像素画
  buildGamma();

  rebuildStrips();                  // 按运行时配置创建各路灯带
  onboardSetup();
  onboardWrite(0);

  lastTick = lastFrame = millis();
  Serial.println("READY ESP32C3-ARGB3");
  sendState();                      // 开机回一次完整状态
}

void loop() {
  // 收指令
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length()) { applyCmd(line); line = ""; }
    } else if (line.length() < 1024) {   // PIXB 分块帧较长，留足缓冲
      line += ch;
    }
  }

  // 改动延后写入 NVS，避免频繁擦写闪存
  if (dirty && millis() - dirtyMs > 1500) { saveSettings(); dirty = false; }
  if (pixDirty && millis() - pixDirtyMs > 1500) { savePix(); pixDirty = false; }

  // 编排推进
  if (cfg.playing && cfg.stepCount > 0) {
    uint32_t dur = (uint32_t)cfg.steps[curStep].secs * 1000UL;
    if (millis() - stepStart >= dur) nextStep();
  }

  // 动画渲染（非阻塞，按速度快慢决定帧间隔）
  uint32_t now = millis();
  uint8_t spd = (cfg.playing && cfg.stepCount > 0) ? cfg.steps[curStep].speed : cfg.speed;
  if (spd < 1) spd = 1;
  if (now - lastFrame >= frameIntervalFor(spd)) {
    lastFrame = now;
    uint32_t dt = now - lastTick;
    lastTick = now;
    animMs += (uint32_t)((uint64_t)dt * spd / 50);   // 速度 50 = 1x
    render();
  }
}
