/*
  M5Core + GNSS 北極星ファインダー（タスク分離版）
  - M5Unified 使用（LCD/IMU）
  - 起動時：IMUキャリブ（磁気 min/max + 3軸回転カバレッジ）
  - タスク構成：
      * IMU Task : IMU取得＆方位/仰角計算 → 共有バッファへ
      * GNSS Task: UART2からNMEA受信 → USB(Serial)へRAWパススルー → TinyGPS++で解析 → 共有バッファへ
      * loop()   : ボタン処理・UI描画のみ
  - 画面ON/OFF：BtnA でバックライトトグル
  - UI：
      上：緯度・経度
      中：コンパスリング＋方位角（0..360°、北=0°）
      下：仰角ゲージ（0..90°）、ターゲット=|緯度|
  - LOCK：|Az|<=5° かつ |Elev-Target|<=3°
*/

#include <M5Unified.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <MadgwickAHRS.h>

// ========= ユーザー設定 =========

static const double DEBUG_LAT = 35.6812; // 東京駅近辺
static const double DEBUG_LON = 139.7671;

static cinst double DECLINATION = 8.0;


static const int   GNSS_BAUD = 38400;
static const int   GNSS_RX   = 16;  // UART2 RX
static const int   GNSS_TX   = 17;  // UART2 TX

// ---- キャリブ系パラメータ ----
static const float MOTION_THRESH_DEG_S  = 60.0f;   // これ未満ならほぼ静止
static const float DOMINANT_RATIO       = 0.75f;   // 角速度ベクトルの75%以上を占める軸を採用
static const int   ROT_TARGET_PER_AXIS  = 4;       // X/Y/Z 各軸ごとに 360deg 回転を 4回検出
static const uint32_t CALIB_MAX_MS      = 60000;   // キャリブ最大時間（安全タイムアウト）

static const float    HEADING_LOCK_TOL_DEG = 5.0f;
static const float    ELEV_LOCK_TOL_DEG    = 3.0f;
// =================================

M5Canvas canvas(&M5.Display); 

// ====== 共有データ構造（IMU/GNSS→描画） ======
struct SharedNavData {
  // GNSS
  bool   fixOK   = false;
  double lat     = DEBUG_LAT;
  double lon     = DEBUG_LON;
  double alt_m   = 0.0;
  double hdop    = 0.0;
  int    sats    = 0;

  // IMU派生
  float  heading_deg = 0.0f;   // 0..360（北=0）
  float  elev_deg    = 0.0f;   // 0..90

  // タイムスタンプ（任意）
  uint32_t imu_ms  = 0;
  uint32_t gnss_ms = 0;
};

// 共有バッファと保護
SharedNavData g_nav;
SemaphoreHandle_t g_navMutex;

// ====== グローバル ======
TinyGPSPlus gps;
bool displayOn = true;

// 磁気キャリブ結果
struct MagCalib {
  float offset[3] = {0,0,0};
  float scale[3]  = {1,1,1};
} magCal;

bool g_useCanvas = false;
Madgwick filter;   // Madgwick姿勢フィルタ

// 角度正規化 helpers
static inline float wrap360(float deg){
  while (deg <   0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static inline float wrap180(float deg){
  while (deg >  180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

// ====== 1次元カルマンフィルタ（方位角用） ======
struct HeadingKalman {
  float x;        // 推定方位角 [deg]
  float p;        // 誤差共分散
  float q;        // プロセスノイズ (大きいほど追従性↑ ノイズ↑)
  float r;        // 観測ノイズ (大きいほどスムージング強め)
  bool  initialized;

  HeadingKalman()
  : x(0.0f), p(0.0f),
    q(0.5f),   // ★追従性：0.1〜1.0くらいで調整
    r(4.0f),   // ★観測ノイズ：2〜10くらいで調整
    initialized(false) {}

  void init(float angle_deg) {
    x = wrap360(angle_deg);
    p = 50.0f;      // 初期不確かさ大きめ
    initialized = true;
  }

  float update(float z_deg) {
    float z = wrap360(z_deg);
    if (!initialized) {
      init(z);
      return x;
    }

    // 予測ステップ：x_k = x_{k-1}, p_k = p_{k-1} + q
    p += q;

    // 観測との偏差（±180内に収める）
    float y = wrap180(z - x);

    // カルマンゲイン
    float S = p + r;
    float K = (S != 0.0f) ? (p / S) : 0.0f;

    // 更新
    x = wrap360(x + K * y);
    p = (1.0f - K) * p;

    return x;
  }
};

HeadingKalman g_headingKF;

// ================== UI ヘルパ ==================
void drawProgress(float ratio, const char* msg) {
  auto& lcd = M5.Display;
  int w = lcd.width();
  int h = 16;
  int x = 10;
  int y = lcd.height()/2 + 30;
  int bw = w - 20;
  lcd.fillRect(x, y, bw, h, DARKGREY);
  int pw = (int)(bw * fmaxf(0.0f, fminf(1.0f, ratio)));
  lcd.fillRect(x, y, pw, h, GREEN);
  lcd.setCursor(10, y - 20);
  lcd.printf("%s", msg);
}

void drawLatLon(M5Canvas &gfx, double lat, double lon, bool fixOK) {
  gfx.setTextDatum(TL_DATUM);
  gfx.setTextSize(2);
  gfx.setTextColor(fixOK ? TFT_WHITE : TFT_SILVER, TFT_BLACK);
  gfx.setCursor(8, 6);
  if (fixOK) gfx.printf("Lat: %.6f  Lon: %.6f", lat, lon);
  else       gfx.printf("GNSS fix... (Lat/Lon pending)");
}

void drawCompass(M5Canvas &gfx, float heading_deg) {
  int cx = gfx.width()/2;
  int cy = gfx.height()/2 - 10;
  int r  = min(gfx.width(), gfx.height())/2 - 20;

  gfx.drawCircle(cx, cy, r, TFT_DARKGREY);
  // N/E/S/W 目盛（Nは赤）
  gfx.drawLine(cx, cy-r, cx, cy-(r-10), TFT_RED);
  gfx.drawLine(cx, cy+r, cx, cy+(r-10), TFT_DARKGREY);
  gfx.drawLine(cx-r, cy, cx-(r-10), cy, TFT_DARKGREY);
  gfx.drawLine(cx+r, cy, cx+(r-10), cy, TFT_DARKGREY);
  // 0°ターゲット
  gfx.drawTriangle(cx-5, cy-(r-14), cx+5, cy-(r-14), cx, cy-(r-2), TFT_RED);

  // 現在方位針
  float rad = heading_deg * M_PI / 180.0f;
  int x2 = cx + (int)((r-6) * sinf(rad));
  int y2 = cy - (int)((r-6) * cosf(rad));
  gfx.drawLine(cx, cy, x2, y2, TFT_WHITE);
  gfx.fillCircle(x2, y2, 3, TFT_WHITE);

  // 数値
  gfx.setTextDatum(MC_DATUM);
  gfx.setTextSize(2);
  gfx.setTextColor(TFT_WHITE, TFT_BLACK);
  gfx.drawString(String("Heading ") + String((int)roundf(heading_deg)) + " deg", cx, cy + r + 8);
}

void drawElevationGauge(M5Canvas &gfx, float elev_deg, float target_deg) {
  int yBase = gfx.height() - 40;
  int x0 = 20, x1 = gfx.width() - 20;
  int w = x1 - x0;

  gfx.drawLine(x0, yBase, x1, yBase, TFT_DARKGREY);
  gfx.drawLine(x0, yBase-5, x0, yBase+5, TFT_DARKGREY);
  gfx.drawLine(x0 + w/2, yBase-5, x0 + w/2, yBase+5, TFT_DARKGREY);
  gfx.drawLine(x1, yBase-5, x1, yBase+5, TFT_DARKGREY);

  gfx.setTextDatum(BR_DATUM);
  gfx.setTextColor(TFT_DARKGREY, TFT_BLACK);
  gfx.setTextSize(1);
  gfx.drawString("0", x0-2, yBase-7);
  gfx.setTextDatum(BC_DATUM);
  gfx.drawString("45", x0 + w/2, yBase-7);
  gfx.setTextDatum(BL_DATUM);
  gfx.drawString("90", x1+2, yBase-7);

  int xt = x0 + (int)roundf(fmaxf(0.0f, fminf(90.0f, target_deg)) / 90.0f * w);
  gfx.drawLine(xt, yBase-12, xt, yBase+12, TFT_YELLOW);

  int xc = x0 + (int)roundf(fmaxf(0.0f, fminf(90.0f, elev_deg)) / 90.0f * w);
  gfx.fillTriangle(xc, yBase-14, xc-6, yBase-2, xc+6, yBase-2, TFT_WHITE);

  gfx.setTextDatum(TC_DATUM);
  gfx.setTextColor(TFT_WHITE, TFT_BLACK);
  gfx.setTextSize(2);
  String line = "Elev " + String(elev_deg, 1) + " deg  Target " + String(target_deg, 1);
  gfx.drawString(line, gfx.width()/2, yBase - 22);
}

// ================== Z軸影響を減らした磁気ベクトル計算 ==================
void getCalibratedMagWithTilt(
    float ax, float ay, float az,          // accel (g)
    float mx_raw, float my_raw, float mz_raw,  // mag raw (uT)
    float &mx_cal, float &my_cal, float &mz_cal, // 補正後
    float &mx_h, float &my_h, float &mz_h,      // 水平成分
    float &nx, float &ny, float &nz             // 正規化水平ベクトル
) {
  // 1) ハード/ソフトアイアン補正
  mx_cal = (mx_raw - magCal.offset[0]) * magCal.scale[0];
  my_cal = (my_raw - magCal.offset[1]) * magCal.scale[1];
  mz_cal = (mz_raw - magCal.offset[2]) * magCal.scale[2];

  // 2) 重力方向の単位ベクトル
  float gNorm = sqrtf(ax*ax + ay*ay + az*az);
  float gx_hat = 0.0f, gy_hat = 0.0f, gz_hat = 0.0f;
  if (gNorm > 1e-3f) {
    gx_hat = ax / gNorm;
    gy_hat = ay / gNorm;
    gz_hat = az / gNorm;
  }

  // 3) 縦成分 m_vert = (m·g_hat)*g_hat
  float dot = mx_cal*gx_hat + my_cal*gy_hat + mz_cal*gz_hat;
  float mx_vert = dot * gx_hat;
  float my_vert = dot * gy_hat;
  float mz_vert = dot * gz_hat;

  // 水平成分
  mx_h = mx_cal - mx_vert;
  my_h = my_cal - my_vert;
  mz_h = mz_cal - mz_vert;  // 理論上ほぼ0

  // 4) 水平成分を正規化
  float hNorm = sqrtf(mx_h*mx_h + my_h*my_h + mz_h*mz_h);
  if (hNorm < 1e-6f) {
    nx = ny = nz = 0.0f;
  } else {
    nx = mx_h / hNorm;
    ny = my_h / hNorm;
    nz = mz_h / hNorm;
  }
}

// 水平磁場ベクトルから方位角（北=0°, 時計回り）を算出
float headingFromHorizMag(float nx, float ny) {
  // 北方向を y 軸正方向とし、東方向を x 軸正方向とした場合の例：
  // float heading = atan2f(nx, ny) * 180.0f / M_PI;  // atan2(x, y)
  float heading = atan2f(ny, nx) * 180.0f / M_PI;  // atan2(x, y)
  if (heading < 0.0f)  heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

float computeElevationDeg(float ax,float ay,float az){
  float elev = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI; // x前方仮定
  if (elev < 0) elev = 0;
  if (elev > 90) elev = 90;
  return elev;
}

// ================== キャリブ ==================
void calibrateIMU() {
  auto& imu = M5.Imu;
  auto& lcd = M5.Display;

  lcd.clear();
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(10, 20);
  lcd.println("IMU/MAG Calibration");
  lcd.setTextSize(1);
  lcd.setCursor(10, 50);
  lcd.println("- X/Y/Z 各軸まわりに 360deg 回転を");
  lcd.setCursor(10, 64);
  lcd.println("  それぞれ4回程度行ってください。");
  lcd.setCursor(10, 88);
  lcd.println("  X/Y/Z の表示が 4/4 になれば終了。");

  float mx, my, mz;
  float minv[3] = { 1e9, 1e9, 1e9};
  float maxv[3] = {-1e9,-1e9,-1e9};

  float angleAccum[3] = {0.0f, 0.0f, 0.0f};  // 各軸の累積角度 [deg]
  int   rotCount[3]   = {0, 0, 0};           // 各軸で360degを何回回ったか

  uint32_t t0    = millis();
  uint32_t lastT = t0;

  while (true) {
    uint32_t now = millis();
    float dt = (now - lastT) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastT = now;

    uint32_t elapsed = now - t0;
    if (elapsed > CALIB_MAX_MS) {
      // タイムアウト（安全装置）
      break;
    }

    // センサ取得
    float ax, ay, az;
    float gx, gy, gz;
    imu.getAccel(&ax, &ay, &az);
    imu.getGyro(&gx, &gy, &gz);   // [deg/s]
    imu.getMag(&mx, &my, &mz);    // [uT]

    // 磁気 min/max 更新
    if (!isnan(mx) && !isnan(my) && !isnan(mz)) {
      if (mx < minv[0]) minv[0] = mx;
      if (mx > maxv[0]) maxv[0] = mx;
      if (my < minv[1]) minv[1] = my;
      if (my > maxv[1]) maxv[1] = my;
      if (mz < minv[2]) minv[2] = mz;
      if (mz > maxv[2]) maxv[2] = mz;
    }

    // 角速度の大きさ
    float gxAbs = fabsf(gx);
    float gyAbs = fabsf(gy);
    float gzAbs = fabsf(gz);
    float gNorm = sqrtf(gx*gx + gy*gy + gz*gz);

    if (gNorm >= MOTION_THRESH_DEG_S) {
      // 支配的な軸を決定
      int   axis  = 0;
      float gAxis = gxAbs;
      if (gyAbs > gAxis) { gAxis = gyAbs; axis = 1; }
      if (gzAbs > gAxis) { gAxis = gzAbs; axis = 2; }

      // その軸が全体の DOMINANT_RATIO 以上なら、その軸の回転として扱う
      if (gAxis >= DOMINANT_RATIO * gNorm) {
        float dtheta = gAxis * dt;  // 角度差 [deg]
        angleAccum[axis] += dtheta;

        // 360度を越えるたびに1カウント
        while (angleAccum[axis] >= 360.0f) {
          angleAccum[axis] -= 360.0f;
          rotCount[axis]++;
        }
      }
    }

    // 全軸とも目標回数に達したら終了
    bool done =
      (rotCount[0] >= ROT_TARGET_PER_AXIS) &&
      (rotCount[1] >= ROT_TARGET_PER_AXIS) &&
      (rotCount[2] >= ROT_TARGET_PER_AXIS);

    // 進捗表示
    float ratio =
      (rotCount[0] + rotCount[1] + rotCount[2]) /
      (float)(ROT_TARGET_PER_AXIS * 3);
    if (ratio > 1.0f) ratio = 1.0f;
    drawProgress(ratio, "Rotate around X/Y/Z...");

    // カウンタ表示
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(10, 120);
    lcd.printf("X: %d / %d", rotCount[0], ROT_TARGET_PER_AXIS);
    lcd.setCursor(10, 144);
    lcd.printf("Y: %d / %d", rotCount[1], ROT_TARGET_PER_AXIS);
    lcd.setCursor(10, 168);
    lcd.printf("Z: %d / %d", rotCount[2], ROT_TARGET_PER_AXIS);

    lcd.setTextSize(1);
    lcd.setCursor(10, 196);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.printf("Elapsed: %lu ms", (unsigned long)elapsed);

    if (done) {
      break;
    }

    delay(10);
  }

  // ハード＆ソフトアイアン補正係数計算
  float span[3];
  float maxSpan = 0.0f;
  for (int i = 0; i < 3; ++i) {
    magCal.offset[i] = (maxv[i] + minv[i]) * 0.5f;
    span[i]          = (maxv[i] - minv[i]) * 0.5f;
    if (span[i] < 1e-3f) span[i] = 1.0f;  // ゼロ割防止
    if (span[i] > maxSpan) maxSpan = span[i];
  }
  if (maxSpan < 1e-3f) maxSpan = 1.0f;

  for (int i = 0; i < 3; ++i) {
    magCal.scale[i] = maxSpan / span[i];
  }

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setCursor(10, 220);
  lcd.println("Calibration OK");
  delay(700);
}

// ================== タスク ==================
// IMU Task：I2Cはこのタスクのみで触る
void vTaskIMU(void* arg){
  const TickType_t period = pdMS_TO_TICKS(20); // 50Hz
  TickType_t last = xTaskGetTickCount();

  while (true) {
    float ax, ay, az;
    float mx, my, mz;
    float gx, gy, gz;

    float ax1, ay1, az1;
    float mx1, my1, mz1;
    float gx1, gy1, gz1;

    M5.Imu.getAccel(&ax,&ay,&az);
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getMag(&mx,&my,&mz);

    // ---- 既存の軸変換（Madgwick用） ----
    ax1 = ax *  -1.0;
    ay1 = ay *  -1.0;
    az1 = az *   1.0;

    gx1 = gx *  -1.0;
    gy1 = gy *  -1.0;
    gz1 = gz *   1.0;

    mx1 = mx *  -1.0;
    my1 = my *   1.0;
    mz1 = mz *  -1.0;

    // Madgwick による姿勢推定（ロール/ピッチの安定に利用）
    filter.update(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1);

    float roll  = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw   = filter.getYaw();

    // ---- 地磁気ヘディング：Z軸の異常を重力で消した水平磁場から算出 ----
    float mx_cal, my_cal, mz_cal;
    float mx_h,   my_h,   mz_h;
    float nx, ny, nz;

    getCalibratedMagWithTilt(
      ax, ay, az,
      mx, my, mz,
      mx_cal, my_cal, mz_cal,
      mx_h, my_h, mz_h,
      nx, ny, nz
    );

    float heading_raw  = headingFromHorizMag(ny, nx); // 生の磁気方位
    float heading_filt = g_headingKF.update(heading_raw); // ★カルマンフィルタで平滑化
    float heading      = heading_filt - 180.0 - DECLINATION;
    if ( heading < 0 ) heading += 360.0;

    // ---- 仰角：ここは必要に応じて調整 ----
    float elev = fabsf(roll);
    if (elev < 0.0f)  elev = 0.0f;
    if (elev > 90.0f) elev = 90.0f;

    if (xSemaphoreTake(g_navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_nav.heading_deg = heading;
      g_nav.elev_deg    = elev;
      g_nav.imu_ms      = millis();
      xSemaphoreGive(g_navMutex);
    }

    vTaskDelayUntil(&last, period);
  }
}

// GNSS Task：UARTから読み、USBへRAWパススルー＋TinyGPS++解析
void vTaskGNSS(void* arg){
  while (true) {
    while (Serial2.available()) {
      int c = Serial2.read();
      Serial.write((char)c);   // 生NMEAをUSBへ出したい場合はコメント解除
      gps.encode(c);           // 解析
    }
    // 解析結果をスナップショット
    bool   fixOK = false;
    double lat   = DEBUG_LAT;
    double lon   = DEBUG_LON;
    double alt   = 0.0;
    double hdop  = 0.0;
    int    sats  = 0;

    const TickType_t period = pdMS_TO_TICKS(20); 

    if (gps.location.isValid())   { lat = gps.location.lat(); lon = gps.location.lng(); fixOK = true; }
    if (gps.altitude.isValid())   alt = gps.altitude.meters();
    if (gps.hdop.isValid())       hdop = gps.hdop.hdop();
    if (gps.satellites.isValid()) sats = (int)gps.satellites.value();

    if (xSemaphoreTake(g_navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_nav.fixOK = fixOK;
      g_nav.lat   = lat;
      g_nav.lon   = lon;
      g_nav.alt_m = alt;
      g_nav.hdop  = hdop;
      g_nav.sats  = sats;
      g_nav.gnss_ms = millis();
      xSemaphoreGive(g_navMutex);
    }
    vTaskDelay(period);
  }
}

// ================== setup / loop ==================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // USB-Serial（PCへRAW NMEAを出す）
  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8N1, 16, 17);

  M5.Display.setRotation(1);
  M5.Display.clear(TFT_BLACK);
  M5.Display.setBrightness(200);

  if (!M5.Imu.begin()) {
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.println("IMU init failed");
    while (true) { delay(1000); }
  }

  // ★ 既存の座標軸設定（ユーザ調整済み）
  M5.Imu.setAxisOrder(
    m5::IMU_Class::axis_t::axis_x_neg,
    m5::IMU_Class::axis_t::axis_y_pos,
    m5::IMU_Class::axis_t::axis_z_neg
  );

  filter.begin(50.0f);  // 50Hz 更新

  // キャンバス（ダブルバッファ）作成
  canvas.setColorDepth(8);
  if (canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr) {
    canvas.setTextWrap(false);
    g_useCanvas = true;
  } else {
    g_useCanvas = false;
  }

  // キャリブ実行（タスク起動前に実施）
  calibrateIMU();
  M5.Display.clear(TFT_BLACK);

  // 共有Mutex
  g_navMutex = xSemaphoreCreateMutex();

  // タスク起動（コア割り当て）
  xTaskCreatePinnedToCore(vTaskIMU,  "IMU_Task",  4096, nullptr, 2, nullptr, 1); // Core1
  xTaskCreatePinnedToCore(vTaskGNSS, "GNSS_Task", 4096, nullptr, 2, nullptr, 0); // Core0
}

void loop() {
  M5.update();

  // 画面ON/OFF（BtnA）
  if (M5.BtnA.wasPressed()) {
    displayOn = !displayOn;
    if (displayOn) {
      M5.Display.setBrightness(200);
      M5.Display.clear(TFT_BLACK);
    } else {
      M5.Display.clear(TFT_BLACK);
      M5.Display.setBrightness(0);
    }
  }

  // OFF中は描画スキップ（省電力）
  if (!displayOn) {
    delay(80);
    return;
  }

  // 共有データのスナップショット
  SharedNavData snap;
  if (xSemaphoreTake(g_navMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    snap = g_nav;
    xSemaphoreGive(g_navMutex);
  }

  // 描画
  if (g_useCanvas) {
    canvas.fillScreen(TFT_BLACK);

    drawLatLon(canvas, snap.lat, snap.lon, snap.fixOK);
    drawCompass(canvas, snap.heading_deg);

    canvas.setTextDatum(MC_DATUM);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_SILVER, TFT_BLACK);
    canvas.drawString("Align to N (0 deg).", canvas.width()/2, canvas.height()/2 + 36);
    canvas.drawString("Lock when |Az|<=5 & |Elev-Target|<=3", canvas.width()/2, canvas.height()/2 + 50);

    float targetElev = fabs(snap.lat);     // 北極星高度 ≒ |緯度|
    drawElevationGauge(canvas, snap.elev_deg, targetElev);

    // LOCK判定
    float azErr = fminf(fabsf(snap.heading_deg - 0.0f),
                        360.0f - fabsf(snap.heading_deg - 0.0f));
    float elErr = fabsf(snap.elev_deg - targetElev);
    bool locked = (azErr <= HEADING_LOCK_TOL_DEG) && (elErr <= ELEV_LOCK_TOL_DEG);
    if (locked) {
      int cx = canvas.width()/2;
      int cy = canvas.height()/2 - 10;
      canvas.fillCircle(cx, cy, 10, TFT_GREEN);
      canvas.setTextDatum(BC_DATUM);
      canvas.setTextColor(TFT_GREEN, TFT_BLACK);
      canvas.setTextSize(2);
      canvas.drawString("LOCKED", cx, cy + 28);
    }

    canvas.pushSprite(0, 0);
  } else {
    // フォールバック：直接 Display に描く（簡易版）
    auto& lcd = M5.Display;
    lcd.startWrite();
    lcd.clear(TFT_BLACK);

    M5Canvas gfx(&lcd);
    gfx.setColorDepth(8);
    gfx.createSprite(lcd.width(), lcd.height());
    gfx.fillScreen(TFT_BLACK);

    drawLatLon(gfx, snap.lat, snap.lon, snap.fixOK);
    drawCompass(gfx, snap.heading_deg);

    float targetElev = fabs(snap.lat);
    drawElevationGauge(gfx, snap.elev_deg, targetElev);

    gfx.pushSprite(0, 0);
    gfx.deleteSprite();

    lcd.endWrite();
  }

  // 描画更新周期
  delay(30);
}
