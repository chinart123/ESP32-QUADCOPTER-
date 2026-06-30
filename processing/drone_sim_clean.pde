import processing.serial.*;

// =====================================================
// DIGITAL TWIN 1S DRONE (doc khong day tu MACH NHAN)
// Dong SIM: roll,pitch,yaw,alt,posX,posY,pwmX,pwmY,dX,dY,armed,rc4_min,...
// =====================================================

Serial myPort;

float tRoll, tPitch, tYaw, tAlt;
float tPosX, tPosY;
float tPwmX, tPwmY;
float tRawDX, tRawDY;
boolean tArmed = false;
float tRc4Min = 2000;
float tDtFlow = 0;   // nhip doc optical flow (ms)
float tLoopUs = 0;   // chu ky vong Core 0 (us)

float dRoll, dPitch, dYaw, dAlt;
float dX, dY;

float floorLevel;
ArrayList<PVector> path = new ArrayList<PVector>();

// ---- Do quang duong chao dao (settling) ----
boolean measuring = false;
float pathLenCm = 0, peakDevCm = 0;
int overshootCount = 0, epStartMs = 0;
float lastPosX = 0, lastPosY = 0;
boolean haveLast = false;
int settledSinceMs = 0;
boolean settledTimerRunning = false;
float prevDev = 0; int devSlope = 0;
float lastEpPathCm = 0, lastEpPeakCm = 0;
int lastEpDurMs = 0, lastEpOvershoots = 0;
boolean haveEpisode = false;
float START_THRESH = 1.5, SETTLE_THRESH = 1.0;
int SETTLE_HOLD_MS = 400;

// ---- Chan doan disarm ----
int lastDisarmMs = -99999;
float disarmRc4 = 2000, disarmAlt = 0;
boolean prevArmed = false;

// ---- Bieu do ----
boolean showGraphs = false;
int GBUF = 300;
float[] gPosX = new float[GBUF];
float[] gPosY = new float[GBUF];
float[] gDev  = new float[GBUF];
float[] gRc4  = new float[GBUF];
int gIdx = 0;

void setup() {
  size(1200, 800, P3D);
  floorLevel = height/2 + 250;
  try {
    // CONG COM CUA MACH NHAN (ground station), baud khop receiver: 921600
    myPort = new Serial(this, "COM8", 921600);
    myPort.bufferUntil('\n');
    println("Connected to Ground Station");
  } catch (Exception e) { println("ERR: Check COM Port Connection."); }
}

void draw() {
  background(20);
  lights();

  dRoll  = lerp(dRoll, tRoll, 0.15);
  dPitch = lerp(dPitch, tPitch, 0.15);
  dYaw   = lerp(dYaw, tYaw, 0.15);
  dAlt   = lerp(dAlt, tAlt, 0.1);
  dX = lerp(dX, tPosX * 10.0, 0.1);
  dY = lerp(dY, -tPosY * 10.0, 0.1);

  if (path.size() == 0 || dist(dX, dY, path.get(path.size()-1).x, path.get(path.size()-1).y) > 2.0) {
    path.add(new PVector(dX, dY));
    if (path.size() > 2000) path.remove(0);
  }

  drawGroundGrid();
  drawPath();

  pushMatrix(); translate(width/2, floorLevel, 0); fill(255,0,0); noStroke(); sphere(5); popMatrix();

  if (abs(dX) > 5 || abs(dY) > 5) {
    stroke(255,50,50,150); strokeWeight(2);
    line(width/2, floorLevel, 0, width/2 + dX, floorLevel - dAlt, dY);
    strokeWeight(1);
  }

  pushMatrix();
  translate(width/2 + dX, floorLevel - dAlt, dY);
  rotateY(radians(-dYaw + 180));
  rotateX(radians(-dPitch));
  rotateZ(radians(-dRoll));
  drawRealistic1SDrone();
  popMatrix();

  hint(DISABLE_DEPTH_TEST);
  camera();
  drawHUD();
  drawEpisodeHUD();
  drawDisarmHUD();
  drawTimingHUD();
  if (showGraphs) drawGraphs();
  else            drawAltitudeRuler();
  hint(ENABLE_DEPTH_TEST);
}

void updateEpisode() {
  float dev = sqrt(tPosX*tPosX + tPosY*tPosY);

  gPosX[gIdx] = tPosX; gPosY[gIdx] = tPosY;
  gDev[gIdx] = dev;    gRc4[gIdx] = tRc4Min;
  gIdx = (gIdx + 1) % GBUF;

  boolean atResetOrigin = (tPosX == 0.0 && tPosY == 0.0);

  if (!measuring) {
    if (!atResetOrigin && dev > START_THRESH) {
      measuring = true; pathLenCm = 0; peakDevCm = dev; overshootCount = 0;
      epStartMs = millis(); lastPosX = tPosX; lastPosY = tPosY; haveLast = true;
      settledTimerRunning = false; prevDev = dev; devSlope = +1;
    }
  } else {
    if (haveLast) pathLenCm += dist(tPosX, tPosY, lastPosX, lastPosY);
    lastPosX = tPosX; lastPosY = tPosY; haveLast = true;
    if (dev > peakDevCm) peakDevCm = dev;
    int newSlope = (dev > prevDev) ? +1 : -1;
    if (devSlope == +1 && newSlope == -1) overshootCount++;
    devSlope = newSlope; prevDev = dev;
    if (dev < SETTLE_THRESH) {
      if (!settledTimerRunning) { settledTimerRunning = true; settledSinceMs = millis(); }
      else if (millis() - settledSinceMs >= SETTLE_HOLD_MS) {
        lastEpPathCm = pathLenCm; lastEpPeakCm = peakDevCm;
        lastEpDurMs = millis() - epStartMs; lastEpOvershoots = overshootCount;
        haveEpisode = true; measuring = false;
      }
    } else settledTimerRunning = false;
    if (atResetOrigin) { measuring = false; settledTimerRunning = false; }
  }

  // Bat su kien disarm: armed chuyen 1 -> 0
  if (prevArmed && !tArmed) {
    lastDisarmMs = millis(); disarmRc4 = tRc4Min; disarmAlt = tAlt;
  }
  prevArmed = tArmed;
}

void drawRealistic1SDrone() {
  stroke(200); fill(30); box(30, 15, 45);
  stroke(100); fill(20);
  pushMatrix(); rotateY(PI/4);  box(85, 4, 4); popMatrix();
  pushMatrix(); rotateY(-PI/4); box(85, 4, 4); popMatrix();
  pushMatrix(); translate(0, -5, 25); fill(0,255,0); noStroke(); box(10,10,10); popMatrix();
}

void drawPath() {
  pushMatrix(); translate(width/2, floorLevel, 0);
  noFill(); stroke(255,200,0); strokeWeight(3);
  beginShape();
  for (PVector p : path) vertex(p.x, 0, p.y);
  endShape();
  strokeWeight(1); popMatrix();
}

void drawAltitudeRuler() {
  int rX = width - 80, rTop = 150, rBot = height - 150;
  float maxAlt = 1500;
  stroke(255,100); line(rX, rTop, rX, rBot);
  textAlign(RIGHT, CENTER); textSize(12);
  for (int i = 0; i <= maxAlt; i += 100) {
    float y = map(i, 0, maxAlt, rBot, rTop);
    line(rX, y, rX-10, y); fill(150); text(i + " mm", rX-15, y);
  }
  float markerY = map(constrain(tAlt, 0, maxAlt), 0, maxAlt, rBot, rTop);
  stroke(0,255,0); line(rX-30, markerY, rX+30, markerY);
  fill(0,255,0); textAlign(LEFT, CENTER); textSize(24);
  text(nf(tAlt,1,0) + " mm", rX+35, markerY);
}

void drawGroundGrid() {
  pushMatrix(); translate(width/2, floorLevel, 0);
  stroke(255,30); noFill();
  int size = 4000, step = 100;
  for (int i = -size/2; i <= size/2; i += step) {
    line(i, 0, -size/2, i, 0, size/2);
    line(-size/2, 0, i, size/2, 0, i);
  }
  popMatrix();
}

void drawHUD() {
  textAlign(LEFT, BASELINE);
  fill(255); textSize(20); text("DIGITAL TWIN: 1S DRONE", 40, 40);

  // Trang thai ARM
  if (tArmed) { fill(0,255,0); text("ARMED", 330, 40); }
  else        { fill(255,60,40); text("DISARMED", 330, 40); }

  if (tAlt > 30) { fill(0,255,0); textSize(16); text("OPTICAL FLOW: ACTIVE", 40, 72); }
  else           { fill(255,50,0); textSize(16); text("OPTICAL FLOW: LOCKED (<30mm)", 40, 72); }

  fill(200); textSize(14);
  text("Roll : " + nf(tRoll,1,1) + " deg", 40, 100);
  text("Pitch: " + nf(tPitch,1,1) + " deg", 40, 120);
  text("Yaw  : " + nf(tYaw,1,1) + " deg", 40, 140);
  text("Alt  : " + nf(tAlt,1,0) + " mm", 40, 160);

  float dev = sqrt(tPosX*tPosX + tPosY*tPosY);
  fill(255,200,0);
  text("Lech X : " + nf(tPosX,1,1) + " cm", 40, 190);
  text("Lech Y : " + nf(tPosY,1,1) + " cm", 40, 210);
  fill(255,150,0);
  text("Lech tong: " + nf(dev,1,1) + " cm", 40, 230);

  fill(0,200,255);
  text("Luc Bu X: " + nf(tPwmX,1,0) + " do", 40, 260);
  text("Luc Bu Y: " + nf(tPwmY,1,0) + " do", 40, 280);

  fill(210,100,255);
  text("Raw dX: " + nf(tRawDX,1,0), 40, 310);
  text("Raw dY: " + nf(tRawDY,1,0), 40, 330);
}

void drawEpisodeHUD() {
  textAlign(LEFT, BASELINE);
  float py = 360;
  fill(0,0,0,140); noStroke(); rect(30, py, 360, 150, 8);
  textSize(16);
  if (measuring) {
    fill(255,80,80); text(">> DANG DO (drone dang chao dao)", 45, py+25);
    fill(255); textSize(14);
    text("Quang duong di lac: " + nf(pathLenCm,1,1) + " cm", 45, py+50);
    text("Overshoot max     : " + nf(peakDevCm,1,1) + " cm", 45, py+70);
    text("So lan ngoanh dau : " + overshootCount, 45, py+90);
    text("Thoi gian         : " + (millis()-epStartMs) + " ms", 45, py+110);
  } else {
    fill(0,255,120); text("KET QUA LAN GAN NHAT", 45, py+25);
    fill(220); textSize(14);
    if (haveEpisode) {
      text("Quang duong sai so: " + nf(lastEpPathCm,1,1) + " cm", 45, py+50);
      text("Overshoot max     : " + nf(lastEpPeakCm,1,1) + " cm", 45, py+70);
      text("So lan ngoanh dau : " + lastEpOvershoots, 45, py+90);
      text("Thoi gian on dinh : " + lastEpDurMs + " ms", 45, py+110);
    } else text("Cho drone lech roi tu ve 0...", 45, py+60);
  }
  fill(150); textSize(12);
  text("SPACE=reset goc  |  G=bieu do  |  R=xoa ket qua", 40, py+145);
}

void drawDisarmHUD() {
  textAlign(LEFT, BASELINE);
  float py = 525;
  // Canh bao glitch cong tac Arm (rc4_min thap dat = co dau hieu nhieu ngat)
  boolean glitchNow  = (tRc4Min < 1450);
  boolean nearGround = (tAlt < 50); // 5cm

  fill(0,0,0,140); noStroke(); rect(30, py, 360, 90, 8);
  textSize(14);
  fill(glitchNow ? color(255,60,40) : color(0,255,120));
  text("rc4_min (cong tac Arm): " + nf(tRc4Min,1,0), 45, py+25);

  if (glitchNow && nearGround)
    { fill(255,50,40); textSize(15); text("!! GLITCH CONG TAC + GAN DAT !!", 45, py+48); }
  else if (glitchNow)
    { fill(255,150,40); textSize(15); text("! rc4_min tut < 1450 (kha nghi)", 45, py+48); }
  else
    { fill(0,255,120); text("Cong tac Arm on dinh", 45, py+48); }

  if (millis() - lastDisarmMs < 4000) {
    fill(255,80,80); textSize(13);
    text("DISARM luc rc4=" + nf(disarmRc4,1,0) + "  alt=" + nf(disarmAlt,1,0) + "mm", 45, py+72);
  }
}

void drawTimingHUD() {
  textAlign(LEFT, BASELINE);
  float py = 625;
  fill(0,0,0,140); noStroke(); rect(410, py, 360, 90, 8);
  textSize(14);
  // dt_flow: binh thuong ~10ms. Cao hon nhieu = nhip doc cam bien bi gian (CHAM)
  fill(tDtFlow > 16 ? color(255,150,40) : color(0,255,120));
  text("Nhip doc flow: " + nf(tDtFlow,1,0) + " ms  (tot ~10)", 425, py+25);
  // loop_us: chu ky vong Core 0. Tang vot = bi chen ngang
  fill(tLoopUs > 6000 ? color(255,150,40) : color(0,255,120));
  text("Chu ky Core 0: " + nf(tLoopUs,1,0) + " us  (tot < 4000)", 425, py+48);
  fill(150); textSize(12);
  text("2 so nay cao = pos hold cham vi nhip doc bi gian", 425, py+72);

void drawGraph(float x, float y, float w, float h, float[] buf, float vmin, float vmax, color col, String label) {
  noFill(); stroke(255,40); rect(x, y, w, h);
  float zeroY = map(0, vmin, vmax, y+h, y);
  stroke(255,60); line(x, zeroY, x+w, zeroY);
  stroke(col); noFill(); strokeWeight(1.5);
  beginShape();
  for (int i = 0; i < GBUF; i++) {
    int idx = (gIdx + i) % GBUF;
    float v = constrain(buf[idx], vmin, vmax);
    vertex(map(i,0,GBUF-1,x,x+w), map(v,vmin,vmax,y+h,y));
  }
  endShape(); strokeWeight(1);
  fill(col); textAlign(LEFT, TOP); textSize(12); text(label, x+6, y+4);
}

void drawGraphs() {
  float gw = 380, gh = 120, gx = width - gw - 20, gy = 150, gap = 16;
  drawGraph(gx, gy,            gw, gh, gPosX, -50, 50,   color(255,200,0),  "pos_x (cm)");
  drawGraph(gx, gy+(gh+gap),   gw, gh, gPosY, -50, 50,   color(0,200,255),  "pos_y (cm)");
  drawGraph(gx, gy+2*(gh+gap), gw, gh, gDev,    0, 60,   color(255,80,80),  "Lech tong (cm)");
  drawGraph(gx, gy+3*(gh+gap), gw, gh, gRc4, 1000, 2100, color(255,160,40), "rc4_min (Arm)");
}

void serialEvent(Serial myPort) {
  try {
    String inString = myPort.readStringUntil('\n');
    if (inString == null) return;
    inString = trim(inString);
    if (!inString.startsWith("SIM,")) return;
    String[] list = split(inString.substring(4), ',');
    if (list.length < 10) return;

    tRoll  = float(list[0]); tPitch = float(list[1]); tYaw = float(list[2]); tAlt = float(list[3]);
    tPosX  = float(list[4]); tPosY  = float(list[5]);
    tPwmX  = float(list[6]); tPwmY  = float(list[7]);
    tRawDX = float(list[8]); tRawDY = float(list[9]);
    if (list.length >= 11) tArmed  = (int(float(list[10])) != 0);
    if (list.length >= 12) tRc4Min = float(list[11]);
    if (list.length >= 17) tDtFlow = float(list[16]);
    if (list.length >= 18) tLoopUs = float(list[17]);
    updateEpisode();
  } catch (Exception e) { println(e); }
}

void keyPressed() {
  if (key == ' ') { dX = 0; dY = 0; path.clear(); }
  if (key == 'g' || key == 'G') showGraphs = !showGraphs;
  if (key == 'r' || key == 'R') { haveEpisode = false; measuring = false; }
}
