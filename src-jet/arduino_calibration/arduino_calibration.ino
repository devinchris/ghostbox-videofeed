/* Automatisches Axis-Tuning + JSON-Ausgabe für Nano 33 BLE Sense Rev2 (ReefwingAHRS)
   - Benötigte Bibliotheken: ReefwingAHRS + Arduino IMU wrapper (IMU.* Aufrufe)
   - Serieller Menü:
       c -> collect poses (6)
       r -> run auto-tune (find mapping & print JSON)
       j -> print last JSON (falls bereits berechnet)
       g -> gyro-bias kalibrierung (still halten)
       m -> mag min/max (drehe 10s)
       l -> Live-Modus mit gefundener Mapping (q beendet Live)
*/

#include <Arduino.h>
#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h> 

ReefwingAHRS ahrs;

// ---- Einstellungen ----
const int SAMPLES_PER_POSE = 300;
const int SAMPLE_DELAY_MS = 5;
const unsigned long LIVE_PRINT_MS = 200;

const char* poseNames[6] = {
  "+X oben", "-X oben",
  "+Y oben", "-Y oben",
  "+Z oben", "-Z oben"
};
const float expectedVecs[6][3] = {
  { 1.0f,  0.0f,  0.0f}, // +X
  {-1.0f,  0.0f,  0.0f}, // -X
  { 0.0f,  1.0f,  0.0f}, // +Y
  { 0.0f, -1.0f,  0.0f}, // -Y
  { 0.0f,  0.0f,  1.0f}, // +Z
  { 0.0f,  0.0f, -1.0f}  // -Z
};

struct SampleAvg { float ax, ay, az; };
SampleAvg poseAvg[6];

int bestPerm[3] = {0,1,2};
int bestSign[3] = {1,1,1};
float lastRMSE = 0.0f;

// Kalibrierungen
float gyroBias[3] = {0.0f, 0.0f, 0.0f};
bool haveGyroBias = false;
float magOffset[3] = {0.0f, 0.0f, 0.0f};
bool haveMagOffsets = false;

// JSON-Cache (wird nach run tuning gesetzt)
String lastCalibJSON = "";

// ----------------- Hilfsfunktionen -----------------
bool readAccel_g(float &ax, float &ay, float &az) {
  if (!IMU.accelerationAvailable()) return false;
  IMU.readAcceleration(ax, ay, az);
  float mag = sqrt(ax*ax + ay*ay + az*az);
  if (mag > 5.0f) { ax /= 9.80665f; ay /= 9.80665f; az /= 9.80665f; }
  return true;
}

void transformVec(const float raw[3], const int perm[3], const int sign[3], float out[3]) {
  out[0] = sign[0] * raw[perm[0]];
  out[1] = sign[1] * raw[perm[1]];
  out[2] = sign[2] * raw[perm[2]];
}

void normalize(float v[3]) {
  float m = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (m > 1e-6f) { v[0] /= m; v[1] /= m; v[2] /= m; }
}

float computeErrorAcrossPoses(const int perm[3], const int sign[3]) {
  float totalSq = 0.0f;
  for (int p=0;p<6;p++) {
    float raw[3] = { poseAvg[p].ax, poseAvg[p].ay, poseAvg[p].az };
    float t[3];
    transformVec(raw, (int*)perm, (int*)sign, t);
    normalize(t);
    float ex = t[0] - expectedVecs[p][0];
    float ey = t[1] - expectedVecs[p][1];
    float ez = t[2] - expectedVecs[p][2];
    totalSq += (ex*ex + ey*ey + ez*ez);
  }
  return sqrt(totalSq / 6.0f);
}

String permToString(const int perm[3], const int sign[3]) {
  String s = "";
  for (int i=0;i<3;i++) {
    char buf[32];
    sprintf(buf, "%c%s", "XYZ"[perm[i]], (sign[i]==-1?"*":" "));
    s += String("out") + (i==0?"X":(i==1?"Y":"Z")) + "<-" + String(buf);
    if (i<2) s += ", ";
  }
  return s;
}

// ----------------- Pose-Sammlung -----------------
void collectPoseAverages() {
  Serial.println(F("\n--- STARTE POSE-SAMMLUNG ---"));
  Serial.println(F("Für jede Pose: positionieren, ENTER drücken, ruhig halten."));
  for (int p=0;p<6;p++) {
    Serial.print(F("Positioniere: "));
    Serial.println(poseNames[p]);
    Serial.println(F("Wenn fertig -> ENTER"));
    while (!Serial.available()) delay(20);
    while (Serial.available()) Serial.read();
    float sx=0, sy=0, sz=0;
    int got=0;
    for (int i=0;i<SAMPLES_PER_POSE;i++) {
      float ax,ay,az;
      if (readAccel_g(ax,ay,az)) {
        sx += ax; sy += ay; sz += az; got++;
      }
      delay(SAMPLE_DELAY_MS);
    }
    if (got == 0) {
      Serial.println(F("Fehler: Keine Accel-Samples erhalten!"));
      return;
    }
    poseAvg[p].ax = sx / (float)got;
    poseAvg[p].ay = sy / (float)got;
    poseAvg[p].az = sz / (float)got;
    Serial.print(F("Mittel (g): "));
    Serial.print(poseAvg[p].ax, 6); Serial.print(", ");
    Serial.print(poseAvg[p].ay, 6); Serial.print(", ");
    Serial.println(poseAvg[p].az, 6);
  }
  Serial.println(F("--- Ende Sammlung ---\n"));
}

// ----------------- Mapping-Finder -----------------
void findBestMapping() {
  int perms[6][3] = {
    {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}
  };
  float bestErr = 1e9;
  int bestPermLocal[3];
  int bestSignLocal[3];
  for (int pi=0; pi<6; pi++) {
    int *perm = perms[pi];
    for (int s=0; s<8; s++) {
      int sign[3];
      sign[0] = (s & 1) ? 1 : -1;
      sign[1] = (s & 2) ? 1 : -1;
      sign[2] = (s & 4) ? 1 : -1;
      float err = computeErrorAcrossPoses(perm, sign);
      if (err < bestErr) {
        bestErr = err;
        for (int i=0;i<3;i++){ bestPermLocal[i] = perm[i]; bestSignLocal[i] = sign[i]; }
      }
    }
  }
  for (int i=0;i<3;i++) { bestPerm[i] = bestPermLocal[i]; bestSign[i] = bestSignLocal[i]; }
  lastRMSE = bestErr;
  Serial.println(F("=== Bestes Mapping gefunden ==="));
  Serial.print(F("RMSE Fehler: ")); Serial.println(bestErr, 6);
  Serial.print(F("Mapping: ")); Serial.println(permToString(bestPerm, bestSign));

  // Erzeuge JSON und speichere in lastCalibJSON
  // JSON-Felder: perm, sign, gyroBias, magOffset, rmse, poses (array of 6 arrays)
  String j = "{";
  // perm
  j += "\"perm\":[";
  j += String(bestPerm[0]); j += ","; j += String(bestPerm[1]); j += ","; j += String(bestPerm[2]);
  j += "],";
  // sign
  j += "\"sign\":[";
  j += String(bestSign[0]); j += ","; j += String(bestSign[1]); j += ","; j += String(bestSign[2]);
  j += "],";
  // gyro bias (if vorhanden else zeros)
  j += "\"gyro\":[";
  j += String(haveGyroBias?gyroBias[0]:0.0, 6); j += ","; j += String(haveGyroBias?gyroBias[1]:0.0, 6); j += ","; j += String(haveGyroBias?gyroBias[2]:0.0, 6);
  j += "],";
  // mag offsets
  j += "\"mag\":[";
  j += String(haveMagOffsets?magOffset[0]:0.0, 6); j += ","; j += String(haveMagOffsets?magOffset[1]:0.0, 6); j += ","; j += String(haveMagOffsets?magOffset[2]:0.0, 6);
  j += "],";
  // rmse
  j += "\"rmse\":"; j += String(lastRMSE, 8); j += ",";
  // poses
  j += "\"poses\":[";
  for (int p=0;p<6;p++) {
    j += "[" + String(poseAvg[p].ax,6) + "," + String(poseAvg[p].ay,6) + "," + String(poseAvg[p].az,6) + "]";
    if (p<5) j += ",";
  }
  j += "]";
  j += "}";
  lastCalibJSON = j;

  // Drucke die JSON einmal formatiert in einer Zeile (einfach kopierbar)
  Serial.println(F("\n--- CALIB JSON (einzeilig, kopierbar) ---"));
  Serial.println(lastCalibJSON);
  Serial.println(F("--- Ende JSON ---\n"));
}

// ----------------- Mag und Gyro Kalibrierung -----------------
void doGyroCalibration() {
  Serial.println(F("Starte Gyro-Bias Kalibrierung — halte Board ruhig und drücke ENTER"));
  while (!Serial.available()) delay(20);
  while (Serial.available()) Serial.read();
  float sx=0, sy=0, sz=0;
  int got=0;
  for (int i=0;i<400;i++) {
    float gx,gy,gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx,gy,gz);
      sx += gx; sy += gy; sz += gz; got++;
    }
    delay(5);
  }
  if (got>0) {
    gyroBias[0] = sx/got; gyroBias[1] = sy/got; gyroBias[2] = sz/got;
    haveGyroBias = true;
    Serial.print(F("Gyro Bias (°/s): "));
    Serial.print(gyroBias[0],6); Serial.print(", "); Serial.print(gyroBias[1],6); Serial.print(", "); Serial.println(gyroBias[2],6);
  } else {
    Serial.println(F("Keine Gyro-Samples erhalten."));
  }
}

void doMagMinMax(int durationMs = 10000) {
  Serial.println(F("Starte Mag min/max Sammlung — drehe das Board langsam (10s)"));
  float minX=1e6, minY=1e6, minZ=1e6;
  float maxX=-1e6, maxY=-1e6, maxZ=-1e6;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    if (IMU.magneticFieldAvailable()) {
      float mx,my,mz;
      IMU.readMagneticField(mx,my,mz);
      if (mx < minX) minX = mx; if (mx > maxX) maxX = mx;
      if (my < minY) minY = my; if (my > maxY) maxY = my;
      if (mz < minZ) minZ = mz; if (mz > maxZ) maxZ = mz;
    }
    delay(20);
  }
  magOffset[0] = (maxX + minX) / 2.0f;
  magOffset[1] = (maxY + minY) / 2.0f;
  magOffset[2] = (maxZ + minZ) / 2.0f;
  haveMagOffsets = true;
  Serial.print(F("Mag offsets (approx): "));
  Serial.print(magOffset[0],6); Serial.print(", ");
  Serial.print(magOffset[1],6); Serial.print(", ");
  Serial.println(magOffset[2],6);
}

// ----------------- Live Mode -----------------
void applyMappingAndRun() {
  if (lastCalibJSON == "") {
    Serial.println(F("Keine Mapping-Daten gefunden. Führe 'c' (collect) und 'r' (run) zuerst aus."));
    return;
  }
  Serial.println(F("Starte Live-Mode mit gefundener Mapping. 'q' beendet Live."));
  unsigned long nextPrint = 0;
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q') { Serial.println(F("Live Mode beendet.")); break; }
    }
    float ax=0,ay=0,az=0,gx=0,gy=0,gz=0,mx=0,my=0,mz=0;
    if (IMU.accelerationAvailable()) IMU.readAcceleration(ax,ay,az);
    if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx,gy,gz);
    if (IMU.magneticFieldAvailable()) IMU.readMagneticField(mx,my,mz);
    // accel units
    float magA = sqrt(ax*ax + ay*ay + az*az);
    if (magA > 5.0f) { ax /= 9.80665f; ay /= 9.80665f; az /= 9.80665f; }
    float rawA[3] = {ax, ay, az};
    float rawG[3] = {gx, gy, gz};
    float rawM[3] = {mx, my, mz};
    float tA[3], tG[3], tM[3];
    transformVec(rawA, bestPerm, bestSign, tA);
    transformVec(rawG, bestPerm, bestSign, tG);
    transformVec(rawM, bestPerm, bestSign, tM);
    // Apply gyro bias and mag offsets if available
    if (haveGyroBias) { tG[0] -= gyroBias[0]; tG[1] -= gyroBias[1]; tG[2] -= gyroBias[2]; }
    if (haveMagOffsets) { tM[0] -= magOffset[0]; tM[1] -= magOffset[1]; tM[2] -= magOffset[2]; }
    SensorData d;
    d.ax = tA[0]; d.ay = tA[1]; d.az = tA[2];
    d.gx = tG[0]; d.gy = tG[1]; d.gz = tG[2];
    d.mx = tM[0]; d.my = tM[1]; d.mz = tM[2];
    ahrs.setData(d, false);
    ahrs.update();
    if (millis() >= nextPrint) {
      nextPrint = millis() + LIVE_PRINT_MS;
      Serial.print("Roll: "); Serial.print(ahrs.angles.roll, 3);
      Serial.print("  Pitch: "); Serial.print(ahrs.angles.pitch, 3);
      Serial.print("  Yaw: "); Serial.println(ahrs.angles.yaw, 3);
    }
  }
}

// ----------------- Setup/Loop -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Automatisches Axis-Tuning + JSON für Nano 33 BLE Rev2 (Reefwing)"));
  if (!IMU.begin()) {
    Serial.println(F("IMU init fehlgeschlagen! Prüfe Bibliothek/Board."));
    while (1) delay(1000);
  }
  ahrs.begin();
  Serial.print(F("Board detected: ")); Serial.println(ahrs.getBoardTypeString());
  Serial.println(F("Menü: 'c' collect poses, 'r' run auto-tune, 'j' print JSON, 'g' gyro-cal, 'm' mag-cal, 'l' live"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') {
      collectPoseAverages();
    } else if (c == 'r') {
      // check
      bool anyZero = false;
      for (int i=0;i<6;i++) if (fabs(poseAvg[i].ax) < 1e-6 && fabs(poseAvg[i].ay) < 1e-6 && fabs(poseAvg[i].az) < 1e-6) anyZero = true;
      if (anyZero) {
        Serial.println(F("Nicht alle Posen gesammelt - drücke 'c' zuerst."));
      } else {
        findBestMapping();
        Serial.println(F("JSON wurde erstellt. Drücke 'j' um sie erneut zu drucken."));
      }
    } else if (c == 'j') {
      if (lastCalibJSON != "") {
        Serial.println(F("\n--- CALIB JSON (erneut) ---"));
        Serial.println(lastCalibJSON);
        Serial.println(F("--- Ende JSON ---\n"));
      } else Serial.println(F("Keine JSON vorhanden. Führe 'r' aus."));
    } else if (c == 'g') {
      doGyroCalibration();
    } else if (c == 'm') {
      doMagMinMax(10000);
    } else if (c == 'l') {
      applyMappingAndRun();
      Serial.println(F("Zurück im Menü."));
    }
    while (Serial.available()) Serial.read();
  }
  delay(50);
}
