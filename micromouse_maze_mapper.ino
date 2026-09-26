// ESP32-C6 Micromouse 2026: stationary sensor map + real flood fill.
// The user carries the mouse between cells and enters its position on Serial.
// No motor pins are driven. Disconnect the motor battery for this test.
// Requires "VL53L0X by Pololu" in Arduino Library Manager.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t WIDTH = 16;
constexpr uint16_t CELL_COUNT = WIDTH * WIDTH;
constexpr uint16_t INF = 0xFFFF;
constexpr uint16_t WALL_LIMIT_MM = 120; // Calibrate from readings in your maze.

// N, E, S, W. Position (0,0) is the start corner; north increases y.
enum Direction : uint8_t { N = 0, E = 1, S = 2, W = 3 };
const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const char DNAME[5] = "NESW";

struct RangingSensor {
  const char *name;
  uint8_t xshut;
  uint8_t address;
  VL53L0X device;
  bool ready;
};

// GPIO6/7 are I2C SDA/SCL. GPIO18/19/20 are XSHUT L/F/R.
// Addresses are assigned here on every reboot. The diagram uses 0x32 for R.
RangingSensor tof[3] = {
  {"L", 18, 0x30, VL53L0X(), false},
  {"F", 19, 0x31, VL53L0X(), false},
  {"R", 20, 0x32, VL53L0X(), false}
};

uint8_t known[CELL_COUNT] = {};
uint8_t walls[CELL_COUNT] = {};
uint16_t distanceToGoal[CELL_COUNT];
uint8_t mouseX = 0, mouseY = 0;
Direction heading = N;

bool inside(int x, int y) {
  return x >= 0 && x < WIDTH && y >= 0 && y < WIDTH;
}

uint16_t indexOf(int x, int y) { return y * WIDTH + x; }
Direction opposite(Direction d) { return Direction((d + 2) & 3); }

void recordWall(int x, int y, Direction d, bool present) {
  if (!inside(x, y)) return;
  int nx = x + DX[d], ny = y + DY[d];
  if (!inside(nx, ny)) present = true; // Outer boundary cannot be opened.

  uint8_t bit = uint8_t(1U << d);
  uint16_t i = indexOf(x, y);
  known[i] |= bit;
  if (present) walls[i] |= bit;
  else walls[i] &= uint8_t(~bit);

  if (inside(nx, ny)) {
    i = indexOf(nx, ny);
    bit = uint8_t(1U << opposite(d));
    known[i] |= bit;
    if (present) walls[i] |= bit;
    else walls[i] &= uint8_t(~bit);
  }
}

bool passable(int x, int y, Direction d) {
  // Unknown interior edges are treated as open during exploration.
  return inside(x + DX[d], y + DY[d]) &&
         !(walls[indexOf(x, y)] & (1U << d));
}

void floodFill() {
  uint16_t queue[CELL_COUNT];
  uint16_t head = 0, tail = 0;
  for (uint16_t i = 0; i < CELL_COUNT; ++i) distanceToGoal[i] = INF;

  for (uint8_t y = 7; y <= 8; ++y) {
    for (uint8_t x = 7; x <= 8; ++x) {
      uint16_t i = indexOf(x, y);
      distanceToGoal[i] = 0;
      queue[tail++] = i;
    }
  }

  while (head < tail) {
    uint16_t i = queue[head++];
    int x = i % WIDTH, y = i / WIDTH;
    for (uint8_t d = 0; d < 4; ++d) {
      if (!passable(x, y, Direction(d))) continue;
      uint16_t j = indexOf(x + DX[d], y + DY[d]);
      if (distanceToGoal[j] != INF) continue;
      distanceToGoal[j] = distanceToGoal[i] + 1;
      queue[tail++] = j;
    }
  }
}

bool startSensor(RangingSensor &s) {
  digitalWrite(s.xshut, HIGH);
  delay(20);
  s.device.setTimeout(100);
  if (!s.device.init()) {
    digitalWrite(s.xshut, LOW);
    return false;
  }
  s.device.setAddress(s.address);
  s.device.startContinuous();
  return true;
}

void showDecision() {
  floodFill();
  uint16_t here = distanceToGoal[indexOf(mouseX, mouseY)];
  Serial.printf("Position (%u,%u), facing %c; flood number ", mouseX, mouseY, DNAME[heading]);
  if (here == INF) { Serial.println("unreachable. No move suggested."); return; }
  Serial.println(here);
  if (here == 0) { Serial.println("You are in the goal block."); return; }

  // Prefer straight on equal-distance options; otherwise consider left/right/back.
  uint8_t order[4] = {uint8_t(heading), uint8_t((heading + 3) & 3),
                      uint8_t((heading + 1) & 3), uint8_t((heading + 2) & 3)};
  Direction best = N;
  uint16_t bestDistance = here;
  bool found = false;
  for (uint8_t k = 0; k < 4; ++k) {
    Direction d = Direction(order[k]);
    if (!passable(mouseX, mouseY, d)) continue;
    uint16_t candidate = distanceToGoal[indexOf(mouseX + DX[d], mouseY + DY[d])];
    if (candidate < bestDistance) {
      bestDistance = candidate;
      best = d;
      found = true;
    }
  }
  if (found) {
    Serial.printf("Suggested NEXT cell: %c -> (%d,%d), flood number %u\n",
                  DNAME[best], mouseX + DX[best], mouseY + DY[best], bestDistance);
    Serial.println("Carry the mouse there, then enter its position with: p x y N/E/S/W");
  } else Serial.println("No open downhill neighbour. Check wall readings and position.");
}

void scanCell() {
  for (auto &s : tof) {
    if (!s.ready) { Serial.printf("%s sensor FAIL; scan cancelled.\n", s.name); return; }
  }

  // Left, front, right relative to the direction the mouse faces.
  Direction directions[3] = {
    Direction((heading + 3) & 3), heading, Direction((heading + 1) & 3)
  };
  uint16_t mm[3];
  for (uint8_t k = 0; k < 3; ++k) {
    mm[k] = tof[k].device.readRangeContinuousMillimeters();
    if (tof[k].device.timeoutOccurred() || mm[k] == 0 || mm[k] == 65535) {
      Serial.printf("%s reading invalid; map NOT updated. Recheck wiring/alignment.\n", tof[k].name);
      return;
    }
  }
  for (uint8_t k = 0; k < 3; ++k) {
    bool wall = mm[k] < WALL_LIMIT_MM;
    Serial.printf("%s: %u mm = %s  ", tof[k].name, mm[k], wall ? "WALL" : "OPEN");
    recordWall(mouseX, mouseY, directions[k], wall);
  }
  Serial.println();
  showDecision();
}

void setup() {
  Serial.begin(115200);
  uint32_t started = millis();
  while (!Serial && millis() - started < 2000) delay(10);
  Serial.println("\nMicromouse maze mapper: sensors + flood fill; MOTORS DISABLED");

  for (auto &s : tof) { pinMode(s.xshut, OUTPUT); digitalWrite(s.xshut, LOW); }
  delay(20);
  Wire.begin(6, 7);
  Wire.setClock(400000);
  for (auto &s : tof) {
    s.ready = startSensor(s);
    Serial.printf("%s XSHUT GPIO%u I2C 0x%02X: %s\n",
                  s.name, s.xshut, s.address, s.ready ? "PASS" : "FAIL");
  }

  for (uint8_t i = 0; i < WIDTH; ++i) {
    recordWall(i, 0, S, true);
    recordWall(i, WIDTH - 1, N, true);
    recordWall(0, i, W, true);
    recordWall(WIDTH - 1, i, E, true);
  }
  Serial.println("Set position: p x y N/E/S/W (example: p 0 0 N)");
  Serial.println("Then send: s  to scan walls and calculate the next cell.");
  Serial.println("All map memory clears when the board resets.");
}

void loop() {
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.equalsIgnoreCase("s")) { scanCell(); return; }

  int x, y;
  char face;
  if (sscanf(command.c_str(), "p %d %d %c", &x, &y, &face) == 3 && inside(x, y)) {
    if (face >= 'a' && face <= 'z') face -= ('a' - 'A');
    for (uint8_t d = 0; d < 4; ++d) {
      if (face != DNAME[d]) continue;
      mouseX = x; mouseY = y; heading = Direction(d);
      Serial.printf("Position set: (%u,%u) facing %c. Send s to scan.\n", mouseX, mouseY, face);
      return;
    }
  }
  Serial.println("Use: p x y N/E/S/W   or: s");
}
