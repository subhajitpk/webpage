let zoomLevel = 1;
let offsetX = 0;
let offsetY = 0;
let stretchY = 1;

let robots = [];
let glowPhase = 0;
let running = false;

// ---------- Setup ----------

function setup() {
  let canvas = createCanvas(900, 600);
    canvas.parent("canvasContainer");
  frameRate(60);
  resetSim();
}

// ---------- Speed ----------

function speedFactor() {
  let sliderValue = parseInt(document.getElementById("speedSlider").value);
  return sliderValue / 50;
}

// ---------- Main Draw Loop ----------

function draw() {

  background(245);
  glowPhase += 0.05;

  let sliderValue = parseInt(document.getElementById("speedSlider").value);
  frameRate(sliderValue);

  push();

  translate(width/2, height/2);
  scale(zoomLevel, zoomLevel * stretchY);
  translate(-width/2, -height/2);

  if (running) {
    simulationStep();
  }

  drawRobots();

  pop();

  drawStatus();
}

// ---------- Initialize Robots ----------

function resetSim() {

  let n = parseInt(document.getElementById("robotCount").value);

  let valid = false;

  while (!valid) {

    robots = [];

    for (let i = 0; i < n; i++) {
      robots.push({
        pos: createVector(
          random(200, width-200),
          random(150, height-150)
        )
      });
    }

    valid = noThreeCollinear();
  }

  running = false;
}

function startSim() { running = true; }
function stopSim() { running = false; }

// ---------- Drawing ----------

function drawRobots() {

  let hull = convexHull(robots.map(r => r.pos));

  for (let r of robots) {

    if (pointOnHull(r.pos, hull)) {
        fill(255, 80 + 50*sin(glowPhase), 80);
        } else {
        fill(50, 100, 255);
    }

    stroke(255);
    strokeWeight(1);
    ellipse(r.pos.x, r.pos.y, 8, 8);
  }

  // draw hull
  stroke(255,0,0);
  noFill();
  beginShape();
  for (let p of hull)
    vertex(p.x, p.y);
  endShape(CLOSE);

  // ---- Display interior angles at hull vertices ----

for (let i = 0; i < hull.length; i++) {

  let prev = hull[(i - 1 + hull.length) % hull.length];
  let curr = hull[i];
  let next = hull[(i + 1) % hull.length];

  let v1 = p5.Vector.sub(prev, curr);
  let v2 = p5.Vector.sub(next, curr);

  let angle = abs(degrees(v1.angleBetween(v2)));

  noStroke();
fill(0, 0, 0, 120);   // black with transparency
textSize(10);         // smaller size
text(angle.toFixed(2), curr.x + 6, curr.y - 6);
}
}

// ---------- Core Algorithm ----------

function simulationStep() {

  if (contractionFinished()) {
    running = false;
    console.log("All robots are strict corners");
    return;
}

  let points = robots.map(r => r.pos);
  let hull = convexHull(points);

  let moves = [];

  for (let i = 0; i < hull.length; i++) {

    let current = hull[i];
    let prev = hull[(i - 1 + hull.length) % hull.length];
    let next = hull[(i + 1) % hull.length];

    let robot = robots.find(r =>
      p5.Vector.dist(r.pos, current) < 0.001
    );

    if (!robot) continue;

    let p_mid1 = p5.Vector.add(current, prev).mult(0.5);
    let p_mid2 = p5.Vector.add(current, next).mult(0.5);
    let targetMid = p5.Vector.add(p_mid1, p_mid2).mult(0.5);

    let interiorRobots = [];

for (let other of robots) {

  if (p5.Vector.dist(other.pos, current) < 0.001)
    continue;

  if (pointInTriangle(other.pos, current, p_mid1, p_mid2)) {
    interiorRobots.push(other);
  }
}

let moveTarget = null;

if (interiorRobots.length === 0) {

  // First branch
  moveTarget = p5.Vector.add(p_mid1, p_mid2).mult(0.5);

} else {

  // Second branch

  // Find nearest interior robot
  let rPrime = interiorRobots[0];
  let minDist = p5.Vector.dist(current, rPrime.pos);

  for (let r2 of interiorRobots) {
    let d = p5.Vector.dist(current, r2.pos);
    if (d < minDist) {
      minDist = d;
      rPrime = r2;
    }
  }

  // Direction of p_mid1 -> p_mid2
  let dir = p5.Vector.sub(p_mid2, p_mid1);
  dir.normalize();

  let L1 = rPrime.pos.copy();
  let L2 = p5.Vector.add(rPrime.pos, dir);

  let p1prime = lineIntersection(current, p_mid1, L1, L2);
  let p2prime = lineIntersection(current, p_mid2, L1, L2);

  if (!p1prime || !p2prime) continue;

  let midpoint1 = p5.Vector.add(p1prime, rPrime.pos).mult(0.5);
  let midpoint2 = p5.Vector.add(p2prime, rPrime.pos).mult(0.5);

  if (p5.Vector.dist(current, midpoint1) <
      p5.Vector.dist(current, midpoint2)) {
    moveTarget = midpoint1;
  } else {
    moveTarget = midpoint2;
  }
}

    moves.push({ robot: robot, newPos: moveTarget });
  }

  // Apply moves synchronously
  for (let m of moves) {
   m.robot.pos = m.newPos;
}
}

// ---------- Geometry Helpers ----------

function cross(a,b,c) {
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}

function convexHull(points) {

  if (points.length < 3) return points;

  let hull = [];
  let leftmost = points[0];

  for (let p of points)
    if (p.x < leftmost.x) leftmost = p;

  let current = leftmost;

  do {
    hull.push(current);
    let next = points[0];

    for (let p of points) {
      if (p === current) continue;

      let val = cross(current, next, p);

      if (next === current || val < 0 ||
         (abs(val) < 0.001 &&
          p5.Vector.dist(current, p) >
          p5.Vector.dist(current, next)))
        next = p;
    }

    current = next;

  } while (p5.Vector.dist(current, leftmost) > 0.001);

  return hull;
}

function pointOnHull(p, hull) {
  for (let h of hull)
    if (p5.Vector.dist(h, p) < 0.001) return true;
  return false;
}

function pointInTriangle(p, a, b, c) {

  let area = abs(cross(a,b,c));
  let area1 = abs(cross(p,b,c));
  let area2 = abs(cross(a,p,c));
  let area3 = abs(cross(a,b,p));

  return abs(area - (area1+area2+area3)) < 0.5;
}

function noThreeCollinear() {

  for (let i = 0; i < robots.length; i++) {
    for (let j = i + 1; j < robots.length; j++) {
      for (let k = j + 1; k < robots.length; k++) {

        if (abs(cross(
          robots[i].pos,
          robots[j].pos,
          robots[k].pos
        )) < 0.01) {

          return false;
        }
      }
    }
  }

  return true;
}


function contractionFinished() {

  let hull = convexHull(robots.map(r => r.pos));

  // Condition 1: all robots are on hull
  if (hull.length !== robots.length)
    return false;

  // Condition 2: no collinear triple
  if (!noThreeCollinear())
    return false;

  return true;
}


function mouseWheel(event) {
  zoomLevel *= (event.delta > 0) ? 0.9 : 1.1;
  return false;
}


function drawStatus() {

  let hull = convexHull(robots.map(r => r.pos));

  fill(0);
  textSize(16);
  noStroke();

  text("Total Robots: " + robots.length, 20, height - 80);
  text("Hull Size: " + hull.length, 20, height - 60);

  if (hull.length === robots.length && noThreeCollinear()) {
    text("Status: ALL CORNERS ✓", 20, height - 40);
  } else {
    text("Status: CONTRACTING...", 20, height - 40);
  }

  let area = hullArea(hull);
    text("Hull Area: " + area.toFixed(2), 20, height - 20);

    let progress = (hull.length / robots.length * 100).toFixed(1);
    text("Corner Percentage: " + progress + "%", 20, height - 100);
}


function keyPressed() {

  if (key === 'W') {
    stretchY *= 1.5;   // stretch vertically
  }

  if (key === 'S') {
    stretchY /= 1.5;   // restore
  }
}


function hullArea(hull) {
  let area = 0;
  for (let i = 0; i < hull.length; i++) {
    let j = (i + 1) % hull.length;
    area += hull[i].x * hull[j].y;
    area -= hull[j].x * hull[i].y;
  }
  return abs(area / 2);
}


function lineIntersection(p1, p2, p3, p4) {

  let x1 = p1.x, y1 = p1.y;
  let x2 = p2.x, y2 = p2.y;
  let x3 = p3.x, y3 = p3.y;
  let x4 = p4.x, y4 = p4.y;

  let denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
  if (abs(denom) < 0.000001) return null;

  let px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom;
  let py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom;

  return createVector(px, py);
}