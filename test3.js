const fs = require('fs');

const lines = fs.readFileSync('rep_data.csv', 'utf8').split('\n');

const START_THRESHOLD = 0.20;
const END_THRESHOLD = 0.02;
const MIN_DURATION_MS = 100;
const LOCKOUT_MS = 500;
const minPeak = 0.15;
const maxVelocity = 5.0;

let isMoving = false;
let currentRepVelocities = [];
let moveStartTime = 0;
let lastRepTime = 0;
let belowThresholdStartTime = 0;
let currentPeak = 0;

let repCount = 0;

for (let line of lines) {
    if(!line || line.startsWith('Exercise')) continue;
    const pts = line.split(',');
    const now = new Date(pts[2]).getTime();
    if (isNaN(now)) continue;
    
    // Simulate isSetRunning = true, isCountingDown = false
    const currentVel = parseFloat(pts[3]);

    if (currentVel > START_THRESHOLD) {
        if (!isMoving && (now - lastRepTime > LOCKOUT_MS)) {
            isMoving = true;
            moveStartTime = now;
            currentPeak = 0;
            currentRepVelocities = [];
            console.log(`\n--- Started moving at ${new Date(now).toISOString().substring(11,23)} with vel ${currentVel}`);
        }
        if (isMoving) {
            currentRepVelocities.push(currentVel);
            if (currentVel > currentPeak) {
                currentPeak = currentVel;
            }
            belowThresholdStartTime = 0;
        }
    } else if (isMoving && currentVel < END_THRESHOLD) {
        currentRepVelocities.push(currentVel);
        if (belowThresholdStartTime === 0) {
            belowThresholdStartTime = now;
            console.log(`  below threshold started at ${new Date(now).toISOString().substring(11,23)} with vel ${currentVel}`);
        }

        if (now - belowThresholdStartTime > 200) {
            isMoving = false;
            belowThresholdStartTime = 0;

            let repPeak = currentPeak; // simplified for test
            const duration = now - moveStartTime;
            
            console.log(`  Evaluating rep. duration: ${duration}, peak: ${repPeak}`);

            if (repPeak >= minPeak && repPeak <= maxVelocity && duration > MIN_DURATION_MS) {
                lastRepTime = now;
                repCount++;
                console.log(`[VALID REP ${repCount}] peak: ${repPeak.toFixed(2)}, start: ${new Date(moveStartTime).toISOString().substring(11,23)}`);
            } else {
                console.log(`[REJECTED REP] peak: ${repPeak.toFixed(2)}, duration: ${duration}`);
            }
        }
    }
}
