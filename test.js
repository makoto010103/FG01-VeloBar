const fs = require('fs');

const lines = fs.readFileSync('full_data.csv', 'utf8').split('\n');
console.log(`Loaded ${lines.length} lines.`);

// Replicate index.html rep logic exact thresholds
const START_THRESHOLD = 0.20;
const END_THRESHOLD = 0.02;
const MIN_DURATION_MS = 100;
const LOCKOUT_MS = 500;
const minPeak = 0.15;
const maxVelocity = 5.0;

let isMoving = false;
let currentRepData = [];
let setRepData = [];
let lastRepTime = 0;
let repAttemptCount = 0;
let lastBelowEndTime = 0;

for (let line of lines) {
    if(!line || line.startsWith('Exercise')) continue;
    const pts = line.split(',');
    
    const now = new Date(pts[2]).getTime();
    if (isNaN(now)) continue;

    const velocity = parseFloat(pts[3]);

    if (!isMoving && (now - lastRepTime > LOCKOUT_MS)) {
        if (velocity >= START_THRESHOLD) {
            isMoving = true;
            currentRepData = [{ timestamp: now, velocity }];
        }
    } else if (isMoving) {
        currentRepData.push({ timestamp: now, velocity });

        if (velocity <= END_THRESHOLD) {
            if (lastBelowEndTime === 0) lastBelowEndTime = now;
        } else {
            lastBelowEndTime = 0; // reset if slightly went up
        }

        if ((lastBelowEndTime > 0 && (now - lastBelowEndTime >= 200)) || velocity <= 0.0) {
            if (currentRepData.length > 3) {
                const duration = currentRepData[currentRepData.length - 1].timestamp - currentRepData[0].timestamp;
                let peak = 0;
                for (let d of currentRepData) {
                    if (d.velocity > peak) peak = d.velocity;
                }
                
                if (duration >= MIN_DURATION_MS) {
                    repAttemptCount++;
                    if (peak >= minPeak && peak <= maxVelocity) {
                        lastRepTime = now;
                        console.log(`[VALID REP ${repAttemptCount}] Peak: ${peak.toFixed(2)}, StartTime: ${new Date(currentRepData[0].timestamp).toISOString().substring(11,23)}`);
                    } else {
                        console.log(`[REJECT Peak] ${peak.toFixed(2)} at ${new Date(currentRepData[0].timestamp).toISOString().substring(11,23)}`);
                    }
                } else {
                    console.log(`[REJECT Short] Dur: ${duration}ms at ${new Date(currentRepData[0].timestamp).toISOString().substring(11,23)}`);
                }
            }
            isMoving = false;
            currentRepData = [];
            lastBelowEndTime = 0;
        }
    }
}
