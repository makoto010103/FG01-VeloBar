const fs = require('fs');
const readline = require('readline');

let isMoving = false;
let currentRepData = [];
let lastRepTime = 0;
let repAttemptCount = 0;
let lastBelowEndTime = 0;

function processRep(velocity, now) {
    if (!isMoving && (now - lastRepTime > 500)) {
        if (velocity >= 0.20) {
            isMoving = true;
            currentRepData = [{ timestamp: now, velocity: velocity }];
        }
    } else if (isMoving) {
        currentRepData.push({ timestamp: now, velocity: velocity });

        if (velocity <= 0.02) {
            if (lastBelowEndTime === 0) lastBelowEndTime = now;
        } else {
            lastBelowEndTime = 0;
        }

        if ((lastBelowEndTime > 0 && (now - lastBelowEndTime >= 200)) || velocity <= 0.0) {
            if (currentRepData.length > 3) {
                const duration = currentRepData[currentRepData.length - 1].timestamp - currentRepData[0].timestamp;
                let peak = 0;
                for (let d of currentRepData) {
                    if (d.velocity > peak) peak = d.velocity;
                }
                
                if (duration >= 100) {
                    repAttemptCount++;
                    if (peak >= 0.15 && peak <= 5.0) {
                        lastRepTime = now;
                        console.log(`[VALID] Peak: ${peak}, Dur: ${duration}`);
                    } else {
                        console.log(`[REJECT bounds] Peak: ${peak}, Dur: ${duration}`);
                    }
                } else {
                    console.log(`[REJECT short] Peak: ${peak}, Dur: ${duration}`);
                }
            }
            isMoving = false;
            currentRepData = [];
            lastBelowEndTime = 0;
        }
    }
}

async function runTest() {
    const rl = readline.createInterface({ 
        input: fs.createReadStream('data.csv'), 
        crlfDelay: Infinity 
    });
    for await (const line of rl) {
        if (line.startsWith('Exercise')) continue;
        const pts = line.split(',');
        processRep(parseFloat(pts[3]), new Date(pts[2]).getTime());
    }
}
runTest();
