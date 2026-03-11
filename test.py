import sys
import csv
from datetime import datetime
import io

data = """Exercise,Weight(kg),Time,Velocity(m/s)
SQ,60,2026-03-11T14:09:01.753Z,0.000
SQ,60,2026-03-11T14:09:02.594Z,0.000
SQ,60,2026-03-11T14:09:03.284Z,0.000
SQ,60,2026-03-11T14:09:03.285Z,0.344
SQ,60,2026-03-11T14:09:03.389Z,1.863
SQ,60,2026-03-11T14:09:03.479Z,2.180
SQ,60,2026-03-11T14:09:03.579Z,2.027
SQ,60,2026-03-11T14:09:04.105Z,0.310
SQ,60,2026-03-11T14:09:04.455Z,0.089
SQ,60,2026-03-11T14:09:04.560Z,0.072
SQ,60,2026-03-11T14:09:04.648Z,0.000
SQ,60,2026-03-11T14:09:04.814Z,0.000
SQ,60,2026-03-11T14:09:04.919Z,0.000
SQ,60,2026-03-11T14:09:05.773Z,0.000
SQ,60,2026-03-11T14:09:05.803Z,0.048
SQ,60,2026-03-11T14:09:05.879Z,1.119
SQ,60,2026-03-11T14:09:05.955Z,1.827
SQ,60,2026-03-11T14:09:06.164Z,2.211
SQ,60,2026-03-11T14:09:06.270Z,1.669
SQ,60,2026-03-11T14:09:06.855Z,0.247
SQ,60,2026-03-11T14:09:06.929Z,0.224
SQ,60,2026-03-11T14:09:07.155Z,0.162
SQ,60,2026-03-11T14:09:07.261Z,0.000
SQ,60,2026-03-11T14:09:07.303Z,0.000
SQ,60,2026-03-11T14:09:08.189Z,0.000
SQ,60,2026-03-11T14:09:08.219Z,0.230
SQ,60,2026-03-11T14:09:08.313Z,1.176
SQ,60,2026-03-11T14:09:08.520Z,2.412
SQ,60,2026-03-11T14:09:09.167Z,0.267
SQ,60,2026-03-11T14:09:09.284Z,0.204
SQ,60,2026-03-11T14:09:09.418Z,0.126
SQ,60,2026-03-11T14:09:09.781Z,0.034
SQ,60,2026-03-11T14:09:09.811Z,0.000
SQ,60,2026-03-11T14:09:10.709Z,0.205
SQ,60,2026-03-11T14:09:11.039Z,2.248
SQ,60,2026-03-11T14:09:11.699Z,0.211
SQ,60,2026-03-11T14:09:12.164Z,0.023
SQ,60,2026-03-11T14:09:12.210Z,0.000"""

START_THRESHOLD = 0.20
END_THRESHOLD = 0.02
MIN_DURATION_MS = 100
LOCKOUT_MS = 500
minPeak = 0.15
maxVelocity = 5.0

isMoving = False
currentRepVelocities = []
moveStartTime = 0
lastRepTime = 0
belowThresholdStartTime = 0
currentPeak = 0
repCount = 0

f = io.StringIO(data)
reader = csv.reader(f)
next(reader, None)
for line in reader:
    try:
        time_str = line[2].replace('Z', '+00:00')
        now_dt = datetime.fromisoformat(time_str)
        now = int(now_dt.timestamp() * 1000)
        currentVel = float(line[3])
    except Exception as e:
        continue

    if currentVel > START_THRESHOLD:
        if not isMoving and (now - lastRepTime > LOCKOUT_MS):
            isMoving = True
            moveStartTime = now
            currentPeak = 0
            currentRepVelocities = []
            print(f'\n--- Started moving at {now_dt.time()} with vel {currentVel}')
        if isMoving:
            currentRepVelocities.append(currentVel)
            if currentVel > currentPeak:
                currentPeak = currentVel
            belowThresholdStartTime = 0
    elif isMoving and currentVel < END_THRESHOLD:
        currentRepVelocities.append(currentVel)
        if belowThresholdStartTime == 0:
            belowThresholdStartTime = now
            print(f'  below threshold started at {now_dt.time()} with vel {currentVel}')

        if now - belowThresholdStartTime > 200:
            isMoving = False
            belowThresholdStartTime = 0

            rawArray = currentRepVelocities if currentRepVelocities else [0]
            weakSmoothed = []
            for i in range(len(rawArray)):
                if i == 0:
                    weakSmoothed.append(rawArray[i])
                else:
                    weakSmoothed.append((rawArray[i] + rawArray[i - 1]) / 2)

            strongSmoothed = []
            for i in range(len(rawArray)):
                s = 0
                count = 0
                for j in range(max(0, i - 4), i + 1):
                    s += rawArray[j]
                    count += 1
                strongSmoothed.append(s / count)

            maxStrongVel = 0
            peakIndex = 0
            for i in range(len(strongSmoothed)):
                if strongSmoothed[i] > maxStrongVel:
                    maxStrongVel = strongSmoothed[i]
                    peakIndex = i
            
            startIndex = 0
            for i in range(peakIndex + 1):
                if weakSmoothed[i] > 0.05:
                    startIndex = i
                    break
            
            mpvSum = 0
            mpvCount = 0
            repPeak = 0
            for i in range(len(weakSmoothed)):
                if weakSmoothed[i] > repPeak:
                    repPeak = weakSmoothed[i]
                if i >= startIndex and i <= peakIndex:
                    mpvSum += weakSmoothed[i]
                    mpvCount += 1
            
            duration = now - moveStartTime
            
            print(f'  Evaluating rep. duration: {duration}, peak: {repPeak}')

            if minPeak <= repPeak <= maxVelocity and duration > MIN_DURATION_MS:
                lastRepTime = now
                repCount += 1
                print(f'[VALID REP {repCount}] peak: {repPeak:.2f}')
            else:
                print(f'[REJECTED REP] peak: {repPeak:.2f}, duration: {duration}')
