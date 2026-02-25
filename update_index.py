import re

with open('index.html', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update CSS
css_old = """        .control-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }"""
css_new = """        .control-panel {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            margin-top: 4px;
        }
        .btn-main {
            grid-column: 1 / -1;
            padding: 16px;
            font-size: 1.25rem;
            border-radius: 12px;
            background: linear-gradient(135deg, #00f3ff 0%, #0088ff 100%);
            color: #0f172a;
            font-family: 'Orbitron', sans-serif;
            font-weight: 700;
            border: none;
            box-shadow: 0 4px 15px rgba(0, 243, 255, 0.4);
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-main:active {
            transform: scale(0.97);
        }
        .btn-main.running {
            background: linear-gradient(135deg, #ef4444 0%, #b91c1c 100%);
            box-shadow: 0 4px 15px rgba(239, 68, 68, 0.4);
            color: white;
        }"""
content = content.replace(css_old, css_new)

# Sub-metrics CSS
css_sub_old = """        .sub-metrics {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 8px;
            margin-top: 16px;
            padding-top: 12px;
            border-top: 1px solid #334155;
        }"""
css_sub_new = """        .sub-metrics {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 8px;
            margin-top: 8px; /* compact */
            padding-top: 8px;
            border-top: 1px solid #334155;
        }"""
content = content.replace(css_sub_old, css_sub_new)

css_metric_old = """        .metric-card {
            background: var(--surface);
            border-radius: 16px;
            padding: 16px;
            /* Compact */
            text-align: center;
"""
css_metric_new = """        .metric-card {
            background: var(--surface);
            border-radius: 16px;
            padding: 12px; /* Ultra compact */
            margin-bottom: 4px;
            text-align: center;
"""
content = content.replace(css_metric_old, css_metric_new)

# Graph height
css_graph_old = """        .graph-container {
            background: var(--surface);
            border-radius: 16px;
            padding: 12px;
            border: 1px solid #334155;
            height: 200px;
            /* Fixed height */
            position: relative;
        }"""
css_graph_new = """        .graph-container {
            background: var(--surface);
            border-radius: 16px;
            padding: 8px;
            border: 1px solid #334155;
            height: 160px; /* Reduced fixed height */
            position: relative;
        }"""
content = content.replace(css_graph_old, css_graph_new)

# Header adjust
css_container_old = """        .container {
            width: 100%;
            max-width: 600px;
            padding: 10px;
            /* Reduced padding */
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 12px;
            /* Reduced gap */
        }"""
css_container_new = """        .container {
            width: 100%;
            max-width: 600px;
            padding: 10px;
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 8px; /* Extremely reduced gap */
        }"""
content = content.replace(css_container_old, css_container_new)

# 2. Setup Modal
pickers_row_old = """            <div class="pickers-row">
                <!-- Exercise Picker -->
                <div class="picker-wrap">
                    <div class="picker-label">種目</div>
                    <div class="picker-container">
                        <div class="picker-selection-band"></div>
                        <div class="picker-scroll" id="exercise-picker">
                            <div class="picker-item" data-value="SQ">SQ</div>
                            <div class="picker-item" data-value="DL">DL</div>
                            <div class="picker-item" data-value="BP">BP</div>
                            <div class="picker-item" data-value="CL">CL</div>
                            <div class="picker-item" data-value="Other">Other</div>
                        </div>
                    </div>
                    <div style="font-size:0.7rem;color:var(--text-dim);" id="exercise-label-hint">スクワット</div>
                </div>
                <!-- Weight Picker -->
                <div class="picker-wrap">
                    <div class="picker-label">重量</div>
                    <div class="picker-container">
                        <div class="picker-selection-band"></div>
                        <div class="picker-scroll" id="weight-picker">
                            <!-- Populated by JS: 0-300 step 2.5 -->
                        </div>
                    </div>
                    <div style="font-size:0.7rem;color:var(--text-dim);">kg</div>
                </div>
            </div>"""
pickers_row_new = """            <div class="pickers-row">
                <!-- Exercise Picker -->
                <div class="picker-wrap" style="flex:0.8;">
                    <div class="picker-label">種目</div>
                    <div class="picker-container">
                        <div class="picker-selection-band"></div>
                        <div class="picker-scroll" id="exercise-picker">
                            <div class="picker-item" data-value="SQ">SQ</div>
                            <div class="picker-item" data-value="DL">DL</div>
                            <div class="picker-item" data-value="BP">BP</div>
                            <div class="picker-item" data-value="CL">CL</div>
                            <div class="picker-item" data-value="Other">Oth</div>
                        </div>
                    </div>
                    <div style="font-size:0.7rem;color:var(--text-dim);" id="exercise-label-hint">スクワット</div>
                </div>
                <!-- Weight Picker -->
                <div class="picker-wrap">
                    <div class="picker-label">重量</div>
                    <div class="picker-container">
                        <div class="picker-selection-band"></div>
                        <div class="picker-scroll" id="weight-picker">
                        </div>
                    </div>
                    <div style="font-size:0.7rem;color:var(--text-dim);">kg</div>
                </div>
                <!-- Target Reps Picker -->
                <div class="picker-wrap" style="flex:0.8;">
                    <div class="picker-label">目標回数</div>
                    <div class="picker-container">
                        <div class="picker-selection-band"></div>
                        <div class="picker-scroll" id="reps-picker">
                        </div>
                    </div>
                    <div style="font-size:0.7rem;color:var(--text-dim);">reps</div>
                </div>
            </div>"""
content = content.replace(pickers_row_old, pickers_row_new)

# 3. Main container HTML replacement
# Replace from <!-- Session Setup --> to <div class="log-container">
container_html_old_pattern = r'<!-- Session Setup -->.*?(?=<div class="log-container">)'
container_html_new = """<!-- Status Bar -->
        <div style="display: flex; justify-content: space-between; font-size: 0.8rem; color: var(--text-dim); align-items: center;">
            <div id="current-setup-info">SET START で設定してください</div>
            <div>
                通知音:
                <select id="sound-select" class="mode-select" style="font-size: 0.75rem; padding: 2px 4px;">
                    <option value="BELL">ベル</option>
                    <option value="GAME">ゲーム</option>
                    <option value="BEEP">電子音</option>
                    <option value="MECHANICAL">機械音</option>
                    <option value="OFF">なし</option>
                </select>
            </div>
        </div>

        <!-- Main Metric ID for JS reference -->
        <div id="metric-card" class="metric-card">
            <div class="metric-label" style="font-size: 0.75rem;">リアルタイム・ピーク速度 (m/s)</div>
            <div class="metric-value-container">
                <span id="velocity-display" class="metric-value" style="font-size: 5rem;">0.00</span>
            </div>

            <div id="rep-change" class="rep-change">-</div>

            <!-- Alerts -->
            <div id="alert-banner" class="alert-banner">パワーゾーン圏外</div>

            <div class="sub-metrics">
                <div class="sub-metric">
                    <span class="label">回数</span>
                    <span id="rep-count" class="value">0 / --</span>
                </div>
                <div class="sub-metric">
                    <span class="label">1Rep Max</span>
                    <span id="last-rep-max" class="value">0.00</span>
                </div>
                <div class="sub-metric">
                    <span class="label">対ベスト疲労度</span>
                    <span id="fatigue-val" class="value fatigue-value">0%</span>
                </div>
            </div>
        </div>

        <div class="graph-container">
            <canvas id="vbtCanvas"></canvas>
        </div>

        <div class="control-panel">
            <button id="main-start-btn" class="btn-main" onclick="handleStartSet()">▶ SET START</button>
            <button id="connect-btn" class="btn"><span>⚡</span> 接続</button>
            <button class="btn" onclick="resetMax()"><span>🔄</span> リセット</button>
            <button class="btn" onclick="copyToClipboard(false)"><span>📋</span> CSV</button>
        </div>
        
        """
content = re.sub(container_html_old_pattern, container_html_new, content, flags=re.DOTALL)

# 4. JavaScript Logic global variables injection
js_globals_old = r"let currentMode = 'POWER';"
js_globals_new = """// --- Application Set State ---
        let currentExercise = 'SQ';
        let currentWeight = 100;
        let setTargetReps = 10;
        let isSetRunning = false;
        
        let currentMode = 'POWER';"""
content = re.sub(js_globals_old, js_globals_new, content)

js_getthresh_old = """        function getExerciseThresholds() {
            const ex = document.getElementById('exercise-name').value || 'Other';
            return EXERCISE_THRESHOLDS[ex] || EXERCISE_THRESHOLDS['Other'];
        }"""
js_getthresh_new = """        function getExerciseThresholds() {
            return EXERCISE_THRESHOLDS[currentExercise] || EXERCISE_THRESHOLDS['Other'];
        }"""
content = content.replace(js_getthresh_old, js_getthresh_new)

js_reset_old = """            document.getElementById('rep-count').innerText = "0";
            document.getElementById('set-best').innerText = "0.00";"""
js_reset_new = """            document.getElementById('rep-count').innerText = `0 / ${setTargetReps}`;
            document.getElementById('last-rep-max').innerText = "0.00";"""
content = content.replace(js_reset_old, js_reset_new)

js_startset_new_funcs = """
        function handleStartSet() {
            if (isSetRunning) {
                // End gracefully
                endSet("STOP BOTTON PRESSED");
            } else {
                // Show modal
                document.getElementById('startup-modal').style.display = 'flex';
                document.getElementById('startup-modal').style.opacity = '1';
            }
        }
        
        function endSet(reason) {
            isSetRunning = false;
            // Stop recording
            isRawRecording = false;
            
            // Update UI
            const btn = document.getElementById('main-start-btn');
            btn.innerHTML = '▶ SET START';
            btn.classList.remove('running');
            
            showSummary(reason);
        }
"""
content = content.replace("// UI Helpers", js_startset_new_funcs + "\n        // UI Helpers")

js_reprecord_old = """function toggleRawRecording() {"""
js_reprecord_new = """// Auto recording wrapper
        function startAutoRecording() {
            isRawRecording = true;
            rawLogBuffer = [];
            // Remove countdown overlay usage since we want instant start
        }

        function toggleRawRecording() {"""
content = content.replace(js_reprecord_old, js_reprecord_new)


js_adjustrep_old_pattern = r"window\.adjustRep.*?}"
js_adjustrep_new = """// Manual rep adjust removed from UI, kept in code just in case
        window.adjustRep = function (delta) {
            repCountInSet += delta;
            if (repCountInSet < 0) repCountInSet = 0;
            document.getElementById('rep-count').innerText = `${repCountInSet} / ${setTargetReps}`;
        }"""
content = re.sub(js_adjustrep_old_pattern, js_adjustrep_new, content, flags=re.DOTALL)


js_clipboard_old_pattern = r"(const exercise = document\.getElementById\('exercise-name'\)\.value \|\| \"NoName\";\s*const load = document\.getElementById\('weight-input'\)\.value \|\| \"0\";)"
js_clipboard_new = """const exercise = currentExercise || "NoName";
            const load = currentWeight || "0";"""
content = re.sub(js_clipboard_old_pattern, js_clipboard_new, content)

# 5. Injection into BLE Rep logic
# In "Rep Validated" block
rep_valid_old = """                                        repCountInSet++;
                                        setRepData.push(currentPeak);
                                        document.getElementById('rep-count').innerText = repCountInSet;"""
rep_valid_new = """                                        repCountInSet++;
                                        setRepData.push(currentPeak);
                                        document.getElementById('rep-count').innerText = `${repCountInSet} / ${setTargetReps}`;
                                        document.getElementById('last-rep-max').innerText = currentPeak.toFixed(2);"""
content = content.replace(rep_valid_old, rep_valid_new)

# In Judgment section
judgment_old = """                                        if (isCriticalDrop) {
                                            status = 'stop';
                                            stopReason = "CRITICAL VELOCITY DROP";
                                        } else if (fatigue >= threshold + 5) {
                                            status = 'stop';
                                            stopReason = "FATIGUE LIMIT EXCEEDED";
                                        } else if (fatigue >= threshold) {
                                            consecutiveWarningCount++;
                                            if (consecutiveWarningCount >= 2) {
                                                status = 'stop';
                                                stopReason = "FATIGUE THRESHOLD (x2)";
                                            } else {
                                                status = 'warning';
                                            }
                                        } else if (fatigue >= 15) {
                                            status = 'warning';
                                            consecutiveWarningCount = 0;
                                        } else {
                                            status = 'good';
                                            consecutiveWarningCount = 0;
                                        }

                                        updateStatusColor(status);

                                        if (status === 'stop') {
                                            toneGen.playStop();
                                            showSummary(stopReason);
                                        } else if (status === 'warning') {
                                            toneGen.playWarning();
                                        } else {
                                            toneGen.playSuccess();
                                        }"""

judgment_new = """                                        if (isCriticalDrop) {
                                            status = 'stop';
                                            stopReason = "速度低下によりセット終了（安全装置）";
                                        } else if (fatigue >= threshold + 5) {
                                            status = 'stop';
                                            stopReason = "目標疲労度オーバー（セット終了）";
                                        } else if (fatigue >= threshold) {
                                            consecutiveWarningCount++;
                                            if (consecutiveWarningCount >= 2) {
                                                status = 'stop';
                                                stopReason = "疲労度閾値に連続到達（セット終了）";
                                            } else {
                                                status = 'warning';
                                            }
                                        } else if (fatigue >= 15) {
                                            status = 'warning';
                                            consecutiveWarningCount = 0;
                                        } else {
                                            status = 'good';
                                            consecutiveWarningCount = 0;
                                        }
                                        
                                        // Also check target reps
                                        if (repCountInSet >= setTargetReps && status !== 'stop') {
                                            status = 'stop';
                                            stopReason = "目標回数達成（セット完了）";
                                        }

                                        updateStatusColor(status);

                                        if (status === 'stop') {
                                            toneGen.playStop();
                                            endSet(stopReason);
                                        } else if (status === 'warning') {
                                            toneGen.playWarning();
                                        } else {
                                            toneGen.playSuccess();
                                        }"""
content = content.replace(judgment_old, judgment_new)

# Modal script changes
modal_script_old_pattern = r"// --- Weight picker.*?(?=// --- Generic picker)"
modal_script_new = """// --- Weight picker: 0, 2.5, 5 ... 300 ---
        (function buildWeightPicker() {
            const scroll = document.getElementById('weight-picker');
            scroll.innerHTML = '';
            for (let w = 0; w <= 300; w += 2.5) {
                const div = document.createElement('div');
                div.className = 'picker-item';
                div.dataset.value = w;
                div.innerHTML = `${w % 1 === 0 ? w : w.toFixed(1)}<span class="picker-suffix">kg</span>`;
                scroll.appendChild(div);
            }
        })();
        
        // --- Reps picker: 1 to 30 ---
        (function buildRepsPicker() {
            const scroll = document.getElementById('reps-picker');
            scroll.innerHTML = '';
            for (let r = 1; r <= 30; r++) {
                const div = document.createElement('div');
                div.className = 'picker-item';
                div.dataset.value = r;
                div.innerHTML = `${r}`;
                scroll.appendChild(div);
            }
        })();

"""
content = re.sub(modal_script_old_pattern, modal_script_new, content, flags=re.DOTALL)

init_weight_picker_old = "initPicker(document.getElementById('weight-picker'), null, 24);"
init_weight_picker_new = "initPicker(document.getElementById('weight-picker'), null, 24);\n        initPicker(document.getElementById('reps-picker'), null, 9); // default 10 reps (index 9)"
content = content.replace(init_weight_picker_old, init_weight_picker_new)

confirm_setup_old = """        function confirmSetup() {
            // Get active exercise
            const exActive = document.querySelector('#exercise-picker .picker-item.active');
            const wtActive = document.querySelector('#weight-picker .picker-item.active');

            if (exActive) {
                document.getElementById('exercise-name').value = exActive.dataset.value;
            }
            if (wtActive) {
                const wt = parseFloat(wtActive.dataset.value);
                // Select nearest option in weight-input select
                const weightSel = document.getElementById('weight-input');
                let closest = null, minDiff = Infinity;
                [...weightSel.options].forEach(opt => {
                    const diff = Math.abs(parseFloat(opt.value) - wt);
                    if (diff < minDiff) { minDiff = diff; closest = opt.value; }
                });
                if (closest !== null) weightSel.value = closest;
            }

            // Close modal with fade-out
            const modal = document.getElementById('startup-modal');
            modal.style.transition = 'opacity 0.3s ease';
            modal.style.opacity = '0';
            setTimeout(() => { modal.style.display = 'none'; }, 300);
        }"""
        
confirm_setup_new = """        function confirmSetup() {
            const exActive = document.querySelector('#exercise-picker .picker-item.active');
            const wtActive = document.querySelector('#weight-picker .picker-item.active');
            const repActive = document.querySelector('#reps-picker .picker-item.active');

            if (exActive) {
                currentExercise = exActive.dataset.value;
            }
            if (wtActive) {
                currentWeight = parseFloat(wtActive.dataset.value);
            }
            if (repActive) {
                setTargetReps = parseInt(repActive.dataset.value);
            }
            
            // Update UI info
            const modeName = document.getElementById('mode-select').options[document.getElementById('mode-select').selectedIndex].text;
            document.getElementById('current-setup-info').innerText = `${EXERCISE_LABELS[currentExercise] || currentExercise} / ${currentWeight}kg / 目標 ${setTargetReps} reps`;

            // Start Set Process
            isSetRunning = true;
            resetSetState();
            startAutoRecording();
            
            // Adjust UI
            const btn = document.getElementById('main-start-btn');
            btn.innerHTML = '⏹ STOP SET';
            btn.classList.add('running');

            // Close modal
            const modal = document.getElementById('startup-modal');
            modal.style.transition = 'opacity 0.3s ease';
            modal.style.opacity = '0';
            setTimeout(() => { modal.style.display = 'none'; }, 300);
        }"""
content = content.replace(confirm_setup_old, confirm_setup_new)

# Remove unused element code
content = content.replace("const weightSelect = document.getElementById('weight-input');", "// NOOP weightSelect")
content = content.replace("const weightInput = document.getElementById('weight-input');", "// NOOP weightInput")

# Delete the specific options loop
content = re.sub(r"for \(let w = 50; w <= 150; w \+= 5\) \{.*?\}", "", content, flags=re.DOTALL)

with open('index.html', 'w', encoding='utf-8') as f:
    f.write(content)
print("Updated index.html successfully.")
