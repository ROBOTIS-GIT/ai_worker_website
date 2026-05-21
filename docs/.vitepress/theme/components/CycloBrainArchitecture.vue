<template>
  <section class="cyclo-brain-architecture" aria-labelledby="cyclo-brain-architecture-title">
    <div class="architecture-header">
      <div>
        <p id="cyclo-brain-architecture-title" class="architecture-title">
          Cyclo Brain System Architecture
        </p>
        <p class="architecture-summary">
          Cyclo Brain separates command handling, model inference, open-source policy backends, and
          robot I/O into distinct runtime areas. The diagram below shows the system blocks first;
          detailed data flow is summarized after the diagram.
        </p>
      </div>
      <div class="legend" aria-label="Flow legend">
        <span><i class="dot command-dot"></i>external command</span>
        <span><i class="dot engine-dot"></i>engine request / action_list</span>
        <span><i class="dot robot-dot"></i>sensor / state read</span>
        <span><i class="dot action-dot"></i>robot command publish</span>
        <span><i class="dot backend-dot"></i>open-source backend</span>
      </div>
    </div>

    <div class="diagram-stage" aria-label="Cyclo Brain architecture map">
      <svg class="flow-overlay desktop-flow" viewBox="0 0 1000 1120" preserveAspectRatio="none" aria-hidden="true">
        <defs>
          <marker id="arch-arrow-command" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-arrow-main" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-arrow-engine" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-arrow-robot" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-arrow-action" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-arrow-backend" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
        </defs>

        <path class="flow-line command-line dashed" d="M 790 112 C 720 160 390 152 172 226" marker-end="url(#arch-arrow-command)" />
        <path class="flow-line main-line" d="M 215 318 L 231 318" marker-end="url(#arch-arrow-main)" />
        <path class="flow-line main-line" d="M 321 356 C 320 388 250 382 240 404" marker-end="url(#arch-arrow-main)" />
        <path class="flow-line engine-line dashed" d="M 412 448 C 500 430 500 330 588 329" marker-end="url(#arch-arrow-engine)" />
        <path class="flow-line engine-line dashed" d="M 588 546 C 520 546 500 598 412 598" marker-end="url(#arch-arrow-engine)" />
        <path class="flow-line main-line" d="M 223 645 C 223 700 124 690 124 740" marker-end="url(#arch-arrow-main)" />
        <path class="flow-line action-line" d="M 215 779 L 231 779" marker-end="url(#arch-arrow-action)" />
        <path class="flow-line backend-line" d="M 425 880 C 440 780 600 720 650 656" marker-end="url(#arch-arrow-backend)" />
        <path class="flow-line backend-line" d="M 650 880 C 650 780 720 720 740 656" marker-end="url(#arch-arrow-backend)" />
        <path class="flow-line backend-line" d="M 875 880 C 850 780 815 720 830 656" marker-end="url(#arch-arrow-backend)" />
        <path class="flow-line robot-line" d="M 790 740 C 790 675 805 620 805 556" marker-end="url(#arch-arrow-robot)" />
        <path class="flow-line robot-line dashed" d="M 250 1049 C 360 980 610 940 790 818" marker-end="url(#arch-arrow-robot)" />
        <path class="flow-line action-line dashed" d="M 321 818 C 445 920 610 1010 750 1049" marker-end="url(#arch-arrow-action)" />
      </svg>

      <svg class="flow-overlay mobile-flow" viewBox="0 0 1000 1750" preserveAspectRatio="none" aria-hidden="true">
        <defs>
          <marker id="arch-mobile-command" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-mobile-engine" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-mobile-robot" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-mobile-action" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
          <marker id="arch-mobile-backend" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" />
          </marker>
        </defs>

        <path class="flow-line command-line dashed" d="M 500 250 C 500 292 500 312 500 350" marker-end="url(#arch-mobile-command)" />
        <path class="flow-line engine-line dashed" d="M 500 587 C 500 735 500 835 500 948" marker-end="url(#arch-mobile-engine)" />
        <path class="flow-line engine-line dashed" d="M 500 1110 C 245 1030 245 705 500 620" marker-end="url(#arch-mobile-engine)" />
        <path class="flow-line robot-line" d="M 500 1230 C 500 1210 500 1200 500 1186" marker-end="url(#arch-mobile-robot)" />
        <path class="flow-line backend-line" d="M 500 1304 C 500 1268 500 1240 500 1214" marker-end="url(#arch-mobile-backend)" />
        <path class="flow-line robot-line dashed" d="M 260 1574 C 650 1500 780 1320 500 1260" marker-end="url(#arch-mobile-robot)" />
        <path class="flow-line action-line dashed" d="M 500 820 C 780 980 780 1400 740 1574" marker-end="url(#arch-mobile-action)" />
      </svg>

      <article class="zone host-zone">
        <p class="zone-label">Host</p>
        <div class="node host-source">
          <span class="lane-tag host-tag">Command Source</span>
          <strong>UI / Orchestrator</strong>
          <span>Standalone CLI args</span>
          <em>same command shape</em>
        </div>
        <div class="node host-command">
          <span class="lane-tag host-tag">InferenceCommand</span>
          <strong>LOAD / START / PAUSE</strong>
          <span>RESUME / STOP / UNLOAD</span>
          <em>external service</em>
        </div>
      </article>

      <section class="zone policy-zone">
        <p class="zone-label container-label">Policy Container: &lt;backend&gt;_server</p>

        <article class="process-panel main-panel">
          <p class="process-title">Main Runtime</p>
          <div class="node main-service">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>ServiceHandler</strong>
            <span>command entry</span>
          </div>
          <div class="node main-session">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>SessionState</strong>
            <span>runtime gate</span>
          </div>
          <div class="node main-requester">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>InferenceRequester</strong>
            <span>one GET_ACTION at a time</span>
            <span>timeout + seq_id stale guard</span>
          </div>
          <div class="node main-actions">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>ActionChunkProcessor</strong>
            <span>buffer + optional match / RTC</span>
            <span>interpolate / blend / smooth</span>
          </div>
          <div class="node main-loop">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>ControlLoop</strong>
            <span>self-running tick</span>
          </div>
          <div class="node main-robot-client">
            <span class="lane-tag main-tag">main-runtime</span>
            <strong>RobotClient</strong>
            <span>command output</span>
          </div>
        </article>

        <article class="process-panel engine-panel">
          <p class="process-title">Engine Process</p>
          <div class="node engine-worker">
            <span class="lane-tag engine-tag">engine-process</span>
            <strong>EngineWorker</strong>
            <span>hosts internal EngineCommand service</span>
            <span>using Zenoh ROS2 SDK</span>
          </div>
          <div class="node engine-backend">
            <span class="lane-tag engine-tag">engine-process</span>
            <strong>Backend InferenceEngine</strong>
            <span>model-specific implementation</span>
            <span>behind stable contract</span>
            <div class="subnode-grid" aria-label="Backend inference stages">
              <span>Policy Load</span>
              <span>Preprocess</span>
              <span>Predict</span>
              <span>Optional Optimize</span>
            </div>
          </div>
          <div class="node engine-robot-client">
            <span class="lane-tag engine-tag">engine-process</span>
            <strong>RobotClient</strong>
            <span>observation only</span>
          </div>
        </article>

        <div class="backend-island" aria-label="Open-source backend island">
          <span class="backend-label">Open-source backend island</span>
          <span>LeRobot</span>
          <span>GR00T</span>
          <span>Future model</span>
        </div>
      </section>

      <article class="zone robot-zone">
        <p class="zone-label">Robot</p>
        <div class="node robot-sensors">
          <span class="lane-tag robot-tag">Sensors / State</span>
          <strong>Observation input</strong>
          <span>camera, joints, base state</span>
        </div>
        <div class="node robot-command">
          <span class="lane-tag robot-tag">Command Topics</span>
          <strong>Command output</strong>
          <span>cmd_vel, trajectory, etc.</span>
        </div>
      </article>

      <span class="flow-label label-service command-flow">service call</span>
      <span class="flow-label label-engine engine-flow">EngineCommand</span>
      <span class="flow-label label-action-list engine-flow">action_list</span>
      <span class="flow-label label-processed main-flow">processed actions</span>
      <span class="flow-label label-tick action-flow">one action / tick</span>
      <span class="flow-label label-observation robot-flow">sensor / state read</span>
      <span class="flow-label label-observation-policy robot-flow">observation to preprocess</span>
      <span class="flow-label label-command-out action-flow">command out</span>
      <span class="flow-label label-backend backend-flow">backend contract</span>
      <span class="flow-marker marker-1 command-marker">1</span>
      <span class="flow-marker marker-2 engine-marker">2</span>
      <span class="flow-marker marker-3 robot-marker">3</span>
      <span class="flow-marker marker-4 engine-marker">4</span>
      <span class="flow-marker marker-5 action-marker">5</span>
    </div>

    <ol class="flow-list" aria-label="Runtime data flow">
      <li><strong>UI/CLI command enters Main.</strong> The command reaches <code>main-runtime</code> through the external service.</li>
      <li><strong>Main requests Engine action.</strong> <code>InferenceRequester</code> sends one <code>EngineCommand</code> at a time.</li>
      <li><strong>Engine reads Robot observation.</strong> The engine-side <code>RobotClient</code> reads camera, joint, and base state.</li>
      <li><strong>Engine returns action_list.</strong> The backend policy loads the model, preprocesses observations, predicts, and optionally optimizes.</li>
      <li><strong>Main buffers/processes/publishes.</strong> <code>ActionChunkProcessor</code> can turn 16 actions into a 100 Hz buffer, and <code>ControlLoop</code> publishes one action per tick.</li>
    </ol>

    <p class="architecture-footer">
      Source of truth: Cyclo Brain runtime structure. When runtime shape changes, update this diagram with it.
    </p>
  </section>
</template>

<style scoped>
.cyclo-brain-architecture {
  --arch-paper: #ffffff;
  --arch-bg: #f6f8fb;
  --arch-ink: #152033;
  --arch-muted: #5d6878;
  --arch-line: #c8d2df;
  --arch-host: #2e5fa7;
  --arch-main: #087f8c;
  --arch-engine: #2f6f4e;
  --arch-robot: #9a6a1d;
  --arch-backend: #6554a8;
  --arch-action: #b34242;
  --arch-flow-opacity: 0.5;
  --arch-shadow: 0 10px 28px rgba(21, 32, 51, 0.08);
  margin: 1rem 0 1.5rem;
  padding: 16px;
  overflow: hidden;
  border: 1px solid var(--arch-line);
  border-radius: 8px;
  background: var(--arch-bg);
  color: var(--arch-ink);
}

.cyclo-brain-architecture *,
.cyclo-brain-architecture *::before,
.cyclo-brain-architecture *::after {
  box-sizing: border-box;
}

.architecture-header {
  display: grid;
  gap: 12px;
}

.architecture-title {
  margin: 0;
  color: var(--arch-ink);
  font-size: 1rem;
  font-weight: 700;
  line-height: 1.35;
}

.architecture-summary {
  margin: 0.35rem 0 0;
  color: var(--arch-muted);
  font-size: 0.86rem;
  line-height: 1.55;
}

.legend {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
}

.legend span {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  min-height: 28px;
  padding: 4px 8px;
  border: 1px solid var(--arch-line);
  border-radius: 999px;
  background: var(--arch-paper);
  color: var(--arch-ink);
  font-size: 0.68rem;
  font-weight: 700;
  line-height: 1.2;
  white-space: nowrap;
}

.dot {
  width: 9px;
  height: 9px;
  border-radius: 999px;
  background: var(--arch-host);
  flex: 0 0 auto;
}

.command-dot { background: var(--arch-host); }
.engine-dot { background: var(--arch-engine); }
.robot-dot { background: var(--arch-robot); }
.backend-dot { background: var(--arch-backend); }
.action-dot { background: var(--arch-action); }

.diagram-stage {
  position: relative;
  height: 1120px;
  margin-top: 14px;
}

.flow-overlay {
  position: absolute;
  inset: 0;
  z-index: 4;
  width: 100%;
  height: 100%;
  overflow: visible;
  pointer-events: none;
}

.mobile-flow {
  display: none;
}

.flow-line {
  fill: none;
  stroke-width: 2.5;
  stroke-opacity: var(--arch-flow-opacity);
  vector-effect: non-scaling-stroke;
}

.flow-line.dashed {
  stroke-dasharray: 9 10;
  stroke-linecap: round;
}

.command-line { stroke: var(--arch-host); marker-end: url(#arch-arrow-command); }
.main-line { stroke: var(--arch-main); marker-end: url(#arch-arrow-main); }
.engine-line { stroke: var(--arch-engine); marker-end: url(#arch-arrow-engine); }
.robot-line { stroke: var(--arch-robot); marker-end: url(#arch-arrow-robot); }
.action-line { stroke: var(--arch-action); marker-end: url(#arch-arrow-action); }
.backend-line { stroke: var(--arch-backend); marker-end: url(#arch-arrow-backend); }

#arch-arrow-command path,
#arch-mobile-command path {
  fill: var(--arch-host);
  fill-opacity: var(--arch-flow-opacity);
}
#arch-arrow-main path {
  fill: var(--arch-main);
  fill-opacity: var(--arch-flow-opacity);
}
#arch-arrow-engine path,
#arch-mobile-engine path {
  fill: var(--arch-engine);
  fill-opacity: var(--arch-flow-opacity);
}
#arch-arrow-robot path,
#arch-mobile-robot path {
  fill: var(--arch-robot);
  fill-opacity: var(--arch-flow-opacity);
}
#arch-arrow-action path,
#arch-mobile-action path {
  fill: var(--arch-action);
  fill-opacity: var(--arch-flow-opacity);
}
#arch-arrow-backend path,
#arch-mobile-backend path {
  fill: var(--arch-backend);
  fill-opacity: var(--arch-flow-opacity);
}

.zone,
.process-panel,
.node,
.backend-island {
  min-width: 0;
  border: 1px solid var(--arch-line);
  border-radius: 8px;
  background: var(--arch-paper);
}

.zone {
  position: absolute;
  box-shadow: var(--arch-shadow);
}

.host-zone {
  top: 0;
  left: 0;
  right: 0;
  height: 126px;
  border-color: rgba(46, 95, 167, 0.55);
}

.policy-zone {
  top: 170px;
  left: 0;
  right: 0;
  height: 800px;
  border-color: rgba(8, 127, 140, 0.45);
}

.robot-zone {
  top: 1005px;
  left: 0;
  right: 0;
  height: 115px;
  border-color: rgba(154, 106, 29, 0.58);
}

.zone-label,
.process-title {
  position: absolute;
  left: 14px;
  right: 14px;
  top: 10px;
  margin: 0;
  padding-bottom: 4px;
  border-bottom: 1px solid var(--arch-line);
  color: var(--arch-ink);
  font-size: 0.82rem;
  font-weight: 700;
  line-height: 1.3;
}

.container-label {
  color: var(--arch-main);
}

.process-title {
  font-size: 0.78rem;
}

.process-panel {
  position: absolute;
}

.main-panel {
  top: 56px;
  left: 16px;
  width: calc(43% - 16px);
  height: 620px;
  border-color: rgba(8, 127, 140, 0.55);
}

.engine-panel {
  top: 56px;
  right: 16px;
  width: calc(43% - 16px);
  height: 620px;
  border-color: rgba(47, 111, 78, 0.55);
}

.node {
  position: absolute;
  z-index: 3;
  display: grid;
  align-content: start;
  gap: 2px;
  padding: 8px;
  overflow-wrap: anywhere;
  background: #fbfdff;
}

.node strong {
  color: var(--arch-ink);
  font-size: 0.72rem;
  line-height: 1.3;
  overflow-wrap: anywhere;
  word-break: normal;
}

.node span,
.node em {
  min-width: 0;
  color: var(--arch-muted);
  font-size: 0.62rem;
  font-style: normal;
  font-weight: 500;
  line-height: 1.3;
}

.node em {
  display: inline-flex;
  width: fit-content;
  max-width: 100%;
  white-space: normal;
  overflow-wrap: anywhere;
  margin-top: 3px;
  padding: 2px 6px;
  border-radius: 999px;
  background: #eef4f8;
  color: #344155;
  font-family: var(--vp-font-family-mono);
  font-size: 0.55rem;
  font-weight: 700;
}

.lane-tag {
  display: inline-flex;
  width: fit-content;
  max-width: 100%;
  margin-bottom: 2px;
  padding: 2px 6px;
  border: 1px solid currentColor;
  border-radius: 999px;
  background: #ffffff;
  font-family: var(--vp-font-family-mono);
  font-size: 0.52rem !important;
  font-weight: 800 !important;
  line-height: 1.2 !important;
}

.host-tag { color: var(--arch-host) !important; }
.main-tag { color: var(--arch-main) !important; }
.engine-tag { color: var(--arch-engine) !important; }
.robot-tag { color: var(--arch-robot) !important; }

.host-source {
  top: 42px;
  left: 20px;
  width: 38%;
  bottom: 14px;
}

.host-command {
  top: 42px;
  right: 20px;
  width: 38%;
  bottom: 14px;
}

.main-service {
  top: 54px;
  left: 18px;
  width: calc(50% - 26px);
  height: 76px;
}

.main-session {
  top: 54px;
  right: 18px;
  width: calc(50% - 26px);
  height: 76px;
}

.main-requester {
  top: 178px;
  left: 18px;
  right: 18px;
  height: 88px;
}

.main-actions {
  top: 325px;
  left: 18px;
  right: 18px;
  height: 94px;
}

.main-loop {
  bottom: 28px;
  left: 18px;
  width: calc(50% - 26px);
  height: 78px;
}

.main-robot-client {
  bottom: 28px;
  right: 18px;
  width: calc(50% - 26px);
  height: 78px;
}

.engine-worker {
  top: 54px;
  left: 18px;
  right: 18px;
  height: 98px;
}

.engine-backend {
  top: 210px;
  left: 18px;
  right: 18px;
  height: 220px;
}

.engine-robot-client {
  bottom: 28px;
  left: 18px;
  right: 18px;
  height: 78px;
}

.robot-sensors {
  top: 44px;
  left: 20px;
  width: calc(50% - 28px);
  bottom: 14px;
}

.robot-command {
  top: 44px;
  right: 20px;
  width: calc(50% - 28px);
  bottom: 14px;
}

.subnode-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
  margin-top: 8px;
}

.subnode-grid span {
  display: grid;
  align-items: center;
  min-height: 34px;
  padding: 5px 6px;
  border: 1px solid var(--arch-line);
  border-radius: 6px;
  background: #fbfdff;
  color: var(--arch-muted);
  font-size: 0.58rem;
  font-weight: 700;
  line-height: 1.25;
  text-align: center;
}

.backend-island {
  position: absolute;
  z-index: 3;
  left: 16px;
  right: 16px;
  bottom: 16px;
  display: grid;
  grid-template-columns: minmax(0, 1.35fr) repeat(3, minmax(0, 1fr));
  gap: 8px;
  height: 74px;
  padding: 10px;
  border-color: rgba(101, 84, 168, 0.5);
}

.backend-island span {
  display: grid;
  align-items: center;
  min-height: 34px;
  padding: 6px 8px;
  border: 1px solid var(--arch-line);
  border-radius: 6px;
  background: #fbfdff;
  color: var(--arch-muted);
  font-size: 0.66rem;
  font-weight: 700;
  line-height: 1.25;
  text-align: center;
}

.backend-island .backend-label {
  justify-items: start;
  border-color: transparent;
  background: transparent;
  color: var(--arch-ink);
  text-align: left;
}

.flow-label,
.flow-marker {
  position: absolute;
  z-index: 5;
  pointer-events: none;
}

.flow-label {
  display: inline-flex;
  max-width: min(100%, 220px);
  padding: 3px 8px;
  border: 1px solid currentColor;
  border-radius: 999px;
  background: var(--arch-paper);
  font-family: var(--vp-font-family-mono);
  font-size: 0.55rem;
  font-weight: 800;
  line-height: 1.2;
  text-align: center;
  overflow-wrap: anywhere;
  transform: translate(-50%, -50%);
}

.flow-marker {
  display: grid;
  place-items: center;
  width: 24px;
  height: 24px;
  border-radius: 999px;
  color: #ffffff;
  font-family: var(--vp-font-family-mono);
  font-size: 0.66rem;
  font-weight: 800;
  line-height: 1;
  transform: translate(-50%, -50%);
}

.command-flow { color: var(--arch-host); }
.main-flow { color: var(--arch-main); }
.engine-flow { color: var(--arch-engine); }
.robot-flow { color: var(--arch-robot); }
.backend-flow { color: var(--arch-backend); }
.action-flow { color: var(--arch-action); }

.command-marker { background: var(--arch-host); }
.engine-marker { background: var(--arch-engine); }
.robot-marker { background: var(--arch-robot); }
.action-marker { background: var(--arch-action); }

.label-service { left: 43%; top: 150px; }
.label-engine { left: 50%; top: 392px; }
.label-action-list { left: 50%; top: 552px; }
.label-processed { left: 29%; top: 698px; }
.label-tick { left: 34%; top: 840px; }
.label-observation { left: 28%; top: 990px; }
.label-observation-policy { left: 77%; top: 700px; }
.label-command-out { left: 74%; top: 990px; }
.label-backend { left: 67%; top: 860px; }
.marker-1 { left: 42%; top: 168px; }
.marker-2 { left: 51%; top: 415px; }
.marker-3 { left: 39%; top: 990px; }
.marker-4 { left: 50%; top: 586px; }
.marker-5 { left: 64%; top: 990px; }

.flow-list {
  display: grid;
  gap: 8px;
  margin: 12px 0 0;
  padding: 0;
  list-style: none;
  counter-reset: cyclo-flow;
}

.flow-list li {
  position: relative;
  min-height: 38px;
  padding: 8px 10px 8px 40px;
  border: 1px solid var(--arch-line);
  border-radius: 6px;
  background: var(--arch-paper);
  color: var(--arch-muted);
  font-size: 0.75rem;
  line-height: 1.5;
  counter-increment: cyclo-flow;
}

.flow-list li::before {
  content: counter(cyclo-flow);
  position: absolute;
  top: 9px;
  left: 10px;
  display: grid;
  place-items: center;
  width: 20px;
  height: 20px;
  border-radius: 999px;
  background: var(--arch-main);
  color: #ffffff;
  font-family: var(--vp-font-family-mono);
  font-size: 0.68rem;
  font-weight: 800;
}

.flow-list strong {
  color: var(--arch-ink);
}

.flow-list code {
  font-size: 0.72rem;
}

.architecture-footer {
  margin: 10px 0 0;
  color: var(--arch-muted);
  font-size: 0.72rem;
  line-height: 1.45;
}

@media (max-width: 720px) {
  .diagram-stage {
    height: 1750px;
  }

  .desktop-flow {
    display: none;
  }

  .mobile-flow {
    display: block;
  }

  .command-line { marker-end: url(#arch-mobile-command); }
  .engine-line { marker-end: url(#arch-mobile-engine); }
  .robot-line { marker-end: url(#arch-mobile-robot); }
  .action-line { marker-end: url(#arch-mobile-action); }
  .backend-line { marker-end: url(#arch-mobile-backend); }

  .host-zone {
    height: 250px;
  }

  .policy-zone {
    top: 290px;
    height: 1190px;
  }

  .robot-zone {
    top: 1530px;
    height: 220px;
  }

  .host-source,
  .host-command,
  .robot-sensors,
  .robot-command {
    left: 20px;
    right: 20px;
    width: auto;
    bottom: auto;
  }

  .host-source { top: 44px; height: 84px; }
  .host-command { top: 150px; height: 84px; }
  .robot-sensors { top: 44px; height: 74px; }
  .robot-command { top: 134px; height: 74px; }

  .main-panel,
  .engine-panel {
    left: 16px;
    right: 16px;
    width: auto;
  }

  .main-panel {
    top: 52px;
    height: 525px;
  }

  .engine-panel {
    top: 610px;
    height: 395px;
  }

  .main-service,
  .main-session,
  .main-requester,
  .main-actions,
  .main-loop,
  .main-robot-client,
  .engine-worker,
  .engine-backend,
  .engine-robot-client {
    left: 18px;
    right: 18px;
    width: auto;
    bottom: auto;
  }

  .main-service { top: 48px; height: 62px; }
  .main-session { top: 128px; height: 62px; }
  .main-requester { top: 208px; height: 74px; }
  .main-actions { top: 292px; height: 74px; }
  .main-loop { top: 382px; height: 62px; }
  .main-robot-client { top: 456px; height: 62px; }

  .engine-worker { top: 48px; height: 76px; }
  .engine-backend { top: 148px; height: 166px; }
  .engine-robot-client { top: 330px; height: 64px; }

  .subnode-grid {
    grid-template-columns: minmax(0, 1fr);
    gap: 4px;
  }

  .subnode-grid span {
    min-height: 24px;
    padding: 3px 6px;
  }

  .backend-island {
    height: 160px;
    grid-template-columns: minmax(0, 1fr);
  }

  .backend-island span {
    min-height: 24px;
  }

  .label-service { left: 50%; top: 265px; }
  .label-engine { left: 64%; top: 882px; }
  .label-action-list { left: 36%; top: 882px; }
  .label-processed,
  .label-tick {
    display: none;
  }
  .label-observation { left: 42%; top: 1518px; }
  .label-observation-policy {
    display: none;
  }
  .label-command-out { left: 72%; top: 1518px; }
  .label-backend {
    display: none;
  }
  .marker-1 { left: 58%; top: 314px; }
  .marker-2 { left: 56%; top: 888px; }
  .marker-3 { left: 34%; top: 1518px; }
  .marker-4 {
    display: none;
  }
  .marker-5 { left: 64%; top: 1518px; }
}

@media (max-width: 520px) {
  .cyclo-brain-architecture {
    padding: 12px;
  }

  .legend span {
    white-space: normal;
  }
}
</style>
