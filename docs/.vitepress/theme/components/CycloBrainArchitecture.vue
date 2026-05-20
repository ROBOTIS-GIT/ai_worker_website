<template>
  <section class="cyclo-brain-architecture" aria-labelledby="cyclo-brain-architecture-title">
    <div class="architecture-header">
      <div>
        <p id="cyclo-brain-architecture-title" class="architecture-title">
          Cyclo Brain System Architecture
        </p>
        <p class="architecture-summary">
          UI/orchestrator and standalone CLI commands enter the same runtime contract. The policy
          container separates command handling from model inference, while robot command output and
          observation input stay on different paths.
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

    <div class="architecture-map" aria-label="Cyclo Brain architecture map">
      <article class="runtime-band host-band">
        <p class="band-label">Host</p>
        <div class="node-row">
          <div class="node">
            <span class="lane-tag host-tag">Command Source</span>
            <strong>UI / Orchestrator</strong>
            <span>Standalone CLI args</span>
            <em>same command shape</em>
          </div>
          <div class="h-arrow command-flow" aria-hidden="true">
            <span>service call</span>
          </div>
          <div class="node">
            <span class="lane-tag host-tag">InferenceCommand</span>
            <strong>LOAD / START / PAUSE</strong>
            <span>RESUME / STOP / UNLOAD</span>
            <em>external service</em>
          </div>
        </div>
      </article>

      <div class="v-arrow down command-flow" aria-hidden="true">
        <span>external service call</span>
      </div>

      <section class="flow-container" aria-label="Runtime data flow across host, policy container, and robot">
        <p class="container-label">Runtime Data Flow</p>

        <article class="flow-step command-step">
          <span class="step-marker">1</span>
          <div class="step-content">
            <p class="step-title">UI/CLI command enters Main.</p>
            <div class="flow-path">
              <div class="node">
                <span class="lane-tag host-tag">Host</span>
                <strong>InferenceCommand</strong>
                <span>common command contract</span>
              </div>
              <div class="path-arrow command-flow" aria-hidden="true">
                <span>external service call</span>
              </div>
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>ServiceHandler</strong>
                <span>command entry</span>
              </div>
              <div class="path-arrow main-flow" aria-hidden="true">
                <span>update session</span>
              </div>
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>SessionState</strong>
                <span>runtime gate</span>
              </div>
            </div>
          </div>
        </article>

        <article class="flow-step engine-step">
          <span class="step-marker">2</span>
          <div class="step-content">
            <p class="step-title">Main requests Engine action.</p>
            <div class="flow-path">
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>InferenceRequester</strong>
                <span>one GET_ACTION at a time</span>
                <span>timeout + seq_id stale guard</span>
              </div>
              <div class="path-arrow engine-flow" aria-hidden="true">
                <span>EngineCommand</span>
              </div>
              <div class="node">
                <span class="lane-tag engine-tag">Engine Process</span>
                <strong>EngineWorker</strong>
                <span>hosts internal EngineCommand service</span>
                <span>using Zenoh ROS2 SDK</span>
              </div>
              <div class="path-arrow engine-flow" aria-hidden="true">
                <span>load / predict</span>
              </div>
              <div class="node">
                <span class="lane-tag engine-tag">Engine Process</span>
                <strong>Backend InferenceEngine</strong>
                <span>model-specific implementation</span>
                <span>behind stable contract</span>
                <div class="subnode-flow" aria-label="Backend inference stages">
                  <span>Policy Load</span>
                  <i aria-hidden="true"></i>
                  <span>Preprocess</span>
                  <i aria-hidden="true"></i>
                  <span>Predict</span>
                  <i aria-hidden="true"></i>
                  <span>Optional Optimize</span>
                </div>
              </div>
            </div>
          </div>
        </article>

        <article class="flow-step robot-step">
          <span class="step-marker">3</span>
          <div class="step-content">
            <p class="step-title">Engine reads Robot observation.</p>
            <div class="flow-path">
              <div class="node">
                <span class="lane-tag robot-tag">Robot</span>
                <strong>Sensors / State</strong>
                <span>camera, joints, base state</span>
              </div>
              <div class="path-arrow robot-flow" aria-hidden="true">
                <span>sensor / state read</span>
              </div>
              <div class="node">
                <span class="lane-tag engine-tag">Engine Process</span>
                <strong>RobotClient</strong>
                <span>observation only</span>
              </div>
              <div class="path-arrow robot-flow" aria-hidden="true">
                <span>observations into policy</span>
              </div>
              <div class="node">
                <span class="lane-tag engine-tag">Engine Process</span>
                <strong>Backend InferenceEngine</strong>
                <span>uses camera, joint, and base state</span>
              </div>
            </div>
          </div>
        </article>

        <article class="flow-step return-step">
          <span class="step-marker">4</span>
          <div class="step-content">
            <p class="step-title">Engine returns action_list.</p>
            <div class="flow-path">
              <div class="node">
                <span class="lane-tag engine-tag">Engine Process</span>
                <strong>Backend InferenceEngine</strong>
                <span>computes action chunk</span>
              </div>
              <div class="path-arrow engine-flow" aria-hidden="true">
                <span>action_list</span>
              </div>
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>ActionChunkProcessor</strong>
                <span>buffer + optional match / RTC</span>
                <span>interpolate / blend / smooth</span>
              </div>
            </div>
          </div>
        </article>

        <article class="flow-step publish-step">
          <span class="step-marker">5</span>
          <div class="step-content">
            <p class="step-title">Main buffers, processes, and publishes.</p>
            <div class="flow-path">
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>ActionChunkProcessor</strong>
                <span>can expand 16 actions to 100 Hz buffer</span>
              </div>
              <div class="path-arrow main-flow" aria-hidden="true">
                <span>processed actions</span>
              </div>
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>ControlLoop</strong>
                <span>self-running tick</span>
              </div>
              <div class="path-arrow action-flow" aria-hidden="true">
                <span>one action / tick</span>
              </div>
              <div class="node">
                <span class="lane-tag main-tag">Main Runtime</span>
                <strong>RobotClient</strong>
                <span>command output</span>
              </div>
              <div class="path-arrow action-flow" aria-hidden="true">
                <span>command publish</span>
              </div>
              <div class="node">
                <span class="lane-tag robot-tag">Robot</span>
                <strong>Command Topics</strong>
                <span>cmd_vel, trajectory, etc.</span>
              </div>
            </div>
          </div>
        </article>

      </section>

      <section class="policy-container" aria-label="Policy container backend server">
        <p class="container-label">Policy Container: &lt;backend&gt;_server</p>

        <article class="runtime-band process-band">
          <p class="band-label">Process Split</p>
          <div class="node-row two-col">
            <div class="node">
              <span class="lane-tag main-tag">main-runtime</span>
              <strong>Command and control owner</strong>
              <span>external command handling</span>
              <span>session state, action buffer, command publish</span>
            </div>
            <div class="node">
              <span class="lane-tag engine-tag">engine-process</span>
              <strong>Model and observation owner</strong>
              <span>model loading, preprocessing, prediction</span>
              <span>robot observation read only</span>
            </div>
          </div>
        </article>

        <div class="backend-island" aria-label="Open-source backend island">
          <span class="backend-label">Open-source backend island</span>
          <span>LeRobot</span>
          <span>GR00T</span>
          <span>Future model</span>
        </div>
      </section>

      <article class="runtime-band robot-band">
        <p class="band-label">Robot</p>
        <div class="node-row two-col">
          <div class="node">
            <span class="lane-tag robot-tag">Sensors / State</span>
            <strong>Observation input</strong>
            <span>camera, joints, base state</span>
          </div>
          <div class="node">
            <span class="lane-tag robot-tag">Command Topics</span>
            <strong>Command output</strong>
            <span>cmd_vel, trajectory, etc.</span>
          </div>
        </div>
      </article>
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

.architecture-map,
.flow-container,
.policy-container,
.flow-path {
  display: grid;
  gap: 10px;
}

.architecture-map {
  margin-top: 14px;
}

.runtime-band,
.flow-container,
.policy-container,
.flow-step {
  min-width: 0;
  border: 1px solid var(--arch-line);
  border-radius: 8px;
  background: var(--arch-paper);
  box-shadow: var(--arch-shadow);
}

.runtime-band {
  display: grid;
  gap: 8px;
  padding: 10px;
}

.flow-container,
.policy-container {
  padding: 10px;
}

.host-band { border-color: rgba(46, 95, 167, 0.55); }
.process-band { border-color: rgba(8, 127, 140, 0.45); }
.flow-container { border-color: rgba(8, 127, 140, 0.45); }
.robot-band { border-color: rgba(154, 106, 29, 0.58); }

.band-label,
.container-label {
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

.node-row {
  display: grid;
  grid-template-columns: minmax(0, 1fr) 92px minmax(0, 1fr);
  gap: 8px;
  align-items: center;
}

.node-row.two-col {
  grid-template-columns: repeat(2, minmax(0, 1fr));
}

.flow-step {
  --step-color: var(--arch-main);
  display: grid;
  grid-template-columns: 28px minmax(0, 1fr);
  gap: 10px;
  padding: 10px;
  border-color: var(--arch-line);
}

.command-step {
  --step-color: var(--arch-host);
  border-color: rgba(46, 95, 167, 0.58);
}

.engine-step,
.return-step {
  --step-color: var(--arch-engine);
  border-color: rgba(47, 111, 78, 0.58);
}

.robot-step {
  --step-color: var(--arch-robot);
  border-color: rgba(154, 106, 29, 0.58);
}

.publish-step {
  --step-color: var(--arch-action);
  border-color: rgba(179, 66, 66, 0.58);
}

.step-marker {
  display: grid;
  place-items: center;
  width: 24px;
  height: 24px;
  margin-top: 2px;
  border-radius: 999px;
  background: var(--step-color);
  color: #ffffff;
  font-family: var(--vp-font-family-mono);
  font-size: 0.72rem;
  font-weight: 800;
  line-height: 1;
}

.step-content {
  min-width: 0;
}

.step-title {
  margin: 0 0 8px;
  color: var(--arch-ink);
  font-size: 0.78rem;
  font-weight: 800;
  line-height: 1.35;
}

.node {
  display: grid;
  align-content: start;
  gap: 2px;
  min-width: 0;
  padding: 8px;
  overflow-wrap: anywhere;
  border: 1px solid var(--arch-line);
  border-radius: 6px;
  background: #fbfdff;
}

.node strong {
  color: var(--arch-ink);
  font-size: 0.75rem;
  line-height: 1.35;
}

.node span,
.node em {
  min-width: 0;
  color: var(--arch-muted);
  font-size: 0.66rem;
  font-style: normal;
  font-weight: 500;
  line-height: 1.35;
}

.node em {
  display: inline-flex;
  width: fit-content;
  max-width: 100%;
  margin-top: 3px;
  padding: 2px 6px;
  border-radius: 999px;
  background: #eef4f8;
  color: #344155;
  font-family: var(--vp-font-family-mono);
  font-size: 0.58rem;
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
  font-size: 0.56rem !important;
  font-weight: 800 !important;
  line-height: 1.2 !important;
}

.host-tag { color: var(--arch-host) !important; }
.main-tag { color: var(--arch-main) !important; }
.engine-tag { color: var(--arch-engine) !important; }
.robot-tag { color: var(--arch-robot) !important; }

.h-arrow,
.v-arrow,
.path-arrow {
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 0;
  color: var(--arch-muted);
}

.h-arrow {
  min-height: 28px;
}

.v-arrow,
.path-arrow {
  min-height: 42px;
}

.h-arrow::before,
.v-arrow::before,
.path-arrow::before {
  content: "";
  position: absolute;
  display: block;
}

.h-arrow::before {
  left: 0;
  right: 0;
  top: 50%;
  border-top: 2px solid currentColor;
}

.h-arrow::after {
  content: "";
  position: absolute;
  right: 0;
  top: 50%;
  width: 8px;
  height: 8px;
  border-top: 2px solid currentColor;
  border-right: 2px solid currentColor;
  transform: translateY(-50%) rotate(45deg);
}

.v-arrow::before,
.path-arrow::before {
  top: 0;
  bottom: 0;
  left: 50%;
  border-left: 2px solid currentColor;
}

.v-arrow::after,
.path-arrow::after {
  content: "";
  position: absolute;
  left: calc(50% - 4px);
  bottom: 0;
  width: 8px;
  height: 8px;
  border-right: 2px solid currentColor;
  border-bottom: 2px solid currentColor;
  transform: rotate(45deg);
}

.h-arrow span,
.v-arrow span,
.path-arrow span {
  position: relative;
  z-index: 1;
  display: inline-flex;
  max-width: min(100%, 260px);
  padding: 3px 8px;
  border: 1px solid currentColor;
  border-radius: 999px;
  background: var(--arch-paper);
  font-family: var(--vp-font-family-mono);
  font-size: 0.58rem;
  font-weight: 800;
  line-height: 1.2;
  text-align: center;
  overflow-wrap: anywhere;
}

.command-flow { color: var(--arch-host); }
.main-flow { color: var(--arch-main); }
.engine-flow { color: var(--arch-engine); }
.robot-flow { color: var(--arch-robot); }
.action-flow { color: var(--arch-action); }

.subnode-flow {
  display: grid;
  grid-template-columns: minmax(74px, 1fr) 16px minmax(74px, 1fr) 16px minmax(74px, 1fr) 16px minmax(84px, 1fr);
  gap: 5px;
  align-items: center;
  margin-top: 6px;
}

.subnode-flow span {
  display: grid;
  align-items: center;
  min-height: 34px;
  padding: 5px 6px;
  border: 1px solid var(--arch-line);
  border-radius: 6px;
  background: #fbfdff;
  color: var(--arch-muted);
  font-size: 0.6rem;
  font-weight: 700;
  line-height: 1.25;
  text-align: center;
}

.subnode-flow i {
  position: relative;
  display: block;
  height: 2px;
  background: var(--arch-engine);
}

.subnode-flow i::after {
  content: "";
  position: absolute;
  right: 0;
  top: -3px;
  width: 8px;
  height: 8px;
  border-top: 2px solid var(--arch-engine);
  border-right: 2px solid var(--arch-engine);
  transform: rotate(45deg);
}

.backend-island {
  display: grid;
  grid-template-columns: minmax(0, 1.4fr) repeat(3, minmax(0, 1fr));
  gap: 8px;
  padding: 10px;
  border: 1px solid rgba(101, 84, 168, 0.5);
  border-radius: 8px;
  background: var(--arch-paper);
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
  font-size: 0.7rem;
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
  .node-row,
  .node-row.two-col,
  .backend-island {
    grid-template-columns: minmax(0, 1fr);
  }

  .h-arrow {
    min-height: 42px;
  }

  .h-arrow::before {
    top: 0;
    bottom: 0;
    left: 50%;
    right: auto;
    border-top: 0;
    border-left: 2px solid currentColor;
  }

  .h-arrow::after {
    right: auto;
    left: calc(50% - 4px);
    top: auto;
    bottom: 0;
    transform: rotate(45deg);
  }

  .subnode-flow {
    grid-template-columns: minmax(0, 1fr);
  }

  .subnode-flow i {
    width: 2px;
    height: 18px;
    margin: 0 auto;
  }

  .subnode-flow i::after {
    top: auto;
    right: -3px;
    bottom: 0;
    transform: rotate(135deg);
  }

  .backend-island .backend-label {
    grid-column: auto;
  }
}

@media (max-width: 520px) {
  .cyclo-brain-architecture {
    padding: 12px;
  }

  .flow-step {
    grid-template-columns: 24px minmax(0, 1fr);
    gap: 8px;
    padding: 8px;
  }

  .step-marker {
    width: 22px;
    height: 22px;
    font-size: 0.66rem;
  }
}
</style>
