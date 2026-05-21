<template>
  <section class="cyclo-data-architecture" aria-labelledby="cyclo-data-architecture-title">
    <div class="architecture-header">
      <div>
        <p id="cyclo-data-architecture-title" class="architecture-title">
          Cyclo Data System Architecture
        </p>
        <p class="architecture-summary">
          Cyclo Data is operated from the Web UI. The user records robot demonstrations,
          edits the collected dataset with Data Tools, converts it to LeRobot format, and
          uploads or downloads datasets and models through Hugging Face.
        </p>
      </div>
      <div class="legend" aria-label="Flow legend">
        <span><i class="dot ui-dot"></i>UI</span>
        <span><i class="dot record-dot"></i>record</span>
        <span><i class="dot tools-dot"></i>data tools</span>
        <span><i class="dot convert-dot"></i>convert</span>
        <span><i class="dot hub-dot"></i>Hub</span>
      </div>
    </div>

    <div class="primary-flow" aria-label="Cyclo Data primary workflow">
      <article class="flow-card ui-card">
        <span class="lane-tag ui-tag">Web UI</span>
        <strong>Operate Everything from UI</strong>
        <span>Select the robot, open Record, and use Data Tools without leaving Cyclo Intelligence.</span>
      </article>

      <div class="flow-arrow ui-flow" aria-hidden="true">v</div>

      <article class="flow-card record-card">
        <span class="lane-tag record-tag">Record Page</span>
        <strong>Acquire Dataset</strong>
        <span>Capture ROS 2 robot topics, camera streams, task metadata, and episode information.</span>
      </article>

      <div class="flow-arrow record-flow" aria-hidden="true">v</div>

      <article class="flow-card tools-card">
        <span class="lane-tag tools-tag">Data Tools</span>
        <strong>Edit and Prepare Dataset</strong>
        <span>Delete episodes, merge datasets, or prepare the dataset for LeRobot conversion.</span>
      </article>

      <div class="flow-arrow tools-flow" aria-hidden="true">v</div>

      <article class="flow-card output-card">
        <span class="lane-tag convert-tag">Outputs</span>
        <strong>LeRobot / Hugging Face</strong>
        <span>Create LeRobot datasets and move datasets or models through Hugging Face.</span>
      </article>
    </div>

    <div class="details-grid" aria-label="Cyclo Data detailed operations">
      <article class="detail-card record-detail">
        <span class="lane-tag record-tag">Dataset Acquisition</span>
        <strong>Record</strong>
        <ul>
          <li>Camera streams are stored as MP4 files with frame timestamps.</li>
          <li>Non-camera ROS 2 topics are stored in MCAP format.</li>
          <li>Camera rotation is saved according to the robot config file.</li>
          <li>Each save creates an episode dataset.</li>
        </ul>
      </article>

      <article class="detail-card edit-detail">
        <span class="lane-tag tools-tag">Dataset Editing</span>
        <strong>Delete / Merge</strong>
        <ul>
          <li>Delete unnecessary or failed episodes.</li>
          <li>Merge multiple collected datasets.</li>
          <li>Keep the dataset organized before conversion.</li>
        </ul>
      </article>

      <article class="detail-card convert-detail">
        <span class="lane-tag convert-tag">Dataset Conversion</span>
        <strong>LeRobot Convert</strong>
        <ul>
          <li>Select the target FPS for LeRobot conversion.</li>
          <li>Align sensor/action timelines and video frames.</li>
          <li>Write LeRobot v2.1 and/or v3.0 output.</li>
        </ul>
      </article>

      <article class="detail-card hub-detail">
        <span class="lane-tag hub-tag">Repository Operation</span>
        <strong>Hugging Face</strong>
        <ul>
          <li>Upload collected or converted datasets.</li>
          <li>Download datasets for editing or conversion.</li>
          <li>Download models for later deployment.</li>
        </ul>
      </article>
    </div>
  </section>
</template>

<style scoped>
.cyclo-data-architecture {
  --arch-paper: #ffffff;
  --arch-node: #fbfdff;
  --arch-bg: #f6f8fb;
  --arch-ink: #152033;
  --arch-muted: #5d6878;
  --arch-line: #c8d2df;
  --arch-ui: #2e5fa7;
  --arch-record: #087f8c;
  --arch-tools: #2f6f4e;
  --arch-convert: #6554a8;
  --arch-hub: #b34242;
  --arch-flow-opacity: 0.5;
  --arch-shadow: 0 10px 28px rgba(21, 32, 51, 0.08);
  margin: 1rem 0 1.5rem;
  padding: 16px;
  border: 1px solid var(--arch-line);
  border-radius: 8px;
  background: var(--arch-bg);
  color: var(--arch-ink);
}

.cyclo-data-architecture *,
.cyclo-data-architecture *::before,
.cyclo-data-architecture *::after {
  box-sizing: border-box;
}

.architecture-header {
  display: grid;
  gap: 12px;
  margin-bottom: 16px;
}

.architecture-title {
  margin: 0;
  color: var(--arch-ink);
  font-size: 1rem;
  font-weight: 700;
  line-height: 1.35;
}

.architecture-summary {
  max-width: 860px;
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
  flex: 0 0 auto;
}

.ui-dot { background: var(--arch-ui); }
.record-dot { background: var(--arch-record); }
.tools-dot { background: var(--arch-tools); }
.convert-dot { background: var(--arch-convert); }
.hub-dot { background: var(--arch-hub); }

.primary-flow {
  display: grid;
  grid-template-columns: 1fr;
  gap: 8px;
}

.flow-card,
.detail-card {
  position: relative;
  display: grid;
  align-content: start;
  gap: 7px;
  padding: 14px;
  border: 1px solid var(--arch-line);
  border-radius: 8px;
  background: var(--arch-node);
  box-shadow: var(--arch-shadow);
}

.flow-card {
  min-height: 92px;
  padding-top: 14px;
}

.flow-card strong,
.detail-card strong {
  color: var(--arch-ink);
  font-size: 1rem;
}

.flow-card span,
.detail-card li {
  color: var(--arch-muted);
  font-size: 0.84rem;
  line-height: 1.42;
}

.lane-tag {
  justify-self: start;
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
  font-weight: 800;
  line-height: 1.2 !important;
}

.ui-card,
.record-detail {
  border-left: 4px solid var(--arch-ui);
}

.record-card {
  border-left: 4px solid var(--arch-record);
}

.tools-card,
.edit-detail {
  border-left: 4px solid var(--arch-tools);
}

.output-card,
.convert-detail {
  border-left: 4px solid var(--arch-convert);
}

.hub-detail {
  border-left: 4px solid var(--arch-hub);
}

.ui-tag { color: var(--arch-ui) !important; }
.record-tag { color: var(--arch-record) !important; }
.tools-tag { color: var(--arch-tools) !important; }
.convert-tag { color: var(--arch-convert) !important; }
.hub-tag { color: var(--arch-hub) !important; }

.flow-arrow {
  min-height: 28px;
  display: grid;
  place-items: center;
  position: relative;
  isolation: isolate;
  color: #ffffff;
  font-size: 1.25rem;
  font-weight: 900;
  text-shadow: 0 1px 1px rgba(0, 0, 0, 0.22);
}

.flow-arrow::after {
  content: "";
  position: absolute;
  width: 28px;
  height: 28px;
  border-radius: 50%;
  background: var(--arrow-color, #32445c);
  opacity: var(--arch-flow-opacity);
  z-index: -1;
}

.ui-flow { --arrow-color: var(--arch-ui); }
.record-flow { --arrow-color: var(--arch-record); }
.tools-flow { --arrow-color: var(--arch-tools); }

.details-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
  margin-top: 14px;
}

.detail-card ul {
  margin: 0;
  padding-left: 18px;
}

@media (max-width: 960px) {
  .primary-flow,
  .details-grid {
    grid-template-columns: 1fr;
  }

  .flow-card {
    min-height: auto;
  }
}

</style>
