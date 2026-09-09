(() => {
  "use strict";

  const canvas = document.querySelector("#energy-canvas");
  const screen = document.querySelector("#screen");
  const fullscreenToggle = document.querySelector("#fullscreen-toggle");
  const fullscreenLabel = fullscreenToggle.querySelector(".fullscreen-label");
  const missionStatus = document.querySelector("#mission-status");
  const missionKicker = document.querySelector("#mission-kicker");
  const missionTitle = document.querySelector("#mission-title");
  const missionDetail = document.querySelector("#mission-detail");
  const context = canvas.getContext("2d", { alpha: true });
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

  let width = 0;
  let height = 0;
  let dpr = 1;
  let lastFrame = 0;
  let burst = 0;
  let wakeLock = null;
  let lastMissionKey = "";
  let missionReceived = false;
  let windowFullscreen = false;
  let fullscreenBusy = false;

  const particles = Array.from({ length: 44 }, (_, index) => ({
    phase: (index / 44) * Math.PI * 2,
    speed: 0.58 + ((index * 17) % 23) / 42,
    size: 0.7 + ((index * 11) % 9) / 7,
    alpha: 0.22 + ((index * 7) % 13) / 20,
  }));

  function resize() {
    const bounds = screen.getBoundingClientRect();
    width = Math.max(1, bounds.width);
    height = Math.max(1, bounds.height);
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function butterflyPoint(angle, scale, centerX, centerY) {
    const sine = Math.sin(angle);
    const cosine = Math.cos(angle);
    const denominator = 1 + sine * sine;
    return {
      x: centerX + (scale * cosine) / denominator,
      y: centerY + (scale * 0.72 * sine * cosine) / denominator,
    };
  }

  function drawRibbons(time) {
    const centerX = width * 0.5;
    const centerY = height * (width > height ? 0.52 : 0.49);
    const fieldWidth = Math.min(width * 0.9, height * 0.82);
    const fieldHeight = Math.min(height * 0.19, width * 0.28);
    const startX = centerX - fieldWidth * 0.5;

    context.save();
    context.globalCompositeOperation = "lighter";
    context.lineCap = "round";

    for (let line = 0; line < 16; line += 1) {
      const depth = line / 15;
      const phase = time * (0.00034 + depth * 0.00008) + line * 0.24;
      context.beginPath();

      for (let step = 0; step <= 90; step += 1) {
        const progress = step / 90;
        const x = startX + progress * fieldWidth;
        const envelope = Math.pow(Math.sin(progress * Math.PI), 2.35);
        const pairedWave =
          Math.sin(progress * Math.PI * 4.1 + phase) * 0.62 +
          Math.sin(progress * Math.PI * 7.7 - phase * 0.72) * 0.24;
        const perspective = (depth - 0.5) * fieldHeight * 0.72;
        const y =
          centerY +
          pairedWave * fieldHeight * envelope * (0.58 + depth * 0.26) +
          perspective * envelope;

        if (step === 0) {
          context.moveTo(x, y);
        } else {
          context.lineTo(x, y);
        }
      }

      const gradient = context.createLinearGradient(
        startX,
        centerY,
        startX + fieldWidth,
        centerY
      );
      gradient.addColorStop(0, "rgba(4, 70, 255, 0)");
      gradient.addColorStop(0.18, `rgba(3, 111, 255, ${0.04 + depth * 0.025})`);
      gradient.addColorStop(0.5, `rgba(55, 225, 241, ${0.12 + depth * 0.05})`);
      gradient.addColorStop(0.82, `rgba(7, 116, 255, ${0.05 + depth * 0.02})`);
      gradient.addColorStop(1, "rgba(3, 81, 255, 0)");
      context.strokeStyle = gradient;
      context.lineWidth = 0.6 + depth * 0.75;
      context.stroke();
    }

    context.restore();
  }

  function drawButterflyParticles(time) {
    const centerX = width * 0.5;
    const centerY = height * (width > height ? 0.485 : 0.455);
    const scale = Math.min(width * 0.37, height * 0.235);
    const speedBoost = 1 + burst * 1.6;

    context.save();
    context.globalCompositeOperation = "lighter";

    for (const particle of particles) {
      const angle =
        particle.phase + time * 0.00042 * particle.speed * speedBoost;
      const point = butterflyPoint(angle, scale, centerX, centerY);
      const previous = butterflyPoint(
        angle - 0.045 * speedBoost,
        scale,
        centerX,
        centerY
      );
      const alpha = Math.min(1, particle.alpha + burst * 0.35);

      const trail = context.createLinearGradient(
        previous.x,
        previous.y,
        point.x,
        point.y
      );
      trail.addColorStop(0, "rgba(24, 119, 255, 0)");
      trail.addColorStop(1, `rgba(95, 240, 248, ${alpha})`);
      context.strokeStyle = trail;
      context.lineWidth = particle.size;
      context.beginPath();
      context.moveTo(previous.x, previous.y);
      context.lineTo(point.x, point.y);
      context.stroke();

      context.fillStyle = `rgba(137, 249, 255, ${Math.min(0.9, alpha)})`;
      context.shadowColor = "#2bd8eb";
      context.shadowBlur = 7 + burst * 10;
      context.beginPath();
      context.arc(point.x, point.y, particle.size * 0.72, 0, Math.PI * 2);
      context.fill();
    }

    context.restore();
  }

  function drawPulse() {
    if (burst <= 0.001) {
      return;
    }

    const centerX = width * 0.5;
    const centerY = height * (width > height ? 0.5 : 0.47);
    const radius = (1 - burst) * Math.min(width, height) * 0.48;
    context.save();
    context.strokeStyle = `rgba(87, 235, 247, ${burst * 0.32})`;
    context.lineWidth = 1.5;
    context.shadowColor = "#2cc6d3";
    context.shadowBlur = 18;
    context.beginPath();
    context.arc(centerX, centerY, radius, 0, Math.PI * 2);
    context.stroke();
    context.restore();
  }

  function render(time) {
    if (!reducedMotion.matches && time - lastFrame < 33) {
      requestAnimationFrame(render);
      return;
    }
    lastFrame = time;

    context.clearRect(0, 0, width, height);
    drawRibbons(time);
    drawButterflyParticles(time);
    drawPulse();
    burst *= 0.92;

    if (!reducedMotion.matches || burst > 0.01) {
      requestAnimationFrame(render);
    }
  }

  function triggerBurst() {
    burst = 1;
    screen.classList.remove("energy-burst");
    void screen.offsetWidth;
    screen.classList.add("energy-burst");
    window.setTimeout(() => screen.classList.remove("energy-burst"), 950);
    if (reducedMotion.matches) {
      requestAnimationFrame(render);
    }
  }

  function updateFullscreenControl(active = windowFullscreen) {
    windowFullscreen = Boolean(active);
    fullscreenToggle.dataset.active = String(active);
    fullscreenLabel.textContent = active ? "退出" : "全屏";
    fullscreenToggle.setAttribute(
      "aria-label",
      active ? "退出全屏" : "进入全屏"
    );
  }

  async function readFullscreenState() {
    try {
      const response = await fetch(`/api/window/fullscreen?_=${Date.now()}`, {
        cache: "no-store",
      });
      const payload = await response.json();
      if (!response.ok || !payload.success) throw new Error(payload.error || "unavailable");
      updateFullscreenControl(payload.enabled === true);
    } catch {
      updateFullscreenControl(windowFullscreen);
    }
  }

  async function toggleFullscreen() {
    if (fullscreenBusy) return;
    fullscreenBusy = true;
    fullscreenToggle.disabled = true;
    try {
      const response = await fetch("/api/window/fullscreen", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: !windowFullscreen }),
      });
      const payload = await response.json();
      if (!response.ok || !payload.success) throw new Error(payload.error || "unavailable");
      updateFullscreenControl(payload.enabled === true);
    } catch {
      await readFullscreenState();
    } finally {
      fullscreenBusy = false;
      fullscreenToggle.disabled = false;
    }
  }

  async function keepScreenAwake() {
    if (!("wakeLock" in navigator) || document.visibilityState !== "visible") {
      return;
    }
    try {
      wakeLock = await navigator.wakeLock.request("screen");
    } catch {
      wakeLock = null;
    }
  }

  function missionPresentation(snapshot) {
    const state = String(snapshot?.state || "idle");
    const current = snapshot?.current || {};
    const location = String(current.location || "");
    const task = String(current.task || "");
    const total = Math.max(0, Number(snapshot?.total) || 0);
    const index = total ? Math.min(total, Math.max(1, Number(snapshot?.current_index) + 1)) : 0;
    const step = total ? `任务 ${index} / ${total}` : "机器人任务状态";
    const values = {
      idle: ["等待任务", "请在任务编排中心下达指令"],
      pending_confirmation: [
        "收到新的任务安排",
        total
          ? `${location || "首项任务"}${task ? ` · ${task}` : ""} · 共 ${total} 项待确认`
          : "等待确认",
      ],
      preparing: ["正在准备任务", location || task || "即将开始"],
      navigating: [location ? `前往 ${location}` : "正在前往任务地点", task ? `随后执行：${task}` : "导航进行中"],
      adjusting_height: [location ? `已到达 ${location}` : "已到达任务地点", "正在确认周围环境"],
      awaiting_action: [task || "执行当前任务", location ? `地点：${location}` : "任务执行中"],
      completed: ["全部任务已完成", total ? `本轮共完成 ${total} 项任务` : "等待新的任务"],
      navigation_failed: ["任务行程待确认", task ? `当前任务：${task}` : "等待操作端处理"],
      failed: ["任务流程待确认", task ? `当前任务：${task}` : "等待系统处理"],
      cancelled: ["任务已更新", "等待新的任务安排"],
    };
    const selected = values[state] || ["任务进行中", String(snapshot?.message || "状态同步中")];
    return { state, step, title: selected[0], detail: selected[1] };
  }

  function renderMission(snapshot) {
    const presentation = missionPresentation(snapshot);
    const key = JSON.stringify([
      snapshot?.version, snapshot?.updated_at, presentation,
      snapshot?.tasks || [], snapshot?.current_index,
    ]);
    if (key === lastMissionKey) return;
    lastMissionKey = key;
    missionStatus.dataset.state = presentation.state;
    missionKicker.textContent = presentation.step;
    missionTitle.textContent = presentation.title;
    missionDetail.textContent = presentation.detail;
    missionStatus.classList.remove("updated");
    void missionStatus.offsetWidth;
    missionStatus.classList.add("updated");
  }

  async function pollMission() {
    try {
      const response = await fetch(`/api/task-status?_=${Date.now()}`, {
        cache: "no-store",
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const payload = await response.json();
      renderMission(payload.task_dispatch || {});
      missionReceived = true;
    } catch {
      if (!missionReceived) {
        missionKicker.textContent = "机器人任务状态";
        missionTitle.textContent = "等待任务";
        missionDetail.textContent = "任务系统连接中";
      }
    } finally {
      window.setTimeout(pollMission, 700);
    }
  }

  window.addEventListener("resize", resize, { passive: true });
  screen.addEventListener("pointerdown", triggerBurst, { passive: true });
  fullscreenToggle.addEventListener("pointerdown", (event) => {
    event.stopPropagation();
  });
  fullscreenToggle.addEventListener("click", toggleFullscreen);
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "visible") {
      keepScreenAwake();
      void readFullscreenState();
      requestAnimationFrame(render);
    }
  });
  reducedMotion.addEventListener?.("change", () => requestAnimationFrame(render));

  resize();
  void readFullscreenState();
  keepScreenAwake();
  pollMission();
  requestAnimationFrame(render);
})();
