// JavaScript: Handling Logic and API calls
console.log("Installing event handlers...");
const Kp_slider = document.getElementById("Kp");
const Ki_slider = document.getElementById("Ki");
const Kd_slider = document.getElementById("Kd");
const pitch_slider = document.getElementById("pitch");

const status = document.getElementById("statusMessage");

async function fetchState() {
	const resp = await fetch("/api/state");
	if (resp.ok) {
		resp.json().then((data) => {
			updateState(data);
		});
	}
}
let stateRefreshTimer = setInterval(fetchState, 1000);

function debounce(func, timeout = 100) {
	let timer;
	return (...args) => {
		clearTimeout(timer);
		timer = setTimeout(() => {
			func.apply(this, args);
		}, timeout);
	};
}

function updateState(data) {
	// Don't fight the user while they're actively dragging the dial
	if (!angleDialDragging) {
		setDialAngle(data.angle, false);
	}
	setEncoderHandleAngle(data.encoderAngle);
}

async function updateValueOnServer(key, value) {
	status.textContent = `Syncing ${key} => ${value}...`;
	try {
		const resp = await fetch(`/api/control?${key}=${value}`, {
			method: "PUT",
			headers: { "Content-Type": "text/json" },
		});

		if (resp.ok) {
			status.textContent = "Synced";
			status.style.color = "green";
			//  Update values from state response
			resp.json().then((data) => {
				updateState(data);
			});
		}
	} catch (error) {
		status.textContent = "Connection error";
		status.style.color = "red";
	}
}

// 3. Create a "debounced" version of our API caller
const processChange = debounce((key, val) => updateValueOnServer(key, val));
const updateFunction = function () {
	// Immediate UI feedback
	let valueDisplay = document.getElementById(`${this.id}Value`);
	if (valueDisplay) {
		valueDisplay.innerHTML = this.value;
	}
	if (this.type === "range") {
		// Debounced API call
		processChange(this.id, this.value);
	} else if (this.type === "checkbox") {
		// Checkbox inputs don't need to be debounced
		updateValueOnServer(this.id, this.checked);
	} else {
		console.log(`Unknown input type: ${this.type}`);
	}
};

Kp_slider.oninput = updateFunction;
Ki_slider.oninput = updateFunction;
Kd_slider.oninput = updateFunction;
pitch_slider.oninput = updateFunction;

// ----- Advanced (PID tuning) section, hidden by default -----
const advancedToggle = document.getElementById("advancedToggle");
const advancedControls = document.getElementById("advancedControls");

advancedToggle.addEventListener("click", () => {
	const showing = advancedControls.hidden;
	advancedControls.hidden = !showing;
	advancedToggle.textContent = showing ? "Hide Advanced" : "Show Advanced";
});

// ----- Circular "Aim" dial (drag-in-a-circle angle control) -----
const angleDial = document.getElementById("angleDial");
const angleHandle = document.getElementById("angleHandle");
const angleValueDisplay = document.getElementById("angleValue");
const encoderHandle = document.getElementById("encoderHandle");
const encoderAngleValueDisplay = document.getElementById("encoderAngleValue");

// Distance (px) from dial center to handle center; keep the 28px handle inside the 180px dial.
const DIAL_HANDLE_RADIUS = 65;

let angleDialDragging = false;
let currentAngle = 0;

// 0 deg is straight up, increasing clockwise, wrapped to [0, 360)
function setDialAngle(deg, sendUpdate) {
	deg = ((deg % 360) + 360) % 360;
	currentAngle = deg;

	const rad = (deg * Math.PI) / 180;
	const x = Math.sin(rad) * DIAL_HANDLE_RADIUS;
	const y = -Math.cos(rad) * DIAL_HANDLE_RADIUS;
	angleHandle.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;

	const rounded = Math.round(deg);
	angleValueDisplay.innerHTML = `${rounded}&deg;`;
	angleDial.setAttribute("aria-valuenow", rounded);

	if (sendUpdate) {
		processChange("angle", deg.toFixed(1));
	}
}

// Positions the read-only encoder marker on the dial; never sends an update.
function setEncoderHandleAngle(deg) {
	deg = ((deg % 360) + 360) % 360;

	const rad = (deg * Math.PI) / 180;
	const x = Math.sin(rad) * DIAL_HANDLE_RADIUS;
	const y = -Math.cos(rad) * DIAL_HANDLE_RADIUS;
	encoderHandle.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;

	encoderAngleValueDisplay.innerHTML = `${Math.round(deg)}&deg;`;
}

function angleFromPointerEvent(e) {
	const rect = angleDial.getBoundingClientRect();
	const cx = rect.left + rect.width / 2;
	const cy = rect.top + rect.height / 2;
	const dx = e.clientX - cx;
	const dy = e.clientY - cy;
	return (Math.atan2(dx, -dy) * 180) / Math.PI;
}

angleDial.addEventListener("pointerdown", (e) => {
	angleDialDragging = true;
	angleDial.setPointerCapture(e.pointerId);
	angleDial.focus();
	setDialAngle(angleFromPointerEvent(e), true);
});

angleDial.addEventListener("pointermove", (e) => {
	if (!angleDialDragging) return;
	setDialAngle(angleFromPointerEvent(e), true);
});

function endAngleDrag(e) {
	if (!angleDialDragging) return;
	angleDialDragging = false;
	try {
		angleDial.releasePointerCapture(e.pointerId);
	} catch (err) {
		// pointer capture may already be released, nothing to do
	}
}
angleDial.addEventListener("pointerup", endAngleDrag);
angleDial.addEventListener("pointercancel", endAngleDrag);

angleDial.addEventListener("keydown", (e) => {
	let delta = 0;
	if (e.key === "ArrowRight" || e.key === "ArrowUp") delta = 1;
	else if (e.key === "ArrowLeft" || e.key === "ArrowDown") delta = -1;
	else return;
	e.preventDefault();
	setDialAngle(currentAngle + delta, true);
});

// Initialize handle position
setDialAngle(currentAngle, false);
