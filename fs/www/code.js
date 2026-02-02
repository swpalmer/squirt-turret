// JavaScript: Handling Logic and API calls
console.log("Installing event handlers...");
const Kp_slider = document.getElementById("Kp");
const Ki_slider = document.getElementById("Ki");
const Kd_slider = document.getElementById("Kd");
const motor_enable = document.getElementById("motorEnable");
const rpm_slider = document.getElementById("rpm");
const angle_slider = document.getElementById("angle");

const status = document.getElementById("statusMessage");

// State Display
const Kp_state = document.getElementById("KpState");
const Ki_state = document.getElementById("KiState");
const Kd_state = document.getElementById("KdState");
const rpm_state = document.getElementById("rpmState");
const angle_state = document.getElementById("angleState");
const motor_enable_state = document.getElementById("motorEnableState");
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
	Kp_state.innerHTML = data.Kp;
	Ki_state.innerHTML = data.Ki;
	Kd_state.innerHTML = data.Kd;
	rpm_state.innerHTML = data.rpm;
	angle_state.innerHTML = data.angle;
	motor_enable_state.innerHTML = data.motor_enable ? "Enabled" : "Disabled";
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
motor_enable.oninput = updateFunction;
rpm_slider.oninput = updateFunction;
angle_slider.oninput = updateFunction;
