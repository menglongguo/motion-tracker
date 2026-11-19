// ============================================================
// EXERCISE DEFINITIONS
// ============================================================
//
// ANGLE CONVENTION:
//
//                  270°
//                    ↑
//                    |
//      180°  ←────── ● ──────→  0°
//                    |
//                    ↓
//                   90°
//
// Positive rotation = CLOCKWISE.
//
// staticArmAngle is an ABSOLUTE angle on the screen.
//
// movingArmStartOffset and movingArmTargetOffset are RELATIVE
// to the static arm.
// ============================================================


let exercises = {

    seatedDumbbellPress: {

        name: "Seated Dumbbell Press",
        mode: "reps",
        staticArmAngle: 270,
        movingArmStartOffset: 90,
        movingArmTargetOffset: 180,
        targetTolerance: 5
    },

    bicepCurl: {

        name: "Bicep Curl",
        mode: "reps",
        staticArmAngle: 200,
        movingArmStartOffset: 180,
        movingArmTargetOffset: 30,
        targetTolerance: 5
    },

    lSit: {

        name: "L-Sit",
        mode: "hold",
        staticArmAngle: 270,
        movingArmStartOffset: 180,
        movingArmTargetOffset: 95,
        targetTolerance: 5
    },

    speedBicepCurl: {

        name: "Speed Bicep Curl",
        mode: "speed",
        staticArmAngle: 200,
        movingArmStartOffset: 180,
        movingArmTargetOffset: 30,
        targetTolerance: 5,
        targetReps: 10
    },

    dumbbellLifts: {

        name: "Dumbbell Lifts",
        mode: "max",
        staticArmAngle: 90,
        movingArmStartOffset: 0,
        movingArmTargetOffset: 15,
        targetTolerance: 5
    }

};


// CURRENT EXERCISE
// Change this line to exercises.bicepCurl to test the curl
//const exercise = exercises.seatedDumbbellPress;
// Current exercise can now change while the app is running, let allows you to change
let exercise = exercises.bicepCurl;
// ============================================================
// IMU QUATERNION MATH
// ============================================================

// Returns the inverse of a normalized quaternion.
// BNO085 rotation-vector quaternions are normalized,
// so the inverse is simply the conjugate.
function quaternionInverse(q) {
    return {
        i: -q.i,
        j: -q.j,
        k: -q.k,
        r:  q.r
    };
}


// Multiplies two quaternions together.
// Quaternion multiplication is how we combine/remove rotations.
function quaternionMultiply(a, b) {
    return {
        r: a.r * b.r - a.i * b.i - a.j * b.j - a.k * b.k,

        i: a.r * b.i + a.i * b.r + a.j * b.k - a.k * b.j,

        j: a.r * b.j - a.i * b.k + a.j * b.r + a.k * b.i,

        k: a.r * b.k + a.i * b.j - a.j * b.i + a.k * b.r
    };
}


// Calculates IMU2's orientation RELATIVE to IMU1.
//
// This removes the global orientation of IMU1.
// For example, rotating your entire arm together should produce
// very little change in this relative quaternion.
function getRelativeQuaternion(imu1, imu2) {

    const imu1Inverse = quaternionInverse(imu1);

    return quaternionMultiply(imu1Inverse, imu2);
}

// Diagnostic only:
// Convert the relative quaternion into X, Y, Z rotation angles.
function quaternionToEuler(q) {

    // Rotation around X axis
    const x = Math.atan2(
        2 * (q.r * q.i + q.j * q.k),
        1 - 2 * (q.i * q.i + q.j * q.j)
    );

    // Rotation around Y axis
    const yInput =
        2 * (q.r * q.j - q.k * q.i);

    const y = Math.asin(
        Math.max(-1, Math.min(1, yInput))
    );

    // Rotation around Z axis
    const z = Math.atan2(
        2 * (q.r * q.k + q.i * q.j),
        1 - 2 * (q.j * q.j + q.k * q.k)
    );

    // Convert radians → degrees
    return {
        x: x * 180 / Math.PI,
        y: y * 180 / Math.PI,
        z: z * 180 / Math.PI
    };
}

// Rotates a 3D vector using a quaternion.
// This lets us ask:
// "Where is this sensor's local +Y/-Y axis pointing in 3D space?"
function rotateVectorByQuaternion(vector, q) {

    // Quaternion vector part.
    const u = {
        x: q.i,
        y: q.j,
        z: q.k
    };

    const s = q.r;

    // Dot product: u · vector
    const dot =
        u.x * vector.x +
        u.y * vector.y +
        u.z * vector.z;

    // Dot product: u · u
    const uu =
        u.x * u.x +
        u.y * u.y +
        u.z * u.z;

    // Cross product: u × vector
    const cross = {
        x: u.y * vector.z - u.z * vector.y,
        y: u.z * vector.x - u.x * vector.z,
        z: u.x * vector.y - u.y * vector.x
    };

    // Quaternion rotation formula.
    return {
        x: 2 * dot * u.x + (s * s - uu) * vector.x + 2 * s * cross.x,
        y: 2 * dot * u.y + (s * s - uu) * vector.y + 2 * s * cross.y,
        z: 2 * dot * u.z + (s * s - uu) * vector.z + 2 * s * cross.z
    };
}

function getBoneAngle(imu1, imu2) {

    // Both sensors are physically mounted with the SAME XYZ orientation.
    //
    // Upper arm:
    // +Y points from elbow toward shoulder.
    const upperLocal = {
        x: 0,
        y: 1,
        z: 0
    };

    // Forearm:
    // -Y points from elbow toward wrist.
    const forearmLocal = {
        x: 0,
        y: -1,
        z: 0
    };

    // Use each sensor's complete quaternion to determine
    // where those two vectors point in 3D space.
    const upperWorld =
        rotateVectorByQuaternion(upperLocal, imu1);

    const forearmWorld =
        rotateVectorByQuaternion(forearmLocal, imu2);

    // Dot product between the two vectors.
    let dot =
        upperWorld.x * forearmWorld.x +
        upperWorld.y * forearmWorld.y +
        upperWorld.z * forearmWorld.z;

    // Protect acos() from tiny floating-point errors.
    dot = Math.max(-1, Math.min(1, dot));

    // acos gives an angle from 0 to 180 degrees.
    return Math.acos(dot) * 180 / Math.PI;
}

// ============================================================
// SENSOR WEBSOCKET
// ============================================================

// Connect the browser to the Feather.
// Replace this IP if the Feather gets a new IP address.
const sensorSocket =
    new WebSocket("ws://192.168.4.34:81");


sensorSocket.onopen = function () {

    console.log("ESP32 connected");
};

sensorSocket.onmessage = function (event) {

    const data = JSON.parse(event.data);

    // Calculate the real joint angle from the two IMUs.
    const boneAngle =
        getBoneAngle(data.imu1, data.imu2);

    // Real sensor angle is now our raw input.
    rawAngle = boneAngle;

    // Apply calibration.
    movingArmCurrentAngle =
        normalizeAngle(rawAngle + calibrationOffset);

    // Convert relative joint angle to its position on the dial.
    const movingArmCurrentAbsolute =
        getAbsoluteMovingAngle(movingArmCurrentAngle);

    // Move the orange arm.
    drawArm(
        "movingArmCurrent",
        movingArmCurrentAbsolute
    );

    // Update the numerical angle.
   // During Speed mode, the main display is reserved for the timer.
    if (!(exercise.mode === "speed" && isExerciseRunning)) {

        angleDisplay.textContent =
            movingArmCurrentAngle.toFixed(1) + "°";
    }
    // Check target/start position for rep counting.
    updateRepState();

    console.log(
        "Bone angle:",
        boneAngle.toFixed(1)
    );
};


sensorSocket.onclose = function () {

    console.log("ESP32 disconnected");
};

// ANGLE HELPERS
function normalizeAngle(angle) {
    return ((angle % 360) + 360) % 360;
}

function getAbsoluteMovingAngle(relativeAngle) {
    return normalizeAngle(
        exercise.staticArmAngle + relativeAngle
    );
}



// ============================================================
// CURRENT MOVEMENT
// ============================================================

// For now the slider directly simulates the relative joint angle.
//
// For the dumbbell press:
// Start = 90°
// Target = 180°

let rawAngle = exercise.movingArmStartOffset;
// Raw sensor reading.
// The slider provides this for now.
// Later this comes from the two BNO085s.

let calibrationOffset = 0;
// Correction applied to the raw sensor angle.

let movingArmCurrentAngle = rawAngle;
// The calibrated joint angle actually used by the exercise.

let isCalibrated = false;
// Becomes true after the user presses Calibrate.

let isExerciseRunning = false;
// Tracks whether we're currently exercising

let sessionStartTime = null;
// Stores the exact time the current exercise session started

let repCount = 0;
// Number of completed reps

let repState = "waitingForTarget";
// We're initially waiting for the user to reach the target

// Extra state used by Hold, Speed, and Max modes
let timerStart = null;
let timerInterval = null;
let holdTime = 0;
let maxAngle = 0;

// ============================================================
// DIAL GEOMETRY
// ============================================================


const dialCenter = 200;                 // Center of 400 x 400 SVG

const armLength = 170;                  // Length of each arm



// ============================================================
// HTML ELEMENTS
// ============================================================


const exerciseNameDisplay =
    document.getElementById("exerciseName");


const movingArmStartDisplay =
    document.getElementById("movingArmStartAngle");


const movingArmTargetDisplay =
    document.getElementById("movingArmTargetAngle");


const angleDisplay =
    document.getElementById("currentAngle");


const angleSlider =
    document.getElementById("angleSlider");

const calibrateButton =
    document.getElementById("calibrateButton");

const startButton =
    document.getElementById("startButton");

const exportButton =
    document.getElementById("exportButton");

const statusMessage =
    document.getElementById("statusMessage");

const repDisplay =
    document.getElementById("repCount");

const metricLabel =
    document.getElementById("metricLabel");

const dialCircle =
    document.querySelector(".dialCircle");

const exerciseSelect =
    document.getElementById("exerciseSelect");

const updateExercisesButton =
    document.getElementById("updateExercisesButton");

const syncResultsButton =
    document.getElementById("syncResultsButton");

const sessionHistory = [];
// Stores every completed exercise during this app session

const pendingCount =
    document.getElementById("pendingCount");


let pendingResults =
    JSON.parse(
        localStorage.getItem("pendingResults")
    ) || [];
// Stores results that Google has not confirmed yet


function savePendingResults() {
    localStorage.setItem(
        "pendingResults",
        JSON.stringify(pendingResults)
    );
}


function updatePendingDisplay() {
    pendingCount.textContent =
        pendingResults.length;
}

// ============================================================
// DRAW AN ARM
// ============================================================

function saveExerciseResult() {

    const sessionEndTime = new Date();

    // Total time from Start Exercise to Stop/Completion
    const durationSeconds =
        (sessionEndTime - sessionStartTime) / 1000;

    // Format: 2026-11-05T21:34
    const startedAt =
        sessionStartTime.getFullYear() + "-" +
        String(sessionStartTime.getMonth() + 1).padStart(2, "0") + "-" +
        String(sessionStartTime.getDate()).padStart(2, "0") + "T" +
        String(sessionStartTime.getHours()).padStart(2, "0") + ":" +
        String(sessionStartTime.getMinutes()).padStart(2, "0") + ":" +
        String(sessionStartTime.getSeconds()).padStart(2, "0");
    // Result means something different depending on mode
    let result;

    if (exercise.mode === "reps") {
        result = repCount;
    }

    else if (exercise.mode === "hold") {

        // Include an active hold that has not ended yet
        result = holdTime;

        if (timerStart !== null) {
            result +=
                (Date.now() - timerStart) / 1000;
        }
    }

    else if (exercise.mode === "speed") {
        result = durationSeconds;
    }

    else if (exercise.mode === "max") {
        result = maxAngle;
    }

    // Our exact CSV structure
    const sessionRow = {
        exercise_name: exercise.name,
        started_at: startedAt,
        mode: exercise.mode,
        duration_seconds: Number(durationSeconds.toFixed(1)),
        result: Number(result.toFixed(1))
    };

    sessionHistory.push(sessionRow);

    // Also save the result to our persistent local queue
    pendingResults.push(sessionRow);

    savePendingResults();

    updatePendingDisplay();


    console.table(sessionHistory);

    console.log(
        "Pending results:",
        pendingResults
    );
}

function drawArm(elementId, absoluteAngle) {

    const radians = absoluteAngle * Math.PI / 180;
    // JavaScript sin/cos use radians


    const x =
        dialCenter + armLength * Math.cos(radians);
    // Horizontal endpoint


    const y =
        dialCenter + armLength * Math.sin(radians);
    // Vertical endpoint
    //
    // IMPORTANT:
    // SVG Y increases DOWNWARD.
    //
    // Using + sin therefore makes positive angles rotate
    // CLOCKWISE, which is our chosen convention.


    const arm = document.getElementById(elementId);


    arm.setAttribute("x2", x);

    arm.setAttribute("y2", y);
}



// ============================================================
// DRAW A FILLED ZONE
// ============================================================


function drawZone(elementId, angle1, angle2) {

    const angle1Radians =
        angle1 * Math.PI / 180;

    const angle2Radians =
        angle2 * Math.PI / 180;


    const x1 =
        dialCenter + armLength * Math.cos(angle1Radians);

    const y1 =
        dialCenter + armLength * Math.sin(angle1Radians);


    const x2 =
        dialCenter + armLength * Math.cos(angle2Radians);

    const y2 =
        dialCenter + armLength * Math.sin(angle2Radians);


    // Find clockwise distance from angle1 → angle2
    const clockwiseDifference =
        normalizeAngle(angle2 - angle1);


    // If clockwise is <= 180°, clockwise is the shorter path.
    // Otherwise, counterclockwise is the shorter path.
    const sweepFlag =
        clockwiseDifference <= 180 ? 1 : 0;


    const path = `
        M ${dialCenter} ${dialCenter}
        L ${x1} ${y1}
        A ${armLength} ${armLength} 0 0 ${sweepFlag} ${x2} ${y2}
        Z
    `;

    // largeArcFlag is ALWAYS 0.
    //
    // That is the important part:
    // SVG is never allowed to draw more than half the circle.
    // sweepFlag simply chooses which direction gives us that smaller area.


    document
        .getElementById(elementId)
        .setAttribute("d", path);
}

// ============================================================
// CALCULATE ABSOLUTE ARM POSITIONS
// ============================================================

const movingArmStartAbsolute =
    getAbsoluteMovingAngle(exercise.movingArmStartOffset);


const movingArmTargetAbsolute =
    getAbsoluteMovingAngle(exercise.movingArmTargetOffset);

// ============================================================
// POPULATE EXERCISE INFORMATION
// ============================================================


exerciseNameDisplay.textContent =
    exercise.name;

movingArmStartDisplay.textContent =
    exercise.movingArmStartOffset;

movingArmTargetDisplay.textContent =
    exercise.movingArmTargetOffset;

// Start slider at this exercise's starting joint angle
angleSlider.value =
    exercise.movingArmStartOffset;

// Display starting joint angle
angleDisplay.textContent =
    movingArmCurrentAngle.toFixed(1) + "\u00B0";

// ============================================================
// DRAW FIXED VISUALS
// ============================================================


drawZone(
    "movementZone",
    movingArmStartAbsolute,
    movingArmTargetAbsolute
);
// Light-green movement region

drawZone(
    "targetZone",
    movingArmTargetAbsolute - exercise.targetTolerance,
    movingArmTargetAbsolute + exercise.targetTolerance
);
// Darker target ± tolerance region

drawArm(
    "staticArm",
    exercise.staticArmAngle
);
// Black static arm

drawArm(
    "movingArmStart",
    movingArmStartAbsolute
);
// Blue starting marker

drawArm(
    "movingArmTarget",
    movingArmTargetAbsolute
);
// Red target marker

drawArm(
    "movingArmCurrent",
    getAbsoluteMovingAngle(movingArmCurrentAngle)
);
// Orange live moving arm

// ============================================================
// TEMPORARY SLIDER / FAKE SENSOR
// ============================================================
function flashTarget() {

    const flashToggle =
        document.getElementById("flashToggle");

    if (!flashToggle.checked) return;
    // Do nothing if Flash is turned off

    dialCircle.classList.add("targetFlash");
    // Turn outer ring green

    setTimeout(function () {

        dialCircle.classList.remove("targetFlash");
        // Return ring to normal
    }, 300);
}

// Create one audio system and reuse it for the whole page.
const audioContext = new AudioContext();
const targetSound = new Audio("beep.mp3");

function playTargetSound() {

    console.log("playTargetSound called");

    const soundToggle =
        document.getElementById("soundToggle");

    console.log("sound checked:", soundToggle.checked);

    if (!soundToggle.checked) return;

    targetSound.currentTime = 0;
    targetSound.play();
}

function updateRepState() {

    if (!isExerciseRunning) return;


    const targetDifference =
        Math.abs(movingArmCurrentAngle - exercise.movingArmTargetOffset);

    const startDifference =
        Math.abs(movingArmCurrentAngle - exercise.movingArmStartOffset);

    if (exercise.mode === "reps") {

        if (
            repState === "waitingForTarget" &&
            targetDifference <= exercise.targetTolerance
        ) {

            repState = "waitingForStart";

            statusMessage.textContent = "Target reached!";

            flashTarget();
            playTargetSound();
        }

        else if (
            repState === "waitingForStart" &&
            startDifference <= exercise.targetTolerance
        ) {

            repCount = repCount + 1;

            repDisplay.textContent = repCount;

            repState = "waitingForTarget";

            statusMessage.textContent = "Rep complete!";
        }
    }


    else if (exercise.mode === "speed") {

        if (
            repState === "waitingForTarget" &&
            targetDifference <= exercise.targetTolerance
        ) {

            repState = "waitingForStart";

            statusMessage.textContent = "Target reached!";

            flashTarget();
            playTargetSound();
        }

        else if (
            repState === "waitingForStart" &&
            startDifference <= exercise.targetTolerance
        ) {

            repCount = repCount + 1;

            repDisplay.textContent =
                exercise.targetReps - repCount;

            repState = "waitingForTarget";

            statusMessage.textContent = "Rep complete!";


            if (repCount >= exercise.targetReps) {

                isExerciseRunning = false;

                saveExerciseResult();

                clearInterval(timerInterval);

                timerInterval = null;

                startButton.textContent = "Start Exercise";

                statusMessage.textContent = "Complete!";
            }
        }
    }


    else if (exercise.mode === "hold") {

        if (movingArmCurrentAngle <=
            exercise.movingArmTargetOffset + exercise.targetTolerance) {

            // Start timing when we first enter the target zone.
            if (timerStart === null) {

                timerStart = Date.now();

                flashTarget();
                playTargetSound();
            }


            const currentHoldTime =
                holdTime +
                (Date.now() - timerStart) / 1000;


            repDisplay.textContent =
                currentHoldTime.toFixed(1) + " s";
        }

        else if (timerStart !== null) {

            // Save the time accumulated before leaving the target.
            holdTime +=
                (Date.now() - timerStart) / 1000;

            timerStart = null;
        }
    }


    else if (exercise.mode === "max") {

        // Do not record a result until the 15° minimum is reached.
        if (
            movingArmCurrentAngle >= exercise.movingArmTargetOffset && movingArmCurrentAngle <= 180
        ) {

            maxAngle =
                Math.max(
                    maxAngle,
                    movingArmCurrentAngle
                );


            repDisplay.textContent =
                maxAngle.toFixed(1) + "°";
        }
    }
}

function loadExercise(exerciseKey) {

    // Change the current exercise
    exercise = exercises[exerciseKey];


    // Update exercise name
    exerciseNameDisplay.textContent =
        exercise.name;


    // Calculate the new absolute arm positions
    const startAbsolute =
        getAbsoluteMovingAngle(exercise.movingArmStartOffset);

    const targetAbsolute =
        getAbsoluteMovingAngle(exercise.movingArmTargetOffset);


    // Update the displayed start/target numbers
    movingArmStartDisplay.textContent =
        exercise.movingArmStartOffset;

    movingArmTargetDisplay.textContent =
        exercise.movingArmTargetOffset;


    // Redraw the movement zones
    drawZone(
        "movementZone",
        startAbsolute,
        targetAbsolute
    );

    drawZone(
        "targetZone",
        targetAbsolute - exercise.targetTolerance,
        targetAbsolute + exercise.targetTolerance
    );


    // Redraw the three fixed arms
    drawArm(
        "staticArm",
        exercise.staticArmAngle
    );

    drawArm(
        "movingArmStart",
        startAbsolute
    );

    drawArm(
        "movingArmTarget",
        targetAbsolute
    );
    // Reset the simulated sensor to the new exercise's start
    rawAngle = exercise.movingArmStartOffset;

    angleSlider.value = rawAngle;


    // Reset calibration
    calibrationOffset = 0;

    isCalibrated = false;

    movingArmCurrentAngle = rawAngle;

    // Move orange arm to the new starting position
    drawArm(
        "movingArmCurrent",
        getAbsoluteMovingAngle(movingArmCurrentAngle)
    );

    angleDisplay.textContent =
        movingArmCurrentAngle.toFixed(1) + "\u00B0";

    // Reset exercise results
    repCount = 0;
    repState = "waitingForTarget";

    timerStart = null;
    holdTime = 0;
    maxAngle = 0;

    clearInterval(timerInterval);
    timerInterval = null;


    // Set the lower display for this mode
    if (exercise.mode === "reps") {

        metricLabel.textContent = "Reps";
        repDisplay.textContent = 0;
    }

    else if (exercise.mode === "speed") {

        metricLabel.textContent = "Reps Remaining";
        repDisplay.textContent = exercise.targetReps;
    }

    else if (exercise.mode === "hold") {

        metricLabel.textContent = "Hold Time";
        repDisplay.textContent = "0.0 s";
    }

    else if (exercise.mode === "max") {

        metricLabel.textContent = "Max Angle";
        repDisplay.textContent = "--";
    }    

    // Stop any running exercise
    isExerciseRunning = false;

    startButton.textContent = "Start Exercise";

    statusMessage.innerHTML = "&nbsp;";
}

exerciseSelect.addEventListener("change", function () {

    loadExercise(exerciseSelect.value);

});

/*
angleSlider.addEventListener("input", function () {

    rawAngle = Number(angleSlider.value);
    // Slider changes the RAW sensor reading.

    movingArmCurrentAngle =
        normalizeAngle(rawAngle + calibrationOffset);
    // Apply our calibration correction.

    const movingArmCurrentAbsolute =
        getAbsoluteMovingAngle(movingArmCurrentAngle);
    // Convert relative joint angle → absolute screen angle.

    drawArm(
        "movingArmCurrent",
        movingArmCurrentAbsolute
    );

    angleDisplay.textContent =
        movingArmCurrentAngle.toFixed(1) + "\u00B0";

    updateRepState();
});
*/

calibrateButton.addEventListener("click", function () {

    calibrationOffset =
        exercise.movingArmStartOffset - rawAngle;
    // Example:
    //
    // Defined start = 180°
    // Raw sensor     = 160°
    //
    // Offset = +20°


    movingArmCurrentAngle =
        normalizeAngle(rawAngle + calibrationOffset);
    // Apply the new calibration.


    isCalibrated = true;


    drawArm(
        "movingArmCurrent",
        getAbsoluteMovingAngle(movingArmCurrentAngle)
    );


    angleDisplay.textContent =
        movingArmCurrentAngle.toFixed(1) + "\u00B0";
});

startButton.addEventListener("click", function () {
    if (audioContext.state === "suspended") {
        audioContext.resume();
    }
    // Don't allow a session to start until calibration is complete
    if (!isCalibrated) {

        statusMessage.textContent = "Please calibrate first.";

        setTimeout(function () {
            statusMessage.innerHTML = "&nbsp;";
        }, 2000);

        return;
    }


    if (!isExerciseRunning) {

        // START the exercise
        isExerciseRunning = true;


        // Save when this session started
        sessionStartTime = new Date();
        // Unlock audio because Safari requires sound to start from a user click.
        targetSound.play().then(function () {

            targetSound.pause();
            targetSound.currentTime = 0;

        }).catch(function (error) {

            console.log("Sound unlock failed:", error);

        });

        // Reset mode-specific result
        repCount = 0;
        repState = "waitingForTarget";
        timerStart = null;
        holdTime = 0;
        maxAngle = 0;


        // Speed mode starts its timer immediately.
        if (exercise.mode === "speed") {

            repDisplay.textContent = exercise.targetReps;

            timerStart = Date.now();

            timerInterval = setInterval(function () {

                const seconds =
                    (Date.now() - timerStart) / 1000;

                angleDisplay.textContent =
                    seconds.toFixed(1) + " s";

            }, 100);
        }

        startButton.textContent = "Stop Exercise";

        console.log("Exercise started");


    } else {

        // STOP the exercise
        isExerciseRunning = false;

        saveExerciseResult();

        repCount = 0;

        repDisplay.textContent = repCount;

        repState = "waitingForTarget";

        startButton.textContent = "Start Exercise";

        console.log("Exercise stopped");
    }
});

exportButton.addEventListener("click", function () {

    if (sessionHistory.length === 0) {
        statusMessage.textContent = "No exercise results to export.";
        return;
    }

    // CSV column names
    const headers = [
        "exercise_name",
        "started_at",
        "mode",
        "duration_seconds",
        "result"
    ];

    // Start CSV with header row
    const rows = [
        headers.join(",")
    ];

    // Add each exercise result
    sessionHistory.forEach(function (session) {

        const row = [
            session.exercise_name,
            session.started_at,
            session.mode,
            session.duration_seconds,
            session.result
        ];

        rows.push(row.join(","));
    });

    // Combine rows into one CSV file
    const csvContent =
        rows.join("\n");

    // Create temporary downloadable file
    const blob =
        new Blob(
            [csvContent],
            { type: "text/csv" }
        );

    const url =
        URL.createObjectURL(blob);

    const downloadLink =
        document.createElement("a");

    downloadLink.href = url;

    downloadLink.download =
        "exercise-results.csv";

    downloadLink.click();

    URL.revokeObjectURL(url);
});

const GOOGLE_SCRIPT_URL =
    "https://script.google.com/macros/s/AKfycbwNwtjP8nkW1QoOH2shVN4Aw0nKAuaNEr21EoGhtPjWNMdWUIRhWSWpy4uxXQbETVPP/exec";


updateExercisesButton.addEventListener("click", async function () {

    try {

        statusMessage.textContent = "Updating exercises...";

        const response =
            await fetch(GOOGLE_SCRIPT_URL);

        const exerciseData =
            await response.json();

        console.log("Exercises from Google:");
        console.table(exerciseData);


        // Convert Google Sheet rows into our exercise format
        const updatedExercises = {};

        exerciseData.forEach(function (row) {

            const exerciseKey =
                row.exercise_name
                    .replace(/\s+/g, "")
                    .replace(/^./, char => char.toLowerCase());

            updatedExercises[exerciseKey] = {
                name: row.exercise_name,
                mode: row.mode,
                staticArmAngle: Number(row.static_arm_angle),
                movingArmStartOffset: Number(row.moving_arm_start_offset),
                movingArmTargetOffset: Number(row.moving_arm_target_offset),
                targetTolerance: Number(row.target_tolerance)
            };

            // Only add targetReps when the sheet contains one
            if (row.target_reps !== "") {
                updatedExercises[exerciseKey].targetReps =
                    Number(row.target_reps);
            }

        });

        exercises = updatedExercises;
        // Clear the old exercise dropdown
        exerciseSelect.innerHTML = "";


        // Add the downloaded exercises to the dropdown
        Object.keys(exercises).forEach(function (exerciseKey) {

            const option =
                document.createElement("option");

            option.value =
                exerciseKey;

            option.textContent =
                exercises[exerciseKey].name;

            exerciseSelect.appendChild(option);

        });

        console.log("Updated exercises:");
        console.log(exercises);

        statusMessage.textContent =
            "Exercise list updated.";

    } catch (error) {

        console.error(
            "Failed to download exercises:",
            error
        );

        statusMessage.textContent =
            "Failed to update exercises.";
    }

});

syncResultsButton.addEventListener("click", async function () {

    if (pendingResults.length === 0) {

        statusMessage.textContent =
            "No results waiting to sync.";

        return;
    }


    try {

        statusMessage.textContent =
            "Syncing " +
            pendingResults.length +
            " result(s)...";


        const response =
            await fetch(GOOGLE_SCRIPT_URL, {

                method: "POST",

                body: JSON.stringify({
                    results: pendingResults
                })

            });


        const syncResult =
            await response.json();


        console.log("Google sync result:");
        console.log(syncResult);


        // Only remove results Google explicitly confirmed
        const confirmed =
            syncResult.confirmed || [];


        pendingResults =
            pendingResults.filter(function(result) {

                return !confirmed.includes(
                    result.started_at
                );

            });


        // Save the reduced queue locally
        savePendingResults();

        // Update the number on screen
        updatePendingDisplay();


        if (pendingResults.length === 0) {

            statusMessage.textContent =
                "All results synced.";

        } else {

            statusMessage.textContent =
                pendingResults.length +
                " result(s) still waiting to sync.";

        }


    } catch (error) {

        console.error(
            "Failed to sync results:",
            error
        );


        // IMPORTANT:
        // We do not delete anything from pendingResults here.

        statusMessage.textContent =
            "Sync failed. " +
            pendingResults.length +
            " result(s) saved locally.";

    }

});

updatePendingDisplay();