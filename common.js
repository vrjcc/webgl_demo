// Settings
const sensorWidth = 1;
const relativeFocalLength = parseFloat(localStorage.getItem('relativeFocalLength')) || 1;
const focalLength = sensorWidth * relativeFocalLength;
const ipd = parseFloat(localStorage.getItem('ipd')) || 65;
const diagonalFov = parseFloat(localStorage.getItem('diagonalFov')) || 78;

// DOM
var video;
var canvas;
var ctx;
var resultDiv;

// Calibration storage
window._calibration = { leftEye: null, rightEye: null };
const calibration = {
  startCalibration: function() {
    const resultEl = document.getElementById('calibrationResult');
    // Disable inputs
    ['relativeFocalLength','diagonalFov'].forEach(id => document.getElementById(id).disabled = true);
    let countdown = 10;
    resultEl.textContent = `Calibration will start in ${countdown} seconds.`;
    const timer = setInterval(() => {
      countdown--;
      if (countdown > 0) {
        resultEl.textContent = `Calibration will start in ${countdown} seconds.`;
      } else {
        clearInterval(timer);
        calibration.doCalibrate();
      }
    }, 1000);
  },
  doCalibrate: function() {
    const resultEl = document.getElementById('calibrationResult');
    const left = window._calibration.leftEye;
    const right = window._calibration.rightEye;
console.log(left + ":" + right);
    if (!left || !right) {
      resultEl.textContent = 'Eyes not detected. Please adjust the camera.';
      ['relativeFocalLength','diagonalFov'].forEach(id => document.getElementById(id).disabled = false);
      return;
    }
    const dx = left.x - right.x;
    const dy = left.y - right.y;
    const eyeDistancePixels = Math.hypot(dx, dy);
console.log(eyeDistancePixels);
    const vw = video.videoWidth;
    const vh = video.videoHeight;
    const pixelSize = sensorWidth / vw;
    const distanceMeters = 0.5;
    // Adjust focal length
    const newFocal = (distanceMeters * 1000) * (eyeDistancePixels * pixelSize) / ipd;
    document.getElementById('relativeFocalLength').value = newFocal.toFixed(3);
console.log(newFocal);
    // Adjust diagonal FOV
    const censorEyeDist = pixelSize * eyeDistancePixels;
    const ipdHFovDeg = Math.atan(censorEyeDist / 2 / newFocal) * 2 * (180/Math.PI);
    const hFov = ipdHFovDeg * (vw / eyeDistancePixels);
    const vFov = hFov * (vh / vw);
    const dFov = Math.hypot(hFov, vFov);
    document.getElementById('diagonalFov').value = Math.floor(dFov);
    // Re-enable inputs
    ['relativeFocalLength','diagonalFov'].forEach(id => document.getElementById(id).disabled = false);
    resultEl.textContent = 'Calibration complete';
  }
};

// Kalman filters
class KalmanFilter {
  constructor(Q,R,P,X){this.Q=Q;this.R=R;this.P=P;this.X=X;}
  update(z){this.P+=this.Q;const K=this.P/(this.P+this.R);this.X+=K*(z-this.X);this.P=(1-K)*this.P;return this.X;}
}
const posXFilter=new KalmanFilter(0.05,0.05,10,0);
const posYFilter=new KalmanFilter(0.05,0.05,10,0);
const posZFilter=new KalmanFilter(0.05,0.5,10,0);
const quatXFilter=new KalmanFilter(0.005,0.1,1,0);
const quatYFilter=new KalmanFilter(0.005,0.1,1,0);
const quatZFilter=new KalmanFilter(0.005,0.1,1,0);
const quatWFilter=new KalmanFilter(0.005,0.1,1,1);

function loadSettings() {
  const settings = ['relativeFocalLength', 'ipd', 'diagonalFov', 'requestedWidth', 'requestedHeight', '_fps'];
  settings.forEach(key => {
    const stored = localStorage.getItem(key);
    if (stored !== null) {
      document.getElementById(key).value = stored;
    }
  });
}
function saveSettings() {
  const settings = ['relativeFocalLength', 'ipd', 'diagonalFov', 'requestedWidth', 'requestedHeight', '_fps'];
  settings.forEach(key => {
    localStorage.setItem(key, document.getElementById(key).value);
  });
}

// --- カメラ選択 ---
function populateCameraSelect() {
  const cameraSelect = document.getElementById('cameraSelect');
  cameraSelect.innerHTML = "";
  navigator.mediaDevices.enumerateDevices().then(devices => {
    const videoDevices = devices.filter(device => device.kind === 'videoinput');
    videoDevices.forEach(device => {
      const option = document.createElement('option');
      option.value = device.deviceId;
      option.text = device.label || `カメラ ${cameraSelect.length + 1}`;
      cameraSelect.appendChild(option);
    });
  }).catch(error => {
    console.error('Error enumerating devices:', error);
  });
}


// Vector math
function subtract(a, b) {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}
function cross(a, b) {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}
function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
function length(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
function normalize(v) {
  const len = length(v);
  return len === 0 ? { x: 0, y: 0, z: 0 } : { x: v.x/len, y: v.y/len, z: v.z/len };
}
function angleBetween(a, b) {
  const nA = normalize(a);
  const nB = normalize(b);
  let cosTheta = dot(nA, nB);
  cosTheta = Math.min(Math.max(cosTheta, -1), 1);
  return Math.acos(cosTheta);
}
function matrixToQuaternion(m) {
  const m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
  const m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
  const m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];
  let trace = m00 + m11 + m22;
  let q = { x: 0, y: 0, z: 0, w: 1 };
  if (trace > 0) {
    let s = Math.sqrt(trace + 1.0) * 2;
    q.w = 0.25 * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  } else if ((m00 > m11) && (m00 > m22)) {
    let s = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    let s = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    let s = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }
  return q;
}
function multiplyQuaternion(q1, q2) {
  return {
    w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
    x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
    y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
    z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
  };
}
function eulerToQuaternion(eulerX, eulerY, eulerZ) {
  const x = eulerX * Math.PI / 180;
  const y = eulerY * Math.PI / 180;
  const z = eulerZ * Math.PI / 180;
  const c1 = Math.cos(x/2), s1 = Math.sin(x/2);
  const c2 = Math.cos(y/2), s2 = Math.sin(y/2);
  const c3 = Math.cos(z/2), s3 = Math.sin(z/2);
  return {
    x: s1 * c2 * c3 - c1 * s2 * s3,
    y: c1 * s2 * c3 + s1 * c2 * s3,
    z: c1 * c2 * s3 - s1 * s2 * c3,
    w: c1 * c2 * c3 + s1 * s2 * s3
  };
}
// --- Euler角（度単位）からクォータニオンに変換する関数 ---
function eulerToQuaternion(eulerX, eulerY, eulerZ) {
  const x = eulerX * Math.PI / 180;
  const y = eulerY * Math.PI / 180;
  const z = eulerZ * Math.PI / 180;
  const c1 = Math.cos(x/2), s1 = Math.sin(x/2);
  const c2 = Math.cos(y/2), s2 = Math.sin(y/2);
  const c3 = Math.cos(z/2), s3 = Math.sin(z/2);
  return {
    x: s1 * c2 * c3 - c1 * s2 * s3,
    y: c1 * s2 * c3 + s1 * c2 * s3,
    z: c1 * c2 * s3 - s1 * s2 * c3,
    w: c1 * c2 * c3 + s1 * s2 * s3
  };
}
function calculateZmeters(eyeDistancePixels, pixelSize, ipd, focalLength, forwardVector) {
  const normForward = normalize(forwardVector);
  const cameraZAxis = { x: 0, y: 0, z: 1 };
  const dotVal = dot(normForward, cameraZAxis);
  const angle = Math.acos(Math.min(Math.max(dotVal, -1), 1));
  const adjustedIpd = ipd * Math.cos(angle);
  const ipdAdjustment = (focalLength * adjustedIpd) / (eyeDistancePixels * pixelSize);
  return -ipdAdjustment / 1000;
}
function getEyeCenter(landmarks, idx1, idx2) {
//console.log("getEyeCenter:" + landmarks + ":" + idx1 + ":" + landmarks.length);
  const x = ((landmarks[idx1].x + landmarks[idx2].x) / 2) * video.videoWidth;
  const y = ((landmarks[idx1].y + landmarks[idx2].y) / 2) * video.videoHeight;
  return { x, y };
}

// MediaPipe FaceMesh setup
const faceMesh=new FaceMesh({locateFile:f=>`https://cdn.jsdelivr.net/npm/@mediapipe/face_mesh/${f}`});
faceMesh.setOptions({maxNumFaces:1,refineLandmarks:true,minDetectionConfidence:0.5,minTrackingConfidence:0.5});
faceMesh.onResults(onResults);

// Camera start
function startCamera(){
  // DOM
  video = document.getElementById('video');
  canvas = document.getElementById('output_canvas') || document.getElementById('canvas');
  ctx = canvas.getContext('2d');
  resultDiv = document.getElementById('result');

  new Camera(video,{onFrame:async()=>await faceMesh.send({image:video}),width:parseInt(localStorage.getItem('requestedWidth')),height:parseInt(localStorage.getItem('requestedHeight')),deviceId:localStorage.getItem('cameraSelect')}).start();
}
window.addEventListener('DOMContentLoaded',startCamera);

// onResults: exact same as index__.html
function onResults(results){
  if(!results.multiFaceLandmarks?.length) return;
  const landmarks = results.multiFaceLandmarks[0];
  
  const leftEyeCenter = getEyeCenter(landmarks, 386, 374);
  const rightEyeCenter = getEyeCenter(landmarks, 159, 145);
  // Store for calibration
  window._calibration.leftEye = leftEyeCenter;
  window._calibration.rightEye = rightEyeCenter;
  const dx = leftEyeCenter.x - rightEyeCenter.x;
  const dy = leftEyeCenter.y - rightEyeCenter.y;
  const eyeDistancePixels = Math.sqrt(dx * dx + dy * dy);
  const pixelSize = sensorWidth / video.videoWidth;
  const aspectRate = video.videoWidth / video.videoHeight;

  const chin3D = { x: landmarks[152].x, y: landmarks[152].y, z: landmarks[152].z };
  const leftEye3D = { x: landmarks[33].x, y: landmarks[33].y, z: landmarks[33].z };
  const rightEye3D = { x: landmarks[263].x, y: landmarks[263].y, z: landmarks[263].z };
  const eyeCenter3D = {
    x: (leftEye3D.x + rightEye3D.x) / 2,
    y: (leftEye3D.y + rightEye3D.y) / 2,
    z: (leftEye3D.z + rightEye3D.z) / 2
  };
  const upVector = normalize(subtract(eyeCenter3D, chin3D));
  const eyeToEyeVector = normalize(subtract(rightEye3D, leftEye3D));
  const forwardVector = normalize(cross(upVector, eyeToEyeVector));

  const xAxis = normalize(cross(upVector, forwardVector));
  const yAxis = upVector;
  const zAxis = forwardVector;
  const rotationMatrix = [
    [xAxis.x, yAxis.x, zAxis.x],
    [xAxis.y, yAxis.y, zAxis.y],
    [xAxis.z, yAxis.z, zAxis.z]
  ];
  let quaternion = matrixToQuaternion(rotationMatrix);
  const rot180 = { x: 0, y: 0, z: 1, w: 0 };
  const rotatedQuaternion = multiplyQuaternion(quaternion, rot180);

  const horizontalFOV = diagonalFov * Math.cos(Math.atan(1 / aspectRate)) * (Math.PI / 180);
  const verticalFOV   = diagonalFov * Math.sin(Math.atan(1 / aspectRate)) * (Math.PI / 180);
  const angleX = (((leftEyeCenter.x + rightEyeCenter.x) / 2) - (video.videoWidth / 2)) * horizontalFOV / video.videoWidth;
  const angleY = (((leftEyeCenter.y + rightEyeCenter.y) / 2) - (video.videoHeight / 2)) * -verticalFOV / video.videoHeight;

  const rotationCorrection = eulerToQuaternion(angleY * (180/Math.PI), angleX * (180/Math.PI), 0);
  const finalQuaternion = multiplyQuaternion(rotatedQuaternion, rotationCorrection);

  let zMeters = calculateZmeters(eyeDistancePixels, pixelSize, ipd, focalLength, forwardVector);
  zMeters = posZFilter.update(zMeters);

  const leftEyeX = ((leftEyeCenter.x - video.videoWidth / 2) * pixelSize / focalLength) * zMeters;
  const leftEyeY = ((leftEyeCenter.y - video.videoHeight / 2) * pixelSize / focalLength) * zMeters;
  const rightEyeX = ((rightEyeCenter.x - video.videoWidth / 2) * pixelSize / focalLength) * zMeters;
  const rightEyeY = ((rightEyeCenter.y - video.videoHeight / 2) * pixelSize / focalLength) * zMeters;
  const eyeCenterX = (leftEyeX + rightEyeX) / 2;
  const eyeCenterY = (leftEyeY + rightEyeY) / 2;

  const filteredPosX = posXFilter.update(eyeCenterX);
  const filteredPosY = posYFilter.update(eyeCenterY);
  const filteredPosZ = zMeters;
  const filteredQuatX = quatXFilter.update(finalQuaternion.x);
  const filteredQuatY = quatYFilter.update(finalQuaternion.y);
  const filteredQuatZ = quatZFilter.update(finalQuaternion.z);
  const filteredQuatW = quatWFilter.update(finalQuaternion.w);

  //ProcessResult(filteredPosX, filteredPosY, filteredPosZ,
  //              filteredQuatX, filteredQuatY, filteredQuatZ, filteredQuatW);


  resultDiv.innerText=`Position:(${filteredPosX.toFixed(3)}, ${filteredPosY.toFixed(3)}, ${filteredPosZ.toFixed(3)})\nRotation(quat):(${filteredQuatX.toFixed(3)}, ${filteredQuatY.toFixed(3)}, ${filteredQuatZ.toFixed(3)}, ${filteredQuatW.toFixed(3)})`;
  if(document.unityInstance?.SendMessage) document.unityInstance.SendMessage('HeadTracker','OnReceiveFaceData',`${filteredPosX.toFixed(3)},${filteredPosY.toFixed(3)},${filteredPosZ.toFixed(3)},${filteredQuatX.toFixed(3)},${filteredQuatY.toFixed(3)},${filteredQuatZ.toFixed(3)},${filteredQuatW.toFixed(3)}`);
}
