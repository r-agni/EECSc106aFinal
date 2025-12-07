var arDrone = require('ar-drone');
var client = arDrone.createClient();

let hasLanded = false;   // <--- the key to canceling .after landing

// Shared landing function
function landNow(reason) {
  if (hasLanded) return;   // prevents landing twice
  hasLanded = true;

  console.log(`[LAND] Landing now (${reason})`);
  client.stop();
  client.land();

  // Exit after 3 seconds to send full land commands
  setTimeout(() => {
    console.log("[EXIT] Done.");
    process.exit(0);
  }, 3000);
}

// Keyboard handler for 'l' key emergency land
if (process.stdin.isTTY) {
  process.stdin.setRawMode(true);
  process.stdin.resume();
  process.stdin.on('data', function (key) {
    key = key.toString();

    if (key === 'l') {
      landNow("manual L key");
    }
  });
}

console.log("Taking off...");
client.takeoff();

// --- AUTO LAND AFTER 10 SECONDS ---
client.after(10000, function() {
  if (hasLanded) {
    console.log("[SKIP] Auto-land ignored because manual land already happened.");
    return;
  }

  landNow("auto 10s");
});