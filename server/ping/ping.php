<?php
/**
 * PoseStudio install-ping endpoint — served at https://www.posestudio.io/ping
 *
 * Receives the app's per-launch ping (src/core/installping.h in the PoseStudio repository),
 * verifies it, and records it. The contract, exactly as the client emits it:
 *
 *   POST, Content-Type: application/json, body under 1 KB, keys sorted:
 *     {"arch":"x86_64","install":"<uuid v4>","kind":"installer|portable","os":"windows",
 *      "os_version":"Windows 11 Version 24H2","qt":"6.10.3","ts":<unix seconds>,"version":"0.3.12"}
 *   Header X-PoseStudio-Signature: lowercase hex HMAC-SHA256 over the RAW body bytes, keyed with
 *   the secret the release build was compiled with (the POSESTUDIO_PING_KEY GitHub secret).
 *
 * Every response is a 204 with no body, accepted or not — a probe learns nothing. Rejections are
 * logged to ping_rejects (reason + hashed address) so abuse stays visible without being answerable.
 *
 * Counting model: one row per INSTALL (installs), keyed on the client's random install id — a
 * thousand pings from one id are one install — plus one row per accepted ping (ping_log) for
 * launches-per-day. Raw addresses are never stored: only a daily-rotating keyed hash, used to cap
 * how many NEW install ids one address may create per day (the one attack a leaked key allows).
 *
 * Setup (see README.md): run schema.sql, then define the shared key in ../includes/posestudio_inc.php:
 *     define('POSESTUDIO_PING_KEY', '<the same value as the GitHub secret>');
 * dbConnect() (mysqli) and getUserIP() come from that include as well.
 *
 * Troubleshooting a deployment: add  define('POSESTUDIO_PING_DEBUG', true);  to that include and
 * the endpoint stops being silent — 400 "rejected: <reason>", 200 "accepted: <id>", 500 with the
 * exception text (a missing table, a bad credential) — and PHP errors are displayed. Remove it after.
 */
declare(strict_types=1);

// Never print errors into the response — an empty 204 is the whole contract. Log them instead.
ini_set('display_errors', '0');
ini_set('log_errors', '1');
mysqli_report(MYSQLI_REPORT_ERROR | MYSQLI_REPORT_STRICT); // failed queries throw; caught at the bottom

include_once '../includes/posestudio_inc.php';

// Debug mode (POSESTUDIO_PING_DEBUG in the include): answer with the outcome instead of a silent 204.
define('PING_DEBUG', defined('POSESTUDIO_PING_DEBUG') && (bool) POSESTUDIO_PING_DEBUG);
if (PING_DEBUG) {
    ini_set('display_errors', '1');
    error_reporting(E_ALL);
}

// ---- Tunables ----------------------------------------------------------------------------------
const PING_MAX_BODY_BYTES       = 1024;
const PING_CLOCK_WINDOW_SECONDS = 6 * 3600; // |client ts - server time| allowed; wrong clocks happen
const PING_MAX_PER_INSTALL_DAY  = 200;      // launches one install may report per day (runaway-script cap)
const PING_MAX_NEW_INSTALLS_IP  = 25;       // new install ids one address may create per day (office NATs are real)
const PING_LOG_REJECTS          = true;     // keep ping_rejects; prune it periodically (README)
const PING_ALLOWED_VERSIONS     = [];       // e.g. ['0.3.12', '0.3.13']; empty = any x.y.z

// ---- Response: always 204, always empty --------------------------------------------------------
http_response_code(204);
header('Cache-Control: no-store');

$pingKey  = defined('POSESTUDIO_PING_KEY') ? (string) POSESTUDIO_PING_KEY : '';
$clientIp = function_exists('getUserIP') ? (string) getUserIP() : (string) ($_SERVER['REMOTE_ADDR'] ?? '');
// Daily-rotating keyed hash of the address: not reversible, different tomorrow, so the tables hold
// no personal data — just enough to enforce the per-address quota within one day.
$ipHash = hash_hmac('sha256', $clientIp, $pingKey . gmdate('Y-m-d'));

function pingDb(): mysqli {
    static $db = null;
    if ($db === null) {
        $db = dbConnect();
    }
    return $db;
}

/** Records why a request was ignored (never telling the sender), then ends the request. */
function pingReject(string $reason, string $ipHash, string $snippet = ''): void {
    $logError = '';
    if (PING_LOG_REJECTS) {
        try {
            $method  = substr((string) ($_SERVER['REQUEST_METHOD'] ?? ''), 0, 8);
            $snippet = substr($snippet, 0, 200);
            $stmt = pingDb()->prepare(
                'INSERT INTO ping_rejects (received_at, reason, method, ip_hash, snippet) VALUES (NOW(), ?, ?, ?, ?)');
            $stmt->bind_param('ssss', $reason, $method, $ipHash, $snippet);
            $stmt->execute();
            $stmt->close();
        } catch (Throwable $e) {
            $logError = $e->getMessage();
            error_log('ping: could not log rejection (' . $reason . '): ' . $logError);
        }
    }
    if (PING_DEBUG) {
        // A database failure while FILING the rejection is the more important news — a table that
        // doesn't exist, a wrong database, a user without INSERT — so it is reported here too.
        http_response_code($logError === '' ? 400 : 500);
        header('Content-Type: text/plain');
        echo 'rejected: ', $reason, "\n";
        if ($logError !== '') {
            echo 'error: could not write ping_rejects: ', $logError, "\n";
        }
    }
    exit;
}

/** Runs a one-column COUNT-style query with one string parameter and returns the integer result. */
function pingScalar(mysqli $db, string $sql, string $value): int {
    $stmt = $db->prepare($sql);
    $stmt->bind_param('s', $value);
    $stmt->execute();
    $result = 0;
    $stmt->bind_result($result);
    $stmt->fetch();
    $stmt->close();
    return (int) $result;
}

try {
    // 1. Shape: POST + JSON + small. A browser visit, a crawler, or a stray curl ends here —
    //    nothing counted, nothing revealed.
    if (($_SERVER['REQUEST_METHOD'] ?? '') !== 'POST') {
        pingReject('method', $ipHash);
    }
    $contentType = strtolower(trim((string) ($_SERVER['CONTENT_TYPE'] ?? $_SERVER['HTTP_CONTENT_TYPE'] ?? '')));
    if (strncmp($contentType, 'application/json', 16) !== 0) {
        pingReject('content_type', $ipHash, $contentType);
    }
    $raw = (string) file_get_contents('php://input');
    if ($raw === '') {
        pingReject('empty_body', $ipHash);
    }
    if (strlen($raw) > PING_MAX_BODY_BYTES) {
        pingReject('too_large', $ipHash);
    }

    // 2. Signature over the RAW bytes. With no key configured nothing is ever accepted (fail closed).
    if ($pingKey === '') {
        pingReject('no_key_configured', $ipHash);
    }
    $signature = strtolower(trim((string) ($_SERVER['HTTP_X_POSESTUDIO_SIGNATURE'] ?? '')));
    if (!preg_match('/^[0-9a-f]{64}$/', $signature)) {
        pingReject('no_signature', $ipHash, $raw);
    }
    if (!hash_equals(hash_hmac('sha256', $raw, $pingKey), $signature)) {
        pingReject('bad_signature', $ipHash, $raw);
    }

    // 3. Fields: exactly what InstallPing::buildPayload() emits, nothing looser. Extra keys are
    //    ignored so a future client can add fields before the server learns about them.
    $ping = json_decode($raw, true, 4);
    if (!is_array($ping)) {
        pingReject('bad_json', $ipHash, $raw);
    }
    $rules = [
        'install'    => '/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/', // QUuid::createUuid() = v4
        'version'    => '/^\d{1,3}\.\d{1,3}\.\d{1,3}$/',      // Constants::APP_VERSION (CMake's project VERSION); pipeline-test builds ship keyless, so they never get here
        'os'         => '/^[a-z0-9_.-]{1,32}$/',              // QSysInfo::productType(): windows, macos, ubuntu, opensuse-leap...
        'os_version' => '/^[^\x00-\x1f\x7f]{1,64}$/u',         // QSysInfo::prettyProductName()
        'arch'       => '/^[a-z0-9_]{1,16}$/',                // x86_64, arm64
        'kind'       => '/^(installer|portable)$/',
        'qt'         => '/^\d{1,2}\.\d{1,3}(\.\d{1,3})?$/',
    ];
    foreach ($rules as $field => $pattern) {
        if (!isset($ping[$field]) || !is_string($ping[$field]) || !preg_match($pattern, $ping[$field])) {
            pingReject('bad_field:' . $field, $ipHash, $raw);
        }
    }
    if (PING_ALLOWED_VERSIONS !== [] && !in_array($ping['version'], PING_ALLOWED_VERSIONS, true)) {
        pingReject('unknown_version', $ipHash, $raw);
    }
    if (!isset($ping['ts']) || !is_int($ping['ts']) || abs($ping['ts'] - time()) > PING_CLOCK_WINDOW_SECONDS) {
        pingReject('stale_ts', $ipHash, $raw);
    }

    // 4. Quotas: new install ids per address per day, and pings per install per day.
    $db = pingDb();
    $isNew = pingScalar($db, 'SELECT COUNT(*) FROM installs WHERE install_id = ?', $ping['install']) === 0;
    if ($isNew) {
        $newToday = pingScalar($db,
            'SELECT COUNT(*) FROM installs WHERE first_ip_hash = ? AND first_seen >= CURDATE()', $ipHash);
        if ($newToday >= PING_MAX_NEW_INSTALLS_IP) {
            pingReject('ip_new_install_limit', $ipHash, $raw);
        }
    } else {
        $today = pingScalar($db,
            'SELECT COUNT(*) FROM ping_log WHERE install_id = ? AND received_at >= CURDATE()', $ping['install']);
        if ($today >= PING_MAX_PER_INSTALL_DAY) {
            pingReject('install_rate_limit', $ipHash, $raw);
        }
    }

    // 5. Record: one row per install (upsert — the install count can never inflate), one per ping.
    //    ON DUPLICATE KEY UPDATE assigns left to right, so ping_days must read last_ping_day
    //    BEFORE the assignment that moves it to today.
    $stmt = $db->prepare(
        'INSERT INTO installs (install_id, first_seen, last_seen, version, os, os_version, arch, kind, qt, ' .
        '                      launches, ping_days, last_ping_day, first_ip_hash) ' .
        'VALUES (?, NOW(), NOW(), ?, ?, ?, ?, ?, ?, 1, 1, CURDATE(), ?) ' .
        'ON DUPLICATE KEY UPDATE ' .
        '  last_seen = NOW(), version = VALUES(version), os = VALUES(os), os_version = VALUES(os_version), ' .
        '  arch = VALUES(arch), kind = VALUES(kind), qt = VALUES(qt), ' .
        '  launches = launches + 1, ' .
        '  ping_days = ping_days + IF(last_ping_day < CURDATE(), 1, 0), ' .
        '  last_ping_day = CURDATE()');
    $stmt->bind_param('ssssssss', $ping['install'], $ping['version'], $ping['os'], $ping['os_version'],
                      $ping['arch'], $ping['kind'], $ping['qt'], $ipHash);
    $stmt->execute();
    $stmt->close();

    $stmt = $db->prepare(
        'INSERT INTO ping_log (install_id, received_at, client_ts, version, ip_hash) VALUES (?, NOW(), ?, ?, ?)');
    $stmt->bind_param('siss', $ping['install'], $ping['ts'], $ping['version'], $ipHash);
    $stmt->execute();
    $stmt->close();

    if (PING_DEBUG) {
        http_response_code(200);
        header('Content-Type: text/plain');
        echo 'accepted: ', $ping['install'], ($isNew ? ' (new install)' : ' (known install)'), "\n";
    }
} catch (Throwable $e) {
    // A database hiccup must not change the response — the client ignores failures anyway.
    error_log('ping: ' . $e->getMessage());
    if (PING_DEBUG) {
        http_response_code(500);
        header('Content-Type: text/plain');
        echo 'error: ', $e->getMessage(), "\n";
    }
}
