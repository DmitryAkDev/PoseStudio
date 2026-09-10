<#
.SYNOPSIS
  Sends one correctly signed PoseStudio install ping, exactly as the app does, so the server
  handler (ping.php) can be exercised without launching the app.

.EXAMPLE
  .\send-test-ping.ps1 -Key devtestkey                       # new random install id -> a row in installs
  .\send-test-ping.ps1 -Key devtestkey -Install <same id>    # repeat -> launches + 1 on that row
  .\send-test-ping.ps1 -Key devtestkey -Tamper               # wrong signature -> a bad_signature row in ping_rejects
  .\send-test-ping.ps1 -Key devtestkey -Url http://127.0.0.1:8765/ping

  In production the response is always an empty HTTP 204 — look in the tables to see whether the
  ping was counted. With POSESTUDIO_PING_DEBUG defined on the server (README, "Troubleshooting")
  the response body says "accepted: …", "rejected: <reason>", or "error: <message>", printed below.
#>
param(
  [string]$Url       = 'https://www.posestudio.io/ping',
  [string]$Key       = '',
  [string]$Install   = '',
  [string]$Version   = '0.3.12',
  [string]$Os        = 'windows',
  [string]$OsVersion = 'Windows 11 Version 24H2',
  [string]$Kind      = 'portable',
  [switch]$Tamper
)
if (-not $Key) { throw 'Pass -Key <the shared HMAC key> (the value defined as POSESTUDIO_PING_KEY on the server).' }
if (-not $Install) { $Install = [guid]::NewGuid().ToString() }   # a v4 UUID, lowercase, like QUuid::createUuid()
$ts = [int64]((Get-Date).ToUniversalTime() - [datetime]'1970-01-01').TotalSeconds

# Keys sorted, compact: byte-for-byte what InstallPing::buildPayload() produces.
$body = '{"arch":"x86_64","install":"' + $Install + '","kind":"' + $Kind + '","os":"' + $Os +
        '","os_version":"' + $OsVersion + '","qt":"6.11.1","ts":' + $ts + ',"version":"' + $Version + '"}'
$bytes = [Text.Encoding]::UTF8.GetBytes($body)

$hmac = New-Object System.Security.Cryptography.HMACSHA256
$hmac.Key = [Text.Encoding]::UTF8.GetBytes($Key)
$sig = ([BitConverter]::ToString($hmac.ComputeHash($bytes))).Replace('-', '').ToLower()
if ($Tamper) { $sig = ('0' * 64) }

$headers = @{ 'X-PoseStudio-Signature' = $sig; 'User-Agent' = "PoseStudio/$Version (send-test-ping.ps1; x86_64)" }
try {
  $resp = Invoke-WebRequest -Uri $Url -Method Post -ContentType 'application/json' -Headers $headers -Body $bytes -UseBasicParsing
  "HTTP $($resp.StatusCode)   install=$Install   tampered=$($Tamper.IsPresent)"
  $content = [string]$resp.Content
  if ($content) { "server says: $($content.Trim())" }
} catch {
  $r = $_.Exception.Response
  if (-not $r) { throw }
  $reader = New-Object IO.StreamReader($r.GetResponseStream())
  "HTTP $([int]$r.StatusCode)   install=$Install   tampered=$($Tamper.IsPresent)"
  "server says: $($reader.ReadToEnd().Trim())"
}
"body: $body"
