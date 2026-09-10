-- PoseStudio install-ping tables (MySQL 5.7+ / MariaDB). See ping.php and README.md.
-- Run once:  mysql -u <user> -p <database> < schema.sql

-- One row per installation, keyed on the client's random install id. This is what "how many
-- installs" is counted from: repeat pings only touch last_seen / launches / ping_days.
CREATE TABLE IF NOT EXISTS installs (
  install_id     CHAR(36)      NOT NULL,
  first_seen     DATETIME      NOT NULL,
  last_seen      DATETIME      NOT NULL,
  version        VARCHAR(16)   NOT NULL,   -- the version last seen running
  os             VARCHAR(32)   NOT NULL,   -- windows / macos / ubuntu / ...
  os_version     VARCHAR(64)   NOT NULL,   -- "Windows 11 Version 24H2"
  arch           VARCHAR(16)   NOT NULL,   -- x86_64 / arm64
  kind           VARCHAR(16)   NOT NULL,   -- installer / portable
  qt             VARCHAR(16)   NOT NULL,
  launches       INT UNSIGNED  NOT NULL DEFAULT 1,  -- accepted pings, ever
  ping_days      INT UNSIGNED  NOT NULL DEFAULT 1,  -- distinct days with a ping ("confirmed" installs have >= 2)
  last_ping_day  DATE          NOT NULL,
  first_ip_hash  CHAR(64)      NOT NULL,   -- daily-rotating keyed hash, never a raw address
  PRIMARY KEY (install_id),
  KEY idx_last_seen (last_seen),
  KEY idx_version   (version),
  KEY idx_first_ip  (first_ip_hash, first_seen)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- One row per accepted ping: launches per day, per version. Prune after ~90 days (README).
CREATE TABLE IF NOT EXISTS ping_log (
  id           BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  install_id   CHAR(36)        NOT NULL,
  received_at  DATETIME        NOT NULL,
  client_ts    BIGINT          NOT NULL,   -- the client's own clock, for skew curiosity only
  version      VARCHAR(16)     NOT NULL,
  ip_hash      CHAR(64)        NOT NULL,
  PRIMARY KEY (id),
  KEY idx_install_day (install_id, received_at),
  KEY idx_received    (received_at)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Every request that was NOT counted, and why: browser visits, crawlers, bad signatures, quota
-- hits. Purely for watching abuse; the sender never learns which reason applied. Prune after ~30 days.
CREATE TABLE IF NOT EXISTS ping_rejects (
  id           BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  received_at  DATETIME        NOT NULL,
  reason       VARCHAR(32)     NOT NULL,
  method       VARCHAR(8)      NOT NULL,
  ip_hash      CHAR(64)        NOT NULL,
  snippet      VARCHAR(200)    NOT NULL DEFAULT '',  -- first 200 bytes of the body (or content type)
  PRIMARY KEY (id),
  KEY idx_received (received_at),
  KEY idx_reason   (reason, received_at)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;
