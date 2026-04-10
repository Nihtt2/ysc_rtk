-- 机器狗路线持久化（与 ysc/db/route_db.py 中 ensure_schema 一致）
-- 用法：mysql -u root -p < schema.sql
-- 或：在 mysql 客户端中逐段执行

CREATE DATABASE IF NOT EXISTS robot_routes
  DEFAULT CHARACTER SET utf8mb4
  DEFAULT COLLATE utf8mb4_unicode_ci;

USE robot_routes;

CREATE TABLE IF NOT EXISTS routes (
  id VARCHAR(128) NOT NULL PRIMARY KEY COMMENT '平台路线唯一 ID',
  name VARCHAR(256) NOT NULL DEFAULT '' COMMENT '路线显示名，可重复',
  next_wp_index INT UNSIGNED NOT NULL DEFAULT 0 COMMENT '下一个航点 0-based 索引',
  status ENUM('active', 'completed', 'aborted') NOT NULL DEFAULT 'active',
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE IF NOT EXISTS route_waypoints (
  route_id VARCHAR(128) NOT NULL COMMENT '外键 -> routes.id',
  seq INT UNSIGNED NOT NULL COMMENT '顺序 1..N',
  name VARCHAR(256) NOT NULL DEFAULT '',
  lon DOUBLE NOT NULL,
  lat DOUBLE NOT NULL,
  action_type INT NOT NULL DEFAULT 0,
  `time` DOUBLE NOT NULL DEFAULT 0,
  PRIMARY KEY (route_id, seq),
  CONSTRAINT fk_route_waypoints_route
    FOREIGN KEY (route_id) REFERENCES routes(id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 授权示例（把用户名、主机改成你的应用账号）：
-- CREATE USER IF NOT EXISTS 'robot'@'localhost' IDENTIFIED BY '你的密码';
-- GRANT SELECT, INSERT, UPDATE, DELETE ON robot_routes.* TO 'robot'@'localhost';
-- FLUSH PRIVILEGES;
