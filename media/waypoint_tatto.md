
## description
🔄 通常の巡回移動 → waypoints巡回中
🚨 /sleep_detect_mode で true を受信
⏸️ 現在のナビゲーションをキャンセル
🎯 睡眠検出用waypointへ移動
🔔 到着後、起こす動作を実行（perform_wake_up_action）
👆 接触センサのタッチ待ち
✅ /sensor_threshold_exceeded で true を受信
🔄 巡回移動に復帰

## How to use

Run waypoint tatto
```bash
ros2 run lucia_navigation2 waypoint_tatto.py
```

Send sleep_detect_mode
```bash
ros2 topic pub --once /sleep_detect_mode std_msgs/msg/Bool "data: true"
```