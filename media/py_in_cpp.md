C++パッケージにPythonファイルを追加する方法

1. `lucia_navigation2` フォルダにpythonファイルを置く
2. `CMakeLists.txt`に以下の内容を追加
```cmake
# Pythonノードをインストール
install(PROGRAMS
  lucia_navigation2/nav2_test.py
  lucia_navigation2/waypoint_navi.py
  lucia_navigation2/[追加したpythonのファイル名]
  DESTINATION lib/${PROJECT_NAME}
)
```
3. 実行権限を付与する
```bash
chmod +x ./src/lucia_navigation2/lucia_navigation2/[追加したpythonのファイル名]
```
4. Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select lucia_navigation2
```