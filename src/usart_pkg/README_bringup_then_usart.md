# bringup 与 usart_node 顺序启动说明

该说明用于通过一个 launch 文件实现以下顺序：

1. 先启动 `leg_bringup` 包中的 `bringup_in_real.launch.py`
2. 等待一段时间（可配置）
3. 再启动 `usart_pkg` 的 `usart_node`

对应 launch 文件：

- `usart_pkg/launch/bringup_then_usart.launch.py`

## 1. 编译

在工作区根目录（`estimations`）执行：

```bash
colcon build --packages-select usart_pkg
source install/setup.bash
```

## 2. 启动（默认延时 25 秒）

```bash
ros2 launch usart_pkg bringup_then_usart.launch.py
```

## 3. 自定义延时启动 usart_node

如果机器启动较慢或你希望更保守，可增大延时：

```bash
ros2 launch usart_pkg bringup_then_usart.launch.py usart_delay_sec:=35.0
```

## 4. 常用参数

- `use_sim_time`：是否使用仿真时间（默认 `false`）
- `usart_delay_sec`：在 `bringup_in_real` 启动后，延迟多少秒再启动 `usart_node`（默认 `25.0`）

示例：

```bash
ros2 launch usart_pkg bringup_then_usart.launch.py use_sim_time:=false usart_delay_sec:=30.0
```

## 5. 说明

`bringup_in_real.launch.py` 本身通常是常驻运行，不会“启动完成后退出”。
因此这里采用“先启动 bringup，再延时启动 usart_node”的方式实现顺序控制。

## 6. 常见报错处理

### 报错 1：旧路径残留（例如指向 `/home/cd/Nav_dog/pre/estimations`）

先清理 `usart_pkg` 的构建缓存再重编译：

```bash
rm -rf build/usart_pkg install/usart_pkg log/latest_build/usart_pkg log/latest_test/usart_pkg
colcon build --packages-select usart_pkg
```

### 报错 2：`No space left on device`

表示磁盘空间不足，先清理大目录再编译，例如：

```bash
df -h .
du -sh build install log
rm -rf log/*
```

必要时清理不需要的包构建产物后再执行：

```bash
colcon build --packages-select usart_pkg
```
