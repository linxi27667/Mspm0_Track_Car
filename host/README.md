# AI 远程调参上位机

PC 端上位机，对接 MSPM0 循迹小车固件（ESP-01S WiFi 透传），实现 AI 远程闭环调参。

**角色边界**：上位机仅做调试与日志，不参与运动依赖。车载运动以已接受指令与本地状态机为准；上位机断连、TCP/Wi-Fi 丢失或心跳缺失不得停止、暂停或改变任何进行中的运动。

## 架构

```
┌─────────────┐   UART1+DMA    ┌──────────┐   TCP/JSON    ┌─────────────┐
│  MSPM0G3507 │ ──────────────>│ ESP-01S  │ ────────────> │   上位机    │
│  循迹小车   │    10Hz 遥测   │  (AT透传)│  <──────────  │  Flask+WS   │
│  (track.c)  │ <─────────────│          │   set/cmd     │             │
└─────────────┘   set/cmd      └──────────┘               └──────┬──────┘
                                                                     │
                                                          ┌──────────┴──────────┐
                                                           │  浏览器实时数据台   │
                                                          │  + LocalRuleEngine  │
                                                          │  + GLM API (可选)   │
                                                          └─────────────────────┘
```

## 文件清单

| 文件 | 作用 |
|------|------|
| `protocol.py` | 协议常量与 JSON 解析/组包（PARAMS/clamp/parse_telemetry/build_set/build_cmd） |
| `tcp_server.py` | TCP server 线程（监听 8888，单连接，按行解析遥测） |
| `tuning_engine.py` | 调参引擎（LocalRuleEngine 6 条规则 + GlmEngine GLM-4-Flash） |
| `config.py` | 配置类（端口/窗口/AI 分析帧数等） |
| `app.py` | Flask + SocketIO 主程序 |
| `templates/index.html` | Web 界面（原始遥测数据台 + 参数面板） |
| `static/app.js` | 前端逻辑（SocketIO 客户端 + 数据更新 + 参数收发） |
| `static/style.css` | 深色主题样式 |
| `simulator.py` | 小车端模拟器（无实车验证用，100Hz 压力遥测 + 接收参数/指令） |
| `requirements.txt` | Python 依赖 |

## 快速开始

### 1. 安装依赖

```powershell
cd host
pip install -r requirements.txt
```

### 2. 无实车验证（模拟器模式）

```powershell
# 终端 1：启动上位机
python app.py

# 终端 2：启动模拟器（模拟小车 100Hz 遥测）
python simulator.py
```

浏览器打开 http://localhost:5000 即可看到实时波形。

### 3. 实车联调

1. 修改固件 `APP/Inc/app_esp01s_cfg.h`：填 WiFi SSID/PASS、上位机 IP（PC 局域网 IP）、端口 8888
2. 烧录固件，ESP-01S 上电后会自动连接上位机
3. 上位机日志显示 `小车连接: x.x.x.x:xxxxx` 即联调成功

## 协议规范

### 下行（PC → 小车）

**参数设置**（字段可选，缺省字段不改）：
```json
{"set":{"KP":3.5,"KI":0.0,"KD":1.5,"GDAMP":0.25,"VBASE":18,"VSTR":20,"VCUR":16,"RATHR":6.0}}
{"set":{"MKP":0.55,"MKI":0.0,"MKD":0.0,"MSL":0.161,"MSR":0.164}}
```

**控制指令**：
```json
{"cmd":"start"}      // 启动循迹
{"cmd":"stop"}       // 停止
{"cmd":"reconnect"}  // 重连 WiFi
```

鱼眼黑白标定改由模块板载按键完成，不再提供旧八路 ADC 的远程白/黑校准命令。电机环与比例尺仅允许 TUNING 构建在车辆静止或电机台架测试时修改。

### 上行（小车 → PC，10Hz，以 `\n` 结尾）

```json
{"v":1,"t":"tele","line":[0,0,0,0,0,1,1,0,0,0,0,0],"lm":96,"bc":2,"sa":7,"se":0,"bp":1,"gz":0.12,"el":20,"er":19,"le":0.0,"po":0,"st":"LF"}
```

| 字段 | 含义 | 范围 |
|------|------|------|
| `line` | A1→A12 统一后的黑线位 | 12 个 `0/1` |
| `lm` | 12 位黑线掩码，bit0=A1 | 0~4095 |
| `bc` | 当前黑点数 | 0~12 |
| `sa` | 最近完整鱼眼帧龄 | ms |
| `se` | 鱼眼通信错误累计值 | ≥0 |
| `bp` | 黑线原始极性 | 0/1 |
| `gz` | 陀螺仪 Z 轴角速度 | deg/s |
| `el`/`er` | 左/右编码器速度 | pulse/5ms |
| `le` | 循迹误差 | -3.5~+3.5 |
| `po` | PID 转向输出 | -100~+100 |
| `st` | 状态缩写 | ID/LF/RA/LL/SL/CR/FN |
| `kp`/`kd` | 当前 PID 参数 | 用于确认参数已生效 |

### 参数钳位范围（与固件 track.c setter 一致）

| 参数 | 范围 | 默认 | 说明 |
|------|------|------|------|
| `KP` | 0~50 | 3.5 | 循迹比例增益 |
| `KI` | 0~5 | 0.0 | 循迹积分增益 |
| `KD` | 0~20 | 1.5 | 循迹微分增益 |
| `GDAMP` | 0~1 | 0.25 | 陀螺仪阻尼系数 |
| `VBASE` | 0~100 | 18 | 基础 PWM 百分比 |
| `VSTR` | 0~100 | 20 | 直道 PWM 百分比 |
| `VCUR` | 0~100 | 16 | 弯道 PWM 百分比 |
| `RATHR` | 5~10（整数） | 6.0 | 直角候选最少黑通道数 |
| `MKP` | 0~5 | 0.55 | 电机速度环 P |
| `MKI` | 0~1 | 0.0 | 电机速度环 I |
| `MKD` | 0~5 | 0.0 | 电机速度环 D |
| `MSL` | 0.01~2 | 0.161 | 左电机前馈比例尺 |
| `MSR` | 0.01~2 | 0.164 | 右电机前馈比例尺 |

## AI 调参

### 本地规则引擎（默认，离线可用）

6 条规则基于遥测历史分析：

| 触发条件 | 建议 | 置信度 |
|----------|------|--------|
| 误差震荡明显（≥30% 帧变号） | KD +50% | 0.7 |
| 误差偏大（平均 |error| > 1.5） | KP +20% | 0.7 |
| 陀螺仪抖动（|gz| > 2 deg/s） | GDAMP +0.05 | 0.6 |
| 弯道频繁丢线（LL 状态 > 5%） | VCUR -5 | 0.6 |
| 直角弯频繁（RA 状态 > 10%） | RATHR +1 | 0.6 |
| 正常 | 无建议 | 0.5 |

### GLM 引擎（配置 API key 后启用）

创建 `host_config.json`：
```json
{
  "GLM_API_KEY": "your_glm_api_key",
  "GLM_MODEL": "glm-4-flash"
}
```

上位机启动时会自动切换到 GlmEngine。

### 调参流程（半自动）

1. 浏览器观察波形，确认小车运行状态
2. 点击「AI 分析」按钮，引擎分析最近 100 帧遥测
3. AI 返回建议参数 + 原因 + 置信度
4. 人工审核后点击「采纳建议」（参数填入面板，不自动下发）
5. 调整后点击「应用参数」下发到小车

## 配置

`host_config.json`（可选，覆盖默认值）：

```json
{
  "TCP_PORT": 8888,
  "WEB_HOST": "127.0.0.1",
  "WEB_PORT": 5000,
  "GLM_API_KEY": "",
  "GLM_MODEL": "glm-4-flash",
  "TELEMETRY_WINDOW": 500,
  "AI_ANALYZE_FRAMES": 100
}
```

Web 默认只监听本机。确需从局域网访问时，显式设置 `WEB_HOST`，并同时做好主机防火墙和访问控制。

## 硬件注意事项（ESP-01S）

- **独立供电**：ESP-01S 必须用独立 AMS1117-3.3（500mA+），不能接 MCU 板载 LDO，否则发送瞬间压降导致 MCU 复位
- **去耦电容**：ESP-01S VCC 与 GND 之间并联 10μF 钽电容 + 0.1μF 陶瓷电容
- **UART 接线**：MSPM0 PA8 (TX) → ESP-01S RXD，PA9 (RX) → ESP-01S TXD，共地
- **天线朝向**：ESP-01S PCB 天线朝上，避免金属遮挡

## 故障排查

| 现象 | 排查 |
|------|------|
| 上位机无连接 | 检查 ESP-01S 是否连上 WiFi、上位机 IP 是否正确、防火墙是否放行 8888 |
| 连接但不显示波形 | 检查浏览器控制台 SocketIO 是否连接、`st` 字段是否合法 |
| 参数下发无效 | 检查固件 `App_Tuning_OnRx` 是否被调用、setter 是否钳位 |
| AI 分析返回空 | 检查遥测历史是否为空（至少需要 1 帧） |
| 模拟器连接被拒 | 确认上位机已启动（`python app.py`），TCP 8888 监听中 |

## 验证状态

- ✅ 协议层单元测试（parse_telemetry/build_set/build_cmd/clamp）
- ✅ TCP 遥测接收（固件 10Hz、模拟器 100Hz，按 `\n` 分割）
- ✅ SocketIO 状态查询（frame_count/client_connected/engine）
- ✅ LocalRuleEngine 震荡规则触发（建议 KD +50%）
- ✅ 参数下发端到端（上位机 → TCP → 模拟器接收）
- ✅ 控制指令下发（start/stop/reconnect）及鱼眼 12 路遥测
- ✅ 浏览器逐帧显示 MCU 遥测字段和原始 JSON
