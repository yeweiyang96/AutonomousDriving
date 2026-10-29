# 连续导航协议 ADR v3

实现基线：`hid_usb` 提交 `b2d73c15ce5f9389490409fdbf77093c68aae106`。新增通道与原 0–10 命令并存。UDP 仍为 8888。整数小端，精确长度，保留字段必须为零。

## 请求（40 字节）

Python 布局 `<4sBBHHHIIIIIB6sB`：

`ADRV, version=3, op, lease_ms, hold_ms, reserved=0, token, boot, epoch, session, seq, modifier, keys[6], reserved=0`。

`op`：0 只读查询；1 OPEN；2 SET；3 STOP；4 RENEW；5 扩展只读诊断。`token` 用于请求回应匹配。`session` 是客户端随机非零整数。`seq` 使用 uint32 半区间比较；重复和旧序号不能延长租约或重启动作。

- QUERY 的状态字段为零，不改变输入。
- OPEN 携带查询所得的 boot/epoch，seq=0，空键，lease=100–300 ms。只有设备可用、原动作和队列为空时接受。接受后 epoch 前进；完成旧引擎零释放才允许新键输出。相同 OPEN 的重复包只返回状态。
- SET 携带当前 boot/epoch/session 和新 seq，设置完整键盘状态；lease=100–300 ms，hold=0 表示持续保持，20–200 ms 表示有界动作。键码为 4–0xDF，零填充，无重复；修饰键使用 modifier。短动作计时从 USB 首次接受目标报告开始。
- RENEW 仅更新会话租约和序号，不改变目标键、不重启 hold。hold=0，modifier 和 keys 清零。短动作到期仅释放键，会话仍需续期。
- STOP 不排队，立即撤销会话、丢弃旧动作并安排 USB 全释放。相同旧会话 STOP 的重复包不能影响后来会话。租约过期同样关闭会话、前进 epoch。迟到 SET/RENEW/OPEN 不能恢复已关闭的会话。

原命令 10 也改为抢占全部释放；新会话占用时拒绝其他旧输入命令。健康查询保持可用。

## 响应（48 字节）

Python 布局 `<4sBBBB7IHHB6sB`：

`ADRS, version=3, result, flags, reserved=0, token, boot, epoch, session, accepted_seq, applied_seq, uptime_ms, lease_remaining_ms, hold_remaining_ms, actual_modifier, actual_keys[6], reserved=0`。

flags：bit0 USB 可用；bit1 Wi-Fi 可用；bit2 会话活动；bit3 USB 状态提交/释放待完成；bit4 短动作已到期；bit5 可 OPEN。

result：0 成功；1 参数错误；2 boot/epoch 过期；3 会话不匹配/关闭；4 忙或不可用；5 旧序号。

`applied_seq` 只记录最近成功提交到 TinyUSB 的 SET 序号，RENEW 绝不推进它。否则丢失 SET 后收到的 RENEW 会掩盖目标键盘状态从未应用的问题。actual keys 是 TinyUSB 最近接受的报告，不证明游戏已经处理。短动作到期后 actual keys 以成功提交的零报告为准。位置和朝向变化仍由游戏遥测确认。

## 客户端约束

默认租约 250 ms，每约 50 ms 基于新鲜遥测续期。非阻塞接收响应，不让网络查询阻塞截图或停止请求。超过 180 ms 没有新的有效序号确认时撤销控制并请求 STOP；固件也必须独立检查租约。重复回应、旧会话和旧序号不能刷新确认有效时间。

活动会话在处理回应前先检查已有确认期限。控制循环停顿超过期限后，积压在接收缓冲区的回应不能按当前时间刷新确认、恢复续期；必须由用户重新开始。

采集/解码与控制调度分线程；采样时间仍在实际截图开始时记录。实际发送 SET/RENEW 前也复核 180 ms 确认期限，避免窗口检查等处理迟延后继续发令。`lease_timeout` 保存停止前的轮询间隔、上次确认年龄、最新发送年龄及已发/已确认序号，区分主机轮询迟延、没有继续续期和仍有状态未确认；`phase` 标明检查发生在轮询还是发令。`lease_status.rtt_ms` 为发送到客户端处理回应的应用层时间，包含轮询调度，不应单独解释为无线延迟。日志中的 reported lease 是设备生成上次回应时的值，不是停止时的实际剩余期限。

直线持续 W；校准、转向和接近拐点使用短 hold。短动作先观察 USB 零状态，再等待其后的新遥测，续期不会重复执行。停止/失焦/战斗/无有效新帧后由用户手动恢复。生产客户端只发送会话限定 STOP，不混发旧全局释放。旧固件不得自动降级为无限按住。

上述时间是控制参数，实际 USB 释放和游戏停止延迟必须实机测量。源码测试、交叉编译和实机验收分别记录。

## 扩展只读诊断（固件 0.5.1）

op=5 请求仍为 40 字节；除 `ADRV, version=3, op=5, token` 外所有字段为零。它不 OPEN、不续租、不改变输入，也不计作任何 SET/RENEW 的确认。原有 48 字节 ADRS 控制响应完全不变。诊断通过独立 socket 返回 160 字节 ADRD，避免大响应进入控制确认通道。

| 偏移 | 布局 | 内容 |
| --- | --- | --- |
| 0 | `<4sBBH8I`，40 字节 | `ADRD, format=1, result, size=160, token, boot, uptime_ms, firmware_version, capabilities, lease_bounds, hold_bounds, validity` |
| 40 | `<5I` | 最近停止 `reason, time_ms, epoch, session, seq` |
| 60 | `<4I` | 最近键盘报告提交 `epoch, session, seq, time_ms` |
| 76 | `<4I` | 最近键盘报告实际 USB 完成，同上 |
| 92 | `<4I` | 最近零报告提交，同上 |
| 108 | `<4I` | 最近零报告实际 USB 完成，同上 |
| 124 | `<6I` | 网络发送错误、USB 错误、Core0 最大循环间隔 µs、Core1 最大循环间隔 µs、未匹配 USB 完成次数、USB 跟踪溢出次数 |
| 148 | `<I` | `protocol_bounds` |
| 152 | 8 字节 | 保留，均为零 |

版本编码为 `major<<24 | minor<<16 | patch<<8 | build`，当前 `0x00050100`。范围字段均为 `max<<16 | min`，当前协议 3–3、租约 100–300 ms、定时动作 20–200 ms。能力位 0–5 依次为连续按键、定时短动作、抢占 STOP、扩展诊断、真实 USB 完成记录、诊断计数；当前 `0x3F`。

`validity` 的 bit0–4 分别说明停止、最近提交、最近完成、零提交、零完成记录有效；bit8、bit9 分别说明最近提交/最近完成是零释放。无效事件不能解释为时间 0 或延迟 0。USB 提交是 TinyUSB 接受报告；完成时间来自 TinyUSB 完成回调，均不证明游戏已经处理输入。完成无法匹配或跟踪溢出时，计数增加且对应诊断可能无有效完成记录。

停止原因：1 客户端 STOP；2 租约到期；3 Wi-Fi 不可用；4 USB 不可用；5 USB 传输失败；6 USB 提交停滞；7 旧命令 10；8 内部取消。首次活动会话被撤销时记录原因和被撤销的 epoch/session/最后 SET 序号；重试 STOP 或随后断线不覆盖该原因。零释放报告继续保留被撤销会话身份，不使用之后的新会话身份。

时间为设备 uint32 毫秒时钟，精度 1 ms，允许回绕；不与电脑时钟直接相减。提交→完成仅在 epoch/session/seq/是否释放全部匹配时计算。停止→零完成仅在同一 boot、被撤销 epoch/session 且时间有向差小于半个 uint32 范围时计算；不要求停止序号等于释放序号，不跨会话相减。记录被后续事件覆盖时显示“无可关联记录”。计数及最大循环间隔从本次固件启动累计；对比时同时核对 boot。

客户端只在用户点击“连接诊断”且未运行时发送一次查询；暂停、故障、停止或到达后，延迟至少 350 ms 记录一次快照，自动快照之间至少间隔 5 秒。开始、继续和校准取消未完成诊断。运行中不增加周期诊断报文或等待；诊断不会刷新控制 ACK、遥测或租约期限，失败只写 `diagnostic_error`，保留原停止原因。

旧 ADR v3 固件若返回 op=5 参数错误 ADRS，直接显示“扩展诊断未提供”，不增加探测。若完全未回应，最多追加一次原协议基本健康查询；成功仅说明基本通道可达，不证明支持连续导航。读取到的协议、能力、租约或动作范围若不兼容，界面明确说明；缓存仍属同一 boot 时，开始会话后、发送移动前再次拒绝不兼容设备。换地址或重新连接清除旧设备摘要，不自动查询。

保存只读诊断用于刷机前后比对：

```powershell
.\.venv\Scripts\python.exe -m adrive.diagnostics --firmware --count 1 --output runtime\firmware-diagnostic.jsonl
```

输出保留完整事件有效位、原始设备时间、版本/能力范围和计数，也保存中文解释。此命令不发送键盘、鼠标、OPEN、RENEW 或 STOP。
