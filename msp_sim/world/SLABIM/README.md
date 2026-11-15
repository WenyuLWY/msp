# SLABIM -> Gazebo Classic 11 楼层资源包

由 `bim2gazebo.py --mode floors` 从 `SLABIM/BIM/BIM/<层>/mesh/*.ply` 生成，
几何保持 BIM 原始坐标，未裁剪、未压缩、未降面，四类语义（墙/门/柱/楼板）全部保留。

## 目录结构

```
SLABIM/
├── readme.md
├── floor1/
│   ├── floor1.world      # 可直接 gzserver 加载
│   ├── model.config      # 模型描述
│   ├── model.sdf         # 同内容的模型包
│   └── meshes/
│       ├── walls.dae
│       ├── doors.dae
│       ├── columns.dae
│       └── floors.dae
├── floor2/ ... floor5/
```

每个 world 与 model.sdf 都用

```xml
<mesh><uri>model://SLABIM/floor1/meshes/walls.dae</uri></mesh>
```

引用网格。Gazebo 11.15.1 的 `SystemPaths::FindFileURI` 对 `model://` 的处理，是把
前缀之后的部分直接拼到 `GAZEBO_MODEL_PATH` 的每一项上再判断存在
（`path = modelPath / suffix`），所以多级路径可行。你的 launch 已设置

```xml
<env name="GAZEBO_MODEL_PATH" value="$(find msp_sim)/world:$(optenv GAZEBO_MODEL_PATH)"/>
```

因此把整个 `SLABIM` 目录放到 `$(find msp_sim)/world/` 下即可命中，
即 `$(find msp_sim)/world/SLABIM/floor1/meshes/walls.dae`。

## 机器人初始位姿（base_footprint）

五层的楼板顶面都在 **z = 0**（板顶 0.00 m，板底 -0.15 m），所以每层都可以把
`base_footprint` 直接放在 `z = 0`，不需要按层做高度补偿。
下表坐标就是 world 坐标（模型 `<pose>` 为 `0 0 0 0 0 0`，与 BIM 坐标一致）：

| 楼层 | x (m) | y (m) | z (m) | yaw (rad) | 最近墙/门/柱 (m) | 可通行面积 (m²) | 30 m 内射线命中 |
|---|---|---|---|---|---|---|---|
| floor1 | `80.85` | `-7.35` | `0.0` | `0.00` | 8.0 | 1886 | 82% |
| floor2 | `186.10` | `-3.83` | `0.0` | `3.58` | 7.8 | 1127 | 97% |
| floor3 | `2.90` | `-27.14` | `0.0` | `5.85` | 5.5 | 203 | 78% |
| floor4 | `62.90` | `-81.58` | `0.0` | `4.80` | 10.2 | 487 | 26% |
| floor5 | `47.37` | `-50.04` | `0.0` | `5.59` | 5.2 | 266 | 46% |

选取方法：

1. 墙、门、柱在 0.3 m 与 1.2 m 两个高度做真实三角面截面（不是顶点投影），
   栅格化成 0.25 m 占据图，再按 0.6 m 净空膨胀；
2. 楼板 `floors` 中 z≈0 的三角面按填充方式栅格化，作为"脚下有板"的判据，
   因此不会选到洞口、楼梯井或楼板轮廓之外；
3. 从栅格边界 flood fill 排除与外界连通的区域，避免落在室外平台或挑檐上；
4. 在剩下的区域里取"到障碍与板边最远"的格子，并要求该点及其周围一圈采样点
   都落在楼板上；yaw 取该点自由行程最长的方向。

表中三个指标各自含义：

- **最近墙/门/柱**：以该点为原点、1.2 m 高度上 72 条水平射线到最近障碍物的距离，
  也就是机器人起步时四周的实际余量。
- **可通行面积**：该点所在的那一块连通自由区（被墙/门/柱和楼板边缘围住）的面积，
  表示机器人不穿墙、不掉边的情况下能走多远。
- **30 m 内射线命中**：72 条射线中在 30 m 内打到墙/门/柱的比例。100% 说明四周
  被完全挡住；比例低说明处在大开敞区或靠近楼板边缘。

**注意：这几层是开敞式平面，最大净空的点通常位于大开敞区，而不是小房间。**
如果你希望从某个具体房间出发，直接改 world 里机器人的 x/y 即可（世界坐标 = BIM 坐标）。

- 若你发布的是 `base_link`，只需在 z 上加上自己机器人的
  `base_footprint -> base_link` 高度差（各机型不同）。
- 局部楼板面实测在 -0.04 m 附近（不是精确 0），若起步有轻微下坠，把 z 抬到
  `0.05` 即可。
- 这些位姿只保证"脚下有楼板 + 净空足够"，**没有**在 Gazebo 里实际运行验证过。

## 几何规模

| 楼层 | 三角形总数（含双面） | DAE 体积 | 各 link |
|---|---|---|---|
| floor1 | 3643896 | 101.1 MB | walls=1130580, doors=2076848, columns=359280, floors=77188 |
| floor2 | 3861074 | 106.9 MB | walls=1427442, doors=2127720, columns=217172, floors=88740 |
| floor3 | 4809026 | 132.0 MB | walls=1517406, doors=2873040, columns=333240, floors=85340 |
| floor4 | 7133524 | 203.4 MB | walls=1511352, doors=5304080, columns=242668, floors=75424 |
| floor5 | 2275708 | 62.4 MB | walls=769156, doors=1211288, columns=243680, floors=51584 |

## 注意事项

- **没有 `ground_plane`**：地面由 BIM 的 `floors` 楼板承担，避免与 z=0 的
  Gazebo 平面重叠产生 z-fighting。楼板上的洞口（楼梯、管井）是真实存在的。
- **碰撞做了双面**：Rhino 导出的是开放曲面（1F 墙有 32.6% 的边只被一个三角形
  使用），单面网格从法线背面可能被穿透，所以每个三角形都复制了一份反向，
  三角形数量因此翻倍。若确认物理表现正常，可用 `--single-sided` 重跑减半。
- **门是关闭的实体**，会挡住通道。需要通行时把 `doors.dae` 从 world 里删掉即可。
- **GUI 可能提示 `Missing model.config`**：`SLABIM` 是容器目录不是模型包，
  Gazebo 的 Insert 列表会扫描 `GAZEBO_MODEL_PATH` 的每个子目录。这条告警不影响加载。
- 4F 的门单独就有 265 万面，加载最慢；若卡顿可用
  `--max-tris-per-link 400000` 降面重跑（会改变几何精度，默认不启用）。
- 楼层 z 范围不统一（1F 到 -5.15 m、3F 到 15.0 m），说明原始 PLY 里含有
  竖井/中庭/屋面等跨层构件，本次按"整层原样"保留，未按高度裁剪。

## 验证

```bash
PKG="$(rospack find msp_sim)/world/SLABIM"
cd "$PKG/floor1"
timeout 30s gzserver --verbose floor1.world 2>&1 | tee /tmp/floor1.log
grep -iE 'err|warn|unable|missing|not found' /tmp/floor1.log

rosservice call /gazebo/get_model_properties "model_name: 'slabim_floor1'"
```
