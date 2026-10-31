# 两个 BeamNG 地图包的只读评估

## 结论

**选择 `american_road` 为底图。保留其坡形和乡村路网，先局部改造窄水泥路、石铺路；不做两个 `.ter` 的直接拼接。** 当前产物是有证据的检查报告和候选路线计划，不是完成的地图，`ready:false`。

American Road 已包含 4 米宽混凝土道路和 `bruk2` 石铺路材质，后者明确声明 `groundType: COBBLESTONE`。为了这两种路面，没有必要引入 Shomaru 的整个地形。Shomaru 的窄山路可以作为形态参考；确需导入时，只考虑经授权且确认碰撞、坐标及资源依赖的局部网格。

不能诚实地承诺现在可以自动拼出一张可开、坡度正确、路面真实颠簸的地图。主要缺口是本机运行验证、地形编码验证、网格裁切和接缝处理，以及素材权利确认。

## 边界与检查方法

- 只打开 `C:\Users\Yu\Desktop\Beamng\2K-Shomaru.zip` 和 `american_road_0.39.zip` 两个来源包。没有读取密钥，也没有打开体积巨大的游戏压缩包。
- 没有启动游戏、安装 mod、解压地图到游戏目录或改动第三方文件。报告写入本工作区 `map_merge/` 固定文件名，不把 ZIP 成员名用作输出路径。
- 检查了全 ZIP 目录、大小、CRC 声明、重复路径、大小写碰撞、路径穿越；两包均未发现上述目录问题。符号链接、加密成员、超大或高压缩比元数据有跳过机制，本次无安全预算跳过。
- 解析 JSON、逐行 JSON；允许有注释或尾逗号的元数据。不会为了读懂 Shomaru 的坏 JSON 而擅自补全其语义。原文保留在报告中。
- 全读选中的文本元数据，计算 SHA-256 并触发 ZIP CRC 校验。另读取 Shomaru 的 DAE，使用标准库 XML 迭代解析，仅记录几何标识、单位、材质绑定和图片引用；拒绝 DTD/实体，不导出顶点或复制模型。American Road 的 DAE 内容未解析，碰撞由对象声明提供线索而非运行确认。
- 本次还对两个 `.ter` 以 1 MiB 分块完整流式校验，未提取、未解码。其余贴图、网格等的目录 CRC 不是完整性验证。不是整个 ZIP 的 SHA-256 鉴定，也不是恶意内容全面审计。
- 两个来源在检查前后的文件大小、修改时间相同，`source_stat_unchanged:true`。这不是排除所有并发改写的密码学证明，但脚本始终只读打开来源。
- 扫描 `C:\Users\Yu\Desktop` 未找到 `BeamNG*.exe`。这不表示电脑其他位置没有安装；本次未发现可据以核对的原生安装编辑器。因此下面操作依据官方文档，不冒充已在本机 0.39.4 验证的精确菜单/API。

## 实际内容

| 项目 | Shomaru | American Road |
|---|---:|---:|
| ZIP 字节数 | 108,991,933 | 519,328,433 |
| 目录项数，含目录 | 545 | 1,684 |
| level ID | `2k_shomaru` | `american_road` |
| TerrainBlock / `.ter` | 1 / 1 | 1 / 1 |
| `.ter` 字节数 | 50,331,687 | 469,762,238 |
| DecalRoad | 140 | 2,390 |
| DecalRoad 原始节点 | 987 | 24,993 |
| MeshRoad | 53 | 748 |
| MeshRoad 原始节点 | 855 | 8,303 |
| TSStatic | 5 | 1,738 |
| TerrainMaterial | 2 | 17 |
| `.forest4.json` 实例文件 | 32 | 102 |
| 可解析的带 `type` 森林实例 | 16,828 | 317,913 |

这里的道路对象数不是独立可驾驶道路数。Shomaru 的 DecalRoad 有 64 个裂纹材质对象、46 个标线对象；American Road 有大量标线、轮胎痕迹、路缘等。MeshRoad 同样可用于墙面、边缘、桥梁，不能一概算成道路。

所有 DecalRoad 的 XY 节点折线总长分别约 7.523 km、587.030 km；MeshRoad 分别约 5.111 km、137.673 km。**这些总长包含重叠和装饰，不是有效路线里程，不拿它们推荐路线。** 所有原始有效节点参与逐对象分布统计，最多 16 个展示节点只是预览样本，不是统计输入的截断。

### 地形参数

Shomaru 来源：`levels/2k_shomaru/main/MissionGroup/items.level.json`。

- `terrainFile`: `/levels/2k_shomaru/theTerrain.ter`。
- `position`: `[-1268.97632, -1401.59692, 242.623001]`。
- `maxHeight:850`，`isRenderEnabled:false`，`castShadows:false`。
- 未显式记录 `squareSize` 或地形旋转/scale；不能把缺省值猜成 1 或据文件大小宣称分辨率。

American Road 来源：`levels/american_road/main/MissionGroup/items.level.json`。

- `terrainFile`: `/levels/american_road/terenia24.ter`。
- `position`: `[-4108.0498, -7074.20996, -570.36499]`。
- `maxHeight:1150`，`squareSize:1.29999995`，`baseTexSize:2048`。
- `materialTextureSet:american_roadTerrainMaterialTextureSet`。
- `info.json` 宣称尺寸 `[11000,11000]`、约 120 平方公里；这是作者展示信息，不是解码出的网格范围。`baseTexSize` 也不是高度图分辨率。

两个二进制头首字节分别为十六进制 `09`、`08`，仅作为原始证据保留，不据此推断版本或布局。未确认地形网格维度、实测海拔范围、孔洞和材质索引编码。`maxHeight` 是配置值，不是测得的最高点。两个位置也不是两条道路接缝的对齐点。

校验结果：

| 地形 | CRC32 验证通过 | SHA-256 |
|---|---|---|
| `theTerrain.ter` | `56c784c0` | `55639555e7b060d41b7783ff1684e0a20d8c7274818de47b166074644d53dc92` |
| `terenia24.ter` | `483e7da1` | `79540b01bf6f603b85859fbb09bad9db5cb5bde65697e459a7da72d1bcf4ad81` |

### Shomaru 实际碰撞结构

`main/MissionGroup/Mesh/items.level.json` 中 `base/base.dae`、`road/road.dae`、`ground/ground.dae` 的 TSStatic 均声明 `collisionType:"Visible Mesh Final"`、`decalType:"Visible Mesh Final"`；`Wall Mesh` 下的 `wallmesh/wallmesh.dae` 也是如此。它们的对象 scale 均约 `[0.8,0.8,0.8]`。`sign/sign.dae` 则声明 `collisionType:"None"`。

这直接支持“道路/地表依赖独立可碰撞场景网格”，而不是只有一张高度图。它不能证明碰撞导入无误或实际轮胎没有穿透。相应 DAE 的单位、轴向、几何节点、材质绑定记录在 `dae_metadata`；变换仍须连同 DAE 内部节点矩阵在编辑器核对。

因此复制 AI Roads 的 DecalRoad 只会复制路径/投影，不会复制承载车轮的山路。对整套网格简单平移，也不等于已完成坡脚连接、地形避让和边界封口。

### 材质与真实物理

- Shomaru 的两个 TerrainMaterial 是 `Grass`、`Grass2`，都指定 `groundmodelName:GRASS`，不是水泥/石铺地形层。
- American Road 的地形层涵盖 `GRASS`、`ROCK`、`ASPHALT`、`DIRT_DUSTY`、`MUD`、`GRAVEL`、`DIRT`、`SAND`、`ASPHALT_OLD`、`DIRT_DUSTY_LOOSE`。
- 特别注意：地形层 `Concrete` 实际指定 `groundmodelName:ASPHALT_OLD`。不能因为名字叫 Concrete 就说轮胎接触模型是独立混凝土模型。
- `mymap_concrete1` 是现成 DecalRoad 外观材质，贴图在 `art/road/`，定义未显式指定 `groundType`。
- `art/supernew/MyRoads/main.materials.json` 的 `bruk2` 指向 `cobbleroad_COL_A.dds`、`cobbleroad_NRM.dds`、roughness、opacity 等文件，目录核查均在包内，且明确 `groundType:COBBLESTONE`。这确认材质存在，不证明已经有一段可驾驶石铺路，也未确认本机物理模型解析结果。
- 普通贴图、normalMap 和渲染 roughness 不会新增轮胎碰撞几何。DecalRoad 本身也不提供碰撞网格。若需真实适度颠簸，应验证接触 ground model，再用小范围可控路面几何/地形起伏试验，不能声称换材质就完成物理粗糙度。
- American Road 约 1.3 m 的地形格距配置不适合逐块表达厘米级鹅卵石。需要可控碰撞路面网格或经验证的物理模型支持；不建议海量单石刚体或全图随机噪声。

## 候选路线

下列数值是原始 DecalRoad 节点 XY 弦长及 `100*abs(dz)/hypot(dx,dy)`。宽度是节点声明的 decal 宽度，不一定是道路净宽；坡度不是轮胎接触面采样，也未求值引擎样条、首末切线或 loop 闭合边。异常大坡度可能来自旧节点、高架/地面投影差异、稀疏采样或真实陡坡，需现场复核。

| 优先级 | 地图 / persistentId 前缀 | 材质 | 长度 m | 节点数 | 宽度中位数 m | 绝对节点坡度 P50 / P90 / 最大 |
|---|---|---|---:|---:|---:|---|
| 1 | American / `30741dc1` | `road_light_nolines` | 1481.89 | 25 | 4 | 2.35% / 5.66% / 7.63% |
| 2 | American / `5024fc52` | `mymap_concrete1` | 586.17 | 17 | 4 | 1.17% / 1.84% / 2.49% |
| 3 | American / `a7bcf844` | `mymap_concrete1` | 699.97 | 14 | 4 | 2.09% / 3.16% / 3.41% |
| 4 | American / `a46d34d3` | `BrightG3` | 802.34 | 68 | 5.5 | 4.90% / 13.02% / 20.24% |
| 参考 | Shomaru / `ab33f7b3` | `DefaultDecalRoadMaterial` | 828.33 | 64 | 3.5 | 4.19% / 8.08% / 9.52% |

**首选优先级 1 作为保坡窄路试作走廊，优先级 2 作为原有水泥路基线。** 优先级 4 是更明显的山坡备选，有陡坡尾部，不能把整条路称作温和坡道。Shomaru 的参考段宽度范围 3 至 4.5 m，但这是 AI 路径声明，实际净宽要看网格。

优先级 1 原始 position 为 `[3371.78101,-5896.49023,-554.798096]`；优先级 2 为 `[5339.2373,-5731.07275,-548.886719]`。完整 UUID、来源路径、位置和其余候选在 `proposed_merge_plan.json`。负 Z 是地图坐标，不表示错误，切勿统一抬到零高度。

没有假定这些段已经互相接通，也没有把全图平均宽度作为乡村路线。候选来自具体材质、长度与宽度筛选，再看每条路的完整原始分布。更长的 `BrightG3` 段 `23ca1b09` 虽有 1615.06 m，P50 坡度约 12.93%、最大 25.31%，不作为第一段温和测试路；`road_light_nolines` 段 `3a4d8ed0` 最大节点坡度约 44.25%，也不因其平均值较小就选入主路线。

## 依赖与授权

路径检查只是文本/DAE 引用审计，不是引擎 VFS 解析器。游戏公共资产、自动派生纹理、导出高度图、未使用材质和历史 DAE 图片 URI 都可能不在当前包内；未解析相对路径、所有脚本语义及所有 American Road DAE 内部依赖，不能称为完整依赖闭包。

American Road 文本引用有 1,197 个唯一引用在包内，4 个不在包内：

- `/levels/american_road/art/supernew/MyRoads/DirtRoads_and_other/mymap_line6_d.png`。
- `/levels/american_road/terenia24.terrainheightmap.png`，可能是编辑器导出记录，不等于运行必需。
- `/levels/utah/scenarios/downhill_rally.jpg`，其他关卡的图片引用。
- `vehicles/common/null2.dds`，可能由游戏公共资源提供。

Shomaru 在仅 JSON/脚本阶段就出现 279 个包外引用线索，其中包括 8 个 `/levels/2k_yabitsu/...` 引用，以及 jungle_rock_island、west_coast_usa 和 `/art/` 公共资源路径。加入 DAE 扫描后，完整结果以报告为准，不把不同扫描范围的数混用。DAE 还保留作者电脑 `C:/Users/Owen/...`、`C:/dev%20build/...`、`/F:/0BLENDER%20MAPS/...` 等绝对图片 URI；它们是字符串证据，脚本没有访问这些路径。材质绑定可能已替代这些历史 URI，需 Resource Checker 判断实际使用与缺失，不能盲目全路径替换。

Shomaru 的 `info.json` 原文声明 `authors:"Techeyy"`、`IDAS3 convert with revamped vegetation and AI traffic`。但其内容缺逗号、包含未引号字段及重复 country，严格 JSON 解析失败。不能据此断言 BeamNG 一定加载失败，因为原生宽松解析器本次未验证；也不能当作已确认兼容 0.39。

American Road 的 `info.json` 声明作者 Primo，并明确称此包是非官方 0.39 兼容更新，关卡、道路、地形和美术仍为原作者作品。这是包内声明，不是经过运行验收的兼容性结论。

两包均未发现文件名包含 license/licence/readme/copyright/credits 的说明文件。元数据没有提供足够的改编、重打包或再分发授权。**免费可下载不等于开源；能本地加载也不等于有权导入、公开发布或商业使用。** Shomaru 自述转换来源，还涉及上游作品权利，不能仅向转换者征得许可就假定权利链完整。公开分享前应核实所有涉及作者、素材与上游权利；本次不生成可分发的派生 mod。

## 为什么不直接拼地形

1. `.ter` 不是可串接图片。本次没有可靠的双版本解码/编码依据；头部与文件大小不能代替解析。若坚持地形迁移，应先用原生 Terrain Editor 导出经过验证的高度图，或实现有已知样本往返测试的解码器。
2. 要处理高度编码、网格维度、squareSize、maxHeight、平移、材质层索引、孔洞、纹理集和高度图方向。不同格距的重采样可能抹平原坡或产生台阶；需要高程控制点和接缝剖面测试。
3. Shomaru 的可见/碰撞主路来自网格，仅导出地形高度图不能保住其山路。导出 TerrainBlock 与提取道路网格是两项任务。
4. DecalRoad、MeshRoad、TSStatic、出生点、AI 路径、森林实例、桥梁/水面/音源须按各自坐标语义统一处理。保坡局部迁移宜首先只用平移和竖轴旋转；不可分别任意拉伸 XY、Z。尤其不能给已经是地图坐标的道路节点再重复叠加 position。
5. Shomaru TSStatic 的 0.8 scale 与 DAE 内部坐标还要一起计算。局部截取大网格可能需要 DCC 工具裁切、补边及碰撞重建；不能声称编辑器复制对象就自动截取一段公路。
6. 两图有 16,828 与 317,913 个可识别森林实例。它们按 forest 类型分文件，包含位置和旋转数据，不能只平移 `Forest` 容器。必须连同实例、类型定义、shapeFile、材质和碰撞设置迁移；按路廊筛选后再处理贴地、道路净空及桥面误吸附。
7. 改 level ID 要审计 VFS 路径、对象名、`persistentId`、材质 `name/mapTo`、TerrainMaterial、ForestItemData 类型与纹理集。新文件夹不能自动解决全局材质名字冲突；不能改写其他已安装地图来凑路径。
8. 若新图仍引用原地图 namespace，它就是有显式依赖的派生地图，不是独立成品。若复制素材到新 namespace，必须有权复制并保留署名。两种方式都不能跳过依赖闭包和授权。

## 可执行的编辑器流程

以下为**下一阶段手动操作方案，未执行**。官方工具文档能确认名称及基本操作；没有本机原生安装验证，不提供臆造的一键“合并地图”、未知菜单路径或未测 Lua 调用。

1. 先确认实际 BeamNG.tech 版本、Launcher 指向的 user folder 和独立测试配置。获得明确安装/启动许可后，分别测试两个原图，不同时启用来掩盖依赖。检查关卡加载、材质、碰撞和出生点日志。不要向本任务目录猜测安装路径。
2. 明确允许创建本地派生工作副本后，为 American Road 建立独立 level ID，如 `yu_rural_ar_local`，审计路径和命名。保留原 ZIP 不变；不要在原图上 Ctrl+S 后误以为没有产生 user-folder override。没有确认独立工作副本前，仅观察，不保存。
3. 在 World Editor 的 Scene Tree / Inspector 中识别 `terenia2`，记录 position、maxHeight、squareSize。使用 Terrain Editor 工具栏的 Heightmap Import / Export 保存基线高度图及所需层数据，并记录导出尺寸、位深和轴向。不要先做全图 Flatten、Smooth 或 Noise。
4. 用计划内 position 将自由相机导航至候选区域，再用 Decal Road Editor 点选现有道路；检查 Inspector 的材质、节点数及位置，与报告 UUID 对照。这里许多道路无 name，不假设 Scene Tree 能按 UUID 搜索，也不要靠名字猜路。相机高度需高于路面，不能把车辆直接放在未验证的元数据 Z 上。
5. 对首选 `30741dc1` 保留原路线和纵向坡形，先在独立副本上截取约 100 至 200 m 试验段。Decal Road Editor 选中节点后，Inspector 有 `Split Road`；需要加节点时按住 ALT 并左键，放开 ALT 结束，节点 Width 在 Inspector 修改。目标净宽建议先 3.5 至 4.5 m，少量会车点再加宽；这是设计目标而非现有路宽保证。
6. 对水泥段先使用原有 `mymap_concrete1`；对石铺外观试用 `bruk2` 并检查 opacity、投影、纹理长度和接缝。改变 decal 宽度只改覆盖带，必要时连同路肩/碰撞路床局部重建，否则宽公路底面仍在。检查 Inspector 的 `overObject`，决定是否投影到网格上；不能把它当成自动生成碰撞。
7. 用 Terrain Painter 核对底层接触材质；双击材质进入 Terrain Material Editor。该包显式使用 `groundmodelName`，应核对当前版本编辑器字段和运行接触结果，不按旧文档的“只改名字”规则盲目操作。不要修改游戏全局 `groundmodels.json`。
8. 真实轻度颠簸先做小样，保留纵坡控制点，仅添加低幅长波路面起伏和少量接缝。Meshroad Editor 可用 ALT+左键建节点，并在 Inspector 调整 Width、Depth、topMaterial、bottomMaterial、sideMaterial。MeshRoad 不是逐石铺路自动器，需更精细石缝碰撞时另做经验证的小型路面网格。所有数值以同车同速测试为准，不未经试车全图套用厘米级噪声。
9. 地形塑形只作用于路廊及接缝。Terrain Editor 有 Smooth Slope、Noise、Align with Mesh (Up/Down)；后者依赖正确碰撞模式。不要用 Flatten 抹掉爬坡，也不要把路面网格下的另一个碰撞面留在几乎同一高度。桥梁、山体穿插和车轮接触需逐段检查。
10. Forest Editor 的 Select Item 可单独操作实例；局部 Erase 清除路面树木，Snap to Terrain 只用于已确认该贴地的植被。不要全森林吸附，否则桥面、网格地面、特殊高度摆放都可能错位。
11. 仅在确实需要且权利允许时导入 Shomaru 的局部网格及完整依赖，保留 0.8 缩放语义，核对 `Visible Mesh Final` 碰撞，再配套迁移裁切范围内的 AI 路径和森林。先解决材质绑定与接缝，不优先搬整座山和整片森林。
12. 调整 DecalRoad 的 drivability、lanesLeft/lanesRight、oneWay 等为实际道路服务，不能给路边裂纹或石墙附属条带添加可行驶性。用官方 Resources Checker 检查实际缺资源，再试车核验材质接触、连续碰撞、净空和 AI 连接。
13. 使用相同车辆、轮胎、载荷和固定低速，例如先 20 km/h，再 40 km/h，对比原路与试验段。至少记录车身垂向加速度、悬架行程、轮胎接触丢失、底盘触地、可用净宽和上下坡两方向行为。目标是可感知但不中断正常行驶的粗糙度；无运行测量前不能标记“适度颠簸已达成”。

“安装两张原图并分别打开预览”只能验证两张独立地图，并不会把它们连成一张。本次未做预览安装器，因为安装目标与权限仍未确认，增加自动复制步骤反而容易越过分析阶段。

## 官方依据

检查时可访问的官方页面：

- Terrain Editor：<https://documentation.beamng.com/world_editor/tools/terrain_editor/>，确认高度图导入/导出、塑形、Terrain Painter 与 ground model 注意事项。文档标注 2025-01-24 更新。
- Decal Road Editor：<https://documentation.beamng.com/world_editor/tools/decalroad_editor/>，确认无碰撞几何、ALT 建节点、Width、Split Road、overObject 与 AI 字段。文档标注 2025-01-24 更新。
- Meshroad Editor：<https://documentation.beamng.com/world_editor/tools/meshroad_editor/>，确认 3D spline 网格道路、Depth、表面材质与节点编辑。文档标注 2021-12-07 更新。
- Forest Editor：<https://documentation.beamng.com/world_editor/tools/forest_editor/>，确认实例选择、贴地、shapeFile、collidable 与关卡绑定。文档标注 2021-12-07 更新，提示工具仍可能变化。

官方文档首页导航也提供 Resources Checker。旧版文档与本机 0.39.4 可能存在界面差异，应以安装后原生编辑器为准。

## 产物与复查

- `inspect_maps.py`：仅 Python 标准库，无需安装依赖。输入固定为两个指定地图包；无启动、安装或来源写入代码。
- `archive_report.json`：目录清单、大小/CRC、元数据 SHA-256、原始关键对象、每路完整分布和有限显示采样、森林计数、材质、DAE 标识/绑定及引用审计。
- `proposed_merge_plan.json`：具体底图、候选 UUID、坐标、指标和开工门槛，明确 `ready:false`。
- `assessment_zh.md`：本报告。

在工作区运行：

```powershell
python "map_merge\inspect_maps.py" --test
python "map_merge\inspect_maps.py" --verify-terrain
python "map_merge\inspect_maps.py" --focus
```

`--test` 覆盖路径、JSON/逐行 JSON、原始节点坡度统计、无效节点不跨接，并逐一验证计划中候选的位置、材质、节点数、长度和坡度与生成报告一致，共 5 项。`--verify-terrain` 重生成本工具自己的报告并流式校验地形；默认不带该选项时仅读地形头，报告会相应标为未校验地形，不沿用旧结论。`--focus` / `--details` 只读取生成报告，不再打开来源。尚未进行引擎加载、截图、驾驶、实际高度图解码、整包 CRC 或重打包验收。
