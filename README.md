# msh_tool

Fluent MSH 网格文件诊断工具。解析、校验、查询、可视化导出，一站式搞定。

## 快速开始

```bash
msh_tool mesh.msh                 # 校验 + 摘要
msh_tool mesh.msh -o              # 导出 VTK（ParaView 可视化）
msh_tool mesh.msh -e              # 列出所有错误面及其原因
msh_tool mesh.msh -c 42           # 查看 cell 42 的几何
msh_tool mesh.msh -f 57434        # 查看面 57434 的连通性 + 方向
```

## 功能

### 解析
- 格式化（文本）.msh 文件的全解析
- 支持所有 11 个网格段：Nodes、Faces、Cells、Periodic Shadow Faces、Face Tree、Cell Tree、Interface Face Parents
- 支持混合类型（element-type=0, face-type=0/5）
- 支持多边形面（任意节点数）
- 自动十六进制→十进制转换
- 自动 utf-8 中文字符支持

### 校验（四层递进）
1. **语法层** — S-expression 括号匹配、非法 token
2. **结构层** — 计数一致性、索引范围重叠、zone 引用完整性
3. **拓扑层** — 节点/单元索引越界、interior 面 c0/c1 非零、边界面 c1=0、树下系有效性
4. **几何层** — 右手定则（法向量定向），默认对所有面执行，不依赖外部库

### 查询
| 命令 | 说明 |
|------|------|
| `-s, --stats` | 统计摘要 |
| `-z, --zone <id>` | zone 面列表详情 |
| `-f, --face <id>` | 面连通性 + 定向 + 坐标 |
| `-c, --cell <id>` | 单元重构（反查所有面） |
| `-n, --nodes` | 节点坐标转储 |
| `-v, --validate` | 完整校验报告 |
| `-e, --error-faces` | 错误面 ID + 原因列表 |

### 可视化导出
```bash
# 同时导出两个文件：
#   mesh.vtk  — 所有面（PolyData，挂 face_id/c0/c1/bc_type）
#   mesh.vtu  — 完整体网格（UnstructuredGrid，挂 cell_id/zone_id）
msh_tool mesh.msh -o [前缀]
```

ParaView 里 `.vtk` 查面、`.vtu` 查体，`Find Data` 支持按 face_id / cell_id / c0 直接定位。

## 用法

```bash
USAGE: msh_tool <file.msh> [options]

Query modes (mutually exclusive):
  -s, --stats           Statistics summary only
  -z, --zone <id>       Face details for a specific zone
  -f, --face <id>       Connectivity & orientation for a specific face
  -c, --cell <id>       Reconstruct cell geometry from faces
  -n, --nodes           Dump all node coordinates
  -v, --validate        Validation report only (no summary)
  -e, --error-faces     List error face IDs with reasons

Validation control:
  -V, --no-validate     Skip all validation checks
  -G, --no-geometry     Skip right-hand-rule geometry check

General:
  -o, --export-vtk [prefix]  Export .vtk + .vtu (default: derived from .msh)
  -S, --scale <factor>       Multiply all coordinates (default: 1000, m→mm)
  -C, --no-color             Disable ANSI color output
  -h, --help                 Print this help text

EXIT CODES:
  0  Validation passed (or --no-validate used)
  1  Validation errors found
  2  File not found / cannot open
  3  Parse error (malformed MSH)
```

## 典型工作流

```bash
# 1. 快速检查一个新 MSH 文件
msh_tool mesh.msh

# 2. 查看错误详情
msh_tool mesh.msh -e
msh_tool mesh.msh -f 57434

# 3. 导出到 ParaView 调试
msh_tool mesh.msh -o
# 打开 mesh.vtu → Find Data → cell_id 输入 ID → Extract Selection

# 4. 只看统计
msh_tool mesh.msh -s
```

## 构建

### 依赖
- C++17 编译器
- CMake ≥ 3.16

### 从源码构建

```bash
git clone https://github.com/yourname/msh_tool.git
cd msh_tool
mkdir build && cd build

# Windows (VS 2022)
cmake .. -G "Visual Studio 17 2022"
cmake --build .

# Windows (MinGW)
cmake .. -G "MinGW Makefiles"
cmake --build .

# Linux / macOS
cmake ..
make -j$(nproc)
```

`nlohmann/json.hpp` 在 `include/nlohmann/` 下，header-only，无需额外安装。

## 格式参考

MSH 格式规范基于 *Ansys Fluent User's Guide Release 2026 R1* 附录 B：Mesh File Format。详见 [Fluent_MSH_Format.md](./Fluent_MSH_Format.md)。

## License

MIT
