# RRT算法演示 (C#版)

一个用于学习和对比多种RRT（Rapidly-exploring Random Tree）路径规划算法的C#实现。

## 项目简介

本项目是Python版本 [RRTDemo](https://github.com/andrealu2012/RRTDemo) 的C#重写版本，实现了5种经典的RRT算法变体，方便用户直观地理解不同算法的特点和性能差异。

## 功能特点

- 🎯 **5种算法实现**：RRT、RRT*、RRT-Connect、RRT*-Connect、RRT-Connect+Path Shortcutting
- 📊 **性能统计**：显示路径节点数、路径长度、计算时间
- 🎨 **清晰架构**：面向对象设计，算法继承结构清晰
- 📝 **中文支持**：完整的中文注释和输出

## 算法说明

### 1. RRT（基础版）
- **特点**：快速探索，找到第一条可行路径即停止
- **优点**：速度快，适合简单场景
- **缺点**：路径通常不是最优的

### 2. RRT*（优化版）
- **特点**：在RRT基础上增加了路径优化机制
- **优点**：生成更短、更平滑的路径
- **缺点**：计算时间相对较长

### 3. RRT-Connect（双向版）
- **特点**：从起点和终点同时生长两棵树，直到连接
- **优点**：通常最快找到路径
- **缺点**：路径质量一般

### 4. RRT*-Connect（混合版）
- **特点**：结合双向搜索和路径优化
- **优点**：平衡速度和路径质量
- **缺点**：实现复杂度较高

### 5. RRT-Connect + Path Shortcutting（后处理优化版）
- **特点**：先用RRT-Connect快速找路径，再进行路径快捷化
- **优点**：速度快且路径质量好，实现简单
- **缺点**：极端情况下可能优化有限

## 环境要求

- .NET 6.0 或更高版本
- Windows / Linux / macOS

## 安装与运行

### 克隆项目

```bash
git clone <your-repo-url>
cd RRTDemoCSharp
```

### 构建项目

```bash
dotnet build
```

### 运行程序

```bash
dotnet run
```

## 项目结构

```
RRTDemoCSharp/
├── RRTAlgorithms/
│   ├── Node.cs                    # 节点类
│   ├── Obstacle.cs                # 障碍物类
│   ├── RRT.cs                     # RRT基础类
│   ├── RRTStar.cs                 # RRT*算法
│   ├── RRTConnect.cs              # RRT-Connect算法
│   ├── RRTStarConnect.cs          # RRT*-Connect算法
│   └── RRTConnectShortcut.cs      # RRT-Connect+路径快捷算法
├── Program.cs                      # 主程序
├── RRTDemoCSharp.csproj           # 项目文件
└── README.md                       # 本文件
```

## 代码架构

- **Node类**：树节点，包含坐标、父节点、代价信息
- **Obstacle类**：矩形障碍物
- **RRT类**：基础RRT算法实现
- **RRTStar类**：继承RRT，增加路径优化
- **RRTConnect类**：继承RRT，实现双向搜索
- **RRTStarConnect类**：继承RRTConnect，结合优化策略
- **RRTConnectShortcut类**：继承RRTConnect，增加路径快捷化

## 自定义参数

编辑 `Program.cs` 的 `Main()` 函数可自定义：

```csharp
var start = (1.0, 1.0);              // 起点坐标
var goal = (9.0, 9.0);               // 终点坐标
var bounds = (0.0, 10.0, 0.0, 10.0); // 空间边界 (x_min, x_max, y_min, y_max)

// 障碍物列表 (x, y, width, height)
var obstacles = new List<Obstacle>
{
    new Obstacle(3, 2, 1, 3),
    new Obstacle(5, 5, 2, 2),
    // ...
};

// 算法参数
stepSize: 0.5        // 扩展步长
maxIter: 2000        // 最大迭代次数
searchRadius: 1.5    // RRT*搜索半径
```

## 示例输出

运行后终端会显示类似以下信息：

```
======================================================================
RRT算法全家桶对比演示 (C#版)
======================================================================

[1/5] 运行RRT算法...
开始RRT路径规划...
找到路径！迭代次数: 234

[2/5] 运行RRT*算法...
开始RRT*路径规划...
找到路径！迭代次数: 456

[3/5] 运行RRT-Connect算法...
开始RRT-Connect路径规划...
找到路径！迭代次数: 123

[4/5] 运行RRT*-Connect算法...
开始RRT*-Connect路径规划...
找到路径！迭代次数: 189

[5/5] 运行RRT-Connect + Path Shortcutting算法...
开始RRT-Connect + Path Shortcutting路径规划...
第1步: 使用RRT-Connect寻找初始路径...
找到路径！迭代次数: 120
第2步: 路径快捷优化...
优化后路径: 15个点, 长度=11.34

======================================================================
算法对比:
======================================================================
RRT                      路径长度=12.45, 路径点数=25, 时间=0.156秒
RRT*                     路径长度=11.23, 路径点数=28, 时间=0.287秒
RRT-Connect              路径长度=12.78, 路径点数=22, 时间=0.098秒
RRT*-Connect             路径长度=11.56, 路径点数=24, 时间=0.198秒
RRT-Connect+Shortcut     路径长度=11.34, 路径点数=15, 时间=0.112秒

算法特点:
- RRT:                   快速探索，找到第一条可行路径
- RRT*:                  路径优化，生成更短更平滑的路径
- RRT-Connect:           双向搜索，通常最快找到路径
- RRT*-Connect:          结合双向搜索和路径优化
- RRT-Connect+Shortcut:  快速找路径后进行路径快捷优化

======================================================================
演示完成！
======================================================================
```

## 技术特点

- **模块化设计**：每个算法独立文件，便于学习和修改
- **继承复用**：通过继承减少代码重复，体现算法演进关系
- **类型安全**：C#强类型系统提供编译时检查
- **中文友好**：完整的中文注释和界面
- **可扩展性**：易于添加新的算法变体

## 学习资源

- **RRT原论文**：S. M. LaValle, "Rapidly-exploring random trees: A new tool for path planning", 1998
- **RRT*论文**：S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning", 2011
- **Python原版**：[andrealu2012/RRTDemo](https://github.com/andrealu2012/RRTDemo)

## 开发说明

本项目专为学习目的设计，代码结构清晰，注释详细，适合：
- 学习路径规划算法
- 理解RRT系列算法原理
- 算法性能对比研究
- 作为课程作业或研究起点
- C#算法实现参考

## 常见问题

**Q: 为什么有时候算法找不到路径？**  
A: 可能是障碍物太密集或maxIter设置太小，可以增大迭代次数或减少障碍物。

**Q: 如何调整算法参数？**  
A: 编辑Program.cs中的Main()函数，修改stepSize、maxIter等参数。

**Q: 可以添加新算法吗？**  
A: 可以！继承RRT基类，实现自己的Plan()方法即可。

**Q: 为什么RRT*比RRT慢？**  
A: RRT*需要在每次添加节点时进行rewire操作，计算量更大，但换来了更优的路径。

**Q: 如何添加可视化？**  
A: 可以使用WPF、WinForms或第三方图形库（如OxyPlot）来可视化结果。

## 许可证

本项目仅供学习交流使用。

## 致谢

感谢原Python版本的作者 [andrealu2012](https://github.com/andrealu2012) 提供的优秀实现。

## 贡献

欢迎提交Issue和Pull Request！
