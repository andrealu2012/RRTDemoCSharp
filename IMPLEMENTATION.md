# RRT算法C#实现详细说明

## 项目特色

本项目是从Python版本 https://github.com/andrealu2012/RRTDemo 迁移到C#的完整实现，保持了原版的所有功能和算法逻辑。

## 核心类设计

### 1. Node.cs - 节点类
```csharp
public class Node
{
    public double X { get; set; }          // X坐标
    public double Y { get; set; }          // Y坐标
    public Node Parent { get; set; }       // 父节点（用于路径追溯）
    public double Cost { get; set; }       // 累积代价（用于RRT*）
}
```

### 2. Obstacle.cs - 障碍物类
```csharp
public class Obstacle
{
    public double X, Y { get; set; }       // 左下角坐标
    public double Width, Height { get; set; } // 宽度和高度
    public bool ContainsPoint(double px, double py) // 碰撞检测
}
```

### 3. 算法类继承关系

```
RRT (基类)
├── RRTStar (继承RRT)
└── RRTConnect (继承RRT)
    ├── RRTStarConnect (继承RRTConnect)
    └── RRTConnectShortcut (继承RRTConnect)
```

## 算法实现细节

### RRT - 基础快速探索随机树
**核心方法：**
- `RandomSample()`: 随机采样（10%概率采样目标点）
- `NearestNode()`: 查找最近节点
- `Steer()`: 朝目标方向移动步长距离
- `IsCollisionFree()`: 碰撞检测
- `Plan()`: 主规划算法

### RRT* - 优化版RRT
**额外方法：**
- `FindNearNodes()`: 查找半径范围内的邻近节点
- `ChooseParent()`: 选择最优父节点（最小代价）
- `Rewire()`: 重连操作，优化邻近节点的路径
- `UpdateChildrenCost()`: 递归更新子节点代价

**优势：** 生成更短、更优的路径

### RRT-Connect - 双向搜索
**额外方法：**
- `ExtendTree()`: 在指定树中扩展节点
- `ConnectTree()`: 尝试连接到目标点
- `ExtractDualPath()`: 提取双向树的完整路径

**优势：** 快速找到路径（通常最快）

### RRT*-Connect - 结合双向搜索和优化
**额外方法：**
- `ExtendTreeWithOptimization()`: 带优化的树扩展
- `ConnectTreeWithOptimization()`: 带优化的树连接
- `RewireTree()`: 在指定树中进行rewire操作

**优势：** 平衡速度和路径质量

### RRT-Connect + Shortcut - 后处理优化
**额外方法：**
- `IsDirectConnectionFree()`: 检查两点直连是否可行
- `PathShortcutting()`: 路径快捷化算法
- `CalculatePathLength()`: 计算路径长度

**优势：** 简单有效的路径优化

## 性能对比（典型场景）

| 算法 | 速度 | 路径质量 | 节点数 | 适用场景 |
|------|------|---------|--------|----------|
| RRT | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 中 | 简单场景，快速探索 |
| RRT* | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 高 | 需要高质量路径 |
| RRT-Connect | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 低 | 快速找到可行路径 |
| RRT*-Connect | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 中 | 平衡速度和质量 |
| RRT-C+Shortcut | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 低 | 实际应用推荐 |

## 使用示例

### 基本使用
```csharp
// 定义环境
var start = (1.0, 1.0);
var goal = (9.0, 9.0);
var bounds = (0.0, 10.0, 0.0, 10.0);
var obstacles = new List<Obstacle> { /* ... */ };

// 运行RRT*算法
var rrtStar = new RRTStar(start, goal, obstacles, bounds, 
    stepSize: 0.5, maxIter: 2000, searchRadius: 1.5);
var path = rrtStar.Plan();

// 路径结果
if (path != null)
{
    Console.WriteLine($"找到路径，共{path.Count}个点");
    foreach (var point in path)
    {
        Console.WriteLine($"({point.Item1:F2}, {point.Item2:F2})");
    }
}
```

### 自定义障碍物
```csharp
var obstacles = new List<Obstacle>
{
    new Obstacle(3, 2, 1, 3),  // x, y, width, height
    new Obstacle(5, 5, 2, 2),
    new Obstacle(2, 6, 2, 1),
    new Obstacle(7, 3, 1, 4)
};
```

## 参数调优指南

### stepSize（步长）
- **小值(0.1-0.3)**：精细探索，适合复杂环境
- **中值(0.4-0.7)**：平衡性能
- **大值(0.8-1.5)**：快速探索，适合开阔环境

### maxIter（最大迭代）
- **简单场景**：500-1000
- **中等复杂度**：1000-2000
- **复杂场景**：2000-5000

### searchRadius（RRT*搜索半径）
- **小值(0.5-1.0)**：局部优化
- **中值(1.0-2.0)**：平衡
- **大值(2.0-3.0)**：全局优化（但较慢）

## 扩展开发

### 添加新算法
```csharp
public class MyCustomRRT : RRT
{
    public MyCustomRRT(/* parameters */) : base(/* base parameters */)
    {
        // 自定义初始化
    }

    public override List<(double, double)> Plan()
    {
        // 自定义规划逻辑
        return myCustomPath;
    }
}
```

### 添加新障碍物类型
```csharp
public class CircleObstacle : Obstacle
{
    public double Radius { get; set; }
    
    public override bool ContainsPoint(double px, double py)
    {
        // 圆形碰撞检测
        double dist = Math.Sqrt(Math.Pow(px - CenterX, 2) + Math.Pow(py - CenterY, 2));
        return dist <= Radius;
    }
}
```

## 与Python版本的差异

1. **类型系统**：C#是强类型语言，提供更好的编译时检查
2. **性能**：C#版本通常运行速度更快
3. **元组**：使用C#的ValueTuple代替Python的tuple
4. **LINQ**：使用LINQ进行集合操作（OrderBy, Where等）
5. **命名空间**：使用namespace组织代码结构

## 常见问题排查

### 问题1：算法总是找不到路径
**解决方案：**
- 增大maxIter
- 检查起点/终点是否在障碍物内
- 减小stepSize以更精细探索

### 问题2：RRT*运行很慢
**解决方案：**
- 减小searchRadius
- 减小maxIter
- 考虑使用RRT-Connect+Shortcut替代

### 问题3：路径质量不理想
**解决方案：**
- 使用RRT*或RRT*-Connect
- 使用RRT-Connect+Shortcut
- 增大searchRadius

## 进一步学习

1. **路径平滑**：添加样条插值
2. **动态环境**：添加移动障碍物支持
3. **3D扩展**：扩展到三维空间
4. **可视化**：集成WPF或其他图形库
5. **并行优化**：使用多线程加速

## 相关资源

- [Python原版](https://github.com/andrealu2012/RRTDemo)
- [RRT论文](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)
- [RRT*论文](https://arxiv.org/abs/1105.1186)

## 贡献指南

欢迎提交Issue和PR！请确保：
- 代码风格一致
- 添加适当的注释
- 更新相关文档
- 测试算法正确性
