# 快速开始指南

## 5分钟上手RRTDemo C#版

### 第一步：环境准备

确保已安装 .NET 6.0 或更高版本：
```bash
dotnet --version
```

### 第二步：克隆并运行

```bash
# 进入项目目录
cd RRTDemoCSharp

# 构建项目
dotnet build

# 运行程序
dotnet run
```

### 第三步：查看结果

程序会自动运行5种算法并显示对比结果。

## 修改示例场景

编辑 `Program.cs` 文件：

```csharp
// 修改起点和终点
var start = (2.0, 2.0);  // 改为(2, 2)
var goal = (8.0, 8.0);   // 改为(8, 8)

// 添加更多障碍物
var obstacles = new List<Obstacle>
{
    new Obstacle(3, 2, 1, 3),
    new Obstacle(5, 5, 2, 2),
    new Obstacle(6, 1, 1, 2),  // 新障碍物
};
```

## 只运行单个算法

如果只想测试某个算法：

```csharp
static void Main(string[] args)
{
    var start = (1.0, 1.0);
    var goal = (9.0, 9.0);
    var bounds = (0.0, 10.0, 0.0, 10.0);
    var obstacles = new List<Obstacle> { /* ... */ };

    // 只运行RRT*算法
    var rrtStar = new RRTStar(start, goal, obstacles, bounds, 
        stepSize: 0.5, maxIter: 2000, searchRadius: 1.5);
    
    var path = rrtStar.Plan();
    
    if (path != null)
    {
        Console.WriteLine($"找到路径，长度：{CalculatePathLength(path):F2}");
        Console.WriteLine($"路径点数：{path.Count}");
    }
}
```

## 导出路径数据

将路径保存到文件：

```csharp
if (path != null)
{
    File.WriteAllLines("path.txt", 
        path.Select(p => $"{p.Item1},{p.Item2}"));
    Console.WriteLine("路径已保存到 path.txt");
}
```

## 性能测试

测试不同参数的影响：

```csharp
// 测试不同步长
foreach (var stepSize in new[] { 0.3, 0.5, 0.7, 1.0 })
{
    var rrt = new RRT(start, goal, obstacles, bounds, 
        stepSize: stepSize, maxIter: 2000);
    
    var stopwatch = Stopwatch.StartNew();
    var path = rrt.Plan();
    stopwatch.Stop();
    
    Console.WriteLine($"步长={stepSize:F1}, 时间={stopwatch.Elapsed.TotalSeconds:F3}秒");
}
```

## 下一步

- 阅读 [README.md](README.md) 了解项目详情
- 阅读 [IMPLEMENTATION.md](IMPLEMENTATION.md) 了解实现细节
- 查看各算法源码学习具体实现
- 尝试调整参数观察效果
- 添加自己的测试场景
