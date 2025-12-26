using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// 主程序 - 运行并对比所有RRT算法
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            Console.OutputEncoding = System.Text.Encoding.UTF8;

            // 定义起点、终点和边界（大规模地图）
            var start = (5.0, 5.0);
            var goal = (95.0, 95.0);
            var bounds = (0.0, 100.0, 0.0, 100.0);

            // 定义障碍物（增加障碍物数量和复杂度，构建更复杂的环境）
            var obstacles = new List<Obstacle>
            {
                // 大型障碍物
                new Obstacle(15, 10, 6, 15),
                new Obstacle(40, 40, 12, 12),
                new Obstacle(20, 60, 10, 6),
                new Obstacle(70, 25, 6, 25),
                new Obstacle(50, 15, 8, 10),
                new Obstacle(80, 50, 6, 15),
                new Obstacle(30, 80, 12, 8),
                new Obstacle(65, 70, 10, 10),
                
                // 中型障碍物
                new Obstacle(10, 30, 6, 12),
                new Obstacle(55, 85, 8, 6),
                new Obstacle(90, 10, 4, 20),
                new Obstacle(25, 45, 6, 8),
                new Obstacle(35, 20, 4, 5),
                new Obstacle(60, 35, 5, 6),
                new Obstacle(45, 65, 4, 4),
                new Obstacle(75, 55, 5, 5),
                new Obstacle(20, 25, 3, 4),
                new Obstacle(85, 75, 4, 6),
                new Obstacle(12, 75, 5, 5),
                new Obstacle(55, 58, 4, 4),
                
                // 新增小型障碍物，形成更密集的环境
                new Obstacle(28, 12, 3, 4),
                new Obstacle(48, 28, 3, 3),
                new Obstacle(38, 55, 3, 4),
                new Obstacle(68, 45, 4, 3),
                new Obstacle(15, 48, 3, 3),
                new Obstacle(82, 35, 3, 4),
                new Obstacle(42, 78, 4, 3),
                new Obstacle(72, 88, 3, 3),
                new Obstacle(8, 55, 3, 4),
                new Obstacle(92, 62, 3, 4),
                new Obstacle(33, 68, 3, 3),
                new Obstacle(58, 22, 4, 3),
                new Obstacle(18, 38, 3, 3),
                new Obstacle(78, 15, 3, 4),
                new Obstacle(52, 48, 3, 3),
                new Obstacle(88, 88, 3, 4),
                new Obstacle(5, 85, 3, 3),
                new Obstacle(95, 28, 3, 3),
                new Obstacle(62, 68, 3, 4),
                new Obstacle(25, 88, 4, 3),
                
                // 额外的小障碍物，进一步增加难度
                new Obstacle(44, 18, 2, 3),
                new Obstacle(66, 58, 3, 2),
                new Obstacle(22, 52, 2, 2),
                new Obstacle(74, 38, 2, 3),
                new Obstacle(36, 82, 3, 2),
                new Obstacle(84, 22, 2, 2),
                new Obstacle(14, 62, 2, 3),
                new Obstacle(58, 75, 3, 2),
                new Obstacle(48, 8, 2, 2),
                new Obstacle(88, 48, 2, 3)
            };

            Console.WriteLine("======================================================================");
            Console.WriteLine("RRT算法全家桶对比演示 (C#版)");
            Console.WriteLine("======================================================================");

            // 运行5种算法
            var results = new Dictionary<string, AlgorithmResult>();

            // 1. RRT
            Console.WriteLine("\n[1/5] 运行RRT算法...");
            var rrt = new RRT(start, goal, obstacles, bounds, stepSize: 2.0, maxIter: 20000);
            results["RRT"] = RunAlgorithm(rrt, () => rrt.Plan());

            // 2. RRT*
            Console.WriteLine("\n[2/5] 运行RRT*算法...");
            var rrtStar = new RRTStar(start, goal, obstacles, bounds, stepSize: 2.0, maxIter: 20000, searchRadius: 5.0);
            results["RRT*"] = RunAlgorithm(rrtStar, () => rrtStar.Plan());

            // 3. RRT-Connect
            Console.WriteLine("\n[3/5] 运行RRT-Connect算法...");
            var rrtConnect = new RRTConnect(start, goal, obstacles, bounds, stepSize: 2.0, maxIter: 20000, randomSeed: 42);
            results["RRT-Connect"] = RunAlgorithm(rrtConnect, () => rrtConnect.Plan());

            // 4. RRT*-Connect
            Console.WriteLine("\n[4/5] 运行RRT*-Connect算法...");
            var rrtStarConnect = new RRTStarConnect(start, goal, obstacles, bounds, stepSize: 2.0, maxIter: 20000, searchRadius: 5.0);
            results["RRT*-Connect"] = RunAlgorithm(rrtStarConnect, () => rrtStarConnect.Plan());

            // 5. RRT-Connect + Shortcut
            Console.WriteLine("\n[5/5] 运行RRT-Connect + Path Shortcutting算法...");
            var rrtConnectShortcut = new RRTConnectShortcut(start, goal, obstacles, bounds, stepSize: 2.0, maxIter: 20000, randomSeed: 42);
            results["RRT-Connect+Shortcut"] = RunAlgorithm(rrtConnectShortcut, () => rrtConnectShortcut.Plan());

            // 打印对比结果
            PrintComparison(results);

            Console.WriteLine("\n算法特点:");
            Console.WriteLine("- RRT:                   快速探索，找到第一条可行路径");
            Console.WriteLine("- RRT*:                  路径优化，生成更短更平滑的路径");
            Console.WriteLine("- RRT-Connect:           双向搜索，通常最快找到路径");
            Console.WriteLine("- RRT*-Connect:          结合双向搜索和路径优化");
            Console.WriteLine("- RRT-Connect+Shortcut:  快速找路径后进行路径快捷优化");

            Console.WriteLine("\n======================================================================");
            Console.WriteLine("演示完成！");
            Console.WriteLine("======================================================================");

            Console.WriteLine("\n按任意键退出...");
            Console.ReadKey();
        }

        /// <summary>
        /// 运行算法并记录时间和结果
        /// </summary>
        static AlgorithmResult RunAlgorithm(RRT algorithm, Func<List<(double, double)>> algorithmFunc)
        {
            var stopwatch = Stopwatch.StartNew();
            var path = algorithmFunc();
            stopwatch.Stop();

            double? length = null;
            if (path != null)
            {
                length = CalculatePathLength(path);
            }

            return new AlgorithmResult
            {
                Path = path,
                Time = stopwatch.Elapsed.TotalSeconds,
                Length = length,
                TotalNodes = algorithm.Nodes.Count
            };
        }

        /// <summary>
        /// 计算路径长度
        /// </summary>
        static double CalculatePathLength(List<(double, double)> path)
        {
            if (path == null || path.Count < 2)
                return 0.0;

            double length = 0.0;
            for (int i = 1; i < path.Count; i++)
            {
                double dx = path[i].Item1 - path[i - 1].Item1;
                double dy = path[i].Item2 - path[i - 1].Item2;
                length += Math.Sqrt(dx * dx + dy * dy);
            }
            return length;
        }

        /// <summary>
        /// 打印算法对比结果
        /// </summary>
        static void PrintComparison(Dictionary<string, AlgorithmResult> results)
        {
            Console.WriteLine("\n======================================================================");
            Console.WriteLine("算法对比:");
            Console.WriteLine("======================================================================");

            foreach (var kvp in results)
            {
                string name = kvp.Key;
                var result = kvp.Value;

                string paddedName = name.PadRight(24);
                if (result.Length.HasValue)
                {
                    Console.WriteLine($"{paddedName} 总节点数={result.TotalNodes}, 路径点数={result.Path.Count}, 路径长度={result.Length.Value:F2}, 时间={result.Time:F3}秒");
                }
                else
                {
                    Console.WriteLine($"{paddedName} 总节点数={result.TotalNodes}, 未找到路径, 时间={result.Time:F3}秒");
                }
            }
        }

        /// <summary>
        /// 算法结果类
        /// </summary>
        class AlgorithmResult
        {
            public List<(double, double)> Path { get; set; }
            public double Time { get; set; }
            public double? Length { get; set; }
            public int TotalNodes { get; set; }
        }
    }
}
