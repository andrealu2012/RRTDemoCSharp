using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT-Connect + Path Shortcutting算法实现
    /// 先用RRT-Connect快速找路径，再进行路径快捷化优化
    /// </summary>
    public class RRTConnectShortcut : RRTConnect
    {
        /// <summary>
        /// 构造函数
        /// </summary>
        public RRTConnectShortcut((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                                  (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                                  int? randomSeed = null)
            : base(start, goal, obstacles, bounds, stepSize, maxIter, randomSeed)
        {
        }

        /// <summary>
        /// 检查两点之间的直接连接是否无碰撞
        /// </summary>
        protected bool IsDirectConnectionFree((double, double) point1, (double, double) point2)
        {
            double x1 = point1.Item1;
            double y1 = point1.Item2;
            double x2 = point2.Item1;
            double y2 = point2.Item2;

            double distance = Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2));
            int steps = Math.Max((int)(distance / 0.1), 1);

            for (int i = 0; i <= steps; i++)
            {
                double t = (double)i / steps;
                double x = x1 + t * (x2 - x1);
                double y = y1 + t * (y2 - y1);

                // 检查是否在边界内
                if (x < bounds.Item1 || x > bounds.Item2 || y < bounds.Item3 || y > bounds.Item4)
                {
                    return false;
                }

                // 检查是否与障碍物碰撞
                foreach (var obs in obstacles)
                {
                    if (obs.ContainsPoint(x, y))
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// 路径快捷化优化
        /// </summary>
        protected List<(double, double)> PathShortcutting(List<(double, double)> path, int maxIterations = 100)
        {
            if (path == null || path.Count <= 2)
            {
                return path;
            }

            Console.WriteLine("开始路径快捷优化...");
            double originalLength = CalculatePathLength(path);
            Console.WriteLine($"原始路径: {path.Count}个点, 长度={originalLength:F2}");

            List<(double, double)> optimizedPath = new List<(double, double)>(path);
            bool improved = true;
            int iteration = 0;

            while (improved && iteration < maxIterations)
            {
                improved = false;
                iteration++;
                int i = 0;

                while (i < optimizedPath.Count - 2)
                {
                    int maxSkip = optimizedPath.Count - 1 - i;
                    
                    // 尝试跳过尽可能多的点
                    for (int skip = maxSkip; skip > 1; skip--)
                    {
                        int j = i + skip;
                        if (IsDirectConnectionFree(optimizedPath[i], optimizedPath[j]))
                        {
                            // 可以直接连接，移除中间的点
                            optimizedPath.RemoveRange(i + 1, j - i - 1);
                            improved = true;
                            Console.WriteLine($"  迭代{iteration}: 跳过{skip - 1}个点，从点{i}直连到点{j}");
                            break;
                        }
                    }
                    i++;
                }

                if (!improved)
                {
                    Console.WriteLine("  无法继续优化，停止");
                }
            }

            double optimizedLength = CalculatePathLength(optimizedPath);
            double improvement = ((originalLength - optimizedLength) / originalLength) * 100;
            Console.WriteLine($"优化后路径: {optimizedPath.Count}个点, 长度={optimizedLength:F2}");
            Console.WriteLine($"路径缩短: {improvement:F1}%");

            return optimizedPath;
        }

        /// <summary>
        /// 计算路径长度
        /// </summary>
        protected double CalculatePathLength(List<(double, double)> path)
        {
            if (path == null || path.Count < 2)
            {
                return 0.0;
            }

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
        /// RRT-Connect + Shortcut路径规划主函数
        /// </summary>
        public override List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT-Connect + Path Shortcutting路径规划...");
            Console.WriteLine("第1步: 使用RRT-Connect寻找初始路径...");

            // 使用基类的RRT-Connect算法找到初始路径
            List<(double, double)> path = base.Plan();

            if (path == null)
            {
                return null;
            }

            Console.WriteLine($"\nRRT-Connect找到路径: {path.Count}个点");
            Console.WriteLine($"树节点总数: {nodes.Count}个");
            Console.WriteLine("\n第2步: 路径快捷优化...");

            // 进行路径快捷化优化
            List<(double, double)> optimizedPath = PathShortcutting(path);

            return optimizedPath;
        }
    }
}
