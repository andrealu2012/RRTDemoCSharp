using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT算法基础实现
    /// </summary>
    public class RRT
    {
        protected Node start;
        protected Node goal;
        protected List<Obstacle> obstacles;
        protected (double, double, double, double) bounds; // (x_min, x_max, y_min, y_max)
        protected double stepSize;
        protected int maxIter;
        protected double goalThreshold;
        protected List<Node> nodes;
        protected Random random;

        public List<Node> Nodes => nodes;
        public Node Start => start;
        public Node Goal => goal;
        public List<Obstacle> Obstacles => obstacles;
        public (double, double, double, double) Bounds => bounds;

        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="start">起点坐标</param>
        /// <param name="goal">终点坐标</param>
        /// <param name="obstacles">障碍物列表</param>
        /// <param name="bounds">空间边界 (x_min, x_max, y_min, y_max)</param>
        /// <param name="stepSize">扩展步长</param>
        /// <param name="maxIter">最大迭代次数</param>
        /// <param name="randomSeed">随机种子（可选）</param>
        public RRT((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                   (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                   int? randomSeed = null)
        {
            this.start = new Node(start.Item1, start.Item2);
            this.goal = new Node(goal.Item1, goal.Item2);
            this.obstacles = obstacles;
            this.bounds = bounds;
            this.stepSize = stepSize;
            this.maxIter = maxIter;
            this.goalThreshold = 0.5;
            this.nodes = new List<Node> { this.start };
            this.random = randomSeed.HasValue ? new Random(randomSeed.Value) : new Random();
        }

        /// <summary>
        /// 随机采样
        /// </summary>
        protected virtual (double, double) RandomSample()
        {
            // 10%的概率采样目标点，促进向目标探索
            if (random.NextDouble() < 0.1)
            {
                return (goal.X, goal.Y);
            }

            double x = random.NextDouble() * (bounds.Item2 - bounds.Item1) + bounds.Item1;
            double y = random.NextDouble() * (bounds.Item4 - bounds.Item3) + bounds.Item3;
            return (x, y);
        }

        /// <summary>
        /// 寻找最近节点
        /// </summary>
        protected virtual Node NearestNode(double x, double y)
        {
            return nodes.OrderBy(node => Math.Pow(node.X - x, 2) + Math.Pow(node.Y - y, 2)).First();
        }

        /// <summary>
        /// 从from_node向目标点移动stepSize距离
        /// </summary>
        protected virtual (double, double) Steer(Node fromNode, double toX, double toY)
        {
            double dx = toX - fromNode.X;
            double dy = toY - fromNode.Y;
            double distance = Math.Sqrt(dx * dx + dy * dy);

            if (distance < stepSize)
            {
                return (toX, toY);
            }

            double ratio = stepSize / distance;
            double newX = fromNode.X + dx * ratio;
            double newY = fromNode.Y + dy * ratio;
            return (newX, newY);
        }

        /// <summary>
        /// 检查从from_node到(to_x, to_y)的路径是否无碰撞
        /// </summary>
        protected virtual bool IsCollisionFree(Node fromNode, double toX, double toY)
        {
            double distance = Math.Sqrt(Math.Pow(toX - fromNode.X, 2) + Math.Pow(toY - fromNode.Y, 2));
            int steps = Math.Max((int)(distance / 0.1), 1);

            for (int i = 0; i <= steps; i++)
            {
                double t = (double)i / steps;
                double x = fromNode.X + t * (toX - fromNode.X);
                double y = fromNode.Y + t * (toY - fromNode.Y);

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
        /// 检查节点是否到达目标
        /// </summary>
        protected virtual bool IsGoalReached(Node node)
        {
            double distance = Math.Sqrt(Math.Pow(node.X - goal.X, 2) + Math.Pow(node.Y - goal.Y, 2));
            return distance < goalThreshold;
        }

        /// <summary>
        /// 路径规划主函数
        /// </summary>
        public virtual List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT路径规划...");
            
            for (int i = 0; i < maxIter; i++)
            {
                var (randX, randY) = RandomSample();
                Node nearest = NearestNode(randX, randY);
                var (newX, newY) = Steer(nearest, randX, randY);

                if (!IsCollisionFree(nearest, newX, newY))
                {
                    continue;
                }

                Node newNode = new Node(newX, newY);
                newNode.Parent = nearest;
                nodes.Add(newNode);

                if (IsGoalReached(newNode))
                {
                    Console.WriteLine($"找到路径！迭代次数: {i + 1}");
                    goal.Parent = newNode;
                    return ExtractPath();
                }

                if ((i + 1) % 100 == 0)
                {
                    Console.WriteLine($"迭代 {i + 1}/{maxIter}...");
                }
            }

            Console.WriteLine("未找到路径，达到最大迭代次数");
            return null;
        }

        /// <summary>
        /// 提取从起点到终点的路径
        /// </summary>
        protected virtual List<(double, double)> ExtractPath()
        {
            List<(double, double)> path = new List<(double, double)>();
            Node current = goal;
            
            while (current != null)
            {
                path.Add((current.X, current.Y));
                current = current.Parent;
            }
            
            path.Reverse();
            return path;
        }

        /// <summary>
        /// 计算两个节点之间的欧氏距离
        /// </summary>
        protected double Distance(Node node1, Node node2)
        {
            return Math.Sqrt(Math.Pow(node1.X - node2.X, 2) + Math.Pow(node1.Y - node2.Y, 2));
        }
    }
}
