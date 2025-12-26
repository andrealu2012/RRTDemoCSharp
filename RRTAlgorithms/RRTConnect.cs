using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT-Connect算法实现 - 双向RRT
    /// 从起点和终点同时生长两棵树，直到连接
    /// </summary>
    public class RRTConnect : RRT
    {
        protected List<Node> startTree;
        protected List<Node> goalTree;
        protected Node connectNodeStart;
        protected Node connectNodeGoal;

        /// <summary>
        /// 构造函数
        /// </summary>
        public RRTConnect((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                          (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                          int? randomSeed = null)
            : base(start, goal, obstacles, bounds, stepSize, maxIter, randomSeed)
        {
            this.startTree = new List<Node> { this.start };
            this.goalTree = new List<Node> { new Node(goal.Item1, goal.Item2) };
            this.connectNodeStart = null;
            this.connectNodeGoal = null;
        }

        /// <summary>
        /// 在指定树中查找最近节点
        /// </summary>
        protected Node NearestNodeInTree(List<Node> tree, double x, double y)
        {
            return tree.OrderBy(node => Math.Pow(node.X - x, 2) + Math.Pow(node.Y - y, 2)).First();
        }

        /// <summary>
        /// 在树中扩展一个新节点
        /// </summary>
        protected Node ExtendTree(List<Node> tree, double randX, double randY)
        {
            Node nearest = NearestNodeInTree(tree, randX, randY);
            var (newX, newY) = Steer(nearest, randX, randY);

            if (!IsCollisionFree(nearest, newX, newY))
            {
                return null;
            }

            Node newNode = new Node(newX, newY);
            newNode.Parent = nearest;
            tree.Add(newNode);
            return newNode;
        }

        /// <summary>
        /// 尝试将树连接到目标点
        /// </summary>
        protected (Node, bool) ConnectTree(List<Node> tree, double targetX, double targetY)
        {
            while (true)
            {
                Node nearest = NearestNodeInTree(tree, targetX, targetY);
                double dx = targetX - nearest.X;
                double dy = targetY - nearest.Y;
                double distance = Math.Sqrt(dx * dx + dy * dy);

                if (distance < stepSize)
                {
                    if (IsCollisionFree(nearest, targetX, targetY))
                    {
                        Node newNode = new Node(targetX, targetY);
                        newNode.Parent = nearest;
                        tree.Add(newNode);
                        return (newNode, true);
                    }
                    return (null, false);
                }

                var (newX, newY) = Steer(nearest, targetX, targetY);
                if (!IsCollisionFree(nearest, newX, newY))
                {
                    return (null, false);
                }

                Node newNode2 = new Node(newX, newY);
                newNode2.Parent = nearest;
                tree.Add(newNode2);

                double distToTarget = Math.Sqrt(Math.Pow(newX - targetX, 2) + Math.Pow(newY - targetY, 2));
                if (distToTarget < stepSize * 0.5)
                {
                    return (newNode2, true);
                }
            }
        }

        /// <summary>
        /// 提取双向树的路径
        /// </summary>
        protected List<(double, double)> ExtractDualPath()
        {
            // 从起点到连接点的路径
            List<(double, double)> pathStart = new List<(double, double)>();
            Node current = connectNodeStart;
            while (current != null)
            {
                pathStart.Add((current.X, current.Y));
                current = current.Parent;
            }
            pathStart.Reverse();

            // 从连接点到终点的路径（不包括连接点）
            List<(double, double)> pathGoal = new List<(double, double)>();
            current = connectNodeGoal.Parent;
            while (current != null)
            {
                pathGoal.Add((current.X, current.Y));
                current = current.Parent;
            }

            // 合并路径
            pathStart.AddRange(pathGoal);
            return pathStart;
        }

        /// <summary>
        /// RRT-Connect路径规划主函数
        /// </summary>
        public override List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT-Connect路径规划...");

            for (int i = 0; i < maxIter; i++)
            {
                var (randX, randY) = RandomSample();
                
                // 在起始树中扩展
                Node newNodeStart = ExtendTree(startTree, randX, randY);
                
                if (newNodeStart != null)
                {
                    // 尝试将目标树连接到新节点
                    var (connectNode, success) = ConnectTree(goalTree, newNodeStart.X, newNodeStart.Y);
                    
                    if (success)
                    {
                        Console.WriteLine($"找到路径！迭代次数: {i + 1}");
                        connectNodeStart = newNodeStart;
                        connectNodeGoal = connectNode;
                        
                        // 合并两棵树
                        nodes = new List<Node>(startTree);
                        nodes.AddRange(goalTree);
                        
                        Console.WriteLine($"树节点总数: {nodes.Count}个 (起始树: {startTree.Count}, 目标树: {goalTree.Count})");
                        return ExtractDualPath();
                    }
                }

                // 交换两棵树
                var temp = startTree;
                startTree = goalTree;
                goalTree = temp;

                if ((i + 1) % 100 == 0)
                {
                    Console.WriteLine($"迭代 {i + 1}/{maxIter}...");
                }
            }

            Console.WriteLine("未找到路径，达到最大迭代次数");
            nodes = new List<Node>(startTree);
            nodes.AddRange(goalTree);
            return null;
        }
    }
}
