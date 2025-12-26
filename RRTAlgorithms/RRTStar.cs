using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT*算法实现 - RRT的优化版本
    /// 通过rewire操作优化路径，生成更短的路径
    /// </summary>
    public class RRTStar : RRT
    {
        protected double searchRadius;

        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="searchRadius">搜索半径，用于寻找邻近节点</param>
        public RRTStar((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                       (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                       double searchRadius = 1.5, int? randomSeed = null)
            : base(start, goal, obstacles, bounds, stepSize, maxIter, randomSeed)
        {
            this.searchRadius = searchRadius;
        }

        /// <summary>
        /// 查找新节点附近的所有节点
        /// </summary>
        protected List<Node> FindNearNodes(Node newNode)
        {
            return nodes.Where(node => Distance(node, newNode) <= searchRadius).ToList();
        }

        /// <summary>
        /// 为新节点选择最优父节点
        /// </summary>
        protected Node ChooseParent(Node newNode, List<Node> nearNodes)
        {
            if (nearNodes.Count == 0)
            {
                return null;
            }

            double minCost = double.MaxValue;
            Node bestParent = null;

            foreach (var nearNode in nearNodes)
            {
                double cost = nearNode.Cost + Distance(nearNode, newNode);
                if (cost < minCost && IsCollisionFree(nearNode, newNode.X, newNode.Y))
                {
                    minCost = cost;
                    bestParent = nearNode;
                }
            }

            if (bestParent != null)
            {
                newNode.Parent = bestParent;
                newNode.Cost = minCost;
            }

            return bestParent;
        }

        /// <summary>
        /// 重连操作：检查是否可以通过新节点改善邻近节点的路径
        /// </summary>
        protected void Rewire(Node newNode, List<Node> nearNodes)
        {
            foreach (var nearNode in nearNodes)
            {
                double newCost = newNode.Cost + Distance(newNode, nearNode);
                if (newCost < nearNode.Cost && IsCollisionFree(newNode, nearNode.X, nearNode.Y))
                {
                    nearNode.Parent = newNode;
                    nearNode.Cost = newCost;
                    UpdateChildrenCost(nearNode);
                }
            }
        }

        /// <summary>
        /// 递归更新所有子节点的代价
        /// </summary>
        protected void UpdateChildrenCost(Node parentNode)
        {
            foreach (var node in nodes)
            {
                if (node.Parent == parentNode)
                {
                    node.Cost = parentNode.Cost + Distance(parentNode, node);
                    UpdateChildrenCost(node);
                }
            }
        }

        /// <summary>
        /// RRT*路径规划主函数
        /// </summary>
        public override List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT*路径规划...");

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
                List<Node> nearNodes = FindNearNodes(newNode);

                // 选择最优父节点
                Node bestParent = ChooseParent(newNode, nearNodes);
                if (bestParent == null)
                {
                    newNode.Parent = nearest;
                    newNode.Cost = nearest.Cost + Distance(nearest, newNode);
                }

                nodes.Add(newNode);

                // Rewire操作
                Rewire(newNode, nearNodes);

                if (IsGoalReached(newNode))
                {
                    Console.WriteLine($"找到路径！迭代次数: {i + 1}");
                    goal.Parent = newNode;
                    goal.Cost = newNode.Cost + Distance(newNode, goal);
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
    }
}
