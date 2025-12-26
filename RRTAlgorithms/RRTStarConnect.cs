using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT*-Connect算法实现 - 结合双向搜索和路径优化
    /// </summary>
    public class RRTStarConnect : RRTConnect
    {
        protected double searchRadius;

        /// <summary>
        /// 构造函数
        /// </summary>
        public RRTStarConnect((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                              (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                              double searchRadius = 1.5, int? randomSeed = null)
            : base(start, goal, obstacles, bounds, stepSize, maxIter, randomSeed)
        {
            this.searchRadius = searchRadius;
        }

        /// <summary>
        /// 在指定树中查找新节点附近的所有节点
        /// </summary>
        protected List<Node> FindNearNodesInTree(List<Node> tree, Node newNode)
        {
            return tree.Where(node => Distance(node, newNode) <= searchRadius).ToList();
        }

        /// <summary>
        /// 在指定树中为新节点选择最优父节点
        /// </summary>
        protected Node ChooseParentInTree(List<Node> tree, Node newNode, List<Node> nearNodes)
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
        /// 在指定树中进行rewire操作
        /// </summary>
        protected void RewireTree(List<Node> tree, Node newNode, List<Node> nearNodes)
        {
            foreach (var nearNode in nearNodes)
            {
                double newCost = newNode.Cost + Distance(newNode, nearNode);
                if (newCost < nearNode.Cost && IsCollisionFree(newNode, nearNode.X, nearNode.Y))
                {
                    nearNode.Parent = newNode;
                    nearNode.Cost = newCost;
                    UpdateChildrenCostInTree(tree, nearNode);
                }
            }
        }

        /// <summary>
        /// 递归更新指定树中所有子节点的代价
        /// </summary>
        protected void UpdateChildrenCostInTree(List<Node> tree, Node parentNode)
        {
            foreach (var node in tree)
            {
                if (node.Parent == parentNode)
                {
                    node.Cost = parentNode.Cost + Distance(parentNode, node);
                    UpdateChildrenCostInTree(tree, node);
                }
            }
        }

        /// <summary>
        /// 在树中扩展一个新节点（带优化）
        /// </summary>
        protected Node ExtendTreeWithOptimization(List<Node> tree, double randX, double randY)
        {
            Node nearest = NearestNodeInTree(tree, randX, randY);
            var (newX, newY) = Steer(nearest, randX, randY);

            if (!IsCollisionFree(nearest, newX, newY))
            {
                return null;
            }

            Node newNode = new Node(newX, newY);
            List<Node> nearNodes = FindNearNodesInTree(tree, newNode);

            // 选择最优父节点
            Node bestParent = ChooseParentInTree(tree, newNode, nearNodes);
            if (bestParent == null)
            {
                newNode.Parent = nearest;
                newNode.Cost = nearest.Cost + Distance(nearest, newNode);
            }

            tree.Add(newNode);

            // Rewire操作
            RewireTree(tree, newNode, nearNodes);

            return newNode;
        }

        /// <summary>
        /// 尝试将树连接到目标点（带优化）
        /// </summary>
        protected (Node, bool) ConnectTreeWithOptimization(List<Node> tree, double targetX, double targetY)
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
                        newNode.Cost = nearest.Cost + Distance(nearest, newNode);
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
                newNode2.Cost = nearest.Cost + Distance(nearest, newNode2);
                tree.Add(newNode2);

                double distToTarget = Math.Sqrt(Math.Pow(newX - targetX, 2) + Math.Pow(newY - targetY, 2));
                if (distToTarget < stepSize * 0.5)
                {
                    return (newNode2, true);
                }
            }
        }

        /// <summary>
        /// RRT*-Connect路径规划主函数
        /// </summary>
        public override List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT*-Connect路径规划...");

            for (int i = 0; i < maxIter; i++)
            {
                var (randX, randY) = RandomSample();

                // 在起始树中扩展（带优化）
                Node newNodeStart = ExtendTreeWithOptimization(startTree, randX, randY);

                if (newNodeStart != null)
                {
                    // 尝试将目标树连接到新节点（带优化）
                    var (connectNode, success) = ConnectTreeWithOptimization(goalTree, newNodeStart.X, newNodeStart.Y);

                    if (success)
                    {
                        Console.WriteLine($"找到路径！迭代次数: {i + 1}");
                        connectNodeStart = newNodeStart;
                        connectNodeGoal = connectNode;

                        // 合并两棵树
                        nodes = new List<Node>(startTree);
                        nodes.AddRange(goalTree);

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
