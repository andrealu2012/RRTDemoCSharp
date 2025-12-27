using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT-Connect算法实现 - 双向RRT
    /// 
    /// 【算法核心思想】
    /// RRT-Connect是RRT的改进版本，采用双向搜索策略：
    /// 1. 同时从起点和终点构建两棵独立的搜索树
    /// 2. 两棵树交替向随机采样点扩展
    /// 3. 每次一棵树扩展后，另一棵树尝试连接到新节点
    /// 4. 当两棵树成功连接时，路径找到
    /// 
    /// 【相比RRT的优势】
    /// - 速度更快：双向搜索减少了搜索空间
    /// - 效率更高：通常只需要RRT一半的迭代次数
    /// - 更容易连接：两棵树相向生长，更容易找到可行路径
    /// </summary>
    public class RRTConnect : RRT
    {
        // 从起点生长的树
        protected List<Node> startTree;
        
        // 从终点生长的树
        protected List<Node> goalTree;
        
        // 两棵树的连接点（在起始树中）
        protected Node connectNodeStart;
        
        // 两棵树的连接点（在目标树中）
        protected Node connectNodeGoal;

        /// <summary>
        /// 构造函数
        /// 初始化两棵独立的搜索树，分别从起点和终点开始
        /// </summary>
        public RRTConnect((double, double) start, (double, double) goal, List<Obstacle> obstacles,
                          (double, double, double, double) bounds, double stepSize = 0.5, int maxIter = 1000,
                          int? randomSeed = null)
            : base(start, goal, obstacles, bounds, stepSize, maxIter, randomSeed)
        {
            // 初始化起始树，包含起点
            this.startTree = new List<Node> { this.start };
            
            // 初始化目标树，包含终点
            this.goalTree = new List<Node> { new Node(goal.Item1, goal.Item2) };
            
            // 连接节点初始为空，找到路径后会设置
            this.connectNodeStart = null;
            this.connectNodeGoal = null;
        }

        /// <summary>
        /// 在指定树中查找最近节点
        /// 遍历树中所有节点，找到距离目标点(x,y)最近的节点
        /// </summary>
        /// <param name="tree">要搜索的树</param>
        /// <param name="x">目标点X坐标</param>
        /// <param name="y">目标点Y坐标</param>
        /// <returns>树中距离目标点最近的节点</returns>
        protected Node NearestNodeInTree(List<Node> tree, double x, double y)
        {
            return tree.OrderBy(node => Math.Pow(node.X - x, 2) + Math.Pow(node.Y - y, 2)).First();
        }

        /// <summary>
        /// 在树中扩展一个新节点（Extend操作）
        /// 
        /// 【步骤】
        /// 1. 在树中找到距离随机点最近的节点
        /// 2. 从最近节点向随机点方向扩展一步（步长为stepSize）
        /// 3. 检查新节点是否与障碍物碰撞
        /// 4. 如果无碰撞，将新节点添加到树中
        /// </summary>
        /// <param name="tree">要扩展的树</param>
        /// <param name="randX">随机采样点X坐标</param>
        /// <param name="randY">随机采样点Y坐标</param>
        /// <returns>成功返回新节点，失败返回null</returns>
        protected Node ExtendTree(List<Node> tree, double randX, double randY)
        {
            // 1. 找到树中距离随机点最近的节点
            Node nearest = NearestNodeInTree(tree, randX, randY);
            
            // 2. 从最近节点向随机点方向扩展一步
            var (newX, newY) = Steer(nearest, randX, randY);

            // 3. 检查是否与障碍物碰撞
            if (!IsCollisionFree(nearest, newX, newY))
            {
                return null;  // 碰撞，扩展失败
            }

            // 4. 创建新节点并添加到树中
            Node newNode = new Node(newX, newY);
            newNode.Parent = nearest;  // 设置父节点，用于后续路径提取
            tree.Add(newNode);
            return newNode;
        }

        /// <summary>
        /// 尝试将树连接到目标点（Connect操作）
        /// 
        /// 【核心思想】
        /// 与Extend不同，Connect会贪心地持续向目标点扩展，
        /// 而不是只扩展一步，直到：
        /// - 成功到达目标点，或
        /// - 遇到障碍物无法继续
        /// 
        /// 【步骤】
        /// 1. 持续循环，每次向目标点靠近一步
        /// 2. 如果距离目标点很近，尝试直接连接
        /// 3. 否则，向目标点方向扩展一步
        /// 4. 如果遇到障碍物，停止并返回失败
        /// </summary>
        /// <param name="tree">要扩展的树</param>
        /// <param name="targetX">目标点X坐标</param>
        /// <param name="targetY">目标点Y坐标</param>
        /// <returns>(最后添加的节点, 是否成功连接)</returns>
        protected (Node, bool) ConnectTree(List<Node> tree, double targetX, double targetY)
        {
            while (true)  // 持续尝试连接，直到成功或失败
            {
                // 找到树中距离目标点最近的节点
                Node nearest = NearestNodeInTree(tree, targetX, targetY);
                double dx = targetX - nearest.X;
                double dy = targetY - nearest.Y;
                double distance = Math.Sqrt(dx * dx + dy * dy);

                // 情况1：距离目标点很近，尝试直接连接
                if (distance < stepSize)
                {
                    if (IsCollisionFree(nearest, targetX, targetY))
                    {
                        // 成功！直接连接到目标点
                        Node newNode = new Node(targetX, targetY);
                        newNode.Parent = nearest;
                        tree.Add(newNode);
                        return (newNode, true);  // 连接成功
                    }
                    return (null, false);  // 碰撞，连接失败
                }

                // 情况2：距离较远，向目标点方向扩展一步
                var (newX, newY) = Steer(nearest, targetX, targetY);
                if (!IsCollisionFree(nearest, newX, newY))
                {
                    return (null, false);  // 碰撞，停止连接
                }

                // 扩展成功，添加新节点并继续
                Node newNode2 = new Node(newX, newY);
                newNode2.Parent = nearest;
                tree.Add(newNode2);

                // 检查是否足够接近目标点，可以结束
                double distToTarget = Math.Sqrt(Math.Pow(newX - targetX, 2) + Math.Pow(newY - targetY, 2));
                if (distToTarget < stepSize * 0.5)
                {
                    return (newNode2, true);  // 足够接近，视为成功
                }
                
                // 继续循环，进一步靠近目标点
            }
        }

        /// <summary>
        /// 提取双向树的完整路径
        /// 
        /// 【路径构建】
        /// 因为是双向搜索，最终路径由两部分组成：
        /// 1. 从起点到连接点（在起始树中）
        /// 2. 从连接点到终点（在目标树中）
        /// 
        /// 需要注意：
        /// - 起始树的路径需要反转（因为是从连接点追溯到起点）
        /// - 目标树的路径顺序正确（从连接点到终点）
        /// - 合并时要避免重复连接点
        /// </summary>
        /// <returns>从起点到终点的完整路径</returns>
        protected List<(double, double)> ExtractDualPath()
        {
            // 第1部分：从起点到连接点的路径
            List<(double, double)> pathStart = new List<(double, double)>();
            Node current = connectNodeStart;
            while (current != null)
            {
                pathStart.Add((current.X, current.Y));
                current = current.Parent;  // 沿着父节点向起点回溯
            }
            pathStart.Reverse();  // 反转，使其从起点到连接点

            // 第2部分：从连接点到终点的路径（不包括连接点本身，避免重复）
            List<(double, double)> pathGoal = new List<(double, double)>();
            current = connectNodeGoal.Parent;  // 从连接点的父节点开始
            while (current != null)
            {
                pathGoal.Add((current.X, current.Y));
                current = current.Parent;  // 沿着父节点向终点回溯
            }
            // 注意：pathGoal不需要反转，因为目标树是从终点向连接点回溯的

            // 合并两部分路径：起点 -> 连接点 -> 终点
            pathStart.AddRange(pathGoal);
            return pathStart;
        }

        /// <summary>
        /// RRT-Connect路径规划主函数
        /// 
        /// 【算法流程】
        /// 1. 随机采样一个点
        /// 2. 在第一棵树中向采样点扩展（Extend）
        /// 3. 如果扩展成功，尝试用第二棵树连接到新节点（Connect）
        /// 4. 如果连接成功，路径找到！
        /// 5. 如果连接失败，交换两棵树的角色
        /// 6. 重复以上步骤
        /// 
        /// 【树的交换策略】
        /// 每次迭代后交换两棵树，使得：
        /// - 起始树和目标树轮流主动扩展
        /// - 保持两棵树平衡生长
        /// - 更容易在中间区域相遇
        /// </summary>
        public override List<(double, double)> Plan()
        {
            Console.WriteLine("开始RRT-Connect路径规划...");

            for (int i = 0; i < maxIter; i++)
            {
                // 步骤1：在空间中随机采样一个点
                var (randX, randY) = RandomSample();
                
                // 步骤2：在第一棵树（起始树）中向随机点扩展
                Node newNodeStart = ExtendTree(startTree, randX, randY);
                
                if (newNodeStart != null)  // 扩展成功
                {
                    // 步骤3：尝试用第二棵树（目标树）连接到新节点
                    // 这是RRT-Connect的核心：一棵树扩展，另一棵树立即尝试连接
                    var (connectNode, success) = ConnectTree(goalTree, 
                    newNodeStart.X, newNodeStart.Y);
                    
                    if (success)  // 连接成功，两棵树相遇！
                    {
                        Console.WriteLine($"找到路径！迭代次数: {i + 1}");
                        
                        // 保存连接点，用于路径提取
                        connectNodeStart = newNodeStart;  // 起始树的连接点
                        connectNodeGoal = connectNode;    // 目标树的连接点
                        
                        // 合并两棵树到nodes列表（用于可视化或统计）
                        nodes = new List<Node>(startTree);
                        nodes.AddRange(goalTree);
                        
                        Console.WriteLine($"树节点总数: {nodes.Count}个 (起始树: {startTree.Count}, 目标树: {goalTree.Count})");
                        return ExtractDualPath();  // 提取并返回完整路径
                    }
                }

                // 如果不成功，则步骤4：交换两棵树的角色
                // 下一次迭代时，原来的目标树变成起始树，反之亦然
                // 这样两棵树轮流主动扩展，保持平衡
                var temp = startTree;
                startTree = goalTree;
                goalTree = temp;

                // 每100次迭代打印一次进度
                if ((i + 1) % 100 == 0)
                {
                    Console.WriteLine($"迭代 {i + 1}/{maxIter}...");
                }
            }

            // 达到最大迭代次数仍未找到路径
            Console.WriteLine("未找到路径，达到最大迭代次数");
            nodes = new List<Node>(startTree);
            nodes.AddRange(goalTree);
            return null;
        }
    }
}
