using System;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT*算法专用的节点类
    /// 继承自Node并添加Cost属性用于路径优化
    /// </summary>
    public class NodeStar : Node
    {
        /// <summary>
        /// 从起点到当前节点的代价（用于RRT*的路径优化）
        /// </summary>
        public double Cost { get; set; }

        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        public NodeStar(double x, double y) : base(x, y)
        {
            Cost = 0.0;
        }
    }
}
