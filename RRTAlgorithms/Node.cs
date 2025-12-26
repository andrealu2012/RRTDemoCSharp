using System;
using System.Collections.Generic;

namespace RRTAlgorithms
{
    /// <summary>
    /// RRT树的节点
    /// </summary>
    public class Node
    {
        /// <summary>
        /// X坐标
        /// </summary>
        public double X { get; set; }

        /// <summary>
        /// Y坐标
        /// </summary>
        public double Y { get; set; }

        /// <summary>
        /// 父节点
        /// </summary>
        public Node Parent { get; set; }

        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        public Node(double x, double y)
        {
            X = x;
            Y = y;
            Parent = null;
        }

        /// <summary>
        /// 获取节点坐标的元组表示
        /// </summary>
        public (double, double) GetPosition()
        {
            return (X, Y);
        }
    }
}
