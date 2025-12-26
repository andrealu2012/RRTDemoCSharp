using System;

namespace RRTAlgorithms
{
    /// <summary>
    /// 障碍物类（矩形障碍物）
    /// </summary>
    public class Obstacle
    {
        /// <summary>
        /// 左下角X坐标
        /// </summary>
        public double X { get; set; }

        /// <summary>
        /// 左下角Y坐标
        /// </summary>
        public double Y { get; set; }

        /// <summary>
        /// 宽度
        /// </summary>
        public double Width { get; set; }

        /// <summary>
        /// 高度
        /// </summary>
        public double Height { get; set; }

        public Obstacle(double x, double y, double width, double height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        /// <summary>
        /// 检测点是否在障碍物内
        /// </summary>
        public bool ContainsPoint(double px, double py)
        {
            return px >= X && px <= X + Width && py >= Y && py <= Y + Height;
        }
    }
}
