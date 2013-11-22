using System;
using System.Collections.Generic;
using System.Windows;
using SharpDX;

namespace BezierCurveFitting
{
	public static class DPReduction
	{
        public static List<Vector3> DouglasPeuckerReduction(List<Vector3> Points, double Tolerance)
		{
			if (Points == null || Points.Count < 3)
			{
				return Points;
			}
			int num = 0;
			int num2 = Points.Count-1;
			List<int> list = new List<int>();
			list.Add(num);
			list.Add(num2);
            /*
			while (Points[num]==Points[num2])
			{
				num2--;
			}
            */
			DPReduction.DouglasPeuckerReduction(Points, num, num2, Tolerance, ref list);
            List<Vector3> list2 = new List<Vector3>();
			list.Sort();
			foreach (int current in list)
			{
				list2.Add(Points[current]);
			}
			return list2;
		}
        private static void DouglasPeuckerReduction(List<Vector3> points, int firstPoint, int lastPoint, double tolerance, ref List<int> pointIndexsToKeep)
		{
			float num = 0.0f;
			int num2 = 0;
			for (int i = firstPoint; i <= lastPoint; i++)
			{
				//float num3 = DPReduction.PerpendicularDistance(points[firstPoint], points[lastPoint], points[i]);
                float num3 = HeroPerpendicularDistance(points[firstPoint], points[lastPoint], points[i]);
				if (num3 > num)
				{
					num = num3;
					num2 = i;
				}
			}
			if (num > tolerance && num2 != 0)
			{
				pointIndexsToKeep.Add(num2);
				DPReduction.DouglasPeuckerReduction(points, firstPoint, num2, tolerance, ref pointIndexsToKeep);
				DPReduction.DouglasPeuckerReduction(points, num2, lastPoint, tolerance, ref pointIndexsToKeep);
			}
		}
        public static float HeroTriangleArea(Vector3 A, Vector3 B, Vector3 C)
        {
            float _ab = (A - B).Length();
            float _ab2 = _ab*_ab;
            float _bc = (C - B).Length();
            float _bc2 = _bc*_bc;
            float _ac = (A - C).Length();
            float _ac2 = _ac*_ac;
            float T = (float)((1.0f / 4.0f) * Math.Sqrt(4 * _bc2 * _ac2 - Math.Pow((_bc2 + _ac2 - _ab2), 2)));
            return T;
        }
        public static float HeroPerpendicularDistance(Vector3 A, Vector3 B, Vector3 C)
        {
            float area = HeroTriangleArea(A, B, C);
            float pD = area * 2 / (B - A).Length();
            return pD;
        }
        public static float PerpendicularDistance(Vector3 Point1, Vector3 Point2, Vector3 Point)
		{
			float num = Math.Abs(0.5f * (Point1.X * Point2.Y + Point2.X * Point.Y + Point.X * Point1.Y - Point2.X * Point1.Y - Point.X * Point2.Y - Point1.X * Point.Y));
			float num2 = (float)Math.Sqrt(Math.Pow(Point1.X - Point2.X, 2.0f) + Math.Pow(Point1.Y - Point2.Y, 2.0f));
			return num / num2 * 2.0f;
		}
	}
}
