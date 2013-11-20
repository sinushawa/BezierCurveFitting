using System;
using System.Collections.Generic;
using System.Windows;
using SharpDX;

namespace BezierCurveFitting
{
	public static class Bezier
	{
		public static List<BezierPoint> CurveSolution(List<Vector3> existingPoints, float reductionTolerance, float error)
		{
			List<Vector3> list = DPReduction.DouglasPeuckerReduction(existingPoints, reductionTolerance);
			return FitCurves.FitCurve(list.ToArray(), error);
		}
	}
}
