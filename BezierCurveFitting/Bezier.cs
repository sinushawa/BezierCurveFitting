using System;
using System.Collections.Generic;
using System.Windows;
using SharpDX;

namespace BezierCurveFitting
{
	public static class Bezier
	{
        public static List<BezierPoint> CurveSolution(Method _method, List<Vector3> existingPoints, float error, int desiredPoints)
		{
            if (_method == Method.Precision)
            {
                List<Vector3> list = DPReduction.DouglasPeuckerReduction(existingPoints, error);
                return FitCurves.FitCurveBy(_method, list.ToArray(), error, 0);
            }
            else
            {
                return FitCurves.FitCurveBy(_method, existingPoints.ToArray(), 0.0f, desiredPoints);
            }
		}
	}
}
