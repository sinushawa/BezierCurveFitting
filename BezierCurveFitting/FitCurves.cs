using System;
using System.Collections.Generic;
using System.Linq;
using SharpDX;
using MoreLinq;

namespace BezierCurveFitting
{
    public enum Method
    {
        Precision,
        Length,
        Path
    }

	public static class FitCurves
	{
		private const int MAXPOINTS = 10000;
        public static List<BezierPoint> FitCurveBy(Method _method, Vector3[] d, float error, int desiredPoints)
        {
            Vector3 tHat;
            Vector3 tHat2;
            if (d[0] == d[d.Length - 1])
            {
                tHat = (d[1] - d[d.Length - 2]);
                tHat.Normalize();
                tHat2 = Vector3.Negate(tHat);
            }
            else
            {
                tHat = FitCurves.ComputeLeftTangent(d, 0);
                tHat2 = FitCurves.ComputeRightTangent(d, d.Length - 1);
            }
            List<BezierPoint> result = new List<BezierPoint>();
            if (_method == Method.Precision)
            {
                FitCurves.FitByPrecision(d, 0, d.Length - 1, tHat, tHat2, error, result);
            }
            else if (_method != Method.Precision)
            {
                FitCurves.FitByParametric(_method, d, 0, d.Length - 1, tHat, tHat2, desiredPoints, result);
            }
            return result;
        }
        private static List<int> GetLengthEqualIDs(Vector3[] d, int first, int last, int nbCtrlPoints)
        {
            List<int> result = new List<int>();
            List<float> array2 = FitCurves.ChordLengthParameterize(d, first, last).ToList();
            for (int i = 0; i < nbCtrlPoints; i++)
            {
                float desiredDiv = (i / (float)(nbCtrlPoints-1));
                result.Add(array2.IndexOf(array2.MinBy(x => Math.Abs(x - desiredDiv))));
            }
            return result;
        }
        private static List<int> GetPathEqualIDs(Vector3[] d, int first, int last, int nbCtrlPoints)
        {
            List<int> result = new List<int>();
            List<float> array2 = FitCurves.ChordLengthParameterize(d, first, last).ToList();
            for (int i = 0; i < nbCtrlPoints; i++)
            {
                int desiredDiv = ((d.Length-1)/(nbCtrlPoints-1))*i;
                result.Add(desiredDiv);
            }
            return result;
        }
        private static void FitByParametric(Method _method, Vector3[] d, int first, int last, Vector3 tHat1, Vector3 tHat2, int nbCtrlPoints, List<BezierPoint> result)
        {
            List<SplineHolder> solutions = new List<SplineHolder>();
            SplineHolder solution = new SplineHolder();
            solution.pointsData = new List<Vector3[]>();
            List<int> ctrlPointsIDs;
            if (_method == Method.Length)
            {
                ctrlPointsIDs = GetLengthEqualIDs(d, first, last, nbCtrlPoints);
            }
            else
            {
                ctrlPointsIDs = GetPathEqualIDs(d, first, last, nbCtrlPoints);
            }
            Vector3 keepHandle1 = tHat1;
            Vector3 keepHandle2 = tHat2;
            for (int i = 0; i < ctrlPointsIDs.Count - 1; i++)
            {
                first = ctrlPointsIDs[i];
                last = ctrlPointsIDs[i + 1];
                if (i == 0)
                {
                    tHat1 = keepHandle1;
                    tHat2 = ComputeCenterTangent(d, last);
                }
                else if (i == ctrlPointsIDs.Count - 2)
                {
                    tHat1 = Vector3.Negate(ComputeCenterTangent(d, first));
                    tHat2 = keepHandle2;
                }
                else
                {
                    tHat1 = Vector3.Negate(ComputeCenterTangent(d, first));
                    tHat2 = ComputeCenterTangent(d, last);
                }
                float[] array2 = FitCurves.ChordLengthParameterize(d, first, last);
                Vector3[] array = FitCurves.GenerateBezier(d, first, last, array2, tHat1, tHat2);
                solution.pointsData.Add(array);
                int num5;
                float num4 = FitCurves.ComputeMaxError(d, first, last, array, array2, out num5);
                solution.errors += num4;
            }
            solutions.Add(solution);
            SplineHolder bestSolution = solutions.Where(x => x.errors == (solutions.Min(y => y.errors))).FirstOrDefault();
            for (int i = 0; i < bestSolution.pointsData.Count; i++)
            {
                if (i != 0)
                {
                    result.Add(new BezierPoint(bestSolution.pointsData[i][0], bestSolution.pointsData[i - 1][2], bestSolution.pointsData[i][1]));
                }
                else
                {
                    result.Add(new BezierPoint(bestSolution.pointsData[i][0], null, bestSolution.pointsData[i][1]));
                }
            }
            result.Add(new BezierPoint(bestSolution.pointsData[bestSolution.pointsData.Count-1][3], bestSolution.pointsData[bestSolution.pointsData.Count-1][2], null));
        }
        private static void FitByPrecision(Vector3[] d, int first, int last, Vector3 tHat1, Vector3 tHat2, float error, List<BezierPoint> result)
		{
			int num = 4;
			float num2 = error * error;
			int num3 = last - first + 1;
			Vector3[] array;
			if (num3 == 2)
			{
				float scalar = (d[first] - d[last]).Length() / 3.0f;
				array = new Vector3[]
				{
					d[first],
					default(Vector3),
					default(Vector3),
					d[last]
				};
				array[1] = tHat1 * scalar + array[0];
				array[2] = tHat2 * scalar + array[3];
				result.Add(new BezierPoint(array[0], null, new Vector3?(array[1])));
				result.Add(new BezierPoint(array[3], new Vector3?(array[2]), null));
				return;
			}
			float[] array2 = FitCurves.ChordLengthParameterize(d, first, last);
			array = FitCurves.GenerateBezier(d, first, last, array2, tHat1, tHat2);
			int num5;
			float num4 = FitCurves.ComputeMaxError(d, first, last, array, array2, out num5);
			if (num4 < error)
			{
				result.Add(new BezierPoint(array[0], null, new Vector3?(array[1])));
				result.Add(new BezierPoint(array[3], new Vector3?(array[2]), null));
				return;
			}
			if (num4 < num2)
			{
				for (int i = 0; i < num; i++)
				{
					float[] array3 = FitCurves.Reparameterize(d, first, last, array2, array);
					array = FitCurves.GenerateBezier(d, first, last, array3, tHat1, tHat2);
					num4 = FitCurves.ComputeMaxError(d, first, last, array, array3, out num5);
					if (num4 < error)
					{
						result.Add(new BezierPoint(array[2], new Vector3?(array[1]), new Vector3?(array[3])));
						return;
					}
					array2 = array3;
				}
			}
			for (int i = 0; i < result.Count - 1; i++)
			{
				if (Vector3.Distance(result[i].point, result[i+1].point)<error)
				{
					result[i].foreHandle = (result[i].foreHandle.HasValue ? result[i].foreHandle : result[i + 1].foreHandle);
					result[i].afterHandle = (result[i].afterHandle.HasValue ? result[i].afterHandle : result[i + 1].afterHandle);
					result.RemoveAt(i + 1);
				}
			}
			Vector3 vector = FitCurves.ComputeCenterTangent(d, num5);
			FitCurves.FitByPrecision(d, first, num5, tHat1, vector, error, result);
            vector = Vector3.Negate(vector);
			FitCurves.FitByPrecision(d, num5, last, vector, tHat2, error, result);
		}
		private static Vector3[] GenerateBezier(Vector3[] d, int first, int last, float[] uPrime, Vector3 tHat1, Vector3 tHat2)
		{
			Vector3[,] array = new Vector3[10000, 2];
			float[,] array2 = new float[2, 2];
			float[] array3 = new float[2];
			Vector3[] array4 = new Vector3[4];
			int num = last - first + 1;
			for (int i = 0; i < num; i++)
			{
				Vector3 vector = tHat1 * FitCurves.B1(uPrime[i]);
				Vector3 vector2 = tHat2 * FitCurves.B2(uPrime[i]);
				array[i, 0] = vector;
				array[i, 1] = vector2;
			}
			array2[0, 0] = 0.0f;
			array2[0, 1] = 0.0f;
			array2[1, 0] = 0.0f;
			array2[1, 1] = 0.0f;
			array3[0] = 0.0f;
			array3[1] = 0.0f;
			for (int i = 0; i < num; i++)
			{
				array2[0, 0] += Vector3.Dot(array[i, 0], array[i, 0]);
				array2[0, 1] += Vector3.Dot(array[i, 0], array[i, 1]);
				array2[1, 0] = array2[0, 1];
				array2[1, 1] += Vector3.Dot(array[i, 1], array[i, 1]);
				Vector3 b = (Vector3)d[first + i] - ((Vector3)d[first] * FitCurves.B0(uPrime[i]) + ((Vector3)d[first] * FitCurves.B1(uPrime[i]) + ((Vector3)d[last] * FitCurves.B2(uPrime[i]) + (Vector3)d[last] * FitCurves.B3(uPrime[i]))));
				array3[0] += Vector3.Dot(array[i, 0], b);
				array3[1] += Vector3.Dot(array[i, 1], b);
			}
			float num2 = array2[0, 0] * array2[1, 1] - array2[1, 0] * array2[0, 1];
			float num3 = array2[0, 0] * array3[1] - array2[1, 0] * array3[0];
			float num4 = array3[0] * array2[1, 1] - array3[1] * array2[0, 1];
			float num5 = (num2 == 0.0) ? 0.0f : (num4 / num2);
			float num6 = (num2 == 0.0) ? 0.0f : (num3 / num2);
			float length = (d[first] - d[last]).Length();
			float num7 = (float)1E-06 * length;
			if (num5 < num7 || num6 < num7)
			{
				float scalar = length / 3.0f;
				array4[0] = d[first];
				array4[3] = d[last];
				array4[1] = tHat1 * scalar + array4[0];
				array4[2] = tHat2 * scalar + array4[3];
				return array4;
			}
			array4[0] = d[first];
			array4[3] = d[last];
			array4[1] = tHat1 * num5 + array4[0];
			array4[2] = tHat2 * num6 + array4[3];
			return array4;
		}
		private static float[] Reparameterize(Vector3[] d, int first, int last, float[] u, Vector3[] bezCurve)
		{
			int num = last - first + 1;
			float[] array = new float[num];
			for (int i = first; i <= last; i++)
			{
				array[i - first] = FitCurves.NewtonRaphsonRootFind(bezCurve, d[i], u[i - first]);
			}
			return array;
		}
		private static float NewtonRaphsonRootFind(Vector3[] Q, Vector3 P, float u)
		{
            Vector3[] array = new Vector3[3];
            Vector3[] array2 = new Vector3[2];
            Vector3 point = FitCurves.BezierII(3, Q, u);
			for (int i = 0; i <= 2; i++)
			{
				array[i].X = (Q[i + 1].X - Q[i].X) * 3.0f;
				array[i].Y = (Q[i + 1].Y - Q[i].Y) * 3.0f;
                array[i].Z = (Q[i + 1].Z - Q[i].Z) * 3.0f;
			}
			for (int i = 0; i <= 1; i++)
			{
				array2[i].X = (array[i + 1].X - array[i].X) * 2.0f;
				array2[i].Y = (array[i + 1].Y - array[i].Y) * 2.0f;
                array2[i].Z = (array[i + 1].Z - array[i].Z) * 2.0f;
			}
            Vector3 point2 = FitCurves.BezierII(2, array, u);
            Vector3 point3 = FitCurves.BezierII(1, array2, u);
            float num = (point.X - P.X) * point2.X + (point.Y - P.Y) * point2.Y + (point.Z - P.Z) * point2.Z;
			float num2 = point2.X * point2.X + point2.Y * point2.Y + point2.Z * point2.Z+ (point.X - P.X) * point3.X + (point.Y - P.Y) * point3.Y + (point.Z-P.Z)*point3.Z;
			if (num2 == 0.0f)
			{
				return u;
			}
			return u - num / num2;
		}
        private static Vector3 BezierII(int degree, Vector3[] V, float t)
		{
            Vector3[] array = new Vector3[degree + 1];
			for (int i = 0; i <= degree; i++)
			{
				array[i] = V[i];
			}
			for (int i = 1; i <= degree; i++)
			{
				for (int j = 0; j <= degree - i; j++)
				{
					array[j].X = (1.0f - t) * array[j].X + t * array[j + 1].X;
					array[j].Y = (1.0f - t) * array[j].Y + t * array[j + 1].Y;
                    array[j].Z = (1.0f - t) * array[j].Z + t * array[j + 1].Z;
				}
			}
			return array[0];
		}
		private static float B0(float u)
		{
			float num = 1.0f - u;
			return num * num * num;
		}
		private static float B1(float u)
		{
			float num = 1.0f - u;
			return 3.0f * u * (num * num);
		}
		private static float B2(float u)
		{
			float num = 1.0f - u;
			return 3.0f * u * u * num;
		}
		private static float B3(float u)
		{
			return u * u * u;
		}
        private static Vector3 ComputeLeftTangent(Vector3[] d, int end)
		{
            Vector3 result = d[end + 1] - d[end];
			result.Normalize();
			return result;
		}
        private static Vector3 ComputeRightTangent(Vector3[] d, int end)
		{
            Vector3 result = d[end - 1] - d[end];
			result.Normalize();
			return result;
		}
        private static Vector3 ComputeCenterTangent(Vector3[] d, int center)
		{
            Vector3 result = default(Vector3);
            Vector3 vector = d[center - 1] - d[center];
            Vector3 vector2 = d[center] - d[center + 1];
			result.X = (vector.X + vector2.X) / 2.0f;
			result.Y = (vector.Y + vector2.Y) / 2.0f;
            result.Z = (vector.Z + vector2.Z) / 2.0f;
			result.Normalize();
			return result;
		}
        private static float[] ChordLengthParameterize(Vector3[] d, int first, int last)
		{
			float[] array = new float[last - first + 1];
            float[] array_unit = new float[last - first + 1];
			array[0] = 0.0f;
            array_unit[0] = 0.0f;
			for (int i = first + 1; i <= last; i++)
			{
				array[i - first] = array[i - first - 1] + (d[i - 1] - d[i]).Length();
                array_unit[i - first] = (d[i - 1] - d[i]).Length();
			}
			for (int i = first + 1; i <= last; i++)
			{
				array[i - first] = array[i - first] / array[last - first];
			}
			return array;
		}
        private static float ComputeMaxError(Vector3[] d, int first, int last, Vector3[] bezCurve, float[] u, out int splitPoint)
		{
			splitPoint = (last - first + 1) / 2;
			float num = 0.0f;
			for (int i = first + 1; i < last; i++)
			{
                Vector3 point = FitCurves.BezierII(3, bezCurve, u[i - first]);
				float lengthSquared = (point - d[i]).LengthSquared();
				if (lengthSquared >= num)
				{
					num = lengthSquared;
					splitPoint = i;
				}
			}
			return num;
		}
	}
}
