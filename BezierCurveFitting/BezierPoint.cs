using System;
using System.Windows;
using SharpDX;

namespace BezierCurveFitting
{
	public class BezierPoint
	{
		public Vector3? foreHandle;
		public Vector3 point;
		public Vector3? afterHandle;
		public BezierPoint(Vector3 _point, Vector3? _foreHandle, Vector3? _afterHandle)
		{
			this.foreHandle = _foreHandle;
			this.point = _point;
			this.afterHandle = _afterHandle;
		}
	}
}
