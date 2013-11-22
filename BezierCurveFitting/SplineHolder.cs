using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpDX;

namespace BezierCurveFitting
{
    public struct SplineHolder
    {
        public List<Vector3[]> pointsData;
        public float errors;
    }
}
