plugin modifier ESplineExt
name:"Edit Spline Ext"
classID:#(0x133067, 0x54374)
extends:Edit_spline
replaceUI:true version:1
(
	local res=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\Scripts\BezierCurveFitting.dll"
	local shp=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\SharpDX.dll"
	local genericType = dotnet.GetType "System.Collections.Generic.List`1"
	local innerType = dotnet.GetType "SharpDX.Vector3" 
	
	local fitCurve = function fitCurve _method _precision _numberPoints =
	(
		curveFit = dotNetClass "BezierCurveFitting.Bezier"
		methodFlag= dotNetClass "BezierCurveFitting.Method"
		specificType = genericType.MakeGenericType #(innerType)
		
		for i in selection do
		(
			sp = splineShape()
			nbSplines= numSplines i
			
			for j=1 to nbSplines do
			(
				nbSegments = numSegments i j
				points = (dotnetclass "System.Activator").CreateInstance specificType
				addnewSpline sp
				newLast = getKnotPoint i j 1
				if isClosed i j do
				(
					open i j
					nLI = getInVec i j 1
					nLO= getOutVec i j 1
					addKnot i j #bezierCorner #Line newLast nLI nLO
				)
				nbVertices=numKnots i j
				for k=1 to k=nbVertices do
				(
					Knot = getKnotPoint i j k
					points.Add (dotnetObject "SharpDX.Vector3" Knot.X Knot.Y Knot.Z)
					if (k<nbVertices AND (getSegmentType i j k) == #curve) do
					(
						nbSteps = i.steps
						for step=1 to nbSteps-1 do
						(
							par = (step/(nbSteps as float))
							Knot = interpBezier3D i j k par pathParam:false
							points.Add (dotnetObject "SharpDX.Vector3" Knot.X Knot.Y Knot.Z)
						)
					)
				)
				method = undefined
				if _method == 1 then
				(
					method = methodFlag.Precision
				)
				else if _method==2 then
				(
					method = methodFlag.Length
				)
				else
				(
					method = methodFlag.Path
				)
				truc=curveFit.CurveSolution method points _precision _numberPoints
				deleteSpline i j
				addnewSpline i
				arr = dotNetObject "BezierCurveFitting.BezierPoint[]" truc.Count
				truc.CopyTo arr 0
				for l = 0 to arr.Length-1 do 
				(
					m=arr.getValue l
					BPoint = point3 (m.point.X as float) (m.point.Y as float) (m.point.Z as float)
					FPoint = BPoint
					APoint = BPoint
					if m.foreHandle != undefined do
					(
						FPoint = point3 (m.foreHandle.X as float) (m.foreHandle.Y as float) (m.foreHandle.Z as float)
					)
					if m.afterHandle != undefined do
					(
						APoint = point3 (m.afterHandle.X as float) (m.afterHandle.Y as float) (m.afterHandle.Z as float)
					)
					addknot i j #bezierCorner #curve BPoint FPoint APoint
				)
				updateShape i
			)
		)
	)
	
  parameters main rollout:params
  (
    NbPoints type:#integer animatable:true ui:_nbPoints default:2
	on NbPoints Set NbPointsVal do recurve NbPointsVal
  )
  rollout params "Parameters"
  (
    spinner _nbPoints "points: " type:#integer range:[2,100,2]
  )
)