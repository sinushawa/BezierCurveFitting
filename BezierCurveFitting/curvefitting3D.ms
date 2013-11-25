--res=dotnet.loadAssembly @"L:\Temp\Jerome\maxscript prod\MaxBezier.dll"
res=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\Scripts\BezierCurveFitting.dll"
shp=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\SharpDX.dll"
if res != undefined do
(
curveFit = dotNetClass "BezierCurveFitting.Bezier"
methodFlag= dotNetClass "BezierCurveFitting.Method"
global truc
global arr
global tempcurves=#()

genericType = dotnet.GetType "System.Collections.Generic.List`1"
innerType = dotnet.GetType "SharpDX.Vector3" 
specificType = genericType.MakeGenericType #(innerType)

fn fitCurve _method _precision _numberPoints =
(
	
	if tempcurves.count>0 do
	(
		for j=1 to tempcurves.count do
		(
			try
			(
				delete tempcurves[j]
			)
			catch()
		)
	)
	tempcurves=#()
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
			print truc.Count
			arr = dotNetObject "BezierCurveFitting.BezierPoint[]" truc.Count
			truc.CopyTo arr 0
			for i = 0 to arr.Length-1 do 
			(
				m=arr.getValue i
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
				addknot sp j #bezierCorner #curve BPoint FPoint APoint
			)
			updateShape sp
			append tempcurves sp
		)
	)
)

rollout CurveFittingRollout "Create Bezier curve from selection" width:240 height:120
(
	label methodLblUI "method: " across:2 align:#left
	radioButtons methodUI labels:#("precision", "length", "path") default:2 align:#right
	spinner precisionUI "precision: " range:[0,1000,1] width:100  enabled:false  align:#left across:2
	spinner numberPointsUI "points: " type:#integer range:[0,500,1] width:100  align:#right
	button fitCurveUI "apply fit curve" offset: [0,16]
	
	on methodUI changed val do
	(
		if val==1 then
		(
			precisionUI.enabled=true
			numberPointsUI.enabled = false
		)
		else
		(
			precisionUI.enabled=false
			numberPointsUI.enabled = true
		)
	)
	
	on fitCurveUI pressed do
	(
		fitCurve methodUI.State precisionUI.Value numberPointsUI.Value
	)
)
CreateDialog CurveFittingRollout
)