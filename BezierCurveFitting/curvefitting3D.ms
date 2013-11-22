--res=dotnet.loadAssembly @"L:\Temp\Jerome\maxscript prod\MaxBezier.dll"
res=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\Scripts\BezierCurveFitting.dll"
shp=dotnet.loadAssembly @"C:\Program Files\Autodesk\3ds Max 2014\SharpDX.dll"
if res != undefined do
(
curveFit = dotNetClass "BezierCurveFitting.Bezier"
global truc
global arr
global tempcurves=#()

genericType = dotnet.GetType "System.Collections.Generic.List`1"
innerType = dotnet.GetType "SharpDX.Vector3" 
specificType = genericType.MakeGenericType #(innerType)

fn fitCurve Fitscale =
(
	
	if tempcurves.count>0 do
	(
		for j=1 to tempcurves.count do
		(
			delete tempcurves[j]
		)
	)
	tempcurves=#()
	for i in selection do
	(
		BBox=nodeLocalBoundingBox i
		w=abs(BBox[2].x-BBox[1].x)
		h=abs(BBox[2].y-BBox[1].y)
		/*
		Fitscale = w/1000
		if w>h do
		(
			Fitscale = h/1000
		)
		*/
		sp = splineShape()
		nbSplines= numSplines i
		
		for j=1 to nbSplines do
		(
			nbSegments = numSegments i j
			points = (dotnetclass "System.Activator").CreateInstance specificType
			addnewSpline sp
			newLast = getKnotPoint i j 1
			Z=newLast.Z
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
					for step=1 to nbSteps do
					(
						par = (step/(nbSteps as float))
						Knot = interpBezier3D i j k par pathParam:false
						points.Add (dotnetObject "SharpDX.Vector3" Knot.X Knot.Y Knot.Z)
					)
				)
			)
			
			truc=curveFit.CurveSolution points Fitscale Fitscale
			print truc
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

rollout CurveFittingRollout "Create Bezier curve from selection" width:400 height:120
(
	spinner SP "precision" range:[0,1,1]	slider FS "" ticks:10 range:[0,1,1]
	button FSCreate "apply fit curve" offset: [0,16]
	
	on FS changed val do
	(
		SP.value = val
	)
	on FS buttonup do
	(
		fitCurve FS.value
	)
	
	on FSCreate pressed do
	(
		if tempcurves.count<1 do
		(
			fitCurve FS.value
		)
		tempcurves=#()
	)
)
CreateDialog CurveFittingRollout
)