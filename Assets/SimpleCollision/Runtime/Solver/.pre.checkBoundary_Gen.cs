//#CsPP
//#	public partial class Program {
//#
//#		static string[] allShapeNames = new [] {
//#			"Point",
//#			"Line",
//#			"HalfSpace",
//#			"Sphere",
//#			"Capsule",
//#			"Edge",
//#			"FaceQuad",
//#			"Quad",
//#			"Triangle",
//#			"Box",
//#			"Circle",
//#			"Cylinder",
//#			"Cone",
//#			"ConvexVerts",
//#		};
//#		static string getArgType(string shapeName) {
//#			if (
//#				shapeName == "Point" ||
//#				shapeName == "Line" ||
//#				shapeName == "HalfSpace" ||
//#				shapeName == "Sphere"
//#			) return shapeName;
//#			return "in " + shapeName;
//#		}
//#		static string type2VarName(string a) => char.ToUpper(a[0]) + a.Substring(1);
//#
//#		public static void Main() {

#if false	//#Ignore  IDE状でエラー表示させ無くするために無効にしておく

using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {
static public partial class Solver {

	// シンプルなバウンダリー判定
	//# foreach (var i in new [] {
	//# 	("Point",		"Point",		"False"),
	//# 	("Point",		"Line",			"False"),
	//# 	("Point",		"HalfSpace",	"True"),
	//# 	("Point",		"Sphere",		"True"),
	//# 	("Point",		"Capsule",		"Point_Simple"),
	//# 	("Point",		"Edge",			"Point_Simple"),
	//# 	("Point",		"FaceQuad",		"Point_Simple"),
	//# 	("Point",		"Quad",			"Point_Simple"),
	//# //	("Point",		"Triangle",		"Simple"),
	//# 	("Point",		"Box",			"Point_Simple"),
	//# 	("Point",		"Circle",		"Point_Simple"),
	//# 	("Point",		"Cylinder",		"Point_Simple"),
	//# 	("Point",		"Cone",			"Point_Simple"),
	//# 	("Point",		"ConvexVerts",	"Point_Simple"),
	//# 	("Line",		"Line",			"False"),
	//# 	("Line",		"HalfSpace",	"False"),
	//# 	("Line",		"Sphere",		"Line_Simple"),
	//# 	("Line",		"Capsule",		"Line_Simple"),
	//# 	("Line",		"Edge",			"Line_Simple"),
	//# 	("Line",		"FaceQuad",		"Line_Simple"),
	//# 	("Line",		"Quad",			"Line_Simple"),
	//# //	("Line",		"Triangle",		"Simple"),
	//# 	("Line",		"Box",			"Line_Simple"),
	//# 	("Line",		"Circle",		"Line_Simple"),
	//# 	("Line",		"Cylinder",		"Line_Simple"),
	//# 	("Line",		"Cone",			"Line_Simple"),
	//# 	("Line",		"ConvexVerts",	"Line_Simple"),
	//# 	("HalfSpace",	"HalfSpace",	"False"),
	//# 	("HalfSpace",	"Sphere",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Capsule",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Edge",			"False"),
	//# 	("HalfSpace",	"FaceQuad",		"False"),
	//# 	("HalfSpace",	"Quad",			"HalfSpace_Simple"),
	//# //	("HalfSpace",	"Triangle",		"Simple"),
	//# 	("HalfSpace",	"Box",			"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Circle",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Cylinder",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Cone",			"HalfSpace_Simple"),
	//# 	("HalfSpace",	"ConvexVerts",	"HalfSpace_Simple"),
	//# 	("Sphere",		"Sphere",		"Simple"),
	//# 	("Sphere",		"Capsule",		"Simple"),
	//# 	("Sphere",		"Edge",			"Simple"),
	//# 	("Sphere",		"FaceQuad",		"Simple"),
	//# 	("Sphere",		"Quad",			"Simple"),
	//# //	("Sphere",		"Triangle",		"Simple"),
	//# 	("Sphere",		"Box",			"Simple"),
	//# 	("Sphere",		"Circle",		"Simple"),
	//# 	("Sphere",		"Cylinder",		"Simple"),
	//# 	("Sphere",		"Cone",			"Simple"),
	//# 	("Sphere",		"ConvexVerts",	"Simple"),
	//# 	("Capsule",		"Capsule",		"Simple"),
	//# 	("Capsule",		"Edge",			"Simple"),
	//# 	("Capsule",		"FaceQuad",		"Simple"),
	//# 	("Capsule",		"Quad",			"Simple"),
	//# 	("Capsule",		"Triangle",		"Triangle_SimpleB"),
	//# 	("Capsule",		"Box",			"Simple"),
	//# 	("Capsule",		"Circle",		"Simple"),
	//# 	("Capsule",		"Cylinder",		"Simple"),
	//# 	("Capsule",		"Cone",			"Simple"),
	//# 	("Capsule",		"ConvexVerts",	"Simple"),
	//# 	("Edge",		"Edge",			"Simple"),
	//# 	("Edge",		"FaceQuad",		"Simple"),
	//# 	("Edge",		"Quad",			"Simple"),
	//# 	("Edge",		"Triangle",		"Triangle_SimpleB"),
	//# 	("Edge",		"Box",			"Simple"),
	//# 	("Edge",		"Circle",		"Simple"),
	//# 	("Edge",		"Cylinder",		"Simple"),
	//# 	("Edge",		"Cone",			"Simple"),
	//# 	("Edge",		"ConvexVerts",	"Simple"),
	//# 	("FaceQuad",	"FaceQuad",		"Simple"),
	//# 	("FaceQuad",	"Quad",			"Simple"),
	//# 	("FaceQuad",	"Triangle",		"Triangle_SimpleB"),
	//# 	("FaceQuad",	"Box",			"Simple"),
	//# 	("FaceQuad",	"Circle",		"Simple"),
	//# 	("FaceQuad",	"Cylinder",		"Simple"),
	//# 	("FaceQuad",	"Cone",			"Simple"),
	//# 	("FaceQuad",	"ConvexVerts",	"Simple"),
	//# 	("Quad",		"Quad",			"Simple"),
	//# 	("Quad",		"Triangle",		"Triangle_SimpleB"),
	//# 	("Quad",		"Box",			"Simple"),
	//# 	("Quad",		"Circle",		"Simple"),
	//# 	("Quad",		"Cylinder",		"Simple"),
	//# 	("Quad",		"Cone",			"Simple"),
	//# 	("Quad",		"ConvexVerts",	"Simple"),
	//# //	("Triangle",	"Triangle",		"Simple"),
	//# 	("Triangle",	"Box",			"Triangle_SimpleA"),
	//# 	("Triangle",	"Circle",		"Triangle_SimpleA"),
	//# 	("Triangle",	"Cylinder",		"Triangle_SimpleA"),
	//# 	("Triangle",	"Cone",			"Triangle_SimpleA"),
	//# 	("Triangle",	"ConvexVerts",	"Triangle_SimpleA"),
	//# 	("Box",			"Box",			"Simple"),
	//# 	("Box",			"Circle",		"Simple"),
	//# 	("Box",			"Cylinder",		"Simple"),
	//# 	("Box",			"Cone",			"Simple"),
	//# 	("Box",			"ConvexVerts",	"Simple"),
	//# 	("Circle",		"Circle",		"Simple"),
	//# 	("Circle",		"Cylinder",		"Simple"),
	//# 	("Circle",		"Cone",			"Simple"),
	//# 	("Circle",		"ConvexVerts",	"Simple"),
	//# 	("Cylinder",	"Cylinder",		"Simple"),
	//# 	("Cylinder",	"Cone",			"Simple"),
	//# 	("Cylinder",	"ConvexVerts",	"Simple"),
	//# 	("Cone",		"Cone",			"Simple"),
	//# 	("Cone",		"ConvexVerts",	"Simple"),
	//# 	("ConvexVerts",	"ConvexVerts",	"Simple"),
	//# }) {
	//# 	var sn0 = i.Item1;
	//# 	var sn1 = i.Item2;
	//# 	var mode = i.Item3;
	//# 	if (mode == "True" || mode == "False") {
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) => 【(mode=="True"?"true":"false")】;
	//# 	} else if (mode == "Simple") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	//# 	} else if (mode == "Point_Simple") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	//# 	} else if (mode == "Line_Simple") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	//# 	} else if (mode == "HalfSpace_Simple") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	//# 	} else if (mode == "Triangle_SimpleA") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	//# 	} else if (mode == "Triangle_SimpleB") {
	static public bool checkBoundary(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		var r = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(r), a.boundarySqR(), lengthsq(a.pos-b.pos) );
	}
	//# 	}
	//# }



	// バウンダリー判定の逆向き引数バージョン
	//# for (int i=0; i<allShapeNames.Length; ++i) for (int j=i+1; j<allShapeNames.Length; ++j) {
	//# 	var sn0 = allShapeNames[i];
	//# 	var sn1 = allShapeNames[j];
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(【getArgType(sn1)】 a, 【getArgType(sn0)】 b) => checkBoundary(b,a);
	//# }
}
}

#endif	//#Ignore

//# } }
