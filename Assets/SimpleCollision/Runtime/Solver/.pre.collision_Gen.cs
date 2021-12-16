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

	// 場合分けでの気合いで、長方形との衝突判定
	//# foreach (var sn0 in new [] {
	//#		"Point",
	//#		"Line",
	//#		"Sphere",
	//#		"Capsule",
	//#		"Edge",
	//#	}) {
	//# 	var sn1 = "Quad";
	static public ColResult collision(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		//#	if (sn0=="Line") {
		//#		// Lineの場合は、垂線方向の面のみと衝突判定
		var ret = collision(a, b.getFace( sign(
			dot( removeDir(a.pos-b.pos, a.dir), b.rot.c2 )
		) ));
		//#	} else {
		//#		// その他の場合は、中心方向の面のみと衝突判定
		var ret = collision(a, b.getFace( sign(
			dot( a.pos-b.pos, b.rot.c2 )
		) ));
		//#	}
		if (ret.isHit) {
			var a2 = a;
			var t = ret.normal * (ret.depth + SEPARATION_ADD_EPSILON);
			a2.pos += t;
			
			var ret2 = collision_CoreProc_EdgeVert(a2, b);
			if ( ret2.isHit ) {
				t += ret2.normal * (ret2.depth + SEPARATION_ADD_EPSILON);
				var tLen = length(t);
				ret.normal = t / max(tLen, 0.00001f);
				ret.depth = tLen;
				ret.pos = ret2.pos;
			}

			return ret;
		}

		// 辺と頂点との衝突判定
		return collision_CoreProc_EdgeVert(a, b);
	}
	static ColResult collision_CoreProc_EdgeVert(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		//#	if (sn0=="Point" || sn0=="Line") {
		//#		// 幅が無い要素同士の衝突の場合は、この判定は不要
		if (b.w < 0.000001f) return default;
		//#	}

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		//#	if (sn0=="Point" || sn0=="Sphere") {
		//#		// Point,Sphereの場合は、中心方向の辺のみと衝突判定
		b.getEdges(a.pos-b.pos, out var e0, out var e1);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
		//#	} else if (sn0=="Line") {
		//#		// Lineの場合は、垂線方向の辺のみと衝突判定
		b.getEdges(removeDir(a.pos-b.pos, a.dir), out var e0, out var e1);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
		//#	} else {
		//#		// その他の場合は、全ての辺と衝突判定
		b.getEdges(out var e0, out var e1, out var e2, out var e3);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var retE2 = collision(a, e2);
		var retE3 = collision(a, e3);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
		if ( retE2.isHit && (!ret.isHit || ret.depth<retE2.depth) ) ret = retE2;
		if ( retE3.isHit && (!ret.isHit || ret.depth<retE3.depth) ) ret = retE3;
		//#	}
		if (ret.isHit) {
			var a2 = a;
			var t = ret.normal * (ret.depth + SEPARATION_ADD_EPSILON);
			a2.pos += t;
			
			var ret2 = collision_CoreProc_Vert(a2, b);
			if ( ret2.isHit ) {
				t += ret2.normal * (ret2.depth + SEPARATION_ADD_EPSILON);
				var tLen = length(t);
				ret.normal = t / max(tLen, 0.00001f);
				ret.depth = tLen;
				ret.pos = ret2.pos;
			}

			return ret;
		}

		// 頂点との衝突判定
		return collision_CoreProc_Vert(a, b);
	}
	static ColResult collision_CoreProc_Vert(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		// 頂点との衝突判定
		//#	if (sn0=="Point" || sn0=="Sphere") {
		//#		// Point,Sphereの場合は、中心方向の頂点のみと衝突判定
		return collision(a, b.getVert(a.pos-b.pos) );
		//#	} else if (sn0=="Line") {
		//#		// Lineの場合は、垂線方向の頂点のみと衝突判定
		return collision(a, b.getVert(removeDir(a.pos-b.pos, a.dir)) );
		//#	} else {
		//#		// その他の場合は、全ての頂点と衝突判定
		b.getVerts(out var v0, out var v1, out var v2, out var v3);
		var retV0 = collision(a, v0);
		var retV1 = collision(a, v1);
		var retV2 = collision(a, v2);
		var retV3 = collision(a, v3);
		var retV = retV0;
		if ( retV1.isHit && (!retV.isHit || retV.depth<retV1.depth) ) retV = retV1;
		if ( retV2.isHit && (!retV.isHit || retV.depth<retV2.depth) ) retV = retV2;
		if ( retV3.isHit && (!retV.isHit || retV.depth<retV3.depth) ) retV = retV3;
		return retV;
		//#	}
	}
	//# }



	// シンプルな衝突判定
	//# foreach (var i in new [] {
	//# 	("Point",		"Point",		"None"),
	//# 	("Point",		"Line",			"None"),
	//# 	("Point",		"Triangle",		"MPR"),
	//# 	("Point",		"Box",			"MPR"),
	//# 	("Point",		"Circle",		"MPR"),
	//# 	("Point",		"Cylinder",		"MPR"),
	//# 	("Point",		"Cone",			"MPR"),
	//# 	("Point",		"ConvexVerts",	"MPR"),
	//# 	("Line",		"Line",			"None"),
	//# 	("Line",		"HalfSpace",	"None"),
	//# 	("HalfSpace",	"HalfSpace",	"None"),
	//# 	("HalfSpace",	"Capsule",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Edge",			"None"),
	//# 	("HalfSpace",	"FaceQuad",		"None"),
	//# 	("HalfSpace",	"Quad",			"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Triangle",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Box",			"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Circle",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Cylinder",		"HalfSpace_Simple"),
	//# 	("HalfSpace",	"Cone",			"HalfSpace_Simple"),
	//# 	("HalfSpace",	"ConvexVerts",	"HalfSpace_Simple"),
	//# 	("Sphere",		"Triangle",		"MPR"),
	//# 	("Sphere",		"Box",			"MPR"),
	//# 	("Sphere",		"Circle",		"MPR"),
	//# 	("Sphere",		"Cylinder",		"MPR"),
	//# 	("Sphere",		"Cone",			"MPR"),
	//# 	("Sphere",		"ConvexVerts",	"MPR"),
	//# 	("Capsule",		"Triangle",		"MPR"),
	//# 	("Capsule",		"Box",			"MPR"),
	//# 	("Capsule",		"Circle",		"MPR"),
	//# 	("Capsule",		"Cylinder",		"MPR"),
	//# 	("Capsule",		"Cone",			"MPR"),
	//# 	("Capsule",		"ConvexVerts",	"MPR"),
	//# 	("Edge",		"Triangle",		"None"),
	//# 	("Edge",		"Box",			"None"),
	//# 	("Edge",		"Circle",		"None"),
	//# 	("Edge",		"Cylinder",		"None"),
	//# 	("Edge",		"Cone",			"None"),
	//# 	("Edge",		"ConvexVerts",	"None"),
	//# 	("FaceQuad",	"FaceQuad",		"None"),
	//# 	("FaceQuad",	"Quad",			"None"),
	//# 	("FaceQuad",	"Triangle",		"None"),
	//# 	("FaceQuad",	"Box",			"None"),
	//# 	("FaceQuad",	"Circle",		"None"),
	//# 	("FaceQuad",	"Cylinder",		"None"),
	//# 	("FaceQuad",	"Cone",			"None"),
	//# 	("FaceQuad",	"ConvexVerts",	"None"),
	//# 	("Quad",		"Quad",			"MPR"),
	//# 	("Quad",		"Triangle",		"MPR"),
	//# 	("Quad",		"Box",			"MPR"),
	//# 	("Quad",		"Circle",		"MPR"),
	//# 	("Quad",		"Cylinder",		"MPR"),
	//# 	("Quad",		"Cone",			"MPR"),
	//# 	("Quad",		"ConvexVerts",	"MPR"),
	//# 	("Triangle",	"Triangle",		"MPR"),
	//# 	("Triangle",	"Box",			"MPR"),
	//# 	("Triangle",	"Circle",		"MPR"),
	//# 	("Triangle",	"Cylinder",		"MPR"),
	//# 	("Triangle",	"Cone",			"MPR"),
	//# 	("Triangle",	"ConvexVerts",	"MPR"),
	//# 	("Box",			"Box",			"MPR"),
	//# 	("Box",			"Circle",		"MPR"),
	//# 	("Box",			"Cylinder",		"MPR"),
	//# 	("Box",			"Cone",			"MPR"),
	//# 	("Box",			"ConvexVerts",	"MPR"),
	//# 	("Circle",		"Circle",		"MPR"),
	//# 	("Circle",		"Cylinder",		"MPR"),
	//# 	("Circle",		"Cone",			"MPR"),
	//# 	("Circle",		"ConvexVerts",	"MPR"),
	//# 	("Cylinder",	"Cylinder",		"MPR"),
	//# 	("Cylinder",	"Cone",			"MPR"),
	//# 	("Cylinder",	"ConvexVerts",	"MPR"),
	//# 	("Cone",		"Cone",			"MPR"),
	//# 	("Cone",		"ConvexVerts",	"MPR"),
	//# 	("ConvexVerts",	"ConvexVerts",	"MPR"),
	//# }) {
	//# 	var sn0 = i.Item1;
	//# 	var sn1 = i.Item2;
	//# 	var mode = i.Item3;
	//# 	if (mode == "None") {
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) => default;
	//# 	} else if (mode == "MPR") {
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	//# 	} else if (mode == "HalfSpace_Simple") {
	static public ColResult collision(【getArgType(sn0)】 a, 【getArgType(sn1)】 b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	//# 	}
	//# }




	// 衝突判定の逆向き引数バージョン
	//# for (int i=0; i<allShapeNames.Length; ++i) for (int j=i+1; j<allShapeNames.Length; ++j) {
	//# 	var sn0 = allShapeNames[i];
	//# 	var sn1 = allShapeNames[j];
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(【getArgType(sn1)】 a, 【getArgType(sn0)】 b) => -collision(b,a);
	//# }
}
}

#endif	//#Ignore

//# } }
