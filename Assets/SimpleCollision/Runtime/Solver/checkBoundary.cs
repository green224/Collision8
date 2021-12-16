using Unity.Mathematics;
using static Unity.Mathematics.math;

//using System.Runtime.CompilerServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
static public partial class Solver {


	/** 点 vs 三角形 のバウンダリー判定 */
	static public bool checkBoundary(Point p, in Triangle t) {
		var d = t.pos - p.pos;
		var rSq = lengthsq(t.vert0);
		rSq = max( rSq, lengthsq(t.vert1) );
		rSq = max( rSq, lengthsq(t.vert2) );
		return checkBoundaryCore( rSq, t.w*t.w, lengthsq(d));
	}

	/** 直線 vs 三角形 のバウンダリー判定 */
	static public bool checkBoundary(Line l, in Triangle t) {
		var d = removeDir(l.pos - t.pos, l.dir);
		var rSq = lengthsq(t.vert0);
		rSq = max( rSq, lengthsq(t.vert1) );
		rSq = max( rSq, lengthsq(t.vert2) );
		return checkBoundaryCore( rSq, t.w*t.w, lengthsq(d));
	}

	/** 半空間 vs 三角形 のバウンダリー判定 */
	static public bool checkBoundary(HalfSpace hs, in Triangle t) {
		var l = dot(hs.pos - t.pos, hs.dir);
		var rSq = lengthsq(t.vert0);
		rSq = max( rSq, lengthsq(t.vert1) );
		rSq = max( rSq, lengthsq(t.vert2) );
		return checkBoundaryCore( rSq, t.w*t.w, l*l);
	}

	/** 球 vs 三角形 のバウンダリー判定 */
	static public bool checkBoundary(Sphere s, in Triangle t) {
		var d = s.pos - t.pos;
		var rSq = lengthsq(t.vert0);
		rSq = max( rSq, lengthsq(t.vert1) );
		rSq = max( rSq, lengthsq(t.vert2) );
		return checkBoundaryCore( rSq, (t.w+s.r)*(t.w+s.r), lengthsq(d));
	}

	/** 三角形 vs 三角形 のバウンダリー判定 */
	static public bool checkBoundary(in Triangle a, in Triangle b) {
		var ra = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		var rb = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(ra), lengthsq(rb), lengthsq(a.pos-b.pos));
	}




}
}
