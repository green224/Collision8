

using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {
static public partial class Solver {

	// 場合分けでの気合いで、長方形との衝突判定
	static public ColResult collision(Point a, in Quad b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		var ret = collision(a, b.getFace( sign(
			dot( a.pos-b.pos, b.rot.c2 )
		) ));
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
	static ColResult collision_CoreProc_EdgeVert(Point a, in Quad b) {
		if (b.w < 0.000001f) return default;

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		b.getEdges(a.pos-b.pos, out var e0, out var e1);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
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
	static ColResult collision_CoreProc_Vert(Point a, in Quad b) {
		// 頂点との衝突判定
		return collision(a, b.getVert(a.pos-b.pos) );
	}
	static public ColResult collision(Line a, in Quad b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		var ret = collision(a, b.getFace( sign(
			dot( removeDir(a.pos-b.pos, a.dir), b.rot.c2 )
		) ));
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
	static ColResult collision_CoreProc_EdgeVert(Line a, in Quad b) {
		if (b.w < 0.000001f) return default;

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		b.getEdges(removeDir(a.pos-b.pos, a.dir), out var e0, out var e1);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
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
	static ColResult collision_CoreProc_Vert(Line a, in Quad b) {
		// 頂点との衝突判定
		return collision(a, b.getVert(removeDir(a.pos-b.pos, a.dir)) );
	}
	static public ColResult collision(Sphere a, in Quad b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		var ret = collision(a, b.getFace( sign(
			dot( a.pos-b.pos, b.rot.c2 )
		) ));
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
	static ColResult collision_CoreProc_EdgeVert(Sphere a, in Quad b) {

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		b.getEdges(a.pos-b.pos, out var e0, out var e1);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
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
	static ColResult collision_CoreProc_Vert(Sphere a, in Quad b) {
		// 頂点との衝突判定
		return collision(a, b.getVert(a.pos-b.pos) );
	}
	static public ColResult collision(in Capsule a, in Quad b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		var ret = collision(a, b.getFace( sign(
			dot( a.pos-b.pos, b.rot.c2 )
		) ));
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
	static ColResult collision_CoreProc_EdgeVert(in Capsule a, in Quad b) {

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		b.getEdges(out var e0, out var e1, out var e2, out var e3);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var retE2 = collision(a, e2);
		var retE3 = collision(a, e3);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
		if ( retE2.isHit && (!ret.isHit || ret.depth<retE2.depth) ) ret = retE2;
		if ( retE3.isHit && (!ret.isHit || ret.depth<retE3.depth) ) ret = retE3;
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
	static ColResult collision_CoreProc_Vert(in Capsule a, in Quad b) {
		// 頂点との衝突判定
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
	}
	static public ColResult collision(in Edge a, in Quad b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;
		
		// 面との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		var ret = collision(a, b.getFace( sign(
			dot( a.pos-b.pos, b.rot.c2 )
		) ));
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
	static ColResult collision_CoreProc_EdgeVert(in Edge a, in Quad b) {

		// 辺との衝突判定。ここで衝突している場合は、aを移動してから再度衝突計算を行う
		b.getEdges(out var e0, out var e1, out var e2, out var e3);
		var retE0 = collision(a, e0);
		var retE1 = collision(a, e1);
		var retE2 = collision(a, e2);
		var retE3 = collision(a, e3);
		var ret = retE0;
		if ( retE1.isHit && (!ret.isHit || ret.depth<retE1.depth) ) ret = retE1;
		if ( retE2.isHit && (!ret.isHit || ret.depth<retE2.depth) ) ret = retE2;
		if ( retE3.isHit && (!ret.isHit || ret.depth<retE3.depth) ) ret = retE3;
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
	static ColResult collision_CoreProc_Vert(in Edge a, in Quad b) {
		// 頂点との衝突判定
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
	}



	// シンプルな衝突判定
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, Point b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, Line b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in Triangle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Point a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Line a, Line b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Line a, HalfSpace b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(HalfSpace a, HalfSpace b) => default;
	static public ColResult collision(HalfSpace a, in Capsule b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(HalfSpace a, in Edge b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(HalfSpace a, in FaceQuad b) => default;
	static public ColResult collision(HalfSpace a, in Quad b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in Triangle b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in Box b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in Circle b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in Cylinder b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in Cone b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	static public ColResult collision(HalfSpace a, in ConvexVerts b) {
		var p = b.supportFunc(-a.dir) + b.pos;
		var depth = dot( a.pos - p, a.dir );
		if (depth < 0) return default;
		return new ColResult( p, -a.dir, depth );
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in Triangle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in Triangle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Triangle b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Box b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Circle b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Cylinder b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Cone b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in ConvexVerts b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in FaceQuad b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Quad b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Triangle b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Box b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Circle b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Cylinder b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Cone b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in ConvexVerts b) => default;
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Quad b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Triangle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Triangle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Box b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Circle b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Cylinder b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Cone b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in ConvexVerts b) {
		if ( !checkBoundary(a, b) ) return default;
		return MPR.solve(a,b);
	}




	// 衝突判定の逆向き引数バージョン
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Line a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(HalfSpace a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, Point b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(HalfSpace a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, Line b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Sphere a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, HalfSpace b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Capsule a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, Sphere b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Edge a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Capsule b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in FaceQuad a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Edge b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Quad a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in FaceQuad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Triangle a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Quad b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Box a, in Triangle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Triangle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Triangle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Triangle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Triangle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Circle a, in Box b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Box b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Box b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Box b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cylinder a, in Circle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Circle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Circle b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in Cone a, in Cylinder b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Cylinder b) => -collision(b,a);
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(in ConvexVerts a, in Cone b) => -collision(b,a);
}
}


