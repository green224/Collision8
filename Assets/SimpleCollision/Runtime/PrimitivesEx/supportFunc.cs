using Unity.Mathematics;
using static Unity.Mathematics.math;

//using System.Runtime.CompilerServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
public static partial class PrimitivesEx {

	// プリミティブごとのサポート写像の得る関数群。

	/** 点のサポート写像 */
	[MI(MO.AggressiveInlining)]
	public static float3 supportFunc(this Point p, float3 dir) => 0;

	/** 球のサポート写像 */
	[MI(MO.AggressiveInlining)]
	public static float3 supportFunc(this Sphere s, float3 dir) => normalize(dir) * s.r;

	/** カプセルのサポート写像 */
	public static float3 supportFunc(this in Capsule c, float3 dir) {
		var dirSign = sign(dot(dir, c.dir));
		return
			c.dir * (dirSign * c.r_h)
			+ normalize(dir) * c.capSphere(dirSign).r;
	}

	/** 長方形のサポート写像 */
	public static float3 supportFunc(this in Quad q, float3 dir) {
		var x = dot(q.rot.c0, dir);
		var y = dot(q.rot.c1, dir);
		var ret =
			(sign(x) * q.r.x) * q.rot.c0 +
			(sign(y) * q.r.y) * q.rot.c1;
		if ( 0.000001f < q.w ) ret += normalize(dir) * q.w;
		return ret;
	}

	/** 三角形のサポート写像 */
	public static float3 supportFunc(this in Triangle t, float3 dir) {
		var p0 = mul(t.rot, t.vert0);
		var p1 = mul(t.rot, t.vert1);
		var p2 = mul(t.rot, t.vert2);
		var a0 = dot(p0, dir);
		var a1 = dot(p1, dir);
		var a2 = dot(p2, dir);

		float3 ret;
		if (a0 < a1)	ret = a1 < a2 ? p2 : p1;
		else			ret = a0 < a2 ? p2 : p0;
		if ( 0.000001f < t.w ) ret += normalize(dir) * t.w;
		return ret;
	}

	/** 直方体のサポート写像 */
	public static float3 supportFunc(this in Box b, float3 dir) {
		var x = dot(b.rot.c0, dir);
		var y = dot(b.rot.c1, dir);
		var z = dot(b.rot.c2, dir);
		var ret =
			(sign(x) * b.r.x) * b.rot.c0 +
			(sign(y) * b.r.y) * b.rot.c1 +
			(sign(z) * b.r.z) * b.rot.c2;
		if ( 0.000001f < b.w ) ret += normalize(dir) * b.w;
		return ret;
	}

	/** 円盤のサポート写像 */
	public static float3 supportFunc(this in Circle c, float3 dir) {
		var dirDot = dot(dir, c.dir);
		var sideNml = normalizesafe(dir - dirDot*c.dir);

		var ret = sideNml * c.r;
		if ( 0.000001f < c.w ) ret += normalize(dir) * c.w;
		return ret;
	}

	/** 円柱のサポート写像 */
	public static float3 supportFunc(this in Cylinder c, float3 dir) {
		var dirDot = dot(dir, c.dir);
		var sideNml = normalizesafe(dir - dirDot*c.dir);

		var ret = c.dir * (sign(dirDot) * c.r_h) + sideNml * c.r_s;
		if ( 0.000001f < c.w ) ret += normalize(dir) * c.w;
		return ret;
	}

	/** 円錐のサポート写像 */
	public static float3 supportFunc(this in Cone c, float3 dir) {
		var sideNml = normalizesafe(dir - dot(dir, c.dir)*c.dir);

		var topP = c.dir * c.r_h;				// 上部の頂点
		var sideP = sideNml * c.r_s - topP;		// dir方向に最も遠い底面の端

		var ret = dot(topP,dir) < dot(sideP,dir) ? sideP : topP;
		if ( 0.000001f < c.w ) ret += normalize(dir) * c.w;
		return ret;
	}

	/** 凸型多面体のサポート写像 */
	public static float3 supportFunc(this in ConvexVerts c, float3 dir) {
//		var invRot = inverse(c.rot);
		var invRot = transpose(c.rot);		//c.rotは正規直行上列なので転置でOK
		var ldir = mul(invRot, dir);

		float maxD = 0;
		float3 maxP = default;
		unsafe {
			var vp = (float3*)c.vertsPtr;
			for (int i=0; i<c.vertsCnt; ++i) {
				var p = vp[i];
				var d = dot(ldir, p);
				if (maxD < d) { maxD = d; maxP = p; }
			}
		}

		var ret = mul(c.rot, maxP);
		if ( 0.000001f < c.w ) ret += normalize(dir) * c.w;
		return ret;
	}

}
}

