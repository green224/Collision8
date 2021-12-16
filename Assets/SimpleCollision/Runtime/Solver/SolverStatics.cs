using System;
using Unity.Mathematics;
using static Unity.Mathematics.math;

//using System.Runtime.CompilerServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
static public partial class Solver {


	/** 指定のベクトルsrcVecから、指定の方向dirの成分のみを除外する */
	[MI(MO.AggressiveInlining)]
	static float3 removeDir(float3 srcVec, float3 dir) => srcVec - dot(srcVec, dir) * dir;

	/** 指定のベクトルsrcVecを、指定の方向dirに投影したベクトルを返す */
	[MI(MO.AggressiveInlining)]
	static float3 projectDir(float3 srcVec, float3 dir) => dot(srcVec, dir) * dir;

	/**
	 * 二つの二乗距離を合体した結果が、指定の二乗距離より小さいか否かをチェックする。
	 * バウンダリー球判定を高速に行うために使用できる
	 *   distance <= r0 + r1
	 */
	static bool checkBoundaryCore(float r0Sq, float r1Sq, float distanceSq) {
		// d <= r0 + r1 は両辺とも正なので二乗できる。
		// 二乗すると  d^2 <= (r0 + r1)^2
		// 変形すると  d^2 - r0^2 - r1^2 <= 2r0r1
		// 右辺が必ず正なので、左辺が0以下の場合はこの式は必ず真になる。
		// 真の場合は更に二乗して (d^2 - r0^2 - r1^2)^2 <= 4 r0^2 r1^2
		// も真になる。したがってこれを条件とすればよい。
		var a = distanceSq - r0Sq - r1Sq;
		return a <= 0 || a*a <= 4*r0Sq*r1Sq;
	}
	
	/** 点 vs 直線 の最短距離 */
	[MI(MO.AggressiveInlining)]
	static public float distanceSq(Point p, Line l) {
		var d = p.pos - l.pos;
		// d から l.dir 方向の成分を除外したベクトルの長さ
		return lengthsq( d - dot(d, l.dir)*l.dir );
	}

	/**
	 * 直線vs直線の、最近傍点の位置mnpへのa.posからの距離を求める。方向はa.dir方向が正。
	 * 計算簡略化のため、a X b の結果を tDir に入れる。
	 */
	[MI(MO.AggressiveInlining)]
	static float getClosestPointDistA(
		float3 a,		// 直線Aの方向
		float3 b,		// 直線Bの方向
		float3 pA,		// 直線A上の任意の点
		float3 pB,		// 直線B上の任意の点
		float3 tDir		// 直線Aの方向
	) {
		// aをX軸としたときの、X軸に垂直なY軸をcとすると、
		var c = cross(a, tDir);
		// このときbをX軸、Y軸成分は、(a・b, c・b)となる。
		// y = ～～ 形式で直線bを表すと
		//   y = (c・b)/(a・b) * (x-x0) + y0
		//   x0 = (Pb-Pa)・a
		//   y0 = (Pb-Pa)・c
		// Pa,Pbは直線b上の任意の点。
		// 最近傍点をMbとすると、y=0により
		//   0 = (c・b)/(a・b) * (x-(Pb-Pa)・a) + (Pb-Pa)・c
		// xについて解くと
		//   x = (Pb-Pa)・{ a - c*(a・b)/(c・b) }
		return dot(
			pB-pA,
			a - c*( dot(a, b) / dot(c, b) )
		);
	}

	/**
	 * 直線 vs 直線 の最近傍点取得の内部処理。
	 * 最近傍点への距離と、直線間の距離を返す。
	 */
	static float getClosestPoint(
		Line a, Line b,
		out float mnpD_a,
		out float mnpD_b
	) {
		// 外積結果が最短距離の方向になる
		var t = cross(a.dir, b.dir);

		// 直線が互いに平行の場合は、特殊処理
		var tLenSq = lengthsq(t);
		if (tLenSq < 0.00000001f) {
			t = a.pos - b.pos;
			var tDb = dot(t, b.dir);

			// 最近傍点は片方を0にして割り当てる
			mnpD_a = 0;
			mnpD_b = tDb;
			t -= t * tDb*b.dir;
			return length(t);
		}

		// 直線 vs 直線の距離
		var tLen = sqrt(tLenSq);
		var tDir = t / tLen;
		var b2a = a.pos - b.pos;
		var llDist = abs(dot(tDir, b2a));

		// カプセル直線状の最近傍点の位置mnpへのb.posからの距離mnpD_bを求める。方向はb.dir方向が正
		mnpD_a = getClosestPointDistA(a.dir, b.dir, a.pos, b.pos, tDir);

		// 最近傍点の位置mnpへのa.posからの距離mnpD_aを求める
		var mnp = a.pos + mnpD_a*a.dir;
		mnpD_b = dot(mnp-b.pos, b.dir);

		return llDist;
	}

	/**
	 * 直線 vs 無限平面 の交点の取得処理。
	 * 直線中央から交点への距離と、交点があるか否かを返す。
	 */
	static bool getIntersection(
		Line l, HalfSpace hs,
		out float lDirK			//!< l.pos + l.dir*lDirK で交点位置が求まる
	) {
		var l2hs = hs.pos - l.pos;

		var a = dot(l2hs, hs.dir);
		if (-0.000001<a && a<0.000001) {
			lDirK = 0;
			return true;
		}

		var b = dot(l.dir, hs.dir);
		if (-0.000001<b && b<0.000001) {
			lDirK = 0;
			return false;
		}

		lDirK = a / b;
		return true;
	}

	/**
	 * 直線 vs 直方体 の交点の取得処理。
	 * 交点は二つあるが、直線の向きの反対側の無限遠からレイを飛ばした際に最初に交わる方の交点を返す。
	 * 直線中央から交点への距離と、交点があるか否かを返す。
	 * なお直方体の厚みは考慮しない。
	 */
	static bool getIntersection(
		Line l, Box b,
		out float lDirK,			//!< l.pos + l.dir*lDirK で交点位置が求まる
		out float3 normal
	) {
		var l2v = b.pos - l.pos;

		var vPosL = mul(b.rot, l2v);
		var lDirL = mul(b.rot, l.dir);

		// X軸の判定
		if (lDirL.x < 0.000001f || 0.000001f < lDirL.x) {
			var a = vPosL.x + (0<lDirL.x ? -b.r.x : b.r.x);
			lDirK = a / lDirL.x;
			var c = lDirL.yz * lDirK;
			if (
				-b.r.y <= c.x && c.x <= b.r.y &&
				-b.r.z <= c.y && c.y <= b.r.z
			) {
				normal = 0<lDirL.x ? -b.rot.c0 : b.rot.c0;
				return true;
			}
		}

		// Y軸の判定
		if (lDirL.y < 0.000001f || 0.000001f < lDirL.y) {
			var a = vPosL.y + (0<lDirL.y ? -b.r.y : b.r.y);
			lDirK = a / lDirL.y;
			var c = lDirL.xz * lDirK;
			if (
				-b.r.x <= c.x && c.x <= b.r.x &&
				-b.r.z <= c.y && c.y <= b.r.z
			) {
				normal = 0<lDirL.y ? -b.rot.c1 : b.rot.c1;
				return true;
			}
		}

		// Z軸の判定
		if (lDirL.z < 0.000001f || 0.000001f < lDirL.z) {
			var a = vPosL.z + (0<lDirL.z ? -b.r.z : b.r.z);
			lDirK = a / lDirL.z;
			var c = lDirL.xz * lDirK;
			if (
				-b.r.x <= c.x && c.x <= b.r.x &&
				-b.r.y <= c.y && c.y <= b.r.y
			) {
				normal = 0<lDirL.z ? -b.rot.c2 : b.rot.c2;
				return true;
			}
		}

		lDirK = 0;
		normal = default;
		return false;
	}

	/** 厚み付きの直線 vs 球 の衝突判定 */
	[MI(MO.AggressiveInlining)]
	static bool isHitCore(
		Line l,
		float lineW,
		Sphere s
	) {
		var sumR = s.r + lineW;
		return distanceSq(s.pos, l) <= sumR*sumR;
	}

	/** 厚み付きの直線 vs 球 の衝突判定 */
	[MI(MO.AggressiveInlining)]
	static ColResult collisionCore(
		Line l,
		float lineW,
		Sphere s
	) {
		var s2l = l.pos - s.pos;
		s2l -= dot(s2l, l.dir)*l.dir;
		var dSqLen = lengthsq(s2l);
		var sumR = s.r + lineW;
		if (sumR*sumR < dSqLen) return default;

		// 衝突
		return genColResult(s, s2l, sqrt(dSqLen), lineW);
	}
	
	/** 直線と四角面要素の衝突判定コア処理 */
	static ColResult collisionCore(
		Line l, in FaceQuad q,
		float3 p		// 交点。簡略化のため外部から入力する
	) {
		// 直線 vs 四角面要素 の衝突判定は
		// そのもの単体で行われることはなく、それぞれ何らかの別のプリミティブの内包部位として
		// 計算される。したがってこの計算ではQuadの厚みを考慮する必要はない。
		// なぜなら厚みを考慮すべき衝突の場合は、かならず頂点との衝突が発生しているため、
		// そちらにゆだねる事が出来るからである。

		// 交点pが面の外側にある場合は未衝突とする
		var qp = p - q.pos;
		var px = dot(qp, q.rot.c0);
		var py = dot(qp, q.rot.c1);
		if (
			px < -q.r.x || q.r.x < px ||
			py < -q.r.y || q.r.y < py
		) return default;

		// シリンダー芯を直線と垂直に、外側の境界上に移動させた場合の、引き離し量vを求める
		float v;
		float3 vDir, s = default;
		{
			// 直線から垂直に引き離す際の方向
			vDir = removeDir(q.rot.c2, l.dir);
			var vDLen = lengthsq(vDir);
			if (vDLen < 0.000001f) {
				
				// 面と垂直に直線が交わっている
				var sx = 0<px ? q.r.x : -q.r.x;
				var sy = 0<py ? q.r.y : -q.r.y;
				var dSx = 0<sx ? (sx-px) : (px-sx);
				var dSy = 0<sy ? (sy-py) : (py-sy);

				if (dSx < dSy)	sy = py;
				else			sx = px;
				s = q.pos + sx * q.rot.c0 + sy * q.rot.c1;

				// シリンダーを面と水平に、外側の境界上に移動させた場合の、引き離し量を求める
				var p2s = s - p;
				var sCD = removeDir(p2s, l.dir);		// 直線を引き離す際のベクトル
				var sCDlen = length(sCD);
				v = sCDlen;								// シリンダー全体を引き離す際の引き離し量
				vDir = sCD / (sCDlen + 0.000001f);

			} else {

				// 面とナナメに直線が交わっている
				vDir /= sqrt(vDLen);

				var invRot = transpose(q.rot);
				var vDirL = mul(invRot, vDir);
				var cPosL = mul(invRot, l.pos-q.pos);
				var dirScl = vDirL.z * vDirL.xy / lengthsq(vDirL.xy);
				var x0 = (-q.r.x - cPosL.x) / dirScl.x;			// ここはNaNになる可能性があるがOK
				var x1 = ( q.r.x - cPosL.x) / dirScl.x;
				var y0 = (-q.r.y - cPosL.y) / dirScl.y;
				var y1 = ( q.r.y - cPosL.y) / dirScl.y;
				var x0cy = cPosL.y + vDirL.y/vDirL.x * (-q.r.x - cPosL.x);
				var x1cy = cPosL.y + vDirL.y/vDirL.x * ( q.r.x - cPosL.x);
				var y0cx = cPosL.x + vDirL.x/vDirL.y * (-q.r.y - cPosL.y);
				var y1cx = cPosL.x + vDirL.x/vDirL.y * ( q.r.y - cPosL.y);
				v = -1;
				float2 sL = default;
				if ( v<x0 && -q.r.y<=x0cy && x0cy<=q.r.y ) {v=x0; sL=float2(-q.r.x,x0cy);}
				if ( v<x1 && -q.r.y<=x1cy && x1cy<=q.r.y ) {v=x1; sL=float2( q.r.x,x1cy);}
				if ( v<y0 && -q.r.x<=y0cx && y0cx<=q.r.x ) {v=y0; sL=float2(y0cx,-q.r.y);}
				if ( v<y1 && -q.r.x<=y1cx && y1cx<=q.r.x ) {v=y1; sL=float2(y1cx, q.r.y);}
				v = (v - cPosL.z) * vDirL.z;
				s = q.pos + mul(q.rot, float3(sL,0));
			}
		}

		return new ColResult(s, vDir, v);
	}




	// プリミティブ間差分位置とその長さ、位置パラメータ等から衝突情報を生成する処理
	static ColResult genColResult(
		HalfSpace hs,
		float3 dPos,		//!< もう一方のプリミティブへの差分位置
		float depthOfs		//!< もう一方のプリミティブの厚さ
	) {
		var a = dot(dPos, hs.dir);
		if (depthOfs < a) return default;

		return new ColResult(
			hs.pos + dPos - a*hs.dir,
			hs.dir,
			-a + depthOfs
		);
	}
	static ColResult genColResult(
		Sphere s,
		float3 dPos,		//!< もう一方のプリミティブへの差分位置
		float dPosLen,		//!< dPosの長さ
		float depthOfs		//!< もう一方のプリミティブの厚さ
	) {
		var nml = dPos / max(dPosLen, 0.000001f);
		return new ColResult(
			s.pos + nml*s.r,
			nml,
			s.r - dPosLen + depthOfs
		);
	}
	static ColResult genColResult(
		in Capsule c,
		float3 dPos,		//!< もう一方のプリミティブへの差分位置
		float dPosLen,		//!< dPosの長さ
		float lenH,			//!< 最短距離を結ぶ直線とカプセル軸直線の交点への、c.posからの距離。c.dir方向が正
		float depthOfs		//!< もう一方のプリミティブの厚さ
	) {
		var nml = dPos / max(dPosLen, 0.000001f);
		return new ColResult(
			c.pos + lenH*c.dir + nml*c.r_s,
			nml,
			c.r_s - dPosLen + depthOfs
		);
	}
	static ColResult genColResult(
		in Edge c,
		float3 dPos,		//!< もう一方のプリミティブへの差分位置
		float dPosLen,		//!< dPosの長さ
		float lenH,			//!< 最短距離を結ぶ直線とカプセル軸直線の交点への、c.posからの距離。c.dir方向が正
		float depthOfs		//!< もう一方のプリミティブの厚さ
	) {
		var nml = dPos / max(dPosLen, 0.000001f);
		return new ColResult(
			c.pos + lenH*c.dir + nml*c.r_s,
			nml,
			c.r_s - dPosLen + depthOfs
		);
	}
	static ColResult genColResult(
		FaceQuad q,
		float3 dPos,		//!< もう一方のプリミティブへの差分位置
		float depthOfs		//!< もう一方のプリミティブの厚さ
	) {
		var dPosZ = dot(dPos, q.rot.c2);
		// 面の裏側にある場合は、たとえ重複していたとしても衝突は無い物とする。
		// そういう相手は、裏側のFaceQuadとの衝突で判定を行う。
		if (dPosZ < 0 || q.w+depthOfs < dPosZ) return default;

		var dPosX = dot(dPos, q.rot.c0);
		if (dPosX < -q.r.x || q.r.x < dPosX) return default;

		var dPosY = dot(dPos, q.rot.c1);
		if (dPosY < -q.r.y || q.r.y < dPosY) return default;

		return new ColResult(
			q.pos + dPos + (q.w-dPosZ)*q.rot.c2,
			q.rot.c2,
			-dPosZ + q.w + depthOfs
		);
	}



}
}
