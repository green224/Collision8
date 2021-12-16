using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
static public partial class Solver {

	/** 引き離し時に、計算誤差を埋めるために追加で足す微小距離 */
	public const float SEPARATION_ADD_EPSILON = 0.00001f;



	/** 点 vs 半空間 の衝突判定 */
	static public ColResult collision(Point p, HalfSpace hs) {
		return genColResult(hs, p.pos-hs.pos, 0);
	}

	/** 点 vs 球 の衝突判定 */
	static public ColResult collision(Point p, Sphere s) {
		var s2p = p.pos - s.pos;
		var dSqLen = lengthsq(p.pos - s.pos);
		if (s.r*s.r < dSqLen) return default;

		// 衝突
		return genColResult(s, s2p, sqrt(dSqLen), 0);
	}

	/** 点 vs カプセル の衝突判定 */
	static public ColResult collision(Point p, in Capsule c) {
		var c2p = p.pos - c.pos;

		// まずはバウンダリー球で衝突判定(ここは球と直線との衝突判定にしてもよい)
		var dSqLen = lengthsq( c2p );
		var sumR_h = c.r_h+c.r_s;
		if (sumR_h*sumR_h < dSqLen) return default;

		// 縦方向の位置により距離を再計算
		var len_h = dot(c2p, c.dir);
		if (len_h < -c.r_h) {		// 下側の球との衝突可能性がある場合
			c2p += c.dir * c.r_h;
			len_h = -c.r_h;
		} else if (len_h < c.r_h) {	// 中央との衝突可能性がある場合
			c2p -= c.dir * len_h;
		} else {					// 上側の球との衝突可能性がある場合
			c2p += c.dir * -c.r_h;
			len_h = c.r_h;
		}

		// 距離判定
		dSqLen = lengthsq( c2p );
		var sumR_s = c.r_s;
		if ( sumR_s*sumR_s < dSqLen ) return default;

		// 衝突
		return genColResult(c, c2p, sqrt(dSqLen), len_h, 0);
	}

	/** 点 vs 辺要素 の衝突判定 */
	static public ColResult collision(Point p, in Edge c) {
		var c2p = p.pos - c.pos;

		// 縦方向の位置により距離を再計算
		var len_h = dot(c2p, c.dir);
		if (len_h < -c.r_h || c.r_h < len_h) return default;

		// 距離判定
		c2p -= c.dir * len_h;
		var dSqLen = lengthsq( c2p );
		var sumR_s = c.r_s;
		if (sumR_s*sumR_s < dSqLen) return default;

		// 衝突
		return genColResult(c, c2p, sqrt(dSqLen), len_h, 0);
	}

	/** 点 vs 四角面要素 の衝突判定 */
	static public ColResult collision(Point p, in FaceQuad q) {
		return genColResult(q, p.pos-q.pos, 0);
	}



	/** 直線 vs 球 の衝突判定 */
	[MI(MO.AggressiveInlining)]
	static public ColResult collision(Line l, Sphere s) => collisionCore(l,0,s);

	/** 直線 vs カプセル の衝突判定 */
	static public ColResult collision(Line l, in Capsule c) {
		var dist = getClosestPoint( l, (Line)c, out var mnpD_l, out var mnpD_c );

		// 最短距離が範囲内でない場合は未衝突
		if (c.r_s < dist) return default;

		// 最近傍点がカプセル範囲内なら衝突
		if (abs(mnpD_c) <= c.r_h) {
			var c2l = l.pos - c.pos;
			c2l += -mnpD_c*c.dir + mnpD_l*l.dir;
			return genColResult(c, c2l, length(c2l), mnpD_c, 0);
		}

		// カプセル両端の球との衝突判定
		return collision( l, c.capSphere(sign(mnpD_c)) );
	}

	/** 直線 vs 辺要素 の衝突判定 */
	static public ColResult collision(Line l, in Edge c) {
		var dist = getClosestPoint( l, (Line)c, out var mnpD_l, out var mnpD_c );

		// 最短距離が範囲内でない場合は未衝突
		if (c.r_s < dist) return default;

		// 最近傍点がカプセル範囲内なら衝突
		if (abs(mnpD_c) <= c.r_h) {
			var c2l = l.pos - c.pos;
			c2l += -mnpD_c*c.dir + mnpD_l*l.dir;
			return genColResult(c, c2l, length(c2l), mnpD_c, 0);
		}

		return default;
	}

	/** 直線 vs 四角面要素 の衝突判定 */
	static public ColResult collision(Line l, in FaceQuad q) {

		// 面とlとの交点pを求める
		float3 p;
		if ( getIntersection( l, new HalfSpace(q.pos, q.rot.c2), out var a) ) {
			p = l.pos + l.dir * a;
		} else {
			return default;
		}

		// 直線を面外側の境界上に移動させる
		return collisionCore( l, q, p );
	}

	/** 直線 vs 三角形 の衝突判定 */
	static public ColResult collision(Line l, in Triangle t) {

		// カプセルとの衝突で代用する
		var trm = max(abs(t.vert0), max(abs(t.vert1), abs(t.vert2)));
		var tr = trm.x+trm.y+trm.z + t.w;
		var pos = removeDir( l.pos-t.pos, l.dir ) + t.pos;

		return collision( new Capsule(pos, l.dir, 0, tr*10), t );
	}

	/** 直線 vs 直方体 の衝突判定 */
	static public ColResult collision(Line l, in Box b) {

		// カプセルとの衝突で代用する
		var r = b.r.x+b.r.y+b.r.z + b.w;
		var pos = removeDir( l.pos-b.pos, l.dir ) + b.pos;

		return collision( new Capsule(pos, l.dir, 0, r*10), b );
	}

	/** 直線 vs 円盤 の衝突判定 */
	static public ColResult collision(Line l, in Circle c) {

		// カプセルとの衝突で代用する
		var r = c.r + c.w;
		var pos = removeDir( l.pos-c.pos, l.dir ) + c.pos;

		return collision( new Capsule(pos, l.dir, 0, r*10), c );
	}

	/** 直線 vs 円柱 の衝突判定 */
	static public ColResult collision(Line l, in Cylinder c) {

		// カプセルとの衝突で代用する
		var r = c.r_s+c.r_h + c.w;
		var pos = removeDir( l.pos-c.pos, l.dir ) + c.pos;

		return collision( new Capsule(pos, l.dir, 0, r*10), c );
	}

	/** 直線 vs 円錐 の衝突判定 */
	static public ColResult collision(Line l, in Cone c) {

		// カプセルとの衝突で代用する
		var r = c.r_s+c.r_h + c.w;
		var pos = removeDir( l.pos-c.pos, l.dir ) + c.pos;

		return collision( new Capsule(pos, l.dir, 0, r*10), c );
	}

	/** 直線 vs 凸型多面体 の衝突判定 */
	static public ColResult collision(Line l, in ConvexVerts c) {

		// カプセルとの衝突で代用する
		var r = c.boundaryR + c.w;
		var pos = removeDir( l.pos-c.pos, l.dir ) + c.pos;

		return collision( new Capsule(pos, l.dir, 0, r*10), c );
	}



	/** 半空間 vs 球 の衝突判定 */
	static public ColResult collision(HalfSpace hs, Sphere s) =>
		-genColResult(hs, s.pos-hs.pos, s.r);



	/** 球 vs 球 の衝突判定 */
	static public ColResult collision(Sphere a, Sphere b) {
		var b2a = a.pos - b.pos;
		var dSqLen = lengthsq(b2a);
		var sumR = a.r + b.r;
		if (sumR*sumR < dSqLen) return default;

		// 衝突
		return genColResult(b, b2a, sqrt(dSqLen), a.r);
	}

	/** 球 vs カプセル の衝突判定 */
	static public ColResult collision(Sphere s, in Capsule c) {
		var c2s = s.pos - c.pos;

		// まずはバウンダリー球で衝突判定
		var dSqLen = lengthsq( c2s );
		var sumR_h = c.r_h+c.r_s + s.r;
		if (sumR_h*sumR_h < dSqLen) return default;

		// 縦方向の位置により距離を再計算
		var len_h = dot(c2s, c.dir);
		if (len_h < -c.r_h) {			// 下側の球との衝突可能性がある場合
			c2s += c.dir * c.r_h;
			len_h = -c.r_h;
		} else if (len_h < c.r_h) {		// 中央との衝突可能性がある場合
			c2s -= c.dir * len_h;
		} else {						// 上側の球との衝突可能性がある場合
			c2s += c.dir * -c.r_h;
			len_h = c.r_h;
		}

		// 距離判定
		dSqLen = lengthsq( c2s );
		var sumR_s = c.r_s + s.r;
		if ( sumR_s*sumR_s < dSqLen ) return default;

		// 衝突
		return genColResult(c, c2s, sqrt(dSqLen), len_h, s.r);
	}

	/** 球 vs 辺要素 の衝突判定 */
	static public ColResult collision(Sphere s, in Edge c) {
		var c2s = s.pos - c.pos;

		// 縦方向の位置により距離を再計算
		var len_h = dot(c2s, c.dir);
		if (len_h < -c.r_h || c.r_h < len_h) return default;

		// 距離判定
		c2s -= c.dir * len_h;
		var dSqLen = lengthsq( c2s );
		var sumR_s = c.r_s + s.r;
		if (sumR_s*sumR_s < dSqLen) return default;

		// 衝突
		return genColResult(c, c2s, sqrt(dSqLen), len_h, s.r);
	}

	/** 球 vs 四角面要素 の衝突判定 */
	static public ColResult collision(Sphere s, in FaceQuad q) =>
		genColResult(q, s.pos-q.pos, s.r);



	/** カプセル vs カプセル の衝突判定 */
	static public ColResult collision(in Capsule a, in Capsule b) {

		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;

		{// 辺との衝突判定
			var retE = collision(a, (Edge)b);
			if (retE.isHit) return retE;
		}

		{// 頂点との衝突判定
			var b2a = a.pos - b.pos;
			var s = b.capSphere(0 < dot(b2a, b.dir) ? 1 : -1);
			return collision(a, s);
		}
	}

	/** カプセル vs 辺要素 の衝突判定 */
	static public ColResult collision(in Capsule a, in Edge b) {
		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;

		// 最短距離が範囲内でない場合は未衝突
		var dist = getClosestPoint( (Line)a, (Line)b, out var mnpD_a, out var mnpD_b );
		if (a.r_s+b.r_s < dist) return default;

		// 最近傍点がどちらの範囲内かどうかで処理を分ける
		var isInsideA = abs(mnpD_a) <= a.r_h;
		var isInsideB = abs(mnpD_b) <= b.r_h;
		if (isInsideA) {
			if (isInsideB) {
				var b2a = a.pos - b.pos + mnpD_a*a.dir - mnpD_b*b.dir;
				return genColResult(b, b2a, length(b2a), mnpD_b, a.r_s);
			}
			return default;
		} else if (isInsideB) {
			return collision( a.capSphere(sign(mnpD_a)), b );
		}

		// 最近傍点がどちらの範囲にも入っていない場合は未衝突
		return default;
	}

	/** カプセル vs 四角面要素 の衝突判定 */
	static public ColResult collision(Capsule c, in FaceQuad q) {

		// これは複雑な衝突判定になるので、まず辺要素部分と衝突判定する
		var ret0 = collision((Edge)c, q);

		// 衝突していたらまずその情報をもとに引き離しを行い、
		// 引き離した結果の状態で、再度両端の球との衝突判定を行う
		if (ret0.isHit) c.pos += ret0.normal * (ret0.depth + SEPARATION_ADD_EPSILON);

		// 両端の球との衝突判定を行う
		var ret1 = collision(
			c.capSphere( 0 < dot(q.rot.c2, c.dir) ? -1 : 1 ),
			q
		);

		// 衝突結果を結合して返す
		if (ret0.isHit) {
			if (ret1.isHit) {
				// 二つとも衝突した場合は、面の上方向への押し出しが行われている
				return new ColResult(
					ret1.pos,
					ret1.normal,
					ret1.depth + (ret0.depth + SEPARATION_ADD_EPSILON)
				);
			}
			return ret0;
		} else {
			if (ret1.isHit) return ret1;
			return default;
		}
	}



	/** 辺要素 vs 辺要素 の衝突判定 */
	static public ColResult collision(in Edge a, in Edge b) {
		// 辺要素同士の衝突は、双方が平面上に並んでいて斜めにギリギリ接している場合には
		// 正常に衝突判定を行えない。
		// ただそもそも辺要素は単体で用いるものではなく、
		// そのような状態では必ず他のパーツが衝突しているので、このままで良しとする。

		// まずはバウンダリー球で衝突判定
		if (!checkBoundary(a,b)) return default;

		// 最短距離が範囲内でない場合は未衝突
		var dist = getClosestPoint( (Line)a, (Line)b, out var mnpD_a, out var mnpD_b );
		if (a.r_s+b.r_s < dist) return default;

		// 最近傍点が双方の範囲内かどうかで処理を分ける
		var isInsideA = abs(mnpD_a) <= a.r_h;
		var isInsideB = abs(mnpD_b) <= b.r_h;
		if (isInsideA && isInsideB) {
			var b2a = a.pos - b.pos + mnpD_a*a.dir - mnpD_b*b.dir;
			return genColResult(b, b2a, length(b2a), mnpD_b, a.r_s);
		}

		// 最近傍点がどちらかの範囲に入っていない場合は未衝突
		return default;
	}

	/** 辺要素 vs 四角面要素 の衝突判定 */
	static public ColResult collision(in Edge c, in FaceQuad q) {
		// 辺要素 vs 四角面要素 の衝突判定は
		// そのもの単体で行われることはなく、それぞれ何らかの別のプリミティブの内包部位として
		// 計算される。したがってナナメに接するような場合や、面に水平に接している際に正確に計算を行う必要はない。
		// また、この計算ではQuadの厚みも考慮する必要はない。
		// なぜなら厚みを考慮すべき衝突の場合は、かならず頂点との衝突が発生しているため、
		// そちらにゆだねる事が出来るからである。

		// 面とシリンダー中心直線との交点pを求める。
		float3 p;
		float lDirK;
		if (getIntersection( (Line)c, new HalfSpace(q.pos, q.rot.c2), out lDirK)) {
			// 単純にシリンダー中心直線との交点がシリンダー範囲外の場合は未衝突として扱う
			if (lDirK < -c.r_h || c.r_h < lDirK) return default;
			p = c.pos + c.dir * lDirK;
		} else {
			return default;
		}

		// シリンダー芯を直線と垂直に、外側の境界上に移動させた場合の、引き離し量vを求める
		float v;
		float3 vDir, s;
		{
			var cc = collisionCore((Line)c, q, p);
			if (!cc.isHit) return default;
			v = cc.depth;
			vDir = cc.normal;
			s = cc.pos;
		}

		float u;
		float3 t;
		{// シリンダーを面から垂直に持ち上げた場合に必要な引き離し量uを求める
			t = c.pos + c.dir * ( 0<lDirK ? c.r_h : -c.r_h );
			// これは範囲外に向けて斜めに突き刺さっている場合には
			// 正確にならないケースがある。その場合はvを垂直に引き離す次のステップの処理が選ばれる
			u = dot(p-t, q.rot.c2);
		}

		float w;
		{// 直線と垂直にv引き離すを処理を、面と垂直に行った場合の引き離し量wを求める
			var d = dot(vDir, q.rot.c2);
			w = d < -0.000001 ? v / -d : -1;
		}

		// 引き離し量から、垂直・水平どちらに引き離すかを決定する。
		if (0 <= u && u < v) {
			// シリンダーを面外側の境界上に移動させるように、面から垂直に持ち上げる
			if (0 <= w && w < u)	return new ColResult( s, q.rot.c2, w );
			// シリンダーを面から垂直に持ち上げる
			else					return -new ColResult( t, -q.rot.c2, u );
		} else {
			// シリンダーを面外側の境界上に移動させるように、面から垂直に持ち上げる
			if (0 <= w && w < u)	return new ColResult( s, q.rot.c2, w );
			// シリンダーを面外側の境界上に移動させるように、直線と垂直に持ち上げる
			else					return new ColResult( s, vDir, v );
		}
	}





}
}
