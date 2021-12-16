

// #define USE_AVERAGE_COLPOS

using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
static public partial class MPR {

	/** MPRアルゴリズムで衝突点と法線、衝突深度をもとめる */
	public static ColResult solve(Sphere a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, Sphere b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Capsule a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Capsule b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Quad a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Quad b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Triangle a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Triangle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, Point b) => -solve(b,a);
	public static ColResult solve(Point a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(Sphere a, Sphere b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Capsule a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Capsule b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Quad a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Quad b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Triangle a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Triangle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, Sphere b) => -solve(b,a);
	public static ColResult solve(Sphere a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Capsule a, in Capsule b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Quad a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Quad b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Triangle a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Triangle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Capsule b) => -solve(b,a);
	public static ColResult solve(in Capsule a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Quad a, in Quad b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Triangle a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in Triangle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Quad b) => -solve(b,a);
	public static ColResult solve(in Quad a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Triangle a, in Triangle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, in Triangle b) => -solve(b,a);
	public static ColResult solve(in Triangle a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, in Triangle b) => -solve(b,a);
	public static ColResult solve(in Triangle a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Triangle b) => -solve(b,a);
	public static ColResult solve(in Triangle a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Triangle b) => -solve(b,a);
	public static ColResult solve(in Triangle a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Triangle b) => -solve(b,a);
	public static ColResult solve(in Triangle a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Box a, in Box b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, in Box b) => -solve(b,a);
	public static ColResult solve(in Box a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Box b) => -solve(b,a);
	public static ColResult solve(in Box a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Box b) => -solve(b,a);
	public static ColResult solve(in Box a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Box b) => -solve(b,a);
	public static ColResult solve(in Box a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Circle a, in Circle b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Circle b) => -solve(b,a);
	public static ColResult solve(in Circle a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Circle b) => -solve(b,a);
	public static ColResult solve(in Circle a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Circle b) => -solve(b,a);
	public static ColResult solve(in Circle a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cylinder a, in Cylinder b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Cylinder b) => -solve(b,a);
	public static ColResult solve(in Cylinder a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Cylinder b) => -solve(b,a);
	public static ColResult solve(in Cylinder a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in Cone a, in Cone b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in Cone b) => -solve(b,a);
	public static ColResult solve(in Cone a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}
	public static ColResult solve(in ConvexVerts a, in ConvexVerts b) {
		//まずミンコフスキー差の中心をもとめる
		var vert0 = a.pos - b.pos;
		// AとBが同じ位置に存在する場合は、適当にオフセットを噛ませておく。じゃないと計算が始まらないので。
		if (lengthsq(vert0) < 0.00000001f) vert0 = float3(0.0001f,0,0);

		//中心から原点方向に最も離れた頂点を求める
		var dir = -vert0;
		var vec01 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos1);
		if ( 0 <= dot(vec01+vert0,vert0) ) return default;		//原点の方が離れている場合は衝突無し

		//さらにその辺から原点方向に最も離れた頂点を求める
		dir = cross( cross(vert0, vec01), vec01 );
		if (lengthsq(dir) < 0.00000001f) {
			// 一発でもう衝突点を検出できてしまった
			var coliNormal = normalize( vert0 );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
		#if USE_AVERAGE_COLPOS
			var coliPos = (supPos1.a + supPos1.b)/2;
		#else
			var coliPos = supPos1.b;
		#endif
			return new ColResult( supPos1.b, coliNormal, coliDepth );
		}
		var vec02 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos2);

		//さらにその面から原点方向に最も離れた頂点を求める
		dir = cross( vec02, vec01 );
		if ( dot(dir,vert0) <= 0 ) {
			var _=vec01; vec01=vec02; vec02=_;	//vert1とvert2をスワップ
			var __=supPos1; supPos1=supPos2; supPos2=__;
		} else {
			dir = -dir;
		}
		var vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos3);

		//ここからがv0を頂点とした無限三角錐ボリューム内に原点を内包させるループ
		for (int mprIterations=0;; ++mprIterations) {
			// 　　　　v3
			// 　　Ｏ／｜＼
			// 　　／　｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//原点はここまでの手順で必ず面012の上側にあるが
			//面032の左、もしくは面013の右側にあるかもしれない
			//その場合はそちら側に単体を作り直す必要がある
			dir = cross(vec03, vec02);
			if ( dot(dir,vert0) <= 0 ) {
				//原点が面032の左側にある
				supPos1 = supPos3;
				vec01 = vec03;
				vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
			} else {
				dir = cross(vec01, vec03);
				if ( dot(dir, vert0) <= 0 ) {
					//原点が面013の右側にある
					supPos2 = supPos3;
					vec02 = vec03;
					vec03 = maxPointInMinkDiffAlongDir(a,b,dir, out supPos3);
				} else {
					//原点はv0を頂点とした無限三角錐ボリューム内にある！
					break;
				}
			}

			if ( mprIterations == MaxIterations - 1 )
				return default;	//収束なし
		}

		//ここからがv0を頂点とした無限三角錐ボリュームを狭めていくループ
		for (int mprIterations=0; mprIterations<MaxIterations; ++mprIterations) {
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//まずは面123の法線方向へ最も離れた頂点v4を求める
			dir = cross( vec02-vec01, vec03-vec01 );
			var vec04 = maxPointInMinkDiffAlongDir(a,b,dir, out var supPos4);
			//v1,v2,v3のどれかが検出された場合は現状態がもっとも狭い無限三角錐ボリューム
			//なので、ループを終了して進入位置・深度を求める
			if ( vec04.Equals(vec01) || vec04.Equals(vec02) || vec04.Equals(vec03) ) break;

			// 　　　　v3    v4
			// 　　　／｜＼  /
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜_ /_＼v1
			//   ＼　　｜ /　／
			//   　＼　｜/ ／
			// 　　　＼｜／
			// 　　　　v0
			//頂点Aと頂点1,2,3を用いて、v0を頂点とした無限三角錐を
			//3種類考えることができるが、
			//その中で原点を内包する無限三角錐を選びたい
			//そのために平面043,042,041に対して原点がどのような関係にあるかを調べる
			var aIsUpperOnFace043 = dot(cross(vec04,vec03), vert0) <= 0;
			var aIsUpperOnFace042 = dot(cross(vec04,vec02), vert0) <= 0;
			var aIsUpperOnFace041 = dot(cross(vec04,vec01), vert0) <= 0;

			if ( aIsUpperOnFace042 && !aIsUpperOnFace043 ) {
				//無限三角錐0423が新しい無限三角錐
				vec01 = vec04;
				supPos1 = supPos4;
			} else if ( aIsUpperOnFace041 && !aIsUpperOnFace042 ) {
				//無限三角錐0124が新しい無限三角錐
				vec03 = vec04;
				supPos3 = supPos4;
			} else {
				//無限三角錐0143が新しい無限三角錐
				vec02 = vec04;
				supPos2 = supPos4;
			}
		}

		{
			//原点を内包し、v0を頂点とするもっとも狭い無限三角錐が得られた
			//したがって進入深度は原点から面123へのベクトルとなる
			// 　　　　v3
			// 　　　／｜＼
			// 　　／Ｏ｜　＼
			// v2∠＿＿｜＿＿＼v1
			//   ＼　　｜　　／
			//   　＼　｜　／
			// 　　　＼｜／
			// 　　　　v0
			//実際にはそうならない場合もありそうだが…？
			var coliNormal = normalizesafe( cross(vec03-vec01, vec02-vec01) );
			var coliDepth = -dot(vec01+vert0, coliNormal);
			if ( coliDepth <= 0.0 ) return default;		//衝突深度がマイナスのときは衝突無し
			// ミンコフスキー差での原点座標の三角錐0123の重心座標を求める。
			var rate = getBarycentric( -vert0, vec01, vec02, vec03 );
		#if USE_AVERAGE_COLPOS
			//A,Bそれぞれの場合について同じ重心座標を用いて算出した座標を平均した衝突位置とする
			var coliPos = supPos1.a * rate.x
					+ supPos2.a * rate.y
					+ supPos3.a * rate.z + a.pos
					+ supPos1.b * rate.x
					+ supPos2.b * rate.y
					+ supPos3.b * rate.z + b.pos;
			coliPos = coliPos/2;
		#else
			// A,Bどちらについて計算しても同じ結果になるので、とりあえずB側の座標のみを使う
			var coliPos =
					supPos1.b * rate.x +
					supPos2.b * rate.y +
					supPos3.b * rate.z + b.pos;
		#endif
			// 衝突点がメッシュにめり込んだ状態で検出されるので、衝突面上の位置に補正する
			coliPos += dot(b.pos + supPos1.b - coliPos, coliNormal) * coliNormal;
			return new ColResult( coliPos, coliNormal, coliDepth );
		}
	}


	/** サポート写像結果。maxPointInMinkDiffAlongDirの返り値として受け取る */
	struct SupportPos {
	#if USE_AVERAGE_COLPOS
		public float3 a;
	#endif
		public float3 b;
	}

	/**
	 * A-Bのミンコフスキー差で、指定の方向に最も離れた頂点座標を求める処理。
	 * ミンコフスキー差にする前のA,Bそれぞれでの頂点座標も返す
	 */
	static float3 maxPointInMinkDiffAlongDir(
		Point a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Point a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		Sphere a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Capsule a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Quad a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Triangle a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Box a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Circle a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cylinder a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in Cone a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, Point b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, Sphere b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Capsule b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Quad b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Triangle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Box b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Circle b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Cylinder b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in Cone b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}
	static float3 maxPointInMinkDiffAlongDir(
		in ConvexVerts a, in ConvexVerts b, float3 dir,
		out SupportPos supPos
	) {
		var vertA = a.supportFunc(dir);
		var vertB = b.supportFunc(-dir);
		supPos = new SupportPos {
		#if USE_AVERAGE_COLPOS
			a = vertA,
		#endif
			b = vertB,
		};
		return vertA - vertB;
	}

}
}


