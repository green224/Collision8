using System;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {

	/** AABBを使用した衝突判定 */
	public partial struct AABB {
		// ------------------------------------- public メンバ ----------------------------------------

		/** 指定の位置から指定位置までのレイキャストを計算する */
		public bool raycast( float3 src, float3 dst ) {
			var maxPos = minPos + size;

			// 開始位置が既にめり込んでいた場合
			if (
				minPos.x < src.x && src.x < maxPos.x &&
				minPos.y < src.y && src.y < maxPos.y &&
				minPos.z < src.z && src.z < maxPos.z
			) {
				return true;
			}

			var s2d = dst - src;
			// X軸方向の判定
			if (s2d.x < -0.000001f || 0.000001f < s2d.x) {
				if (src.x < minPos.x) {
					if (dst.x < minPos.x) return false;

					var a = src.x / s2d.x;
					var b = src.y - s2d.y*a;
					var c = src.z - s2d.z*a;
					if ( minPos.y<=b && b<=maxPos.y && minPos.z<=c && c<=maxPos.z ) {
						return true;
					}
				} else if (maxPos.x < src.x) {
					if (maxPos.x < dst.x) return false;

					var a = (src.x-maxPos.x) / s2d.x;
					var b = src.y - s2d.y*a;
					var c = src.z - s2d.z*a;
					if ( minPos.y<=b && b<=maxPos.y && minPos.z<=c && c<=maxPos.z ) {
						return true;
					}
				}
			}

			// Y軸方向の判定
			if (s2d.y < -0.000001f || 0.000001f < s2d.y) {
				if (src.y < minPos.y) {
					if (dst.y < minPos.y) return false;

					var a = src.y / s2d.y;
					var b = src.x - s2d.x*a;
					var c = src.z - s2d.z*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.z<=c && c<=maxPos.z ) {
						return true;
					}
				} else if (maxPos.y < src.y) {
					if (maxPos.y < dst.y) return false;

					var a = (src.y-maxPos.y) / s2d.y;
					var b = src.x - s2d.x*a;
					var c = src.z - s2d.z*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.z<=c && c<=maxPos.z ) {
						return true;
					}
				}
			}

			// Z軸方向の判定
			if (s2d.z < -0.000001f || 0.000001f < s2d.z) {
				if (src.z < minPos.z) {
					if (dst.z < minPos.z) return false;

					var a = src.z / s2d.z;
					var b = src.x - s2d.x*a;
					var c = src.y - s2d.y*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.y<=c && c<=maxPos.y ) {
						return true;
					}
				} else if (maxPos.z < src.z) {
					if (maxPos.z < dst.z) return false;

					var a = (src.z-maxPos.z) / s2d.z;
					var b = src.x - s2d.x*a;
					var c = src.y - s2d.y*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.y<=c && c<=maxPos.y ) {
						return true;
					}
				}
			}

			return false;
		}

		/** 指定の位置から指定位置までのレイキャストを計算する。dstは衝突位置に更新される */
		public bool raycast( float3 src, ref float3 dst, ref float3 normal, out bool isInside ) {
			var maxPos = minPos + size;

			// 開始位置が既にめり込んでいた場合
			if (
				minPos.x < src.x && src.x < maxPos.x &&
				minPos.y < src.y && src.y < maxPos.y &&
				minPos.z < src.z && src.z < maxPos.z
			) {
				dst = src;
				isInside = true;
				return true;
			} else {
				isInside = false;
			}

			var s2d = dst - src;
			// X軸方向の判定
			if (s2d.x < -0.000001f || 0.000001f < s2d.x) {
				if (src.x < minPos.x) {
					if (dst.x < minPos.x) return false;

					var a = src.x / s2d.x;
					var b = src.y - s2d.y*a;
					var c = src.z - s2d.z*a;
					if ( minPos.y<=b && b<=maxPos.y && minPos.z<=c && c<=maxPos.z ) {
							normal = float3(-1,0,0);
							dst = float3(minPos.x, b, c);
						return true;
					}
				} else if (maxPos.x < src.x) {
					if (maxPos.x < dst.x) return false;

					var a = (src.x-maxPos.x) / s2d.x;
					var b = src.y - s2d.y*a;
					var c = src.z - s2d.z*a;
					if ( minPos.y<=b && b<=maxPos.y && minPos.z<=c && c<=maxPos.z ) {
							normal = float3(1,0,0);
							dst = float3(maxPos.x, b, c);
						return true;
					}
				}
			}

			// Y軸方向の判定
			if (s2d.y < -0.000001f || 0.000001f < s2d.y) {
				if (src.y < minPos.y) {
					if (dst.y < minPos.y) return false;

					var a = src.y / s2d.y;
					var b = src.x - s2d.x*a;
					var c = src.z - s2d.z*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.z<=c && c<=maxPos.z ) {
							normal = float3(0,-1,0);
							dst = float3(b, minPos.y, c);
						return true;
					}
				} else if (maxPos.y < src.y) {
					if (maxPos.y < dst.y) return false;

					var a = (src.y-maxPos.y) / s2d.y;
					var b = src.x - s2d.x*a;
					var c = src.z - s2d.z*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.z<=c && c<=maxPos.z ) {
							normal = float3(0,1,0);
							dst = float3(b, maxPos.y, c);
						return true;
					}
				}
			}

			// Z軸方向の判定
			if (s2d.z < -0.000001f || 0.000001f < s2d.z) {
				if (src.z < minPos.z) {
					if (dst.z < minPos.z) return false;

					var a = src.z / s2d.z;
					var b = src.x - s2d.x*a;
					var c = src.y - s2d.y*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.y<=c && c<=maxPos.y ) {
							normal = float3(0,0,-1);
							dst = float3(b, c, minPos.z);
						return true;
					}
				} else if (maxPos.z < src.z) {
					if (maxPos.z < dst.z) return false;

					var a = (src.z-maxPos.z) / s2d.z;
					var b = src.x - s2d.x*a;
					var c = src.y - s2d.y*a;
					if ( minPos.x<=b && b<=maxPos.x && minPos.y<=c && c<=maxPos.y ) {
							normal = float3(0,0,1);
							dst = float3(b, c, maxPos.z);
						return true;
					}
				}
			}

			return false;
		}



		// --------------------------------- private / protected メンバ -------------------------------

		
		// --------------------------------------------------------------------------------------------
	}


}

