#if false	//#Ignore  IDE状でエラー表示させ無くするために無効にしておく
using System;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {

	/** AABBを使用した衝突判定 */
	public partial struct AABB {
		// ------------------------------------- public メンバ ----------------------------------------

		//# for (int i=0; i<2; ++i) {
		//# 	if (i==0) {
		/** 指定の位置から指定位置までのレイキャストを計算する */
		public bool raycast( float3 src, float3 dst ) {
		//# 	} else {
		/** 指定の位置から指定位置までのレイキャストを計算する。dstは衝突位置に更新される */
		public bool raycast( float3 src, ref float3 dst, ref float3 normal, out bool isInside ) {
		//# 	}
			var maxPos = minPos + size;

			// 開始位置が既にめり込んでいた場合
			if (
				minPos.x < src.x && src.x < maxPos.x &&
				minPos.y < src.y && src.y < maxPos.y &&
				minPos.z < src.z && src.z < maxPos.z
			) {
		//# 	if (i==1) {
				dst = src;
				isInside = true;
				return true;
			} else {
				isInside = false;
			}
		//# 	} else {
				return true;
			}
		//# 	}

			var s2d = dst - src;
		//# 	for (int j=0; j<3; ++j) {
		//# 		var axis = j==0 ? new[]{"x","y","z"} : ( j==1 ? new[]{"y","x","z"} : new[]{"z","x","y"} );
			【"// " + char.ToUpper(axis[0][0]) + "軸方向の判定"】
			if (s2d.【axis[0]】 < -0.000001f || 0.000001f < s2d.【axis[0]】) {
				if (src.【axis[0]】 < minPos.【axis[0]】) {
					if (dst.【axis[0]】 < minPos.【axis[0]】) return false;

					var a = src.【axis[0]】 / s2d.【axis[0]】;
					var b = src.【axis[1]】 - s2d.【axis[1]】*a;
					var c = src.【axis[2]】 - s2d.【axis[2]】*a;
					if ( minPos.【axis[1]】<=b && b<=maxPos.【axis[1]】 && minPos.【axis[2]】<=c && c<=maxPos.【axis[2]】 ) {
		//# 		if (i==1) {
		//# 			if (j==0) {
							normal = float3(-1,0,0);
							dst = float3(minPos.【axis[0]】, b, c);
		//# 			} else if (j==1) {
							normal = float3(0,-1,0);
							dst = float3(b, minPos.【axis[0]】, c);
		//# 			} else {
							normal = float3(0,0,-1);
							dst = float3(b, c, minPos.【axis[0]】);
		//# 			}
		//# 		}
						return true;
					}
				} else if (maxPos.【axis[0]】 < src.【axis[0]】) {
					if (maxPos.【axis[0]】 < dst.【axis[0]】) return false;

					var a = (src.【axis[0]】-maxPos.【axis[0]】) / s2d.【axis[0]】;
					var b = src.【axis[1]】 - s2d.【axis[1]】*a;
					var c = src.【axis[2]】 - s2d.【axis[2]】*a;
					if ( minPos.【axis[1]】<=b && b<=maxPos.【axis[1]】 && minPos.【axis[2]】<=c && c<=maxPos.【axis[2]】 ) {
		//# 		if (i==1) {
		//# 			if (j==0) {
							normal = float3(1,0,0);
							dst = float3(maxPos.【axis[0]】, b, c);
		//# 			} else if (j==1) {
							normal = float3(0,1,0);
							dst = float3(b, maxPos.【axis[0]】, c);
		//# 			} else {
							normal = float3(0,0,1);
							dst = float3(b, c, maxPos.【axis[0]】);
		//# 			}
		//# 		}
						return true;
					}
				}
			}

		//# 	}
			return false;
		}

		//# }


		// --------------------------------- private / protected メンバ -------------------------------

		
		// --------------------------------------------------------------------------------------------
	}


}

#endif	//#Ignore
