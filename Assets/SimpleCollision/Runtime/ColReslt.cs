using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {

	/** 衝突判定結果 */
	public struct ColResult {
		/** 衝突しているか否か。これがfalseの場合は、他のパラメータに有効な値は入らない */
		public bool isHit;

		/**
		 * 衝突点。衝突対象がA,Bの場合、B上の点。
		 * この値からでも、衝突深度の半分をnormalに乗算することで、A上の点を算出することもできる。
		 */
		public float3 pos;

		/** 衝突法線。衝突対象がA,Bの場合、BからAへ向かう方向。 */
		public float3 normal;

		/** 衝突深度。引き離し深度でもある */
		public float depth;

		public ColResult(float3 pos, float3 normal, float depth) {
			isHit = true;
			this.pos = pos;
			this.normal = normal;
			this.depth = depth;
		}
		static public ColResult operator-(ColResult a) {
			a.pos -= a.normal * a.depth;
			a.normal = -a.normal;
			return a;
		}
	}


}

