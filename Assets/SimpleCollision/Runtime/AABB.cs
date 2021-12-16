using System;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {

	/** AABBを使用した衝突判定 */
	public partial struct AABB {
		// ------------------------------------- public メンバ ----------------------------------------

		public float3 minPos;
		public float3 size;

		public AABB( float3 minPos, float3 size ) {
			this.minPos = minPos;
			this.size = size;
		}


		// --------------------------------- private / protected メンバ -------------------------------

		
		// --------------------------------------------------------------------------------------------
	}


}

