using Unity.Mathematics;
using static Unity.Mathematics.math;
using Unity.Burst;

//using System.Runtime.CompilerServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 



namespace SimpleCollision {
static public partial class MPR {

	/** 計算イテレーション回数の最大値 */
	const int MaxIterations = 20;

	/**
	 * posをベクトルX、ベクトルY、ベクトルZの重心座標で表したときの成分x,y,zを求める。
	 * 計算の簡略化のため、法線ベクトルも計算して入れておく。
	 */
	static float3 getBarycentric(
		float3 pos,
		float3 vecX,
		float3 vecY,
		float3 vecZ
	) {
		var mtx = float3x3(vecX, vecY, vecZ);
		return mul( inverse(mtx), pos );
	}

}
}
