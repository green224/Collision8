using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {
public static partial class PrimitivesEx {

	// プリミティブのバウンダリー球半径の二乗を得る


	/** 球のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Sphere self) => self.r*self.r;

	/** カプセルのバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Capsule self) {
		var a = self.r_h + self.r_s;
		return a * a;
	}

	/** 辺要素のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Edge self) =>
		lengthsq( float2(self.r_h, self.r_s) );

	/** 四角面要素のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in FaceQuad self) =>
		lengthsq( float3( self.r, self.w ) );

	/** 長方形のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Quad self) =>
		lengthsq( self.r + self.w );

//	/** 三角形のバウンダリー球半径の二乗 */
//	[MI(MO.AggressiveInlining)]
//	static public float boundarySqR(this in Triangle self) {
//		var ret = lengthsq( abs(self.vert0) + self.w );
//		ret = max( ret, lengthsq( abs(self.vert1) + self.w ) );
//		ret = max( ret, lengthsq( abs(self.vert2) + self.w ) );
//		return ret;
//	}

	/** 直方体のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Box self) =>
		lengthsq( self.r + self.w );

	/** 円盤のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Circle self) =>
		(self.r + self.w) * (self.r + self.w);

	/** 円柱のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Cylinder self) =>
		lengthsq( float2(self.r_h, self.r_s) + self.w );

	/** 円錐のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in Cone self) =>
		lengthsq( float2(self.r_h, self.r_s) + self.w );

	/** 凸型多面体のバウンダリー球半径の二乗 */
	[MI(MO.AggressiveInlining)]
	static public float boundarySqR(this in ConvexVerts self) =>
		(self.boundaryR + self.w) * (self.boundaryR + self.w);

}
}

