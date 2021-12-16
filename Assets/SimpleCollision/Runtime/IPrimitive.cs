using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {

	/** プリミティブ形状のインターフェイス */
	public interface IPrimitive {
	}

	public partial struct PrimitiveWrapper
	{
		/** プリミティブの中心位置 */
		public float3 pos {
			[MI(MO.AggressiveInlining)]
			// どんなプリミティブでもかならず最初にposが来る。
			// したがってこのアクセス方法で問題なく全プリミティブのposにアクセスできる
			get => point.pos;
		}
		
	}

}

