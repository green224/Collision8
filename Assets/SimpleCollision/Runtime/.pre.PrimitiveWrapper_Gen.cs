//#CsPP
//#	public partial class Program {
//#
//#		static string[] shapeNames = new [] {
//#			"Point",
//#			"Line",
//#			"HalfSpace",
//#			"Sphere",
//#			"Capsule",
//#	//		"Edge",
//#	//		"FaceQuad",
//#			"Quad",
//#			"Triangle",
//#			"Box",
//#			"Circle",
//#			"Cylinder",
//#			"Cone",
//#			"ConvexVerts",
//#		};
//#		static string getArgType(string shapeName) {
//#			if (
//#				shapeName == "Point" ||
//#				shapeName == "Line" ||
//#				shapeName == "HalfSpace" ||
//#				shapeName == "Sphere"
//#			) return shapeName;
//#			return "in " + shapeName;
//#		}
//#		static string type2VarName(string a) => char.ToLower(a[0]) + a.Substring(1);
//#
//#		public static void Main() {

#if false	//#Ignore  IDE状でエラー表示させ無くするために無効にしておく

using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {

	/**
	 * Primitiveを種別関係なく1つの型でまとめて扱えるようにしたもの。
	 * 不要なオーバーヘッドが掛かるが、デバッグ用途などには便利。
	 */
	[StructLayout(LayoutKind.Explicit)]
	public partial struct PrimitiveWrapper {
		public enum ShapeType : int {
			//# foreach (var i in shapeNames) {
			【i】,
			//# }
		}
		[FieldOffset(0)] public ShapeType shapeType;

		//# foreach (var i in shapeNames) {
		[FieldOffset(8)] public 【i】 【type2VarName(i)】;
		//# }

		//# foreach (var i in shapeNames) {
		public PrimitiveWrapper(【i】 【type2VarName(i)】) {
			shapeType = ShapeType.【i】;
			//# foreach (var j in shapeNames) {
			//#		if (i==j) continue;
			【type2VarName(j)】 = default;
			//# }
			this.【type2VarName(i)】 = 【type2VarName(i)】;
		}
		//# }

		// これはボクシングを引き起こすので注意すること
		public IPrimitive BoxedValue {
			get {
				switch (shapeType) {
				//# foreach (var i in shapeNames) {
				case ShapeType.【i】 : return 【type2VarName(i)】;
				//# }
				}
				return null;
			}
			set {
				//# for (int i=0; i<shapeNames.Length; ++i) {
				//#		var sn = shapeNames[i];
				【(i==0?"if":"} else if")】 (value is 【sn】) {
					shapeType = ShapeType.【sn】;
					【type2VarName(sn)】 = (【sn】)value;
				//# }
				}
			}
		}

		/** 指定の移動・回転を行った結果を得る */
		public PrimitiveWrapper translate(float3 pos, float3x3 rot) {
			switch (shapeType) {
			//# foreach (var a in shapeNames) {
			case ShapeType.【a】 : return new PrimitiveWrapper(【type2VarName(a)】.translate(pos,rot));
			//# }
			}
			return default;
		}

		// 衝突判定
		static public ColResult collision(in PrimitiveWrapper a, in PrimitiveWrapper b) {
			switch (a.shapeType) {
			//# foreach (var a in shapeNames) {
			case ShapeType.【a】 : return collision(a.【type2VarName(a)】, b);
			//# }
			}
			return default;
		}
		//# foreach (var a in shapeNames) {
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, 【getArgType(a)】 b) => -collision(b, a);
		static public ColResult collision(【getArgType(a)】 a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			//# foreach (var b in shapeNames) {
			case ShapeType.【b】 : return Solver.collision(a, b.【type2VarName(b)】);
			//# }
			}
			return default;
		}
		//# }

		// バウンダリー判定
		static public bool checkBoundary(in PrimitiveWrapper a, in PrimitiveWrapper b) {
			switch (a.shapeType) {
			//# foreach (var a in shapeNames) {
			case ShapeType.【a】 : return checkBoundary(a.【type2VarName(a)】, b);
			//# }
			}
			return default;
		}
		//# foreach (var a in shapeNames) {
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, 【getArgType(a)】 b) => checkBoundary(b, a);
		static public bool checkBoundary(【getArgType(a)】 a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			//# foreach (var b in shapeNames) {
			case ShapeType.【b】 : return Solver.checkBoundary(a, b.【type2VarName(b)】);
			//# }
			}
			return default;
		}
		//# }
	}

}

#endif	//#Ignore

//# } }
