using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {

	/** 点 */
	public struct Point : IPrimitive {
		public float3 pos;		//!< 位置

		public Point(float3 pos) {
			this.pos = pos;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Point translate(float3 pos, float3x3 rot) =>
			new Point{pos = this.pos+pos};

		/** 点は利便性のためfloat3から直接変換できるようにしておく */
		static public implicit operator Point(float3 src) => new Point(src);
	}

	/** 直線 */
	public struct Line : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 方向

		public Line(float3 pos, float3 dir) {
			this.pos = pos; this.dir = dir;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Line translate(float3 pos, float3x3 rot) => new Line{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
		};
	}

	/** 半空間 */
	public struct HalfSpace : IPrimitive {
		public float3 pos;		//!< 境界面上の一点
		public float3 dir;		//!< 境界面方向。この方向の空間のみが存在可能な状態となる。

		public HalfSpace(float3 pos, float3 dir) {
			this.pos = pos; this.dir = dir;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public HalfSpace translate(float3 pos, float3x3 rot) => new HalfSpace{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
		};
	}

	/** 球 */
	public struct Sphere : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float r;			//!< 半径

		public Sphere(float3 pos, float r) {
			this.pos = pos; this.r = r;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Sphere translate(float3 pos, float3x3 rot) => new Sphere{
			pos = this.pos + pos,
			r = r,
		};
	}

	/** カプセル */
	public struct Capsule : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 縦方向の向き
		public float r_s;		//!< 横方向の半径
		public float r_h;		//!< 縦方向の長さの半分(r_s部分は含まない)

		public Capsule(float3 pos, float3 dir, float r_s, float r_h) {
			this.pos = pos; this.dir = dir;
			this.r_s = r_s; this.r_h = r_h;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Capsule translate(float3 pos, float3x3 rot) => new Capsule{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
			r_s = r_s,
			r_h = r_h,
		};

		/** 軸部分をLineでも取得できるようにする */
		static public explicit operator Line(in Capsule src) =>
			new Line(src.pos, src.dir);
		/** 筒部分をEdgeでも取得できるようにする */
		static public explicit operator Edge(in Capsule src) =>
			new Edge(src.pos, src.dir, src.r_s, src.r_h);
	}

	/** 辺要素。シリンダー側面部分のみの形状。カプセルとはr_hの値が異なるので注意 */
	public struct Edge : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 縦方向の向き
		public float r_s;		//!< 横方向の半径
		public float r_h;		//!< 縦方向の長さの半分

		public Edge(float3 pos, float3 dir, float r_s, float r_h) {
			this.pos = pos; this.dir = dir;
			this.r_s = r_s; this.r_h = r_h;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Edge translate(float3 pos, float3x3 rot) => new Edge{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
			r_s = r_s,
			r_h = r_h,
		};

		/** 軸部分をLineでも取得できるようにする */
		static public explicit operator Line(in Edge src) =>
			new Line(src.pos, src.dir);
	}

	/** 四角面要素。長方形の片面のみで、辺部分を除いた面部分のみの形状。+Z方向のみ衝突する */
	public struct FaceQuad : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3x3 rot;	//!< 回転行列
		public float2 r;		//!< X,Y方向の半径
		public float w;			//!< 厚み

		public FaceQuad(float3 pos, float3x3 rot, float2 r, float w) {
			this.pos = pos; this.rot = rot;
			this.r = r; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public FaceQuad translate(float3 pos, float3x3 rot) => new FaceQuad{
			pos = this.pos + pos,
			rot = mul( rot, this.rot ),
			r = r,
			w = w,
		};

		/** 自身を含んだ半空間で表す（厚みは考慮しない） */
		static public explicit operator HalfSpace(in FaceQuad src) =>
			new HalfSpace(src.pos, src.rot.c2);
	}

	/** 長方形。Z方向が面の向き */
	public struct Quad : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3x3 rot;	//!< 回転行列
		public float2 r;		//!< 半径
		public float w;			//!< 厚み

		public Quad(float3 pos, float3x3 rot, float2 r, float w) {
			this.pos = pos; this.rot = rot;
			this.r = r; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Quad translate(float3 pos, float3x3 rot) => new Quad{
			pos = this.pos + pos,
			rot = mul( rot, this.rot ),
			r = r,
			w = w,
		};
	}

	/** 三角形。左手系なので、pos0,1,2の順に左ネジの方向が面の向き */
	public struct Triangle : IPrimitive {
		public float3 pos;					//!< 中心位置
		public float3x3 rot;				//!< 回転行列
		public float3 vert0, vert1, vert2;	//!< 頂点位置。中心からの相対位置
		public float w;						//!< 厚み

		public Triangle(
			float3 vert0, float3 vert1, float3 vert2,
			float3 pos, float3x3 rot, float w
		) {
			this.vert0 = vert0; this.vert1 = vert1; this.vert2 = vert2;
			this.pos = pos; this.rot = rot; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Triangle translate(float3 pos, float3x3 rot) {
			var ret = this;
			ret.pos += pos;
			ret.rot = mul( rot, ret.rot );
			return ret;
		}
	}

	/** 直方体 */
	public struct Box : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3x3 rot;	//!< 回転行列
		public float3 r;		//!< X,Y,Z方向の半径
		public float w;			//!< 厚み

		public Box(float3 pos, float3x3 rot, float3 r, float w) {
			this.pos = pos; this.rot = rot;
			this.r = r; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Box translate(float3 pos, float3x3 rot) => new Box{
			pos = this.pos + pos,
			rot = mul( rot, this.rot ),
			r = r,
			w = w,
		};
	}
	
	/** 円盤 */
	public struct Circle : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 縦方向の向き
		public float r;			//!< 半径
		public float w;			//!< 厚み

		public Circle(float3 pos, float3 dir, float r, float w) {
			this.pos = pos; this.dir = dir;
			this.r = r; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Circle translate(float3 pos, float3x3 rot) => new Circle{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
			r = r,
			w = w,
		};
	}
	
	/** 円柱 */
	public struct Cylinder : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 縦方向の向き
		public float r_s;		//!< 横方向の半径
		public float r_h;		//!< 縦方向の長さの半分
		public float w;			//!< 厚み

		public Cylinder(float3 pos, float3 dir, float r_s, float r_h, float w) {
			this.pos = pos; this.dir = dir;
			this.r_s = r_s; this.r_h = r_h; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Cylinder translate(float3 pos, float3x3 rot) => new Cylinder{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
			r_s = r_s,
			r_h = r_h,
			w = w,
		};

		/** 軸部分をLineでも取得できるようにする */
		static public explicit operator Line(in Cylinder src) =>
			new Line(src.pos, src.dir);
		/** 軸部分をEdgeでも取得できるようにする */
		static public explicit operator Edge(in Cylinder src) =>
			new Edge(src.pos, src.dir, src.r_s+src.w, src.r_h);
	}
	
	/** 円錐 */
	public struct Cone : IPrimitive {
		public float3 pos;		//!< 中央位置
		public float3 dir;		//!< 縦方向の向き
		public float r_s;		//!< 横方向の半径
		public float r_h;		//!< 縦方向の長さの半分
		public float w;			//!< 厚み

		public Cone(float3 pos, float3 dir, float r_s, float r_h, float w) {
			this.pos = pos; this.dir = dir;
			this.r_s = r_s; this.r_h = r_h; this.w = w;
		}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public Cone translate(float3 pos, float3x3 rot) => new Cone{
			pos = this.pos + pos,
			dir = mul( rot, dir ),
			r_s = r_s,
			r_h = r_h,
			w = w,
		};

		/** 軸部分をLineでも取得できるようにする */
		static public explicit operator Line(in Cone src) =>
			new Line(src.pos, src.dir);
	}
	
	/** 凸型多面体。重複しない頂点配列のみを持つ */
	public struct ConvexVerts : IPrimitive {
		public float3 pos;			//!< 位置
		public float3x3 rot;		//!< 回転行列
		public IntPtr vertsPtr;		//!< 頂点配列のポインタ。float3の配列
		public int vertsCnt;		//!< 頂点配列の個数
		public float w;				//!< 厚み
		public float boundaryR;		//!< 厚みを除いたバウンダリー球半径。計算簡略化のために入れておくこと

		unsafe public ConvexVerts(IntPtr vertsPtr, int vertsCnt, float3 pos, float3x3 rot, float w, float boundaryR) {
			this.vertsPtr = vertsPtr; this.vertsCnt = vertsCnt;
			this.pos = pos; this.rot = rot; this.w = w;
			this.boundaryR = boundaryR;
		}
		unsafe public ConvexVerts(float3* vertsPtr, int vertsCnt, float3 pos, float3x3 rot, float w, float boundaryR) 
			: this((IntPtr)vertsPtr, vertsCnt, pos, rot, w, boundaryR) {}
		public ConvexVerts(IntPtr vertsPtr, int vertsCnt, float3 pos, float3x3 rot, float w) {
			this.vertsPtr = vertsPtr; this.vertsCnt = vertsCnt;
			this.pos = pos; this.rot = rot; this.w = w;

			// バウンダリー半径を自動計算
			boundaryR = 0;
			unsafe {
				var vp = (float3*)vertsPtr;
				for (int i=0; i<vertsCnt; ++i)
					boundaryR = max( boundaryR, length(vp[i]) );
			}
		}
		unsafe public ConvexVerts(float3* vertsPtr, int vertsCnt, float3 pos, float3x3 rot, float w)
			: this((IntPtr)vertsPtr, vertsCnt, pos, rot, w) {}

		/** 指定の移動・回転を行った結果を得る */
		[MI(MO.AggressiveInlining)]
		public ConvexVerts translate(float3 pos, float3x3 rot) {
			var ret = this;
			ret.pos += pos;
			ret.rot = mul( rot, ret.rot );
			return ret;
		}
	}
	

}

