using System;
using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using static Unity.Mathematics.math;


namespace SimpleCollision {


	/** PrimitiveWrapperをインスペクタで扱えるようにシリアライズ可能にしたもの */
	[Serializable]
	public sealed class PrimitiveWrapperAuthoring {
		// ------------------------------------- public メンバ ----------------------------------------

		public interface IHandle : IDisposable {
			public PrimitiveWrapper body {get;}
		}

		public PrimitiveWrapper.ShapeType shapeType = default;
		public float4 r = 1;
		public float3[] verts = null;
		public float3 pos = 0;
		public quaternion rot = Unity.Mathematics.quaternion.identity;

		/** PrimitiveWrapper本体をアロケートする。不要になったらDisposeすること */
		public IHandle alloc() => new Handle(this);


		// --------------------------------- private / protected メンバ -------------------------------

		/** PrimitiveWrapper本体をアロケートしたハンドル */
		sealed class Handle : IHandle {
			public PrimitiveWrapper body {get; private set;}

			public Handle(PrimitiveWrapperAuthoring parent) {
				var dir = mul(parent.rot, float3(0,0,1));
				var rotMtx = new float3x3(parent.rot);
				switch (parent.shapeType) {
				case PrimitiveWrapper.ShapeType.Point:
					body = new PrimitiveWrapper(new Point(parent.pos));
					break;
				case PrimitiveWrapper.ShapeType.Line:
					body = new PrimitiveWrapper(new Line(parent.pos, dir));
					break;
				case PrimitiveWrapper.ShapeType.HalfSpace:
					body = new PrimitiveWrapper(new HalfSpace(parent.pos, dir));
					break;
				case PrimitiveWrapper.ShapeType.Sphere:
					body = new PrimitiveWrapper(new Sphere(parent.pos, parent.r.x));
					break;
				case PrimitiveWrapper.ShapeType.Capsule:
					body = new PrimitiveWrapper(
						new Capsule(parent.pos, dir, parent.r.x, parent.r.y)
					);
					break;
				case PrimitiveWrapper.ShapeType.Quad:
					body = new PrimitiveWrapper(
						new Quad(parent.pos, rotMtx, parent.r.xy, parent.r.z)
					);
					break;
				case PrimitiveWrapper.ShapeType.Triangle:
					body = new PrimitiveWrapper( new Triangle(
						parent.verts[0], parent.verts[1], parent.verts[2],
						parent.pos, rotMtx, parent.r.w
					) );
					break;
				case PrimitiveWrapper.ShapeType.Box:
					body = new PrimitiveWrapper(
						new Box(parent.pos, rotMtx, parent.r.xyz, parent.r.w)
					);
					break;
				case PrimitiveWrapper.ShapeType.Circle:
					body = new PrimitiveWrapper(
						new Circle(parent.pos, dir, parent.r.x, parent.r.z)
					);
					break;
				case PrimitiveWrapper.ShapeType.Cylinder:
					body = new PrimitiveWrapper(
						new Cylinder(parent.pos, dir, parent.r.x, parent.r.y, parent.r.w)
					);
					break;
				case PrimitiveWrapper.ShapeType.Cone:
					body = new PrimitiveWrapper(
						new Cone(parent.pos, dir, parent.r.x, parent.r.y, parent.r.w)
					);
					break;
				case PrimitiveWrapper.ShapeType.ConvexVerts:
					_vertsAry = new NativeArray<float3>(parent.verts, Allocator.Persistent);
					unsafe {
						var vertsPtr = (float3*)_vertsAry.GetUnsafeReadOnlyPtr();
						body = new PrimitiveWrapper(
							new ConvexVerts(
								vertsPtr,
								_vertsAry.Length,
								parent.pos,
								rotMtx,
								parent.r.w
							)
						);
					}
					break;
				default: throw new SystemException();
				}
			}
			public void Dispose() {
				Debug.Assert(!_isDisposed);
				_isDisposed = true;
				if (_vertsAry.IsCreated) _vertsAry.Dispose();
			}

			NativeArray<float3> _vertsAry;		//!< PrimitiveがConvexVertsだったときの頂点配列
			bool _isDisposed = false;

		#if UNITY_EDITOR
			~Handle() { if (!_isDisposed) throw new SystemException(); }
		#endif
		}


		// --------------------------------------------------------------------------------------------
	}

}

