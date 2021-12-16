#if UNITY_EDITOR
using UnityEditor;
#endif

using System;
using System.Linq;
using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;



namespace SimpleCollision {
[ExecuteInEditMode]
public sealed class CollisionDebugger : MonoBehaviour
{
#if UNITY_EDITOR
	// ----------------------------- インスペクタに公開しているフィールド -------------------------

	[SerializeField] PrimitiveWrapper.ShapeType _shapeType =
		PrimitiveWrapper.ShapeType.Point;

	[Range(0,15)][SerializeField] int innerShape = 0;

	// box等用設定
	[Range(0,3)][SerializeField] float _w = 0.1f;

	// triangle用設定
	[SerializeField] float3 _vert0 = float3(1,0,0);
	[SerializeField] float3 _vert1 = float3(1,0,0);
	[SerializeField] float3 _vert2 = float3(1,0,0);

	// convexVerts用設定
	[SerializeField] Mesh _mesh = null;


	// ------------------------------------- public メンバ ----------------------------------------

	public float3 pos => transform.position;
	public quaternion rot => transform.rotation;
	public float4 r => float4(transform.localScale, _w);

	public Point point => new Point(pos);
	public Line line => new Line( pos, mul(rot, float3(0,0,1)) );
	public HalfSpace halfSpace => new HalfSpace(pos, mul(rot, float3(0,0,1)) );
	public Sphere sphere => new Sphere(pos, r.x);
	public Capsule capsule => new Capsule(pos, mul(rot,float3(0,0,1)), r.x, r.z);
	public Edge edge => new Edge(pos, mul(rot,float3(0,0,1)), r.x, r.z);
	public FaceQuad faceQuad => new FaceQuad(pos, float3x3(rot), r.xy, r.z);
	public Quad quad => new Quad(pos, float3x3(rot), r.xy, r.z);
	public Triangle triangle => new Triangle(_vert0, _vert1, _vert2, pos, float3x3(rot), r.w);
	public Box box => new Box(pos, float3x3(rot), r.xyz, r.w);
	public Circle circle => new Circle(pos, mul(rot,float3(0,0,1)), r.x, r.z);
	public Cylinder cylinder => new Cylinder(pos, mul(rot,float3(0,0,1)), r.x, r.z, r.w);
	public Cone cone => new Cone(pos, mul(rot,float3(0,0,1)), r.x, r.z, r.w);
	public ConvexVerts convexVerts {get{
		if (!_meshVerts.IsCreated && _mesh!=null) {
			var v = _mesh.vertices.Select(i=>(float3)i).ToArray();
			_meshVerts = new NativeArray<float3>(v, Allocator.Persistent);
		}
		unsafe {
			var vertsPtr = (float3*)_meshVerts.GetUnsafeReadOnlyPtr();
			return new ConvexVerts(vertsPtr, _meshVerts.Length, pos, float3x3(rot), r.w);
		}
	}}
	public PrimitiveWrapper primitiveWrapper {get{
		switch (_shapeType) {
		case PrimitiveWrapper.ShapeType.Point:		return new PrimitiveWrapper(point);
		case PrimitiveWrapper.ShapeType.Line:		return new PrimitiveWrapper(line);
		case PrimitiveWrapper.ShapeType.HalfSpace:	return new PrimitiveWrapper(halfSpace);
		case PrimitiveWrapper.ShapeType.Sphere:		return new PrimitiveWrapper(sphere);
		case PrimitiveWrapper.ShapeType.Capsule:
//			if (innerShape == 1) return new PrimitiveWrapper((Edge)capsule);
			if (innerShape == 2) return new PrimitiveWrapper(capsule.capSphere(1));
			if (innerShape == 3) return new PrimitiveWrapper(capsule.capSphere(-1));
			if (innerShape == 4) return new PrimitiveWrapper((Line)capsule);
			return new PrimitiveWrapper(capsule);
//		case PrimitiveWrapper.ShapeType.Edge:
//			if (innerShape == 1) return new PrimitiveWrapper((Line)edge);
//			return new PrimitiveWrapper(edge);
//		case PrimitiveWrapper.ShapeType.FaceQuad:
//			if (innerShape == 1) return new PrimitiveWrapper((HalfSpace)faceQuad);
//			return new PrimitiveWrapper(faceQuad);
		case PrimitiveWrapper.ShapeType.Quad:
			quad.getEdges(out var e0, out var e1, out var e2, out var e3);
			quad.getVerts(out var v0, out var v1, out var v2, out var v3);
//			if (innerShape == 1) return new PrimitiveWrapper(quad.getFace(1));
//			if (innerShape == 2) return new PrimitiveWrapper(quad.getFace(-1));
//			if (innerShape == 3) return new PrimitiveWrapper(e0);
//			if (innerShape == 4) return new PrimitiveWrapper(e1);
//			if (innerShape == 5) return new PrimitiveWrapper(e2);
//			if (innerShape == 6) return new PrimitiveWrapper(e3);
			if (innerShape == 7) return new PrimitiveWrapper(v0);
			if (innerShape == 8) return new PrimitiveWrapper(v1);
			if (innerShape == 9) return new PrimitiveWrapper(v2);
			if (innerShape == 10) return new PrimitiveWrapper(v3);
			return new PrimitiveWrapper(quad);
		case PrimitiveWrapper.ShapeType.Triangle:
			return new PrimitiveWrapper(triangle);
		case PrimitiveWrapper.ShapeType.Box:
			return new PrimitiveWrapper(box);
		case PrimitiveWrapper.ShapeType.Circle:
			return new PrimitiveWrapper(circle);
		case PrimitiveWrapper.ShapeType.Cylinder:
			return new PrimitiveWrapper(cylinder);
		case PrimitiveWrapper.ShapeType.Cone:
			return new PrimitiveWrapper(cone);
		case PrimitiveWrapper.ShapeType.ConvexVerts:
			return new PrimitiveWrapper(convexVerts);
		default : throw new InvalidProgramException();
		}
	}}


	// --------------------------------- private / protected メンバ -------------------------------

	MeshBuilderBase.Handle<MeshBuilder_Premitive> _meshBuilderHdl = null;
	NativeArray<float3> _meshVerts;

	void OnEnable() {
		_meshBuilderHdl = MeshBuilder_Premitive.generate(gameObject);
		Update();
	}

	void OnDisable() {
		if (_meshBuilderHdl != null) {
			_meshBuilderHdl.Dispose();
			_meshBuilderHdl = null;
		}
		if (_meshVerts.IsCreated) _meshVerts.Dispose();
	}

	void Update() {
		if (_meshBuilderHdl != null) {

			switch (_shapeType) {
			case PrimitiveWrapper.ShapeType.Point:
				_meshBuilderHdl.Parent.clear();
				break;
			case PrimitiveWrapper.ShapeType.Line:
				_meshBuilderHdl.Parent.clear();
				break;
			case PrimitiveWrapper.ShapeType.HalfSpace:
				_meshBuilderHdl.Parent.build_FaceQuad(100, 0);
				break;
			case PrimitiveWrapper.ShapeType.Sphere:
				_meshBuilderHdl.Parent.build_Sphere(r.x);
				break;
			case PrimitiveWrapper.ShapeType.Capsule:
				_meshBuilderHdl.Parent.build_Capsule(r.xz);
				break;
//			case PrimitiveWrapper.ShapeType.Edge:
//				_meshBuilderHdl.Parent.build_Edge(r.xz);
//				break;
//			case PrimitiveWrapper.ShapeType.FaceQuad:
//				_meshBuilderHdl.Parent.build_FaceQuad(r.xy, r.z);
//				break;
			case PrimitiveWrapper.ShapeType.Quad:
				_meshBuilderHdl.Parent.build_Quad(r.xy, r.z);
				break;
			case PrimitiveWrapper.ShapeType.Triangle:
				_meshBuilderHdl.Parent.build_Triangle(_vert0,_vert1,_vert2, r.w);
				break;
			case PrimitiveWrapper.ShapeType.Box:
				_meshBuilderHdl.Parent.build_Box(r.xyz, r.w);
				break;
			case PrimitiveWrapper.ShapeType.Circle:
				_meshBuilderHdl.Parent.build_Circle(r.x, r.z);
				break;
			case PrimitiveWrapper.ShapeType.Cylinder:
				_meshBuilderHdl.Parent.build_Cylinder(r.xz, r.w);
				break;
			case PrimitiveWrapper.ShapeType.Cone:
				_meshBuilderHdl.Parent.build_Cone(r.xz, r.w);
				break;
			case PrimitiveWrapper.ShapeType.ConvexVerts:
				_meshBuilderHdl.Parent.clear();
				break;
			default : throw new InvalidProgramException();
			}
		}
	}

	bool isValidNum(float3 a) =>
		!float.IsNaN(a.x) && !float.IsNaN(a.y) && !float.IsNaN(a.z) &&
		!float.IsInfinity(a.x) && !float.IsInfinity(a.y) && !float.IsInfinity(a.z);

	void OnDrawGizmosSelected()
	{
		var dbgLst = FindObjectsOfType<CollisionDebugger>();

		var hitCol = new Color(1,0,0,0.5f);
		var nohitCol = new Color(1,1,0,0.5f);
//		var errCol = new Color(1,0,1,1);

		// 他のCollisionとのあたり判定
		var isHitAny = false;
		foreach (var i in dbgLst) {
			if (i==this) continue;

			var clsn = PrimitiveWrapper.collision(primitiveWrapper, i.primitiveWrapper);

			if (clsn.isHit) {
				if (!isValidNum(clsn.pos) || !isValidNum(clsn.normal) || !isValidNum(clsn.depth))
					throw new SystemException();
				Handles.Label(
					i.pos,
					""+clsn.depth
				);
				Gizmos.color = new Color(0,1,0,1);
				Gizmos.DrawLine(clsn.pos, clsn.pos-clsn.normal*clsn.depth);
				Gizmos.DrawWireSphere(clsn.pos, HandleUtility.GetHandleSize(clsn.pos)*0.1f);
			}

			Gizmos.color = clsn.isHit ? hitCol : nohitCol;
			GizmosDrawer.draw( i.primitiveWrapper, 0 );
			if (clsn.isHit) {
				Gizmos.color = new Color(0,1,1,0.5f);
				GizmosDrawer.draw( i.primitiveWrapper, clsn.normal * -clsn.depth );
			}
			isHitAny |= clsn.isHit;
		}

		// 自身の形状を表示
		Gizmos.color = isHitAny ? hitCol : nohitCol;
		GizmosDrawer.draw( primitiveWrapper, 0 );
	}


	// --------------------------------------------------------------------------------------------
#endif
}
}

