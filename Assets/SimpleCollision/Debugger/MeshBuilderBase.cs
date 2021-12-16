#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {
[ExecuteInEditMode]
public abstract class MeshBuilderBase : MonoBehaviour
{
	// ------------------------------------- public メンバ ----------------------------------------

	public sealed class Handle<T> : IDisposable
	where T : MeshBuilderBase {
		public T Parent {get; private set;}
		public Handle(T parent) {
			Parent = parent;
		}
		public void Dispose() {
			if (Parent != null && Parent.gameObject != null)
				DestroyImmediate( Parent.gameObject );
			Parent = null;
		}
	}

	public void clear() {
		_poss.Clear();
		_tris.Clear();

		_idxBase = 0;
		_rot = Unity.Mathematics.quaternion.identity;
		_posBase = 0;

		_mesh.Clear();
		_meshFilter.sharedMesh = _mesh;
	}


	// --------------------------------- private / protected メンバ -------------------------------

	const int CIRCLE_DIV_CNT = 40;

	Mesh _mesh;
	MeshFilter _meshFilter;
	MeshRenderer _meshRenderer;

	List<Vector3> _poss = new List<Vector3>();
	List<int> _tris = new List<int>();

	protected int _idxBase = 0;
	protected quaternion _rot = Unity.Mathematics.quaternion.identity;
	protected float3 _posBase = 0;

	protected static Handle<T> generateCore<T>(GameObject go)
	where T : MeshBuilderBase {
		var obj = new GameObject();
		obj.transform.SetParent( go.transform, false );
		obj.hideFlags = HideFlags.HideAndDontSave;

		var ret = obj.AddComponent<T>();
		ret.hideFlags = HideFlags.HideAndDontSave;
		ret._mesh = new Mesh();

		ret._meshFilter   = obj.AddComponent<MeshFilter>();
		ret._meshRenderer = obj.AddComponent<MeshRenderer>();
		ret._meshFilter.hideFlags   = HideFlags.HideAndDontSave;
		ret._meshRenderer.hideFlags = HideFlags.HideAndDontSave;
		ret._meshRenderer.sharedMaterial =
			UnityEditor.AssetDatabase.LoadAssetAtPath<Material>(
				"Packages/com.unity.render-pipelines.universal/Runtime/Materials/Lit.mat"
			);

		ret._meshFilter.sharedMesh = ret._mesh;
		return new Handle<T>(ret);
	}

	protected void buildCore() {
		_mesh.SetVertices(_poss);
		_mesh.SetTriangles(_tris, 0);
		_mesh.RecalculateNormals();
		_meshFilter.sharedMesh = _mesh;
		transform.localScale = 1/max(abs(transform.parent.localScale),0.00001f);
	}

	protected void addVert(float3 pos) {
		_poss.Add( _posBase + mul(_rot, pos) );
	}

	protected void addTri(int idxOfs0, int idxOfs1, int idxOfs2) {
		_tris.Add(_idxBase + idxOfs0);
		_tris.Add(_idxBase + idxOfs1);
		_tris.Add(_idxBase + idxOfs2);
	}

	protected void addSphere(float3 pos, float r, quaternion rot) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		addVert(float3( 0, -r, 0 ));
		addVert(float3( 0, r, 0 ));
		for (int i=0; i<CIRCLE_DIV_CNT; ++i)
		for (int j=1; j<CIRCLE_DIV_CNT/2; ++j) {
			var theta = PI*2 * i/CIRCLE_DIV_CNT;
			var phi = PI * j/CIRCLE_DIV_CNT*2;

			var (cosT, sinT) = (cos(theta), sin(theta));
			var (cosP, sinP) = (cos(phi), sin(phi));

			addVert( r * float3(cosT*sinP, cosP, sinT*sinP) );
		}

		for (int i=0; i<CIRCLE_DIV_CNT; ++i) {
			int idx0 = 2 + i * (CIRCLE_DIV_CNT/2-1);
			int idx1 = 2 + ((i+1)%CIRCLE_DIV_CNT) * (CIRCLE_DIV_CNT/2-1);
			addTri(1, idx1, idx0);
			addTri(0, idx0+CIRCLE_DIV_CNT/2-2, idx1+CIRCLE_DIV_CNT/2-2);
			for (int j=0; j<CIRCLE_DIV_CNT/2-2; ++j) {
				addTri(idx0+j+0, idx1+j+0, idx0+j+1);
				addTri(idx0+j+1, idx1+j+0, idx1+j+1);
			}
		}
	}

	protected void addQuad(float3 pos, float2 r, quaternion rot) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		addVert( float3(-r.x, 0, -r.y) );
		addVert( float3( r.x, 0, -r.y) );
		addVert( float3(-r.x, 0,  r.y) );
		addVert( float3( r.x, 0,  r.y) );
		addTri(0, 2, 1);
		addTri(2, 3, 1);
	}

	protected void addTri(float3 pos0, float3 pos1, float3 pos2, float3 pos, quaternion rot, bool flipFace=false) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		addVert( pos0 );
		addVert( pos1 );
		addVert( pos2 );
		if (flipFace)	addTri(0, 2, 1);
		else			addTri(0, 1, 2);
	}

	protected void addCylinder(float3 pos, float2 r_s, float r_h, quaternion rot) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		for (int i=0; i<CIRCLE_DIV_CNT; ++i) {

			int idx0 = i * 2;
			int idx1 = ((i+1)%CIRCLE_DIV_CNT) * 2;

			var theta = PI*2 * i/CIRCLE_DIV_CNT;
			var (cosT, sinT) = (cos(theta), sin(theta));

			addVert( float3(r_s.x*cosT, -r_h, r_s.x*sinT) );
			addVert( float3(r_s.y*cosT,  r_h, r_s.y*sinT) );
			addTri(idx0,   idx0+1, idx1);
			addTri(idx0+1, idx1+1, idx1);
		}
	}

	protected void addCircle(float3 pos, float r, quaternion rot) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		addVert( 0 );
		for (int i=0; i<CIRCLE_DIV_CNT; ++i) {

			int idx0 = 1 + i;
			int idx1 = 1 + (i+1)%CIRCLE_DIV_CNT;

			var theta = PI*2 * i/CIRCLE_DIV_CNT;
			var (cosT, sinT) = (cos(theta), sin(theta));

			addVert( float3(r*cosT, 0, r*sinT) );
			addTri(idx0, 0, idx1);
		}
	}

	protected void addTorus(float3 pos, float r, float w, quaternion rot) {
		_posBase = pos;
		_rot = rot;
		_idxBase = _poss.Count;

		for (int i=0; i<CIRCLE_DIV_CNT; ++i) {

			int idx0 = i * CIRCLE_DIV_CNT;
			int idx1 = ((i+1)%CIRCLE_DIV_CNT) * CIRCLE_DIV_CNT;

			var theta = PI*2 * i/CIRCLE_DIV_CNT;
			var (cosT, sinT) = (cos(theta), sin(theta));

			for (int j=0; j<CIRCLE_DIV_CNT; ++j) {

				var phi = PI*2 * j/CIRCLE_DIV_CNT;
				var (cosP, sinP) = (cos(phi), sin(phi));

				var u = float3(cosT, 0, sinT);
				var v = float3(0, 1, 0);
				addVert( r*u + u*cosP*w + v*sinP*w );
				var k = (j + 1) % CIRCLE_DIV_CNT;
				addTri(idx0+j, idx0+k, idx1+j);
				addTri(idx0+k, idx1+k, idx1+j);
			}
		}
	}


	// --------------------------------------------------------------------------------------------
}
}


#endif