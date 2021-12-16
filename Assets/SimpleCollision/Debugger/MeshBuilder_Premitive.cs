#if UNITY_EDITOR
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {
[ExecuteInEditMode]
public sealed class MeshBuilder_Premitive : MeshBuilderBase
{
	// ------------------------------------- public メンバ ----------------------------------------

	static public Handle<MeshBuilder_Premitive> generate(GameObject parent) {
		return generateCore<MeshBuilder_Premitive>(parent);
	}

	public void build_Sphere(float r) {
		clear();

		addSphere(0, r, Unity.Mathematics.quaternion.identity);

		buildCore();
	}

	public void build_Capsule(float2 r) {
		clear();

		addSphere(float3(0,0, r.y), r.x, Unity.Mathematics.quaternion.identity);
		addSphere(float3(0,0,-r.y), r.x, Unity.Mathematics.quaternion.identity);
		addCylinder(0, r.x, r.y, Unity.Mathematics.quaternion.Euler(PI/2,0,0));

		buildCore();
	}

	public void build_Edge(float2 r) {
		clear();

		addCylinder(0, r.x, r.y, Unity.Mathematics.quaternion.Euler(PI/2,0,0));

		buildCore();
	}

	public void build_FaceQuad(float2 r, float w) {
		clear();

		addQuad(float3(0,0,w), r, Unity.Mathematics.quaternion.Euler(PI/2,0,0));

		buildCore();
	}

	public void build_Quad(float2 r, float w) {
		clear();

		addQuad(float3(0,0,w), r, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addQuad(float3(0,0,-w), r, Unity.Mathematics.quaternion.Euler(-PI/2,0,0));
		addCylinder(float3(r.x,0,0),  w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(-r.x,0,0), w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(0,r.y,0),  w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addCylinder(float3(0,-r.y,0), w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addSphere(float3(-r.x,-r.y,0), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x,-r.y,0), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3(-r.x, r.y,0), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x, r.y,0), w, Unity.Mathematics.quaternion.identity);

		buildCore();
	}

	public void build_Triangle(float3 pos0, float3 pos1, float3 pos2, float w) {
		quaternion calcDirRotation(float3 src, float3 dst) {
			src = normalizesafe(src);
			dst = normalizesafe(dst);
			var axis = cross( src, dst );
			var agl = atan2( length(axis), dot(src,dst) );
			return Unity.Mathematics.quaternion.AxisAngle(normalizesafe(axis), agl);
		}
		clear();

		var nml = normalizesafe(cross(pos1-pos0, pos2-pos0));
		addTri(pos0, pos1, pos2, nml* w, Unity.Mathematics.quaternion.identity);
		addTri(pos0, pos1, pos2, nml*-w, Unity.Mathematics.quaternion.identity, true);
		addCylinder(
			(pos0+pos1)/2,
			w, length(pos0-pos1)/2,
			calcDirRotation(float3(0,1,0), pos0-pos1)
		);
		addCylinder(
			(pos1+pos2)/2,
			w, length(pos1-pos2)/2,
			calcDirRotation(float3(0,1,0), pos1-pos2)
		);
		addCylinder(
			(pos2+pos0)/2,
			w, length(pos2-pos0)/2,
			calcDirRotation(float3(0,1,0), pos2-pos0)
		);
		addSphere(pos0, w, Unity.Mathematics.quaternion.identity);
		addSphere(pos1, w, Unity.Mathematics.quaternion.identity);
		addSphere(pos2, w, Unity.Mathematics.quaternion.identity);

		buildCore();
	}

	public void build_Box(float3 r, float w) {
		clear();

		addQuad(float3(0,0, r.z+w), r.xy, Unity.Mathematics.quaternion.Euler( PI/2,0,0));
		addQuad(float3(0,0,-r.z-w), r.xy, Unity.Mathematics.quaternion.Euler(-PI/2,0,0));
		addQuad(float3(0, r.y+w,0), r.xz, Unity.Mathematics.quaternion.Euler( 0,0,0));
		addQuad(float3(0,-r.y-w,0), r.xz, Unity.Mathematics.quaternion.Euler(PI,0,0));
		addQuad(float3( r.x+w,0,0), r.zy, Unity.Mathematics.quaternion.Euler( PI/2,PI/2,0));
		addQuad(float3(-r.x-w,0,0), r.zy, Unity.Mathematics.quaternion.Euler(-PI/2,PI/2,0));
		addCylinder(float3( r.x,0,r.z),  w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(-r.x,0,r.z),  w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(0, r.y,r.z),  w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addCylinder(float3(0,-r.y,r.z),  w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addCylinder(float3( r.x,0,-r.z), w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(-r.x,0,-r.z), w, r.y, Unity.Mathematics.quaternion.Euler(0,0,0));
		addCylinder(float3(0, r.y,-r.z), w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addCylinder(float3(0,-r.y,-r.z), w, r.x, Unity.Mathematics.quaternion.Euler(0,0,PI/2));
		addCylinder(float3( r.x, r.y,0), w, r.z, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addCylinder(float3(-r.x, r.y,0), w, r.z, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addCylinder(float3( r.x,-r.y,0), w, r.z, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addCylinder(float3(-r.x,-r.y,0), w, r.z, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addSphere(float3(-r.x,-r.y,-r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x,-r.y,-r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3(-r.x, r.y,-r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x, r.y,-r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3(-r.x,-r.y, r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x,-r.y, r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3(-r.x, r.y, r.z), w, Unity.Mathematics.quaternion.identity);
		addSphere(float3( r.x, r.y, r.z), w, Unity.Mathematics.quaternion.identity);

		buildCore();
	}

	public void build_Circle(float r, float w) {
		clear();

		addCircle(float3(0,0, w), r, Unity.Mathematics.quaternion.Euler( PI/2,0,0));
		addCircle(float3(0,0,-w), r, Unity.Mathematics.quaternion.Euler(-PI/2,0,0));
		addTorus(0, r, w, Unity.Mathematics.quaternion.Euler( PI/2,0,0));

		buildCore();
	}

	public void build_Cylinder(float2 r, float w) {
		clear();

		addCircle(float3(0,0, r.y+w), r.x, Unity.Mathematics.quaternion.Euler( PI/2,0,0));
		addCircle(float3(0,0,-r.y-w), r.x, Unity.Mathematics.quaternion.Euler(-PI/2,0,0));
		addTorus(float3(0,0, r.y), r.x, w, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addTorus(float3(0,0,-r.y), r.x, w, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addCylinder(0, r.x+w, r.y, Unity.Mathematics.quaternion.Euler(PI/2,0,0));

		buildCore();
	}

	public void build_Cone(float2 r, float w) {
		clear();

		var t = normalize(float2(r.x,r.y*2));
		addCircle(float3(0,0,-r.y-w), r.x, Unity.Mathematics.quaternion.Euler(-PI/2,0,0));
		addTorus(float3(0,0,-r.y), r.x, w, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addCylinder(float3(0,0,t.x*w), float2(r.x+t.y*w,t.y*w), r.y, Unity.Mathematics.quaternion.Euler(PI/2,0,0));
		addSphere(float3(0,0,r.y), w, Unity.Mathematics.quaternion.identity);

		buildCore();
	}



	// --------------------------------- private / protected メンバ -------------------------------


	// --------------------------------------------------------------------------------------------
}
}


#endif