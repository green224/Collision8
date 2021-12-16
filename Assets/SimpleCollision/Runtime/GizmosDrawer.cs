#if UNITY_EDITOR
using UnityEditor;
using System;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;



namespace SimpleCollision {
public static class GizmosDrawer
{
	static public void draw(
		PrimitiveWrapper prim,
		float3 offsetPos
	) {
		switch (prim.shapeType) {
			case PrimitiveWrapper.ShapeType.Point:{
				var p = prim.point.pos + offsetPos;
				var t = 0.1f;
				Gizmos.DrawLine(p-float3(t,0,0), p+float3(t,0,0));
				Gizmos.DrawLine(p-float3(0,t,0), p+float3(0,t,0));
				Gizmos.DrawLine(p-float3(0,0,t), p+float3(0,0,t));
			break;}
			case PrimitiveWrapper.ShapeType.Line:{
				var a = prim.line;
				var p = a.pos + offsetPos;
				Gizmos.DrawLine(p-a.dir*10, p+a.dir*10);
			break;}
			case PrimitiveWrapper.ShapeType.HalfSpace:{
				var a = prim.halfSpace;
				Gizmos.DrawLine(a.pos, a.pos+a.dir*3);
				drawWireCircleGUI(
					a.pos + offsetPos, 3,
					float3x3( mul(
						Unity.Mathematics.quaternion.LookRotation( a.dir, float3(0,0,1) ),
						Unity.Mathematics.quaternion.Euler(PI/2, 0, 0)
					) ),
					0, 1
				);
			break;}
			case PrimitiveWrapper.ShapeType.Sphere:{
				var a = prim.sphere;
				Gizmos.DrawWireSphere(a.pos + offsetPos, a.r);
			break;}
			case PrimitiveWrapper.ShapeType.Capsule:{
				var a = prim.capsule;
				float3 x,y,z;
				y = a.dir;
				x = y.Equals(float3(1,0,0)) ?
					normalize( cross(a.dir, float3(0,1,0)) ) :
					normalize( cross(a.dir, float3(1,0,0)) );
				z = cross(x, y);

				drawWireCylinderGUI((Edge)a, offsetPos);

				var t =  y*a.r_h + a.pos + offsetPos;
				var b = -y*a.r_h + a.pos + offsetPos;
				drawWireCircleGUI(t, a.r_s, float3x3(z,x,y), 0, 0.5f);
				drawWireCircleGUI(t, a.r_s, float3x3(x,z,y), 0, 0.5f);
				drawWireCircleGUI(b, a.r_s, float3x3(z,x,y), 0.5f, 1);
				drawWireCircleGUI(b, a.r_s, float3x3(x,z,y), 0.5f, 1);
			break;}
//			case PrimitiveWrapper.ShapeType.Edge:{
//				drawWireCylinderGUI(prim.edge, offsetPos);
//			break;}
//			case PrimitiveWrapper.ShapeType.FaceQuad:{
//				var a = prim.faceQuad;
//				var x = a.rot.c0 * a.r.x;
//				var y = a.rot.c1 * a.r.y;
//				var z = a.rot.c2 * a.w;
//				var p = a.pos + offsetPos;
//				Gizmos.DrawLine(p-x-y+z, p+x-y+z);
//				Gizmos.DrawLine(p-x+y+z, p+x+y+z);
//				Gizmos.DrawLine(p-x-y+z, p-x+y+z);
//				Gizmos.DrawLine(p+x-y+z, p+x+y+z);
//			break;}
			case PrimitiveWrapper.ShapeType.Quad:{
				var a = prim.quad;
				var x = a.rot.c0 * a.r.x;
				var y = a.rot.c1 * a.r.y;
				var z = a.rot.c2 * a.w;
				var p = a.pos + offsetPos;
				var p0 = p-x-y;
				var p1 = p+x-y;
				var p2 = p-x+y;
				var p3 = p+x+y;
				Gizmos.DrawLine(p0+z, p1+z);
				Gizmos.DrawLine(p2+z, p3+z);
				Gizmos.DrawLine(p0+z, p2+z);
				Gizmos.DrawLine(p1+z, p3+z);
				Gizmos.DrawLine(p0-z, p1-z);
				Gizmos.DrawLine(p2-z, p3-z);
				Gizmos.DrawLine(p0-z, p2-z);
				Gizmos.DrawLine(p1-z, p3-z);
				x = a.rot.c0;
				y = a.rot.c1;
				z = a.rot.c2;
				drawWireCircleGUI(p0, a.w, float3x3(z,x,y), 0.5f, 1);
				drawWireCircleGUI(p0, a.w, float3x3(z,y,x), 0.5f, 1);
				drawWireCircleGUI(p1, a.w, float3x3(z,x,y), 0.5f, 1);
				drawWireCircleGUI(p1, a.w, float3x3(z,y,x), 0, 0.5f);
				drawWireCircleGUI(p2, a.w, float3x3(z,x,y), 0, 0.5f);
				drawWireCircleGUI(p2, a.w, float3x3(z,y,x), 0.5f, 1);
				drawWireCircleGUI(p3, a.w, float3x3(z,x,y), 0, 0.5f);
				drawWireCircleGUI(p3, a.w, float3x3(z,y,x), 0, 0.5f);
			break;}
			case PrimitiveWrapper.ShapeType.Triangle:{
				var a = prim.triangle;
				var v0 = a.pos + mul(a.rot, a.vert0) + offsetPos;
				var v1 = a.pos + mul(a.rot, a.vert1) + offsetPos;
				var v2 = a.pos + mul(a.rot, a.vert2) + offsetPos;
				var z = normalizesafe(cross( v1-v0, v2-v0 ));
				Gizmos.DrawLine(v0 + z*a.w, v1 + z*a.w);
				Gizmos.DrawLine(v1 + z*a.w, v2 + z*a.w);
				Gizmos.DrawLine(v2 + z*a.w, v0 + z*a.w);
				Gizmos.DrawLine(v0 - z*a.w, v1 - z*a.w);
				Gizmos.DrawLine(v1 - z*a.w, v2 - z*a.w);
				Gizmos.DrawLine(v2 - z*a.w, v0 - z*a.w);

				void drawCircle(float3 p0, float3 p1, float w) {
					var x = normalizesafe( p1-p0 );
					var y = cross(x, z);
					drawWireCircleGUI(p0, w, float3x3(z,x,y), 0, 0.5f);
					drawWireCircleGUI(p1, w, float3x3(z,x,y), 0, 0.5f);
				};
				drawCircle(v0, v1, a.w);
				drawCircle(v1, v2, a.w);
				drawCircle(v2, v0, a.w);
			break;}
			case PrimitiveWrapper.ShapeType.Box:{
				void drawPlane(float3x3 rot,float3 r,float w,float3 pos) {
					pos += rot.c2 * r.z;

					Gizmos.DrawLine(pos - rot.c0*(r.x+w) - rot.c1*r.y, pos - rot.c0*(r.x+w) + rot.c1*r.y);
					Gizmos.DrawLine(pos + rot.c0*(r.x+w) - rot.c1*r.y, pos + rot.c0*(r.x+w) + rot.c1*r.y);
					Gizmos.DrawLine(pos - rot.c1*(r.y+w) - rot.c0*r.x, pos - rot.c1*(r.y+w) + rot.c0*r.x);
					Gizmos.DrawLine(pos + rot.c1*(r.y+w) - rot.c0*r.x, pos + rot.c1*(r.y+w) + rot.c0*r.x);

					var cRot = float3x3(rot.c1,rot.c2,rot.c0);
					drawWireCircleGUI(pos-rot.c0*r.x-rot.c1*r.y, w, cRot, 0.5f, 0.75f);
					drawWireCircleGUI(pos+rot.c0*r.x-rot.c1*r.y, w, cRot, 0.25f, 0.5f);
					drawWireCircleGUI(pos-rot.c0*r.x+rot.c1*r.y, w, cRot, 0.75f, 1.0f);
					drawWireCircleGUI(pos+rot.c0*r.x+rot.c1*r.y, w, cRot, 0.0f, 0.25f);
				}
				var a = prim.box;
				var x = a.rot.c0;
				var y = a.rot.c1;
				var z = a.rot.c2;
				var pos = a.pos + offsetPos;
				drawPlane( float3x3(x,y, z), a.r.xyz, a.w, pos);
				drawPlane( float3x3(x,y,-z), a.r.xyz, a.w, pos);
				drawPlane( float3x3(x,z, y), a.r.xzy, a.w, pos);
				drawPlane( float3x3(x,z,-y), a.r.xzy, a.w, pos);
				drawPlane( float3x3(z,y, x), a.r.zyx, a.w, pos);
				drawPlane( float3x3(z,y,-x), a.r.zyx, a.w, pos);
			break;}
			case PrimitiveWrapper.ShapeType.Circle:{
				var a = prim.circle;
				var pos = a.pos+offsetPos;
				var z = a.dir;
				var x = normalize( cross(a.dir, a.dir.Equals(float3(1,0,0)) ? float3(0,1,0) : float3(1,0,0)) );
				var y = cross(x, z);
				drawWireCircleGUI(pos-z*a.w, a.r, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos+z*a.w, a.r, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos, a.r+a.w, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos+x*a.r, a.w, float3x3(x,y,z), 0.75f, 1.25f);
				drawWireCircleGUI(pos-x*a.r, a.w, float3x3(x,y,z), 0.25f, 0.75f);
				drawWireCircleGUI(pos+y*a.r, a.w, float3x3(y,x,z), 0.75f, 1.25f);
				drawWireCircleGUI(pos-y*a.r, a.w, float3x3(y,x,z), 0.25f, 0.75f);
			break;}
			case PrimitiveWrapper.ShapeType.Cylinder:{
				var a = prim.cylinder;
				var pos = a.pos+offsetPos;
				var z = a.dir;
				var x = normalize( cross(a.dir, a.dir.Equals(float3(1,0,0)) ? float3(0,1,0) : float3(1,0,0)) );
				var y = cross(x, z);
				drawWireCylinderGUI((Edge)a, offsetPos);
				drawWireCircleGUI(pos+z*(a.r_h+a.w), a.r_s, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos-z*(a.r_h+a.w), a.r_s, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos+z*a.r_h, a.r_s+a.w, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos-z*a.r_h, a.r_s+a.w, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos-z*a.r_h+x*a.r_s, a.w, float3x3(z,y,x), 0.25f, 0.5f);
				drawWireCircleGUI(pos-z*a.r_h-x*a.r_s, a.w, float3x3(z,y,x), 0.5f, 0.75f);
				drawWireCircleGUI(pos+z*a.r_h+x*a.r_s, a.w, float3x3(z,y,x), 0, 0.25f);
				drawWireCircleGUI(pos+z*a.r_h-x*a.r_s, a.w, float3x3(z,y,x), 0.75f, 1);
				drawWireCircleGUI(pos-z*a.r_h+y*a.r_s, a.w, float3x3(z,x,y), 0.25f, 0.5f);
				drawWireCircleGUI(pos-z*a.r_h-y*a.r_s, a.w, float3x3(z,x,y), 0.5f, 0.75f);
				drawWireCircleGUI(pos+z*a.r_h+y*a.r_s, a.w, float3x3(z,x,y), 0, 0.25f);
				drawWireCircleGUI(pos+z*a.r_h-y*a.r_s, a.w, float3x3(z,x,y), 0.75f, 1);
			break;}
			case PrimitiveWrapper.ShapeType.Cone:{
				var a = prim.cone;
				var pos = a.pos+offsetPos;
				var z = a.dir;
				var x = normalize( cross(a.dir, a.dir.Equals(float3(1,0,0)) ? float3(0,1,0) : float3(1,0,0)) );
				var y = cross(x, z);
				var t = normalize(float2(a.r_h*2,a.r_s));
				var thetaRate = atan2(t.y,t.x) / (2*PI);
				drawWireCircleGUI(pos-z*(a.r_h+a.w), a.r_s, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos+z*(-a.r_h+t.y*a.w), a.r_s+t.x*a.w, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos+z*(a.r_h+t.y*a.w), t.x*a.w, float3x3(x,z,y), 0, 1);
				drawWireCircleGUI(pos-z*a.r_h+x*a.r_s, a.w, float3x3(z,y,x), 0.25f-thetaRate, 0.5f);
				drawWireCircleGUI(pos-z*a.r_h-x*a.r_s, a.w, float3x3(z,y,x), 0.5f, 0.75f+thetaRate);
				drawWireCircleGUI(pos-z*a.r_h+y*a.r_s, a.w, float3x3(z,x,y), 0.25f-thetaRate, 0.5f);
				drawWireCircleGUI(pos-z*a.r_h-y*a.r_s, a.w, float3x3(z,x,y), 0.5f, 0.75f+thetaRate);
				Gizmos.DrawLine(
					pos - z*a.r_h + x*a.r_s + (t.x*x + t.y*z)*a.w,
					pos + z*a.r_h + (t.x*x + t.y*z)*a.w
				);
				Gizmos.DrawLine(
					pos - z*a.r_h - x*a.r_s + (-t.x*x + t.y*z)*a.w,
					pos + z*a.r_h + (-t.x*x + t.y*z)*a.w
				);
				Gizmos.DrawLine(
					pos - z*a.r_h + y*a.r_s + (t.x*y + t.y*z)*a.w,
					pos + z*a.r_h + (t.x*y + t.y*z)*a.w
				);
				Gizmos.DrawLine(
					pos - z*a.r_h - y*a.r_s + (-t.x*y + t.y*z)*a.w,
					pos + z*a.r_h + (-t.x*y + t.y*z)*a.w
				);
			break;}
			case PrimitiveWrapper.ShapeType.ConvexVerts:{
				var a = prim.convexVerts;
				var t = 0.1f;
				var iMax = 40;
				unsafe {
					var vp = (float3*)a.vertsPtr;
					for (int i=0; i<iMax; ++i) {
						var v = vp[ (a.vertsCnt-1) * i / (iMax-1) ];
						var p = a.pos + offsetPos + mul(a.rot,v);
						Gizmos.DrawLine(p-float3(t,0,0), p+float3(t,0,0));
						Gizmos.DrawLine(p-float3(0,t,0), p+float3(0,t,0));
						Gizmos.DrawLine(p-float3(0,0,t), p+float3(0,0,t));
					}
				}
			break;}
			default : throw new InvalidProgramException();
		}
	}

	/** 円を描画 */
	static void drawWireCircleGUI(
		float3 center,
		float r,
		float3x3 rot,
		float from,
		float to
	) {
		int segNum = 30;
		Func<int,float> calcTheta = i => (PI*2)*( from + (to-from)*i/segNum );

		for (int i=0; i<segNum; ++i) {
			var theta0 = calcTheta( i );
			var theta1 = calcTheta( i+1 );
			var p0 = mul( rot, float3(cos(theta0),0,sin(theta0)) ) * r + center;
			var p1 = mul( rot, float3(cos(theta1),0,sin(theta1)) ) * r + center;
			Gizmos.DrawLine(p0, p1);
		}
	}

	/** 辺要素を描画 */
	static void drawWireCylinderGUI(Edge a, float3 offsetPos) {
		float3 x,y,z;
		y = a.dir;
		x = y.Equals(float3(1,0,0)) ?
			normalize( cross(a.dir, float3(0,1,0)) ) :
			normalize( cross(a.dir, float3(1,0,0)) );
		z = cross(x, y);

		var p = a.pos + offsetPos;
		drawWireCircleGUI( p, a.r_s, float3x3(x,y,z), 0, 1 );
		var t =  y*a.r_h + p;
		var b = -y*a.r_h + p;
		drawWireCircleGUI(t, a.r_s, float3x3(x,y,z), 0, 1);
		drawWireCircleGUI(b, a.r_s, float3x3(x,y,z), 0, 1);
		Gizmos.DrawLine( b + x*a.r_s, t + x*a.r_s );
		Gizmos.DrawLine( b + z*a.r_s, t + z*a.r_s );
		Gizmos.DrawLine( b - x*a.r_s, t - x*a.r_s );
		Gizmos.DrawLine( b - z*a.r_s, t - z*a.r_s );
	}

}
}

#endif
