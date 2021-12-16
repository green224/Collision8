using System;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Mathematics.math;

using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {
public static partial class PrimitivesEx {

	/** 指定の正負方向のキャップ部分を、Sphereとして取得。dirには-1か1を入れる */
	static public Sphere capSphere(this in Capsule self, float dir) =>
		new Sphere( self.pos + dir*self.r_h*self.dir, self.r_s );




	/** Quadから面部分のみを取り出す。dirには-1か1を入れる */
	static public FaceQuad getFace(this in Quad self, float dir) =>
		new FaceQuad(
			self.pos,
			// 軸の左手系・右手系が反転しないようにする
			new float3x3( self.rot.c0, self.rot.c1*dir, self.rot.c2*dir ),
			self.r.xy,
			self.w
		);

	/** Quadから辺部分のみを取り出す */
	static public void getEdges(
		this in Quad self,
		out Edge edge0,
		out Edge edge1,
		out Edge edge2,
		out Edge edge3
	) {
		edge0 = new Edge(
			self.pos + self.rot.c0*self.r.x,
			self.rot.c1, self.w, self.r.y
		);
		edge1 = new Edge(
			self.pos - self.rot.c0*self.r.x,
			self.rot.c1, self.w, self.r.y
		);
		edge2 = new Edge(
			self.pos + self.rot.c1*self.r.y,
			self.rot.c0, self.w, self.r.x
		);
		edge3 = new Edge(
			self.pos - self.rot.c1*self.r.y,
			self.rot.c0, self.w, self.r.x
		);
	}

	/** Quadから辺部分のみを取り出す。指定の方向に近い要素のみを取り出す */
	static public void getEdges(
		this in Quad self,
		float3 dir,
		out Edge edge0,
		out Edge edge1
	) {
		var xDir = sign( dot(dir, self.rot.c0) );
		var yDir = sign( dot(dir, self.rot.c1) );
		edge0 = new Edge(
			self.pos + self.rot.c0*(xDir*self.r.x),
			self.rot.c1, self.w, self.r.y
		);
		edge1 = new Edge(
			self.pos + self.rot.c1*(yDir*self.r.y),
			self.rot.c0, self.w, self.r.x
		);
	}

	/** Quadから頂点部分のみを取り出す */
	static public void getVerts(
		this in Quad self,
		out Sphere vert0,
		out Sphere vert1,
		out Sphere vert2,
		out Sphere vert3
	) {
		vert0 = new Sphere(
			self.pos - self.rot.c0*self.r.x - self.rot.c1*self.r.y,
			self.w
		);
		vert1 = new Sphere(
			self.pos + self.rot.c0*self.r.x - self.rot.c1*self.r.y,
			self.w
		);
		vert2 = new Sphere(
			self.pos - self.rot.c0*self.r.x + self.rot.c1*self.r.y,
			self.w
		);
		vert3 = new Sphere(
			self.pos + self.rot.c0*self.r.x + self.rot.c1*self.r.y,
			self.w
		);
	}

	/** Quadから頂点部分のみを取り出す。指定の方向に近い要素のみを取り出す */
	static public Sphere getVert( this in Quad self, float3 dir ) {
		var xDir = sign( dot(dir, self.rot.c0) );
		var yDir = sign( dot(dir, self.rot.c1) );
		return new Sphere(
			self.pos
			+ self.rot.c0 * (xDir * self.r.x)
			+ self.rot.c1 * (yDir * self.r.y),
			self.w
		);
	}




	/** Boxから面部分のみを取り出す */
	static public void getFaces(
		this in Box self,
		out FaceQuad face0,
		out FaceQuad face1,
		out FaceQuad face2,
		out FaceQuad face3,
		out FaceQuad face4,
		out FaceQuad face5
	) {
		face0 = new FaceQuad(		// +Z
			self.pos + self.rot.c2*self.r.z,
			new float3x3( self.rot.c0, self.rot.c1, self.rot.c2 ),
			self.r.xy, self.w
		);
		face1 = new FaceQuad(		// -Z
			self.pos - self.rot.c2*self.r.z,
			new float3x3( self.rot.c0, -self.rot.c1, -self.rot.c2 ),
			self.r.xy, self.w
		);
		face2 = new FaceQuad(		// +X
			self.pos + self.rot.c0*self.r.x,
			new float3x3( -self.rot.c2, self.rot.c1, self.rot.c0 ),
			self.r.zy, self.w
		);
		face3 = new FaceQuad(		// -X
			self.pos - self.rot.c0*self.r.x,
			new float3x3( self.rot.c2, self.rot.c1, -self.rot.c0 ),
			self.r.zy, self.w
		);
		face4 = new FaceQuad(		// +Y
			self.pos + self.rot.c1*self.r.y,
			new float3x3( self.rot.c0, -self.rot.c2, self.rot.c1 ),
			self.r.xz, self.w
		);
		face5 = new FaceQuad(		// -Y
			self.pos - self.rot.c1*self.r.y,
			new float3x3( self.rot.c0, self.rot.c2, -self.rot.c1 ),
			self.r.xz, self.w
		);
	}

	/** Boxから面部分のみを取り出す。指定の方向に近い要素のみを取り出す */
	static public void getFaces(
		this in Box self,
		float3 dir,
		out FaceQuad face0,
		out FaceQuad face1,
		out FaceQuad face2
	) {
		var xDir = sign( dot(dir, self.rot.c0) );
		var yDir = sign( dot(dir, self.rot.c1) );
		var zDir = sign( dot(dir, self.rot.c2) );
		face0 = new FaceQuad(		// ±Z
			self.pos + self.rot.c2*(zDir*self.r.z),
			new float3x3( self.rot.c0, zDir*self.rot.c1, zDir*self.rot.c2 ),
			self.r.xy, self.w
		);
		face1 = new FaceQuad(		// ±X
			self.pos + self.rot.c0*(xDir*self.r.x),
			new float3x3( -xDir*self.rot.c2, self.rot.c1, xDir*self.rot.c0 ),
			self.r.zy, self.w
		);
		face2 = new FaceQuad(		// ±Y
			self.pos + self.rot.c1*(yDir*self.r.y),
			new float3x3( self.rot.c0, -yDir*self.rot.c2, yDir*self.rot.c1 ),
			self.r.xz, self.w
		);
	}

	/** Boxから辺部分のみを取り出す */
	static public void getEdges(
		this in Box self,
		out Edge edge0, out Edge edge1, out Edge edge2, out Edge edge3,
		out Edge edge4, out Edge edge5, out Edge edge6, out Edge edge7,
		out Edge edge8, out Edge edge9, out Edge edge10, out Edge edge11
	) {
		edge0 = new Edge(
			self.pos + self.rot.c0*self.r.x + self.rot.c2*self.r.z,
			self.rot.c1, self.w, self.r.y
		);
		edge1 = new Edge(
			self.pos - self.rot.c0*self.r.x + self.rot.c2*self.r.z,
			self.rot.c1, self.w, self.r.y
		);
		edge2 = new Edge(
			self.pos + self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.rot.c0, self.w, self.r.x
		);
		edge3 = new Edge(
			self.pos - self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.rot.c0, self.w, self.r.x
		);
		edge4 = new Edge(
			self.pos + self.rot.c0*self.r.x - self.rot.c2*self.r.z,
			self.rot.c1, self.w, self.r.y
		);
		edge5 = new Edge(
			self.pos - self.rot.c0*self.r.x - self.rot.c2*self.r.z,
			self.rot.c1, self.w, self.r.y
		);
		edge6 = new Edge(
			self.pos + self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.rot.c0, self.w, self.r.x
		);
		edge7 = new Edge(
			self.pos - self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.rot.c0, self.w, self.r.x
		);
		edge8 = new Edge(
			self.pos + self.rot.c0*self.r.x + self.rot.c1*self.r.y,
			self.rot.c2, self.w, self.r.z
		);
		edge9 = new Edge(
			self.pos + self.rot.c0*self.r.x - self.rot.c1*self.r.y,
			self.rot.c2, self.w, self.r.z
		);
		edge10 = new Edge(
			self.pos - self.rot.c0*self.r.x + self.rot.c1*self.r.y,
			self.rot.c2, self.w, self.r.z
		);
		edge11 = new Edge(
			self.pos - self.rot.c0*self.r.x - self.rot.c1*self.r.y,
			self.rot.c2, self.w, self.r.z
		);
	}

	/** Boxから辺部分のみを取り出す。指定の方向に近い要素のみを取り出す */
	static public void getEdges(
		this in Box self,
		float3 dir,
		out Edge edge0,
		out Edge edge1,
		out Edge edge2
	) {
		var xDir = sign( dot(dir, self.rot.c0) );
		var yDir = sign( dot(dir, self.rot.c1) );
		var zDir = sign( dot(dir, self.rot.c2) );
		edge0 = new Edge(
			self.pos + self.rot.c1*(yDir*self.r.y) + self.rot.c2*(zDir*self.r.z),
			self.rot.c0, self.w, self.r.x
		);
		edge1 = new Edge(
			self.pos + self.rot.c0*(xDir*self.r.x) + self.rot.c2*(zDir*self.r.z),
			self.rot.c1, self.w, self.r.y
		);
		edge2 = new Edge(
			self.pos + self.rot.c0*(xDir*self.r.x) + self.rot.c1*(yDir*self.r.y),
			self.rot.c2, self.w, self.r.z
		);
	}

	/** Boxから頂点部分のみを取り出す */
	static public void getVerts(
		this in Box self,
		out Sphere vert0, out Sphere vert1, out Sphere vert2, out Sphere vert3,
		out Sphere vert4, out Sphere vert5, out Sphere vert6, out Sphere vert7
	) {
		vert0 = new Sphere(
			self.pos - self.rot.c0*self.r.x - self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.w
		);
		vert1 = new Sphere(
			self.pos + self.rot.c0*self.r.x - self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.w
		);
		vert2 = new Sphere(
			self.pos - self.rot.c0*self.r.x + self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.w
		);
		vert3 = new Sphere(
			self.pos + self.rot.c0*self.r.x + self.rot.c1*self.r.y + self.rot.c2*self.r.z,
			self.w
		);
		vert4 = new Sphere(
			self.pos - self.rot.c0*self.r.x - self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.w
		);
		vert5 = new Sphere(
			self.pos + self.rot.c0*self.r.x - self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.w
		);
		vert6 = new Sphere(
			self.pos - self.rot.c0*self.r.x + self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.w
		);
		vert7 = new Sphere(
			self.pos + self.rot.c0*self.r.x + self.rot.c1*self.r.y - self.rot.c2*self.r.z,
			self.w
		);
	}

	/** Boxから頂点部分のみを取り出す。指定の方向に近い要素のみを取り出す */
	static public Sphere getVert( this in Box self, float3 dir ) {
		var xDir = sign( dot(dir, self.rot.c0) );
		var yDir = sign( dot(dir, self.rot.c1) );
		var zDir = sign( dot(dir, self.rot.c2) );
		return new Sphere(
			self.pos
			+ self.rot.c0 * (xDir * self.r.x)
			+ self.rot.c1 * (yDir * self.r.y)
			+ self.rot.c2 * (zDir * self.r.z),
			self.w
		);
	}


}
}

