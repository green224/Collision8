

using Unity.Mathematics;
using static Unity.Mathematics.math;
using System.Runtime.InteropServices;
using MI = System.Runtime.CompilerServices.MethodImplAttribute; 
using MO = System.Runtime.CompilerServices.MethodImplOptions; 


namespace SimpleCollision {
static public partial class Solver {

	// シンプルなバウンダリー判定
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Point a, Point b) => false;
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Point a, Line b) => false;
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Point a, HalfSpace b) => true;
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Point a, Sphere b) => true;
	static public bool checkBoundary(Point a, in Capsule b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Edge b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in FaceQuad b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Quad b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Box b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Circle b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Cylinder b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in Cone b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	static public bool checkBoundary(Point a, in ConvexVerts b) =>
		lengthsq(a.pos - b.pos) < b.boundarySqR();
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Line a, Line b) => false;
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Line a, HalfSpace b) => false;
	static public bool checkBoundary(Line a, Sphere b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Capsule b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Edge b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in FaceQuad b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Quad b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Box b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Circle b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Cylinder b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in Cone b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	static public bool checkBoundary(Line a, in ConvexVerts b) =>
		lengthsq(removeDir(a.pos-b.pos, a.dir)) < b.boundarySqR();
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(HalfSpace a, HalfSpace b) => false;
	static public bool checkBoundary(HalfSpace a, Sphere b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in Capsule b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(HalfSpace a, in Edge b) => false;
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(HalfSpace a, in FaceQuad b) => false;
	static public bool checkBoundary(HalfSpace a, in Quad b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in Box b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in Circle b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in Cylinder b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in Cone b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(HalfSpace a, in ConvexVerts b) {
		var l = dot(a.pos - b.pos, a.dir);
		return l*l < b.boundarySqR();
	}
	static public bool checkBoundary(Sphere a, Sphere b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Capsule b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Edge b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in FaceQuad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Quad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(Sphere a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Capsule b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Edge b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in FaceQuad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Quad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Triangle b) {
		var r = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(r), a.boundarySqR(), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Capsule a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Capsule a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Edge b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in FaceQuad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Quad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Triangle b) {
		var r = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(r), a.boundarySqR(), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Edge a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Edge a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in FaceQuad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in Quad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in Triangle b) {
		var r = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(r), a.boundarySqR(), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in FaceQuad a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in FaceQuad a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in Quad b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in Triangle b) {
		var r = max(abs(b.vert0), max(abs(b.vert1), abs(b.vert2))) + b.w;
		return checkBoundaryCore( lengthsq(r), a.boundarySqR(), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Quad a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Quad a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Triangle a, in Box b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Triangle a, in Circle b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Triangle a, in Cylinder b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Triangle a, in Cone b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Triangle a, in ConvexVerts b) {
		var r = max(abs(a.vert0), max(abs(a.vert1), abs(a.vert2))) + a.w;
		return checkBoundaryCore( b.boundarySqR(), lengthsq(r), lengthsq(a.pos-b.pos) );
	}
	static public bool checkBoundary(in Box a, in Box b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Box a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Box a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Box a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Box a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Circle a, in Circle b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Circle a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Circle a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Circle a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Cylinder a, in Cylinder b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Cylinder a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Cylinder a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Cone a, in Cone b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in Cone a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );
	static public bool checkBoundary(in ConvexVerts a, in ConvexVerts b) =>
		checkBoundaryCore( a.boundarySqR(), b.boundarySqR(), lengthsq(a.pos - b.pos) );



	// バウンダリー判定の逆向き引数バージョン
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Line a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(HalfSpace a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Sphere a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Capsule a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Edge a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, Point b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(HalfSpace a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Sphere a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Capsule a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Edge a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, Line b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(Sphere a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Capsule a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Edge a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, HalfSpace b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Capsule a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Edge a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, Sphere b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Edge a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Capsule b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in FaceQuad a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Edge b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Quad a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in FaceQuad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Triangle a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Quad b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Box a, in Triangle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in Triangle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Triangle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Triangle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Triangle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Circle a, in Box b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Box b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Box b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Box b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cylinder a, in Circle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Circle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Circle b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in Cone a, in Cylinder b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Cylinder b) => checkBoundary(b,a);
	[MI(MO.AggressiveInlining)]
	static public bool checkBoundary(in ConvexVerts a, in Cone b) => checkBoundary(b,a);
}
}


