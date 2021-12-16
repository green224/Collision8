

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
			Point,
			Line,
			HalfSpace,
			Sphere,
			Capsule,
			Quad,
			Triangle,
			Box,
			Circle,
			Cylinder,
			Cone,
			ConvexVerts,
		}
		[FieldOffset(0)] public ShapeType shapeType;

		[FieldOffset(8)] public Point point;
		[FieldOffset(8)] public Line line;
		[FieldOffset(8)] public HalfSpace halfSpace;
		[FieldOffset(8)] public Sphere sphere;
		[FieldOffset(8)] public Capsule capsule;
		[FieldOffset(8)] public Quad quad;
		[FieldOffset(8)] public Triangle triangle;
		[FieldOffset(8)] public Box box;
		[FieldOffset(8)] public Circle circle;
		[FieldOffset(8)] public Cylinder cylinder;
		[FieldOffset(8)] public Cone cone;
		[FieldOffset(8)] public ConvexVerts convexVerts;

		public PrimitiveWrapper(Point point) {
			shapeType = ShapeType.Point;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.point = point;
		}
		public PrimitiveWrapper(Line line) {
			shapeType = ShapeType.Line;
			point = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.line = line;
		}
		public PrimitiveWrapper(HalfSpace halfSpace) {
			shapeType = ShapeType.HalfSpace;
			point = default;
			line = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.halfSpace = halfSpace;
		}
		public PrimitiveWrapper(Sphere sphere) {
			shapeType = ShapeType.Sphere;
			point = default;
			line = default;
			halfSpace = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.sphere = sphere;
		}
		public PrimitiveWrapper(Capsule capsule) {
			shapeType = ShapeType.Capsule;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.capsule = capsule;
		}
		public PrimitiveWrapper(Quad quad) {
			shapeType = ShapeType.Quad;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.quad = quad;
		}
		public PrimitiveWrapper(Triangle triangle) {
			shapeType = ShapeType.Triangle;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.triangle = triangle;
		}
		public PrimitiveWrapper(Box box) {
			shapeType = ShapeType.Box;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			circle = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.box = box;
		}
		public PrimitiveWrapper(Circle circle) {
			shapeType = ShapeType.Circle;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			cylinder = default;
			cone = default;
			convexVerts = default;
			this.circle = circle;
		}
		public PrimitiveWrapper(Cylinder cylinder) {
			shapeType = ShapeType.Cylinder;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cone = default;
			convexVerts = default;
			this.cylinder = cylinder;
		}
		public PrimitiveWrapper(Cone cone) {
			shapeType = ShapeType.Cone;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			convexVerts = default;
			this.cone = cone;
		}
		public PrimitiveWrapper(ConvexVerts convexVerts) {
			shapeType = ShapeType.ConvexVerts;
			point = default;
			line = default;
			halfSpace = default;
			sphere = default;
			capsule = default;
			quad = default;
			triangle = default;
			box = default;
			circle = default;
			cylinder = default;
			cone = default;
			this.convexVerts = convexVerts;
		}

		// これはボクシングを引き起こすので注意すること
		public IPrimitive BoxedValue {
			get {
				switch (shapeType) {
				case ShapeType.Point : return point;
				case ShapeType.Line : return line;
				case ShapeType.HalfSpace : return halfSpace;
				case ShapeType.Sphere : return sphere;
				case ShapeType.Capsule : return capsule;
				case ShapeType.Quad : return quad;
				case ShapeType.Triangle : return triangle;
				case ShapeType.Box : return box;
				case ShapeType.Circle : return circle;
				case ShapeType.Cylinder : return cylinder;
				case ShapeType.Cone : return cone;
				case ShapeType.ConvexVerts : return convexVerts;
				}
				return null;
			}
			set {
				if (value is Point) {
					shapeType = ShapeType.Point;
					point = (Point)value;
				} else if (value is Line) {
					shapeType = ShapeType.Line;
					line = (Line)value;
				} else if (value is HalfSpace) {
					shapeType = ShapeType.HalfSpace;
					halfSpace = (HalfSpace)value;
				} else if (value is Sphere) {
					shapeType = ShapeType.Sphere;
					sphere = (Sphere)value;
				} else if (value is Capsule) {
					shapeType = ShapeType.Capsule;
					capsule = (Capsule)value;
				} else if (value is Quad) {
					shapeType = ShapeType.Quad;
					quad = (Quad)value;
				} else if (value is Triangle) {
					shapeType = ShapeType.Triangle;
					triangle = (Triangle)value;
				} else if (value is Box) {
					shapeType = ShapeType.Box;
					box = (Box)value;
				} else if (value is Circle) {
					shapeType = ShapeType.Circle;
					circle = (Circle)value;
				} else if (value is Cylinder) {
					shapeType = ShapeType.Cylinder;
					cylinder = (Cylinder)value;
				} else if (value is Cone) {
					shapeType = ShapeType.Cone;
					cone = (Cone)value;
				} else if (value is ConvexVerts) {
					shapeType = ShapeType.ConvexVerts;
					convexVerts = (ConvexVerts)value;
				}
			}
		}

		/** 指定の移動・回転を行った結果を得る */
		public PrimitiveWrapper translate(float3 pos, float3x3 rot) {
			switch (shapeType) {
			case ShapeType.Point : return new PrimitiveWrapper(point.translate(pos,rot));
			case ShapeType.Line : return new PrimitiveWrapper(line.translate(pos,rot));
			case ShapeType.HalfSpace : return new PrimitiveWrapper(halfSpace.translate(pos,rot));
			case ShapeType.Sphere : return new PrimitiveWrapper(sphere.translate(pos,rot));
			case ShapeType.Capsule : return new PrimitiveWrapper(capsule.translate(pos,rot));
			case ShapeType.Quad : return new PrimitiveWrapper(quad.translate(pos,rot));
			case ShapeType.Triangle : return new PrimitiveWrapper(triangle.translate(pos,rot));
			case ShapeType.Box : return new PrimitiveWrapper(box.translate(pos,rot));
			case ShapeType.Circle : return new PrimitiveWrapper(circle.translate(pos,rot));
			case ShapeType.Cylinder : return new PrimitiveWrapper(cylinder.translate(pos,rot));
			case ShapeType.Cone : return new PrimitiveWrapper(cone.translate(pos,rot));
			case ShapeType.ConvexVerts : return new PrimitiveWrapper(convexVerts.translate(pos,rot));
			}
			return default;
		}

		// 衝突判定
		static public ColResult collision(in PrimitiveWrapper a, in PrimitiveWrapper b) {
			switch (a.shapeType) {
			case ShapeType.Point : return collision(a.point, b);
			case ShapeType.Line : return collision(a.line, b);
			case ShapeType.HalfSpace : return collision(a.halfSpace, b);
			case ShapeType.Sphere : return collision(a.sphere, b);
			case ShapeType.Capsule : return collision(a.capsule, b);
			case ShapeType.Quad : return collision(a.quad, b);
			case ShapeType.Triangle : return collision(a.triangle, b);
			case ShapeType.Box : return collision(a.box, b);
			case ShapeType.Circle : return collision(a.circle, b);
			case ShapeType.Cylinder : return collision(a.cylinder, b);
			case ShapeType.Cone : return collision(a.cone, b);
			case ShapeType.ConvexVerts : return collision(a.convexVerts, b);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, Point b) => -collision(b, a);
		static public ColResult collision(Point a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, Line b) => -collision(b, a);
		static public ColResult collision(Line a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, HalfSpace b) => -collision(b, a);
		static public ColResult collision(HalfSpace a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, Sphere b) => -collision(b, a);
		static public ColResult collision(Sphere a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Capsule b) => -collision(b, a);
		static public ColResult collision(in Capsule a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Quad b) => -collision(b, a);
		static public ColResult collision(in Quad a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Triangle b) => -collision(b, a);
		static public ColResult collision(in Triangle a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Box b) => -collision(b, a);
		static public ColResult collision(in Box a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Circle b) => -collision(b, a);
		static public ColResult collision(in Circle a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Cylinder b) => -collision(b, a);
		static public ColResult collision(in Cylinder a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in Cone b) => -collision(b, a);
		static public ColResult collision(in Cone a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public ColResult collision(in PrimitiveWrapper a, in ConvexVerts b) => -collision(b, a);
		static public ColResult collision(in ConvexVerts a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.collision(a, b.point);
			case ShapeType.Line : return Solver.collision(a, b.line);
			case ShapeType.HalfSpace : return Solver.collision(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.collision(a, b.sphere);
			case ShapeType.Capsule : return Solver.collision(a, b.capsule);
			case ShapeType.Quad : return Solver.collision(a, b.quad);
			case ShapeType.Triangle : return Solver.collision(a, b.triangle);
			case ShapeType.Box : return Solver.collision(a, b.box);
			case ShapeType.Circle : return Solver.collision(a, b.circle);
			case ShapeType.Cylinder : return Solver.collision(a, b.cylinder);
			case ShapeType.Cone : return Solver.collision(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.collision(a, b.convexVerts);
			}
			return default;
		}

		// バウンダリー判定
		static public bool checkBoundary(in PrimitiveWrapper a, in PrimitiveWrapper b) {
			switch (a.shapeType) {
			case ShapeType.Point : return checkBoundary(a.point, b);
			case ShapeType.Line : return checkBoundary(a.line, b);
			case ShapeType.HalfSpace : return checkBoundary(a.halfSpace, b);
			case ShapeType.Sphere : return checkBoundary(a.sphere, b);
			case ShapeType.Capsule : return checkBoundary(a.capsule, b);
			case ShapeType.Quad : return checkBoundary(a.quad, b);
			case ShapeType.Triangle : return checkBoundary(a.triangle, b);
			case ShapeType.Box : return checkBoundary(a.box, b);
			case ShapeType.Circle : return checkBoundary(a.circle, b);
			case ShapeType.Cylinder : return checkBoundary(a.cylinder, b);
			case ShapeType.Cone : return checkBoundary(a.cone, b);
			case ShapeType.ConvexVerts : return checkBoundary(a.convexVerts, b);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, Point b) => checkBoundary(b, a);
		static public bool checkBoundary(Point a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, Line b) => checkBoundary(b, a);
		static public bool checkBoundary(Line a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, HalfSpace b) => checkBoundary(b, a);
		static public bool checkBoundary(HalfSpace a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, Sphere b) => checkBoundary(b, a);
		static public bool checkBoundary(Sphere a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Capsule b) => checkBoundary(b, a);
		static public bool checkBoundary(in Capsule a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Quad b) => checkBoundary(b, a);
		static public bool checkBoundary(in Quad a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Triangle b) => checkBoundary(b, a);
		static public bool checkBoundary(in Triangle a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Box b) => checkBoundary(b, a);
		static public bool checkBoundary(in Box a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Circle b) => checkBoundary(b, a);
		static public bool checkBoundary(in Circle a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Cylinder b) => checkBoundary(b, a);
		static public bool checkBoundary(in Cylinder a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in Cone b) => checkBoundary(b, a);
		static public bool checkBoundary(in Cone a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
		[MI(MO.AggressiveInlining)]
		static public bool checkBoundary(in PrimitiveWrapper a, in ConvexVerts b) => checkBoundary(b, a);
		static public bool checkBoundary(in ConvexVerts a, in PrimitiveWrapper b) {
			switch (b.shapeType) {
			case ShapeType.Point : return Solver.checkBoundary(a, b.point);
			case ShapeType.Line : return Solver.checkBoundary(a, b.line);
			case ShapeType.HalfSpace : return Solver.checkBoundary(a, b.halfSpace);
			case ShapeType.Sphere : return Solver.checkBoundary(a, b.sphere);
			case ShapeType.Capsule : return Solver.checkBoundary(a, b.capsule);
			case ShapeType.Quad : return Solver.checkBoundary(a, b.quad);
			case ShapeType.Triangle : return Solver.checkBoundary(a, b.triangle);
			case ShapeType.Box : return Solver.checkBoundary(a, b.box);
			case ShapeType.Circle : return Solver.checkBoundary(a, b.circle);
			case ShapeType.Cylinder : return Solver.checkBoundary(a, b.cylinder);
			case ShapeType.Cone : return Solver.checkBoundary(a, b.cone);
			case ShapeType.ConvexVerts : return Solver.checkBoundary(a, b.convexVerts);
			}
			return default;
		}
	}

}


