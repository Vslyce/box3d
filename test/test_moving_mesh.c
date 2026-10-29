// SPDX-License-Identifier: MIT
// Moving-mesh regressions adapted from the standalone Esoterica reproduction.
// All fixtures use only Box3D; no Engine, planet data, renderer or external files are required.

#include "shape.h"
#include "test_macros.h"

#include "box3d/box3d.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static b3Quat MeshYaw( float angle )
{
	return (b3Quat){ { 0.0f, sinf( 0.5f * angle ), 0.0f }, cosf( 0.5f * angle ) };
}

static b3MeshData* MakeMovingPlane( float half, int cells )
{
	int vertexCount = ( cells + 1 ) * ( cells + 1 );
	int triangleCount = 2 * cells * cells;
	b3Vec3* vertices = malloc( vertexCount * sizeof( b3Vec3 ) );
	int32_t* indices = malloc( 3 * triangleCount * sizeof( int32_t ) );
	if ( vertices == NULL || indices == NULL )
	{
		free( vertices );
		free( indices );
		return NULL;
	}
	for ( int y = 0; y <= cells; ++y )
	{
		for ( int x = 0; x <= cells; ++x )
		{
			vertices[y * ( cells + 1 ) + x] =
				(b3Vec3){ -half + 2.0f * half * x / cells, -half + 2.0f * half * y / cells, 0 };
		}
	}
	int index = 0;
	for ( int y = 0; y < cells; ++y )
	{
		for ( int x = 0; x < cells; ++x )
		{
			int a = y * ( cells + 1 ) + x, b = a + 1, c = a + cells + 1, d = c + 1;
			indices[index++] = a;
			indices[index++] = b;
			indices[index++] = d;
			indices[index++] = a;
			indices[index++] = d;
			indices[index++] = c;
		}
	}
	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = vertexCount;
	def.stride = sizeof( b3Vec3 );
	def.indices = indices;
	def.triangleCount = triangleCount;
	def.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	free( vertices );
	free( indices );
	return mesh;
}

static b3ShapeId AddMovingConvex( b3BodyId body, const b3ShapeDef* def, int kind )
{
	if ( kind == 0 )
	{
		b3Sphere sphere = { { 0, 0, 0 }, 0.5f };
		return b3CreateSphereShape( body, def, &sphere );
	}
	if ( kind == 1 )
	{
		b3Capsule capsule = { { 0, 0, -0.5f }, { 0, 0, 0.5f }, 0.25f };
		return b3CreateCapsuleShape( body, def, &capsule );
	}
	b3BoxHull box = b3MakeBoxHull( 0.35f, 0.5f, 0.25f );
	return b3CreateHullShape( body, def, &box.base );
}

static float MovingConvexExtent( int kind, b3Quat rotation, b3Vec3 normal )
{
	if ( kind == 0 )
		return 0.5f;
	if ( kind == 1 )
		return 0.25f +
			   0.5f * fabsf( b3Dot( b3RotateVector( rotation, (b3Vec3){ 0, 0, 1 } ), normal ) );
	return 0.35f * fabsf( b3Dot( b3RotateVector( rotation, (b3Vec3){ 1, 0, 0 } ), normal ) ) +
		   0.5f * fabsf( b3Dot( b3RotateVector( rotation, (b3Vec3){ 0, 1, 0 } ), normal ) ) +
		   0.25f * fabsf( b3Dot( b3RotateVector( rotation, (b3Vec3){ 0, 0, 1 } ), normal ) );
}

static bool StepMovingMesh( b3WorldId world, float interval, int* impactSplits )
{
	double remaining = interval;
	for ( int split = 0; remaining > 1e-9 && split < 32; ++split )
	{
		float fraction = 1.0f;
		int pairs = 0;
		if ( !b3World_QueryKinematicMeshImpact( world, (float)remaining, 64, &fraction, &pairs ) )
			return false;
		double h = remaining;
		if ( fraction > 0.0f && fraction < 0.999f )
		{
			h = fmax( 1e-6, remaining * fraction );
			*impactSplits += 1;
		}
		b3World_Step( world, (float)h, 1 );
		remaining -= h;
	}
	return remaining <= 1e-9;
}

static int RelativeMeshSweeps( void )
{
	b3MeshData* mesh = MakeMovingPlane( 4, 8 );
	ENSURE( mesh != NULL );
	b3Shape a = { 0 }, b = { 0 };
	a.type = b3_meshShape;
	a.mesh = (b3Mesh){ mesh, { 1, 1, 1 } };
	a.sensorIndex = b.sensorIndex = B3_NULL_INDEX;
	b.type = b3_sphereShape;
	b.sphere.radius = 0.25f;
	b3Sweep from = {
		.q1 = b3Quat_identity, .q2 = b3Quat_identity, .c1 = { 0, 0, -5 }, .c2 = { 0, 0, 5 } };
	b3Sweep fixed = { .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	b3TOIOutput translated = b3ShapeTimeOfImpact( &a, &b, &from, &fixed, 1.0f );
	ENSURE( translated.state == b3_toiStateHit );
	ENSURE_SMALL( translated.fraction - 0.475f, 0.001f );
	from.c1 = from.c2 = b3Vec3_zero;
	from.q1 = MeshYaw( 0.25f * B3_PI );
	from.q2 = MeshYaw( -0.25f * B3_PI );
	fixed.c1 = fixed.c2 = (b3Vec3){ 2, 0, 0 };
	b3TOIOutput angular = b3ShapeTimeOfImpact( &a, &b, &from, &fixed, 1.0f );
	// Independent analytic crossing, evaluated at Box3D's normalized-linear quaternion
	// interpolation.
	float expected = 0.5f * ( 1.0f + tanf( -0.5f * asinf( 0.125f ) ) / tanf( B3_PI / 8.0f ) );
	ENSURE( angular.state == b3_toiStateHit );
	ENSURE_SMALL( angular.fraction - expected, 0.002f );
	// Equivalent quaternion signs must not change the swept candidate coverage.
	from.q2.v = b3Neg( from.q2.v );
	from.q2.s = -from.q2.s;
	b3TOIOutput signedAngular = b3ShapeTimeOfImpact( &a, &b, &from, &fixed, 1.0f );
	ENSURE_SMALL( signedAngular.fraction - angular.fraction, 0.002f );
	b3DestroyMesh( mesh );
	return 0;
}

static int MovingTriangleCacheAt( b3Pos origin )
{
	b3MeshData* mesh = MakeMovingPlane( 8, 64 );
	ENSURE( mesh != NULL );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = (b3Vec3){ 0, 0, -9.81f };
	wd.enableSleep = false;
	b3WorldId world = b3CreateWorld( &wd );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_kinematicBody;
	bd.position = origin;
	b3BodyId terrain = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.baseMaterial.friction = 0;
	b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
	bd.type = b3_dynamicBody;
	bd.position = b3OffsetPos( origin, (b3Vec3){ 0, 0, 0.505f } );
	b3BodyId sphere = b3CreateBody( world, &bd );
	AddMovingConvex( sphere, &sd, 0 );
	float minimum = 1.0f;
	for ( int i = 0; i < 480; ++i )
	{
		b3Body_SetLinearVelocity( terrain, (b3Vec3){ 3, 0, 0 } );
		b3World_Step( world, 1.0f / 240.0f, 1 );
		minimum = b3MinFloat( minimum, b3SubPos( b3Body_GetPosition( sphere ), origin ).z );
	}
	ENSURE_SMALL( minimum - 0.505f, 0.035f );
	b3DestroyWorld( world );
	b3DestroyMesh( mesh );
	return 0;
}

static int MovingTriangleCache( void )
{
	ENSURE( MovingTriangleCacheAt( b3Pos_zero ) == 0 );
	ENSURE( MovingTriangleCacheAt( (b3Pos){ 10000, 10000, 10000 } ) == 0 );
#if defined( BOX3D_DOUBLE_PRECISION )
	ENSURE( MovingTriangleCacheAt( (b3Pos){ 100000000, 100000000, 100000000 } ) == 0 );
#endif
	return 0;
}

static int SleepingMeshContacts( void )
{
	for ( int kind = 0; kind < 3; ++kind )
	{
		for ( int angular = 0; angular < 2; ++angular )
		{
			b3MeshData* mesh = MakeMovingPlane( 8, 32 );
			ENSURE( mesh != NULL );
			b3WorldDef wd = b3DefaultWorldDef();
			wd.gravity = (b3Vec3){ 0, 0, -9.81f };
			b3WorldId world = b3CreateWorld( &wd );
			b3BodyDef bd = b3DefaultBodyDef();
			bd.type = b3_kinematicBody;
			b3BodyId terrain = b3CreateBody( world, &bd );
			b3ShapeDef sd = b3DefaultShapeDef();
			sd.baseMaterial.friction = 0.8f;
			b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
			bd.type = b3_dynamicBody;
			bd.position = (b3Pos){
				2, 0, MovingConvexExtent( kind, b3Quat_identity, (b3Vec3){ 0, 0, 1 } ) + 0.005f };
			b3BodyId body = b3CreateBody( world, &bd );
			AddMovingConvex( body, &sd, kind );
			for ( int i = 0; i < 240; ++i )
				b3World_Step( world, 1.0f / 240, 1 );
			b3Body_SetAwake( body, false );
			ENSURE( !b3Body_IsAwake( body ) );
			float penetration = 0;
			int contacts = 0;
			for ( int i = 0; i < 360; ++i )
			{
				if ( angular )
					b3Body_SetAngularVelocity( terrain, (b3Vec3){ 0, -0.2f, 0 } );
				else
					b3Body_SetLinearVelocity( terrain, (b3Vec3){ 0, 0, 2 } );
				b3World_Step( world, 1.0f / 240, 1 );
				b3WorldTransform tp = b3Body_GetTransform( terrain ),
								 bp = b3Body_GetTransform( body );
				b3Vec3 normal = b3RotateVector( tp.q, (b3Vec3){ 0, 0, 1 } );
				penetration =
					b3MaxFloat( penetration, MovingConvexExtent( kind, bp.q, normal ) -
												 b3Dot( b3SubPos( bp.p, tp.p ), normal ) );
				b3ContactData data[32];
				contacts += b3Body_GetContactData( body, data, 32 ) > 0;
			}
			ENSURE( penetration <= 0.035f && contacts >= 350 && b3Body_IsAwake( body ) );
			b3DestroyWorld( world );
			b3DestroyMesh( mesh );
		}
	}
	return 0;
}

static int RotatingMeshContinuity( void )
{
	// Thirty seconds on a rotating platform, with gravity following its surface normal.
	b3MeshData* mesh = MakeMovingPlane( 4, 32 );
	ENSURE( mesh != NULL );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.enableSleep = false;
	b3WorldId world = b3CreateWorld( &wd );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_kinematicBody;
	b3BodyId terrain = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.baseMaterial.friction = 1;
	b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 0, 0, 0.255f };
	bd.angularVelocity = (b3Vec3){ 0, 0.2f, 0 };
	bd.linearVelocity = (b3Vec3){ 0.051f, 0, 0 };
	b3BodyId body = b3CreateBody( world, &bd );
	AddMovingConvex( body, &sd, 2 );
	float penetration = 0;
	int contacts = 0, splits = 0;
	for ( int i = 0; i < 7200; ++i )
	{
		b3Vec3 normal = b3RotateVector( b3Body_GetRotation( terrain ), (b3Vec3){ 0, 0, 1 } );
		b3World_SetGravity( world, b3MulSV( -9.81f, normal ) );
		b3Body_SetAngularVelocity( terrain, (b3Vec3){ 0, 0.2f, 0 } );
		ENSURE( StepMovingMesh( world, 1.0f / 240, &splits ) );
		b3WorldTransform bp = b3Body_GetTransform( body );
		normal = b3RotateVector( b3Body_GetRotation( terrain ), (b3Vec3){ 0, 0, 1 } );
		penetration = b3MaxFloat( penetration, MovingConvexExtent( 2, bp.q, normal ) -
												   b3Dot( b3ToVec3( bp.p ), normal ) );
		b3ContactData data[32];
		contacts += b3Body_GetContactData( body, data, 32 ) > 0;
	}
	ENSURE( penetration <= 0.035f && contacts >= 7100 );
	b3DestroyWorld( world );
	b3DestroyMesh( mesh );
	return 0;
}

static int MovingMeshContactCases( void )
{
	int splits = 0;
	for ( int kind = 0; kind < 3; ++kind )
	{
		// Linear mesh, angular mesh, and fast convex approaching a frozen kinematic mesh.
		for ( int mode = 0; mode < 3; ++mode )
		{
			b3MeshData* mesh = MakeMovingPlane( 8, 32 );
			ENSURE( mesh != NULL );
			b3WorldDef wd = b3DefaultWorldDef();
			wd.gravity = b3Vec3_zero;
			b3WorldId world = b3CreateWorld( &wd );
			b3BodyDef bd = b3DefaultBodyDef();
			bd.type = b3_kinematicBody;
			if ( mode == 1 )
				bd.rotation = MeshYaw( 0.25f * B3_PI );
			else
				bd.position = (b3Pos){ 0, 0, -2 };
			b3BodyId terrain = b3CreateBody( world, &bd );
			b3ShapeDef sd = b3DefaultShapeDef();
			sd.baseMaterial.friction = 0;
			b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
			bd = b3DefaultBodyDef();
			bd.type = b3_dynamicBody;
			bd.position = (b3Pos){ 2, 0, 0 };
			b3BodyId body = b3CreateBody( world, &bd );
			AddMovingConvex( body, &sd, kind );
			b3Body_SetAwake( body, false );
			ENSURE( !b3Body_IsAwake( body ) );
			float penetration = 0, maxSpeed = 0;
			bool contact = false, woken = false;
			for ( int i = 0; i < ( mode == 1 ? 120 : 60 ); ++i )
			{
				if ( mode == 1 )
					b3Body_SetAngularVelocity( terrain, (b3Vec3){ 0, -2, 0 } );
				else if ( mode == 2 && i == 0 )
					b3Body_SetLinearVelocity( body, (b3Vec3){ 0, 0, -20 } );
				else if ( mode == 0 )
					b3Body_SetLinearVelocity( terrain, (b3Vec3){ 0, 0, 20 } );
				ENSURE( StepMovingMesh( world, 1.0f / 240.0f, &splits ) );
				b3WorldTransform tp = b3Body_GetTransform( terrain ),
								 bp = b3Body_GetTransform( body );
				b3Vec3 normal = b3RotateVector( tp.q, (b3Vec3){ 0, 0, 1 } );
				float extent = MovingConvexExtent( kind, bp.q, normal );
				penetration =
					b3MaxFloat( penetration, extent - b3Dot( b3SubPos( bp.p, tp.p ), normal ) );
				maxSpeed = b3MaxFloat( maxSpeed, b3Length( b3Body_GetLinearVelocity( body ) ) );
				b3ContactData data[32];
				contact |= b3Body_GetContactData( body, data, 32 ) > 0;
				woken |= b3Body_IsAwake( body );
			}
			printf( "    moving mesh shape %d mode %d: penetration %.6g m\n", kind, mode,
					penetration );
			ENSURE( contact && woken && maxSpeed > 1.0f );
			ENSURE( penetration <= 0.035f );
			b3DestroyWorld( world );
			b3DestroyMesh( mesh );
		}
	}
	ENSURE( splits > 0 );
	return 0;
}

typedef struct MeshQueryFixture
{
	b3WorldId world;
	b3BodyId terrain, body;
	b3ShapeId meshShape, convexShape;
	b3MeshData* mesh;
} MeshQueryFixture;

static MeshQueryFixture MakeMeshQueryFixture( b3Pos offset )
{
	MeshQueryFixture f = { 0 };
	f.mesh = MakeMovingPlane( 4, 8 );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = b3Vec3_zero;
	f.world = b3CreateWorld( &wd );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_kinematicBody;
	bd.position = b3OffsetPos( offset, (b3Vec3){ 0, 0, -5 } );
	bd.linearVelocity = (b3Vec3){ 0, 0, 20 };
	f.terrain = b3CreateBody( f.world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.enableCustomFiltering = true;
	f.meshShape = b3CreateMeshShape( f.terrain, &sd, f.mesh, (b3Vec3){ 1, 1, 1 } );
	bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = offset;
	f.body = b3CreateBody( f.world, &bd );
	f.convexShape = AddMovingConvex( f.body, &sd, 0 );
	b3Body_SetAwake( f.body, false );
	return f;
}

static void DestroyMeshQueryFixture( MeshQueryFixture* f )
{
	b3DestroyWorld( f->world );
	b3DestroyMesh( f->mesh );
}

static int MeshImpactQueryContract( void )
{
	MeshQueryFixture f = MakeMeshQueryFixture( b3Pos_zero );
	b3MeshImpactQueryDef def = b3DefaultMeshImpactQueryDef();
	def.timeStep = 0.5f;
	def.maxPairs = 1;
	def.maxTriangles = 0;
	b3MeshImpactResult result = { 0 };
	b3WorldTransform before = b3Body_GetTransform( f.terrain );
	int64_t bytes = b3GetByteCount();
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE_SMALL( result.fraction - 0.45f, 0.001f );
	ENSURE( B3_ID_EQUALS( result.meshShapeId, f.meshShape ) &&
			B3_ID_EQUALS( result.shapeId, f.convexShape ) );
	ENSURE( result.pairCount == 1 && result.triangleCount > 0 );
	b3WorldTransform after = b3Body_GetTransform( f.terrain );
	ENSURE( b3LengthSquared( b3SubPos( before.p, after.p ) ) == 0.0f );
	ENSURE( !b3Body_IsAwake( f.body ) && bytes == b3GetByteCount() );
	// Repeated queries are deterministic and read-only.
	b3MeshImpactResult repeated = { 0 };
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &repeated ) ==
			b3_meshImpactComplete );
	ENSURE( repeated.fraction == result.fraction &&
			repeated.triangleCount == result.triangleCount );

	b3MeshImpactResult saved = result;
	def.maxTriangles = 1;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactCapacityExceeded );
	ENSURE( memcmp( &saved, &result, sizeof( result ) ) == 0 );
	def.maxTriangles = 0;
	def.timeStep = NAN;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactInvalidInput );
	def.timeStep = 0;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactInvalidInput );
	def.timeStep = 0.5f;
	def.maxPairs = -1;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactInvalidInput );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, NULL, &result ) ==
			b3_meshImpactInvalidInput );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( b3_nullWorldId, &def, &result ) ==
			b3_meshImpactInvalidInput );
	ENSURE( memcmp( &saved, &result, sizeof( result ) ) == 0 );
	def.maxPairs = 1;
	// A second eligible pair exhausts the budget, even if an earlier pair already found a hit.
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 1, 0, 0 };
	b3BodyId other = b3CreateBody( f.world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	AddMovingConvex( other, &sd, 0 );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactCapacityExceeded );
	ENSURE( memcmp( &saved, &result, sizeof( result ) ) == 0 );
	float fraction = 0.125f;
	int pairs = -7;
	ENSURE( !b3World_QueryKinematicMeshImpact( f.world, 0.5f, 1, &fraction, &pairs ) );
	ENSURE( fraction == 0.125f && pairs == -7 );
	def.maxPairs = 0;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 2 );
	b3Body_SetLinearVelocity( f.terrain, b3Vec3_zero );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.fraction == 1.0f && B3_IS_NULL( result.meshShapeId ) &&
			B3_IS_NULL( result.shapeId ) );
	DestroyMeshQueryFixture( &f );
	return 0;
}

typedef struct MeshFilterContext
{
	b3WorldId world;
	int calls;
	bool reject;
	bool locked;
} MeshFilterContext;

static bool FilterMeshQuery( b3ShapeId a, b3ShapeId b, void* rawContext )
{
	MAYBE_UNUSED( a );
	MAYBE_UNUSED( b );
	MeshFilterContext* context = rawContext;
	context->calls += 1;
	b3MeshImpactQueryDef def = b3DefaultMeshImpactQueryDef();
	b3MeshImpactResult result = { .fraction = 0.123f };
	context->locked = b3World_QueryKinematicMeshImpactWithOptions(
						  context->world, &def, &result ) == b3_meshImpactWorldLocked &&
					  result.fraction == 0.123f;
	return !context->reject;
}

static int MeshImpactFiltering( void )
{
	MeshQueryFixture f = MakeMeshQueryFixture( b3Pos_zero );
	b3MeshImpactQueryDef def = { 0.5f, 1, 0 };
	b3MeshImpactResult result;
	b3Filter filter = b3Shape_GetFilter( f.convexShape );
	filter.maskBits = 0;
	b3Shape_SetFilter( f.convexShape, filter, true );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 0 && result.fraction == 1.0f );
	// Positive group overrides masks AND zero category bits.
	filter.groupIndex = 3;
	filter.categoryBits = 0;
	b3Shape_SetFilter( f.convexShape, filter, true );
	b3Shape_SetFilter( f.meshShape, filter, true );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 1 && result.fraction < 1.0f );
	filter.groupIndex = -3;
	b3Shape_SetFilter( f.convexShape, filter, true );
	b3Shape_SetFilter( f.meshShape, filter, true );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 0 );
	filter = b3DefaultFilter();
	b3Shape_SetFilter( f.convexShape, filter, true );
	b3Shape_SetFilter( f.meshShape, filter, true );
	// Both fixture shapes enable custom filtering at creation.
	MeshFilterContext context = { .world = f.world, .reject = true };
	b3World_SetCustomFilterCallback( f.world, FilterMeshQuery, &context );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( context.calls > 0 && context.locked && result.pairCount == 0 );
	context.reject = false;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( context.locked && result.fraction < 1.0f );
	b3Body_Disable( f.body );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 0 );
	b3Body_Enable( f.body );
	// Non-colliding joints exclude the pair even though the spatial query returns it.
	b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
	jointDef.base.bodyIdA = f.terrain;
	jointDef.base.bodyIdB = f.body;
	jointDef.base.collideConnected = false;
	b3JointId joint = b3CreateDistanceJoint( f.world, &jointDef );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 0 );
	b3DestroyJoint( joint, true );
	DestroyMeshQueryFixture( &f );
	return 0;
}

static int MeshQueryLargeWorldPopulation( void )
{
	MeshQueryFixture f = MakeMeshQueryFixture( b3Pos_zero );
	b3BodyDef bd = b3DefaultBodyDef();
	b3ShapeDef sd = b3DefaultShapeDef();
	for ( int i = 0; i < 2000; ++i )
	{
		bd.type = ( i & 1 ) ? b3_dynamicBody : b3_staticBody;
		bd.position = (b3Pos){ 1000.0f + 2.0f * i, 0, 0 };
		b3BodyId body = b3CreateBody( f.world, &bd );
		AddMovingConvex( body, &sd, 0 );
		if ( i % 3 == 0 )
			b3DestroyBody( body ); // Sparse IDs must not consume a collision budget.
	}
	b3MeshImpactQueryDef def = { 0.5f, 1, 0 };
	b3MeshImpactResult result;
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.pairCount == 1 );
	ENSURE_SMALL( result.fraction - 0.45f, 0.001f );
	DestroyMeshQueryFixture( &f );
	return 0;
}

static int MeshQueryMotionPrediction( void )
{
	MeshQueryFixture f = MakeMeshQueryFixture( b3Pos_zero );
	b3MeshImpactQueryDef def = { 0.5f, 0, 0 };
	b3MeshImpactResult result;
	// Equal velocities preserve separation; opposite velocities halve the impact time.
	b3Body_SetLinearVelocity( f.body, (b3Vec3){ 0, 0, 20 } );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.fraction == 1.0f );
	b3Body_SetLinearVelocity( f.body, (b3Vec3){ 0, 0, -20 } );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE_SMALL( result.fraction - 0.225f, 0.001f );
	// Pade damping halves the convex velocity for h=0.5 and damping=2.
	b3Body_SetLinearVelocity( f.body, (b3Vec3){ 0, 0, 20 } );
	b3Body_SetLinearDamping( f.body, 2 );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	// Five meters of relative travel: allow the solver's 5 mm contact target plus roundoff.
	ENSURE_SMALL( result.fraction - 0.9f, 0.0011f );
	b3Body_SetMotionLocks( f.terrain, (b3MotionLocks){ .linearZ = true } );
	b3Body_SetLinearVelocity( f.terrain, (b3Vec3){ 0, 0, 20 } );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE( result.fraction == 1.0f );
	b3Body_SetMotionLocks( f.terrain, (b3MotionLocks){ 0 } );
	b3Body_SetLinearDamping( f.body, 0 );
	b3Body_SetLinearVelocity( f.body, b3Vec3_zero );
	b3World_SetGravity( f.world, (b3Vec3){ 0, 0, -20 } );
	b3Body_ApplyForceToCenter( f.body, (b3Vec3){ 0, 0, -20 * b3Body_GetMass( f.body ) }, true );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE_SMALL( result.fraction - 0.225f, 0.001f );
	ENSURE( b3LengthSquared( b3Body_GetLinearVelocity( f.body ) ) == 0 );
	DestroyMeshQueryFixture( &f );

	// Multiple hits must report the earliest shape, independently of candidate order.
	f = MakeMeshQueryFixture( b3Pos_zero );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 1, 0, -2 };
	b3BodyId closer = b3CreateBody( f.world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3ShapeId closerShape = AddMovingConvex( closer, &sd, 0 );
	ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
			b3_meshImpactComplete );
	ENSURE_SMALL( result.fraction - 0.25f, 0.001f );
	ENSURE( B3_ID_EQUALS( result.shapeId, closerShape ) && result.pairCount == 2 );
	DestroyMeshQueryFixture( &f );
	return 0;
}

static int MeshQuerySensors( void )
{
	for ( int meshSensor = 0; meshSensor < 2; ++meshSensor )
	{
		MeshQueryFixture f = MakeMeshQueryFixture( b3Pos_zero );
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.isSensor = true;
		if ( meshSensor )
		{
			b3DestroyShape( f.meshShape, true );
			b3CreateMeshShape( f.terrain, &sd, f.mesh, (b3Vec3){ 1, 1, 1 } );
		}
		else
		{
			b3DestroyShape( f.convexShape, true );
			AddMovingConvex( f.body, &sd, 0 );
		}
		b3MeshImpactQueryDef def = { 0.5f, 1, 0 };
		b3MeshImpactResult result;
		ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
				b3_meshImpactComplete );
		ENSURE( result.pairCount == 0 && result.fraction == 1.0f );
		DestroyMeshQueryFixture( &f );
	}
	return 0;
}

static int MeshWakeManyIslands( void )
{
	b3MeshData* mesh = MakeMovingPlane( 8, 8 );
	ENSURE( mesh != NULL );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = b3Vec3_zero;
	b3WorldId world = b3CreateWorld( &wd );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_kinematicBody;
	bd.position = (b3Pos){ 0, 0, -1 };
	bd.linearVelocity = (b3Vec3){ 0, 0, 20 };
	b3BodyId terrain = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	// Two shapes on one body exercise resuming iteration after wake-up reallocates solver arrays.
	b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
	b3CreateMeshShape( terrain, &sd, mesh, (b3Vec3){ 0.9f, 0.9f, 1 } );
	b3BodyId bodies[3][24];
	bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	for ( int group = 0; group < 3; ++group )
	{
		sd = b3DefaultShapeDef();
		if ( group == 1 )
			sd.filter.maskBits = 0;
		if ( group == 2 )
			sd.isSensor = true;
		for ( int i = 0; i < 24; ++i )
		{
			bd.position = (b3Pos){ 1.5f * ( i % 6 ) - 4, 1.5f * ( i / 6 ) - 3, 0 };
			bodies[group][i] = b3CreateBody( world, &bd );
			AddMovingConvex( bodies[group][i], &sd, 0 );
			b3Body_SetAwake( bodies[group][i], false );
		}
	}
	b3World_Step( world, 0.05f, 4 );
	for ( int group = 0; group < 3; ++group )
	{
		for ( int i = 0; i < 24; ++i )
			ENSURE( b3Body_IsAwake( bodies[group][i] ) == ( group == 0 ) );
	}
	b3DestroyWorld( world );
	b3DestroyMesh( mesh );
	return 0;
}

static int MeshQueryOffsets( void )
{
	// Common translation cancels before float relative geometry is formed.
	const double offsets[] = {
		0,
		10000,
#if defined( BOX3D_DOUBLE_PRECISION )
		100000000,
#endif
	};
	for ( int i = 0; i < ARRAY_COUNT( offsets ); ++i )
	{
		b3Pos origin = { offsets[i], offsets[i], offsets[i] };
		MeshQueryFixture f = MakeMeshQueryFixture( origin );
		b3MeshImpactQueryDef def = { 0.5f, 1, 0 };
		b3MeshImpactResult result;
		ENSURE( b3World_QueryKinematicMeshImpactWithOptions( f.world, &def, &result ) ==
				b3_meshImpactComplete );
		ENSURE_SMALL( result.fraction - 0.45f, 0.001f );
		DestroyMeshQueryFixture( &f );
	}
	return 0;
}

int MovingMeshTest( void )
{
	int64_t bytes = b3GetByteCount();
	RUN_SUBTEST( RelativeMeshSweeps );
	RUN_SUBTEST( MovingTriangleCache );
	RUN_SUBTEST( SleepingMeshContacts );
	RUN_SUBTEST( MovingMeshContactCases );
	RUN_SUBTEST( RotatingMeshContinuity );
	RUN_SUBTEST( MeshImpactQueryContract );
	RUN_SUBTEST( MeshImpactFiltering );
	RUN_SUBTEST( MeshQueryLargeWorldPopulation );
	RUN_SUBTEST( MeshQueryMotionPrediction );
	RUN_SUBTEST( MeshQuerySensors );
	RUN_SUBTEST( MeshWakeManyIslands );
	RUN_SUBTEST( MeshQueryOffsets );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}
