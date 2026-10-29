// SPDX-License-Identifier: MIT
// Public-API example: advance the whole world in small intervals, splitting at predicted mesh
// impacts.
#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>

static bool Advance( b3WorldId world, float interval, int* splits )
{
	float remaining = interval;
	for ( int step = 0; remaining > 0.0f && step < 32; ++step )
	{
		b3MeshImpactQueryDef def = b3DefaultMeshImpactQueryDef();
		def.timeStep = remaining;
		b3MeshImpactResult result;
		b3MeshImpactQueryStatus status =
			b3World_QueryKinematicMeshImpactWithOptions( world, &def, &result );
		if ( status != b3_meshImpactComplete )
		{
			fprintf( stderr, "Mesh query stopped: status %d, %.9g seconds unconsumed\n", status,
					 remaining );
			return false;
		}
		float h = remaining * result.fraction;
		if ( h <= 0.0f || ( h < remaining && h < 1e-7f ) )
		{
			fprintf( stderr, "Impact interval too small; %.9g seconds unconsumed\n", remaining );
			return false;
		}
		*splits += result.fraction < 1.0f;
		// A world step clears applied forces. This fixture only uses constant kinematic velocity.
		// Applications using persistent forces must reapply them for each partial step.
		b3World_Step( world, h, 1 );
		remaining -= h;
	}
	if ( remaining > 0.0f )
	{
		fprintf( stderr, "Subdivision limit reached; %.9g seconds unconsumed\n", remaining );
		return false;
	}
	return true;
}

int main( void )
{
	b3Vec3 vertices[] = { { -4, -4, 0 }, { 4, -4, 0 }, { 4, 4, 0 }, { -4, 4, 0 } };
	int32_t indices[] = { 0, 1, 2, 0, 2, 3 };
	b3MeshDef md = { 0 };
	md.vertices = vertices;
	md.vertexCount = 4;
	md.stride = sizeof( b3Vec3 );
	md.indices = indices;
	md.triangleCount = 2;
	md.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &md, NULL, 0 );
	if ( mesh == NULL )
		return EXIT_FAILURE;
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = b3Vec3_zero;
	b3WorldId world = b3CreateWorld( &wd );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_kinematicBody;
	bd.position = (b3Pos){ 0, 0, -2 };
	bd.linearVelocity = (b3Vec3){ 0, 0, 20 };
	b3BodyId platform = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.baseMaterial.friction = 0;
	b3CreateMeshShape( platform, &sd, mesh, (b3Vec3){ 1, 1, 1 } );
	bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	b3BodyId body = b3CreateBody( world, &bd );
	b3Sphere sphere = { { 0, 0, 0 }, 0.5f };
	b3CreateSphereShape( body, &sd, &sphere );
	b3Body_SetAwake( body, false );
	int splits = 0;
	bool complete = true;
	for ( int i = 0; i < 60 && complete; ++i )
		complete = Advance( world, 1.0f / 240, &splits );
	printf( "Completed: %s; impact splits: %d; sphere height: %.6f m\n", complete ? "yes" : "no",
			splits, (double)b3Body_GetPosition( body ).z );
	b3DestroyWorld( world );
	b3DestroyMesh( mesh );
	return complete && splits > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
