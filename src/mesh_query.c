// SPDX-License-Identifier: MIT

#include "body.h"
#include "dynamic_tree.h"
#include "shape.h"
#include "solver_set.h"

#include "box3d/box3d.h"

#include <limits.h>

b3MeshImpactQueryDef b3DefaultMeshImpactQueryDef( void )
{
	return (b3MeshImpactQueryDef){ 1.0f / 60.0f, 1024, 65536 };
}

// Predict free motion only. Contacts, joints and the gyroscopic solve can change this motion.
// Sleeping bodies use zero initial velocity but include forces/gravity, anticipating a mesh wake.
static bool b3PredictMeshMotion( b3World* world, b3Body* body, float h, b3Vec3* translation,
								 b3Vec3* rotation )
{
	b3BodySim* sim = b3GetBodySim( world, body );
	b3BodyState* state = b3GetBodyState( world, body );
	b3Vec3 v = state != NULL ? state->linearVelocity : b3Vec3_zero;
	b3Vec3 w = state != NULL ? state->angularVelocity : b3Vec3_zero;
	float gravityScale = sim->invMass > 0.0f ? sim->gravityScale : 0.0f;
	b3Vec3 dv = b3Blend2( h * sim->invMass, sim->force, h * gravityScale, world->gravity );
	b3Vec3 dw = b3MulSV( h, b3MulMV( sim->invInertiaWorld, sim->torque ) );
	v = b3MulAdd( dv, 1.0f / ( 1.0f + h * sim->linearDamping ), v );
	w = b3MulAdd( dw, 1.0f / ( 1.0f + h * sim->angularDamping ), w );
	uint32_t flags = body->flags;
	v.x = ( flags & b3_lockLinearX ) ? 0.0f : v.x;
	v.y = ( flags & b3_lockLinearY ) ? 0.0f : v.y;
	v.z = ( flags & b3_lockLinearZ ) ? 0.0f : v.z;
	w.x = ( flags & b3_lockAngularX ) ? 0.0f : w.x;
	w.y = ( flags & b3_lockAngularY ) ? 0.0f : w.y;
	w.z = ( flags & b3_lockAngularZ ) ? 0.0f : w.z;
	float speed = b3Length( v );
	float spin = b3Length( w );
	if ( !b3IsValidFloat( speed ) || !b3IsValidFloat( spin ) )
	{
		return false;
	}
	if ( speed > world->maxLinearSpeed )
	{
		v = b3MulSV( world->maxLinearSpeed / speed, v );
	}
	if ( h * spin > B3_MAX_ROTATION && ( flags & b3_allowFastRotation ) == 0 )
	{
		w = b3MulSV( ( B3_MAX_ROTATION / h ) / spin, w );
	}
	*translation = b3MulSV( h, v );
	*rotation = b3MulSV( h, w );
	return b3IsValidVec3( *translation ) && b3IsValidVec3( *rotation ) &&
		   b3IsValidFloat( b3Length( *rotation ) );
}

static b3Sweep b3MeshQuerySweep( const b3BodySim* sim, b3Pos base, b3Vec3 translation,
								 b3Vec3 rotation )
{
	b3Sweep sweep;
	sweep.c1 = b3SubPos( sim->center, base );
	sweep.c2 = b3Add( sweep.c1, translation );
	sweep.q1 = sim->transform.q;
	sweep.q2 = b3IntegrateRotation( sweep.q1, rotation );
	sweep.localCenter = sim->localCenter;
	return sweep;
}

// Every point moves at most |translation| + radius * angle, and rotation cannot move it
// more than the diameter. This bound covers intermediate orientations as well as endpoints.
static float b3MeshMotionMargin( const b3BodySim* sim, b3Vec3 translation, b3Vec3 rotation )
{
	return b3Length( translation ) +
		   b3Length( sim->maxExtent ) * b3MinFloat( b3Length( rotation ), 2.0f );
}

static b3ShapeId b3MeshQueryShapeId( const b3World* world, const b3Shape* shape )
{
	return (b3ShapeId){ shape->id + 1, world->worldId, shape->generation };
}

static bool b3MeshPairCanCollide( b3World* world, b3Shape* mesh, b3Shape* shape )
{
	if ( shape->sensorIndex != B3_NULL_INDEX ||
		 !b3ShouldShapesCollide( mesh->filter, shape->filter ) )
	{
		return false;
	}
	b3Body* a = world->bodies.data + mesh->bodyId;
	b3Body* b = world->bodies.data + shape->bodyId;
	if ( b->type != b3_dynamicBody || !b3ShouldBodiesCollide( world, a, b ) )
	{
		return false;
	}
	if ( ( ( mesh->flags | shape->flags ) & b3_enableCustomFiltering ) != 0 &&
		 world->customFilterFcn != NULL )
	{
		return world->customFilterFcn( b3MeshQueryShapeId( world, mesh ),
									   b3MeshQueryShapeId( world, shape ),
									   world->customFilterContext );
	}
	return true;
}

typedef struct b3MeshWakeContext
{
	b3World* world;
	int meshId;
} b3MeshWakeContext;

static bool b3WakeMeshNeighbor( int proxyId, uint64_t shapeId, void* rawContext )
{
	B3_UNUSED( proxyId );
	b3MeshWakeContext* context = rawContext;
	b3World* world = context->world;
	b3Shape* shape = world->shapes.data + (int)shapeId;
	b3Body* body = world->bodies.data + shape->bodyId;
	if ( body->setIndex >= b3_firstSleepingSet &&
		 b3MeshPairCanCollide( world, world->shapes.data + context->meshId, shape ) )
	{
		b3WakeBody( world, body );
	}
	return true;
}

void b3WakeMovingMeshNeighbors( b3World* world, float timeStep )
{
	// Inspect only the original awake bodies. Waking islands appends sims and may relocate the
	// solver arrays, so never retain a sim/set pointer across the tree callback.
	int awakeCount = world->solverSets.data[b3_awakeSet].bodySims.count;
	for ( int i = 0; i < awakeCount; ++i )
	{
		int bodyId = world->solverSets.data[b3_awakeSet].bodySims.data[i].bodyId;
		b3Body* body = world->bodies.data + bodyId;
		if ( body->type != b3_kinematicBody )
		{
			continue;
		}
		// Kinematic bodies have no acceleration. Undamped velocities bound all substeps,
		// even when damping makes the final velocity smaller than the average velocity.
		b3BodyState* state = b3GetBodyState( world, body );
		b3Vec3 translation = b3MulSV( timeStep, state->linearVelocity );
		b3Vec3 rotation = b3MulSV( timeStep, state->angularVelocity );
		float travel = b3MeshMotionMargin( b3GetBodySim( world, body ), translation, rotation );
		if ( travel <= 0.0f || !b3IsValidFloat( travel ) )
		{
			continue;
		}
		for ( int id = body->headShapeId; id != B3_NULL_INDEX;
			  id = world->shapes.data[id].nextShapeId )
		{
			b3Shape* mesh = world->shapes.data + id;
			if ( mesh->type != b3_meshShape || mesh->sensorIndex != B3_NULL_INDEX )
			{
				continue;
			}
			b3AABB bounds = b3ComputeFatShapeAABB( mesh, b3GetBodySim( world, body )->transform,
												   travel + B3_SPECULATIVE_DISTANCE );
			b3MeshWakeContext context = { world, id };
			b3TreeView view = b3MakeTreeView( world->broadPhase.trees + b3_dynamicBody );
			// A positive collision group can override zero category/mask bits.
			b3TreeView_Query( &view, bounds, 0, true, b3WakeMeshNeighbor, &context );
		}
	}
}

typedef struct b3MeshQueryContext
{
	b3World* world;
	const b3MeshImpactQueryDef* def;
	b3MeshImpactResult result;
	b3MeshImpactBudget budget;
	b3Shape* mesh;
	b3Pos base;
	b3Sweep meshSweep;
	b3AABB meshBounds;
} b3MeshQueryContext;

static bool b3QueryMeshPair( int proxyId, uint64_t shapeId, void* rawContext )
{
	B3_UNUSED( proxyId );
	b3MeshQueryContext* context = rawContext;
	b3World* world = context->world;
	b3Shape* shape = world->shapes.data + (int)shapeId;
	if ( !b3MeshPairCanCollide( world, context->mesh, shape ) )
	{
		return true;
	}
	b3Body* body = world->bodies.data + shape->bodyId;
	b3Vec3 translation, rotation;
	if ( !b3PredictMeshMotion( world, body, context->def->timeStep, &translation, &rotation ) )
	{
		context->budget.status = b3_meshImpactInvalidInput;
		return false;
	}
	b3BodySim* sim = b3GetBodySim( world, body );
	float travel = b3MeshMotionMargin( sim, translation, rotation );
	b3AABB bounds =
		b3ComputeFatShapeAABB( shape, sim->transform, travel + B3_SPECULATIVE_DISTANCE );
	if ( !b3AABB_Overlaps( context->meshBounds, bounds ) )
	{
		return true;
	}
	if ( !b3IsConvex( shape->type ) )
	{
		context->budget.status = b3_meshImpactUnsupportedShape;
		return false;
	}
	if ( context->result.pairCount == INT_MAX ||
		 ( context->def->maxPairs > 0 && context->result.pairCount >= context->def->maxPairs ) )
	{
		context->budget.status = b3_meshImpactCapacityExceeded;
		return false;
	}
	context->result.pairCount += 1;
	b3Sweep sweep = b3MeshQuerySweep( sim, context->base, translation, rotation );
	if ( !b3IsValidVec3( sweep.c1 ) || !b3IsValidVec3( sweep.c2 ) || !b3IsValidQuat( sweep.q2 ) )
	{
		context->budget.status = b3_meshImpactInvalidInput;
		return false;
	}
	b3TOIOutput impact =
		b3ShapeRelativeTimeOfImpact( context->mesh, shape, &context->meshSweep, &sweep,
									 context->result.fraction, &context->budget );
	if ( context->budget.status != b3_meshImpactComplete )
	{
		return false;
	}
	if ( impact.fraction > 0.0f && impact.fraction < context->result.fraction )
	{
		context->result.fraction = impact.fraction;
		context->result.meshShapeId = b3MeshQueryShapeId( world, context->mesh );
		context->result.shapeId = b3MeshQueryShapeId( world, shape );
	}
	return true;
}

b3MeshImpactQueryStatus b3World_QueryKinematicMeshImpactWithOptions(
	b3WorldId worldId, const b3MeshImpactQueryDef* def, b3MeshImpactResult* result )
{
	if ( def == NULL || result == NULL || !b3World_IsValid( worldId ) ||
		 !b3IsValidFloat( def->timeStep ) || def->timeStep <= 0.0f || def->maxPairs < 0 ||
		 def->maxTriangles < 0 )
	{
		return b3_meshImpactInvalidInput;
	}
	b3World* world = b3GetWorldFromId( worldId );
	if ( world->locked )
	{
		return b3_meshImpactWorldLocked;
	}
	b3MeshQueryContext context = { 0 };
	context.world = world;
	context.def = def;
	context.result.fraction = 1.0f;
	context.budget.maxTriangles = def->maxTriangles;

	// One pass over live sims finds a conservative movement allowance for the dynamic tree.
	// This also includes sleeping bodies (gravity after waking), and ignores sparse ID holes.
	float dynamicTravel = 0.0f;
	for ( int setIndex = b3_awakeSet; setIndex < world->solverSets.count; ++setIndex )
	{
		b3SolverSet* set = world->solverSets.data + setIndex;
		for ( int i = 0; i < set->bodySims.count; ++i )
		{
			b3BodySim* sim = set->bodySims.data + i;
			b3Body* body = world->bodies.data + sim->bodyId;
			if ( body->type != b3_dynamicBody )
			{
				continue;
			}
			b3Vec3 translation, rotation;
			if ( !b3PredictMeshMotion( world, body, def->timeStep, &translation, &rotation ) )
			{
				return b3_meshImpactInvalidInput;
			}
			float travel = b3MeshMotionMargin( sim, translation, rotation );
			if ( !b3IsValidFloat( travel ) )
			{
				return b3_meshImpactInvalidInput;
			}
			dynamicTravel = b3MaxFloat( dynamicTravel, travel );
		}
	}

	// Lock out mutations/reentrant steps while invoking user filters. Always release before
	// returning.
	world->locked = true;
	for ( int setIndex = b3_awakeSet; setIndex < world->solverSets.count; ++setIndex )
	{
		b3SolverSet* set = world->solverSets.data + setIndex;
		for ( int i = 0; i < set->bodySims.count; ++i )
		{
			b3BodySim* sim = set->bodySims.data + i;
			b3Body* body = world->bodies.data + sim->bodyId;
			if ( body->type != b3_kinematicBody )
			{
				continue;
			}
			b3Vec3 translation, rotation;
			if ( !b3PredictMeshMotion( world, body, def->timeStep, &translation, &rotation ) )
			{
				context.budget.status = b3_meshImpactInvalidInput;
				goto finish;
			}
			float travel = b3MeshMotionMargin( sim, translation, rotation );
			context.base = sim->center;
			context.meshSweep = b3MeshQuerySweep( sim, context.base, translation, rotation );
			if ( !b3IsValidFloat( travel + dynamicTravel ) ||
				 !b3IsValidQuat( context.meshSweep.q2 ) )
			{
				context.budget.status = b3_meshImpactInvalidInput;
				goto finish;
			}
			for ( int id = body->headShapeId; id != B3_NULL_INDEX;
				  id = world->shapes.data[id].nextShapeId )
			{
				context.mesh = world->shapes.data + id;
				if ( context.mesh->type != b3_meshShape ||
					 context.mesh->sensorIndex != B3_NULL_INDEX )
				{
					continue;
				}
				// Inflate in the local frame before outward rounding to the float broad phase.
				context.meshBounds = b3ComputeFatShapeAABB( context.mesh, sim->transform,
															travel + B3_SPECULATIVE_DISTANCE );
				b3AABB bounds =
					b3ComputeFatShapeAABB( context.mesh, sim->transform,
										   travel + dynamicTravel + B3_SPECULATIVE_DISTANCE );
				b3TreeView view = b3MakeTreeView( world->broadPhase.trees + b3_dynamicBody );
				b3TreeView_Query( &view, bounds, 0, true, b3QueryMeshPair, &context );
				if ( context.budget.status != b3_meshImpactComplete )
				{
					goto finish;
				}
			}
		}
	}
finish:
	world->locked = false;
	if ( context.budget.status == b3_meshImpactComplete )
	{
		context.result.triangleCount = context.budget.triangleCount;
		*result = context.result;
	}
	return context.budget.status;
}

bool b3World_QueryKinematicMeshImpact( b3WorldId worldId, float timeStep, int maxPairs,
									   float* fraction, int* pairCount )
{
	if ( maxPairs <= 0 || fraction == NULL || pairCount == NULL )
	{
		return false;
	}
	b3MeshImpactQueryDef def = { timeStep, maxPairs, 0 };
	b3MeshImpactResult result;
	if ( b3World_QueryKinematicMeshImpactWithOptions( worldId, &def, &result ) !=
		 b3_meshImpactComplete )
	{
		return false;
	}
	*fraction = result.fraction;
	*pairCount = result.pairCount;
	return true;
}
