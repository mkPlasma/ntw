#pragma once


// Rigid body physics update timing
#define NTW_PHYS_UPDATES_PER_SECOND 20
#define NTW_PHYS_UPDATE_TIME_MICRO	(1000000 / NTW_PHYS_UPDATES_PER_SECOND)

// Rigid body physics time delta
#define NTW_PHYS_TIME_DELTA	(1.0f / NTW_PHYS_UPDATES_PER_SECOND)

// time / (updates 1000000 / updates)


// OBSOLETE

// Physics timestep multiplier
// TEMPORARY! use a variable later
// KEEP AS FLOAT !!!
#define PHYS_TIMESCALE 1.0f

// Physics timestep value 'dt'
#define PHYS_TIMESTEP (PHYS_TIMESCALE / NTW_PHYS_UPDATES_PER_SECOND)

// Delta multiplier for rendering
#define PHYS_DELTA_MULT (PHYS_TIMESCALE * NTW_PHYS_UPDATES_PER_SECOND)


// Maximum iterations for simple dynamic collision resolution
#define PHYS_MAX_SIMPLE_COLLISION_ITER 10


// Maximum number of constraint solve iterations per update
#define PHYS_MAX_CONSTRAINT_ITER 20

// Maximum absolute value of constraint value for it to be considered solved
#define PHYS_CONSTRAINT_THRESHOLD 0.001f

// Maximum distance between contact points over two updates
// for them to be considered the same contact
#define PHYS_CONTACT_UPDATE_THRESHOLD 1.0f * PHYS_TIMESTEP

// Lagrangian sum multiplier for warm start
#define PHYS_WARM_START_LAMBDA_MULTIPLIER 0.2f


// Maximum iteration count for collision algorithms
#define PHYS_MPR_MAX_ITER 15
#define PHYS_EPA_MAX_ITER 20

// Return from EPA if contact distance is within this threshold
// Higher values are slightly more stable but less accurate
#define PHYS_EPA_THRESHOLD 0.0f

// Baumgarte stabilization factor for collisions
#define NTW_PHYS_BAUMGARTE_FAC 0.05f

// Maximum penetration distance before using Baumgarte stabilization
#define NTW_PHYS_PENETRATION_SLOP 0.005f

// Reduce restitution amount
#define NTW_PHYS_RESTITUTION_SLOP 0.5f
