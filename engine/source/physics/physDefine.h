#pragma once


// Physics update timing
#define PHYS_UPDATES_PER_SECOND 30
#define PHYS_UPDATE_TIME_MICRO	1000000/PHYS_UPDATES_PER_SECOND

// Physics timestep multiplier
// TEMPORARY! use a variable later
// KEEP AS FLOAT !!!
#define PHYS_TIMESCALE 1.0f

// Physics timestep value 'dt'
#define PHYS_TIMESTEP (PHYS_TIMESCALE / PHYS_UPDATES_PER_SECOND)

// Delta multiplier for rendering
#define PHYS_DELTA_MULT (PHYS_TIMESCALE * PHYS_UPDATES_PER_SECOND)


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
#define PHYS_BAUMGARTE_FAC 0.2f

// Maximum penetration distance before using Baumgarte stabilization
#define PHYS_PENETRATION_SLOP 0.00f

// Reduce restitution amount
#define PHYS_RESTITUTION_SLOP 0.0f
