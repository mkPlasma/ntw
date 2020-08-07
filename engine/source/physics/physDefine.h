#pragma once


// Rigid body physics update timing
#define NTW_PHYS_UPDATES_PER_SECOND 60
#define NTW_PHYS_UPDATE_TIME_MICRO	(1000000 / NTW_PHYS_UPDATES_PER_SECOND)

// Rigid body physics time delta
#define NTW_PHYS_TIME_DELTA	(1.0f / NTW_PHYS_UPDATES_PER_SECOND)


// Margin to enlarge parent AABBs in AABB tree by
#define NTW_AABB_MARGIN 0.2f

// Margin to enlarge portal AABBs by
#define NTW_AABB_PORTAL_MARGIN 0.25f


// Maximum number of constraint resolution iterations for rigid bodies
#define NTW_PHYS_MAX_CONSTRAINT_ITER 10

// Threshold for considering a constraint solved
#define NTW_PHYS_CONSTRAINT_THRESHOLD 0.0001f


// Baumgarte stabilization factor for collisions
#define NTW_PHYS_BAUMGARTE_FAC 0.1f

// Maximum penetration distance before using Baumgarte stabilization
#define NTW_PHYS_PENETRATION_SLOP 0.005f

// Reduce restitution amount
#define NTW_PHYS_RESTITUTION_SLOP 0.5f
