#pragma once

enum class PhysicsType{
    NONE,
    STATIC,
    SEMI_DYNAMIC,
    DYNAMIC,
    DYNAMIC_SIMPLE
};

enum class HitboxType{
    NONE,
    MESH,
    PREDEFINED,
    SPHERE
};
