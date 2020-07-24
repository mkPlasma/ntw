#pragma once

/*
 *	player.h
 *
 *	Controllable player object with physics.
 *
 */

#include"physicsObject.h"
#include"core/options.h"

#define NTW_PLAYER_EYE_LEVEL 0.9f


class Player : public PhysicsObject{

    ControlOptions& cOptions_;
    Window& window_;

    float yaw_;
    float pitch_;
    float yawDifference_;
    float pitchDifference_;

    bool noclip_;

    // Eye/camera position
    Vec3 eyePosition_;

    // Directional vectors
    Vec3 look_;
    Vec3 lookRight_;
    Vec3 lookUp_;
    Vec3 move_;

    // Held object properties
    PhysicsObject* heldObject_;

    bool heldObjectUseGravity_;
    PhysicsType heldObjectPhysicsType_;

public:
    Player(World& world, Model* model, ControlOptions& cOptions, Window& window);

    void updatePlayer(float timeDelta);

    void updateLookVectors();


    float getYaw();
    float getPitch();

    const Vec3& getEyePosition();

    const Vec3& getLookVector();
    const Vec3& getLookRightVector();
    const Vec3& getLookUpVector();
};
