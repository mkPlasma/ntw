#pragma once

/*
 *	player.h
 *
 *	Controllable player object with physics.
 *
 */

#include"physicsObject.h"
#include"core/options.h"


class Player : public PhysicsObject{

    ControlOptions& cOptions_;
    Window& window_;

    float yaw_;
    float pitch_;
    
    bool noclip_;

    // Directional vectors
    Vec3 look_;
    Vec3 lookRight_;
    Vec3 lookUp_;
    Vec3 move_;

public:
    Player(ControlOptions& cOptions, Window& window);

    void initPlayer();
    void updatePlayer(bool updatePhysics);

    void updateLookVectors();


    float getYaw();
    float getPitch();

    const Vec3& getLookVector();
    const Vec3& getLookRightVector();
    const Vec3& getLookUpVector();
};
