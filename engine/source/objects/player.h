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

    Vec3 look_;
    Vec3 lookLat_;

public:
    Player(ControlOptions& cOptions, Window& window);

    void initPlayer();
    void updatePlayer(const bool& updatePhysics);

    void updateLookVectors();


    float getYaw();
    float getPitch();
};
